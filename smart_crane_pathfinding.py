"""
智能起重机寻路模块 (smart_crane_pathfinding)

=== 模块概述 ===
这是一个用于工业起重机 (Gantry Crane/Overhead Crane) 的 2.5D 运动规划模块。
它结合了传统的网格搜索算法 (A*) 和现代路径优化技术 (捷径优化 + 贝塞尔平滑)，
能够在存在静态和动态障碍物的车间环境中生成安全、平滑且高效的路径。

=== 核心特性 ===
1. **配置空间 (C-Space) 生成**:
   - 自动根据起重机尺寸膨胀障碍物。
   - 支持基于 SciPy 的 EDT (欧几里得距离变换) 加速计算。
2. **混合寻路策略**:
   - 阶段一: 膨胀网格上的 A* 搜索 (保证无碰撞)。
   - 阶段二: 贪婪视线优化 (去除多余拐点)。
   - 阶段三: 贝塞尔曲线平滑 (符合机械运动学)。
3. **线程安全**:
   - MapManager 使用线程锁保护共享状态，支持高并发请求。
"""

import heapq
import math
import threading
import copy
import logging
import time
from typing import List, Tuple, Optional, Dict, Callable, Union, Set

# 获取模块级日志记录器
logger = logging.getLogger(__name__)

# === 性能优化依赖 ===
try:
    import numpy as np
    from scipy.ndimage import distance_transform_edt

    HAS_SCIPY = True
    logger.info("检测到 numpy/scipy: 启用高性能网格膨胀算法 (EDT)")
except ImportError:
    HAS_SCIPY = False
    logger.warning("未检测到 numpy/scipy: 性能优化未启用，将回退到纯 Python 慢速算法")

from path_optimizer import GreedyShortcutOptimizer, BezierCurveSmoother

# --- 类型别名定义 ---
Grid = List[List[int]]  # 2D 网格矩阵 (0=空, 1=阻挡)
Point2D = Tuple[int, int]  # 网格坐标 (row, col)
Point3D_M = Tuple[float, float, float]  # 物理世界坐标 (x, y, z) 米
LoggerFunc = Callable[[str], None]  # 日志回调函数类型


# =========================================================================
# === 部分 1: 底层 A* 寻路算法 (Path Solver)
# =========================================================================


def _create_inflated_grid(grid: Grid, safety_margin: Union[int, float]) -> Grid:
    """
    生成膨胀网格 (Configuration Space Generation)。

    === 算法原理 ===
    为了将这一问题简化为“质点运动规划”，我们需要将地图上的所有障碍物
    向外“膨胀”一个起重机的半径距离。这样，只要质点不碰到膨胀后的障碍物，
    实际体积的起重机就不会碰到原始障碍物。

    参数:
        grid: 原始 0/1 网格
        safety_margin: 膨胀半径 (单位: 格子数)。支持浮点数。

    返回:
        膨胀后的网格 (1 代表这一格不可通行，因为离障碍物太近)
    """

    # 如果没有网格,返回空网格
    if not grid or not grid[0]:
        return []

    # 解析行列数
    rows, cols = len(grid), len(grid[0])

    # 记录耗时
    start_time = time.time()

    # --- 方案 A: SciPy EDT 高性能算法 (推荐) ---
    if HAS_SCIPY:
        # 1. 转换 grid 为 numpy 数组 (int8 节省内存)
        # 原 grid 是一个二维数组
        np_grid = np.array(grid, dtype=np.int8)

        # 2. 生成二值掩膜 (0 为障碍物背景)
        # 假设原网格np_grid为:
        # 0 0 1
        # 0 1 1
        # 0 0 0
        # mask将为:
        # True True False
        # True False False
        # True True True
        # 其中True表示空地可以通行(通行区), Fasle表示禁止通行(障碍物)
        # distance_transform_edt 计算每个非零点(True)(通行区)到最近零点(False)(障碍物)的欧氏距离
        #
        mask = np_grid == 0
        dist_map = distance_transform_edt(mask)

        # 3. 阈值过滤
        # 如果某点距离最近障碍物的距离 <= 安全半径，则标记为不可通行(1)
        # 1. (dist_map <= safety_margin): 生成一个布尔网格 (Boolean Mask)。
        #    - True : 该点距离障碍物太近（<= 安全半径），属于危险区域。
        #    - False: 该点距离障碍物足够远，属于安全区域。
        # 2. .astype(int): 将布尔值转换为整数。
        #    - True  -> 1 (表示“不可通行/障碍物”)
        #    - False -> 0 (表示“可通行/空地”)
        #
        # 最终结果 inflated_np: 一个由 0 和 1 组成的膨胀后网格，
        # 其中 1 代表原来的障碍物加上了向外膨胀的一圈“安全缓冲区”。
        inflated_np = (dist_map <= safety_margin).astype(int)

        # 4. 边界处理 (防止撞墙)
        # 强制将地图四周的 safety_margin 范围内设为障碍
        if safety_margin > 0:
            sm_int = int(math.ceil(safety_margin))
            sm_int = min(sm_int, min(rows, cols))
            inflated_np[0:sm_int, :] = 1  # 上边缘
            inflated_np[rows - sm_int : rows, :] = 1  # 下边缘
            inflated_np[:, 0:sm_int] = 1  # 左边缘
            inflated_np[:, cols - sm_int : cols] = 1  # 右边缘

        elapsed = (time.time() - start_time) * 1000
        logger.debug(f"EDT 网格膨胀耗时: {elapsed:.2f}ms")
        return inflated_np.tolist()

    # --- 方案 B: 纯 Python 回退算法 (仅当缺少依赖时使用) ---
    # 这是一个简单的暴力膨胀，效率较低 O(N*M * R^2)
    safety_margin_int = int(math.ceil(safety_margin))
    inflated_grid = [[0 for _ in range(cols)] for _ in range(rows)]

    # 1. 边界填充
    if safety_margin_int > 0:
        for r in range(rows):
            for c in range(cols):
                if (
                    r < safety_margin_int
                    or r >= rows - safety_margin_int
                    or c < safety_margin_int
                    or c >= cols - safety_margin_int
                ):
                    inflated_grid[r][c] = 1

    # 2. 障碍物扩散
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                # 遍历该障碍物周围的矩形区域
                r_min = max(0, r - safety_margin_int)
                r_max = min(rows, r + safety_margin_int + 1)
                c_min = max(0, c - safety_margin_int)
                c_max = min(cols, c + safety_margin_int + 1)
                for nr in range(r_min, r_max):
                    for nc in range(c_min, c_max):
                        inflated_grid[nr][nc] = 1
    return inflated_grid


def _heuristic(a: Point2D, b: Point2D) -> float:
    """
    A* 启发式函数 (Heuristic Function)。

    === 原理: Octile Distance (八方向距离) ===
    由于起重机可以沿对角线移动，我们使用 Octile Distance 而非 Manhattan Distance。

    公式:
    直走步数 = |dx - dy|
    斜走步数 = min(dx, dy)
    总代价 = 直走步数 * 1.0 + 斜走步数 * 1.414

    简化写法: (max - min) + 1.414 * min
    """
    # 如果没有启发式函数 h(n) ,就退化成了 Dijkstra 算法,
    # 像往水里扔石头激起的波纹，一圈一圈均匀地向四周扩散。
    # 它不管终点在东边还是西边，反正四面八方都查一遍，
    # 直到碰到终点。这样做非常慢，因为它检查了大量无关的格子。

    # 假如传入(0,0)和(2,4),则dx为2,dy为4
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    # 返回值则为(4-2)+1.414*2,是一个只允许走直线和对角线斜行的距离,
    return (max(dx, dy) - min(dx, dy)) + 1.414 * min(dx, dy)


def _reconstruct_path(
    came_from: Dict[Point2D, Point2D], current: Point2D
) -> List[Point2D]:
    """从终点回溯到起点重建路径。"""
    path = []

    # 倒推循环：只要当前点在字典的 key 里，说明它有 父节点
    while current in came_from:
        # 先把终点/当前点记下来
        path.append(current)
        # 把当前点再作为父节点,去找父父节点..
        current = came_from[current]

    # 加上起点：循环结束时，current 就是起点（因为起点没有父亲，不在 key 里）
    path.append(current)
    # [终点, ..., 起点]，变成 [起点, ..., 终点]
    return path[::-1]  # 反转列表


def find_crane_path(
    grid: Grid,
    start_pos: Point2D,
    end_pos: Point2D,
    shortcut_optimizer: Optional[GreedyShortcutOptimizer] = None,
) -> Optional[List[Point2D]]:
    """
    执行 A* 寻路算法。

    注意：传入的 grid 必须已经是【膨胀后】的网格。
    如果起点或终点落在膨胀层内 (例如太靠近墙壁)，算法将立即返回失败。

    参数:
        grid: 膨胀后的 2D 网格
        start_pos: 起点 (row, col)
        end_pos: 终点 (row, col)
        shortcut_optimizer: (可选) 路径优化器实例

    返回:
        路径点列表 [(r, c), ...] 或 None
    """
    if not grid or not grid[0]:
        return None
    rows, cols = len(grid), len(grid[0])

    # 1. 越界检查
    if not (0 <= start_pos[0] < rows and 0 <= start_pos[1] < cols):
        logger.warning(f"A* 失败: 起点 {start_pos} 越界")
        return None
    if not (0 <= end_pos[0] < rows and 0 <= end_pos[1] < cols):
        logger.warning(f"A* 失败: 终点 {end_pos} 越界")
        return None

    # 2. 碰撞检查 (检查起终点是否在安全区内)
    if grid[start_pos[0]][start_pos[1]] == 1:
        logger.warning(f"A* 失败: 起点 {start_pos} 位于障碍物或安全缓冲区内")
        return None
    if grid[end_pos[0]][end_pos[1]] == 1:
        logger.warning(f"A* 失败: 终点 {end_pos} 位于障碍物或安全缓冲区内")
        return None

    # 3. A* 初始化
    # open_set: 优先队列，存储待探索节点 (f_cost, node)
    # 起点总代价是0
    open_set = [(0, start_pos)]

    # came_from: 记录路径树，key=当前点, value=父节点
    came_from: Dict[Point2D, Point2D] = {}

    # g_cost: 从起点到当前点的实际代价
    g_cost: Dict[Point2D, float] = {start_pos: 0}

    # f_cost: 估算总代价 (g + h)
    f_cost: Dict[Point2D, float] = {start_pos: _heuristic(start_pos, end_pos)}

    # open_set_hash: 用于快速查找 open_set 中是否包含某节点
    # 集合（Set）是一个无序且不包含重复元素的容器。
    open_set_hash: Set[Point2D] = {start_pos}

    # 路径列表, [(x, y), (x, y), ...]
    raw_path = None
    # 计数器,统计寻找了多少个格子
    nodes_explored = 0

    # 4. 主循环
    while open_set:

        # heappop 取出 f 值最小的节点
        _, current_node = heapq.heappop(open_set)
        # 如果我们在探索过程中发现了一条通往某节点更近的路，
        # 我们需要更新优先队列里那个节点的 f_cost。
        # 但在 Python 的 heapq 库中，
        # “修改队列里某个元素的值”是非常慢的操作。
        # 此处检查,如果节点在 open_set_hash 中,需要处理;
        # open_set_hash 和 open_set 存储所有“我发现了，但还没去探索”的节点。
        if current_node in open_set_hash:
            # 则移除该条目
            open_set_hash.remove(current_node)
        # 如果节点已经不在 open_set_hash 中了,说明已经被处理过了
        else:
            # 这是一个过期的条目 (懒惰删除 lazy removal)
            continue

        nodes_explored += 1

        # 到达终点
        if current_node == end_pos:
            # came_from 示例
            # {
            #     # === 有效路径部分 ===
            #     (2, 3): (2, 2),   # 记录：我是从 (2, 2) 走到 (2, 3) 的
            #     (2, 4): (2, 3),   # 记录：我是从 (2, 3) 走到 (2, 4) 的
            #     (2, 5): (2, 4),   # 记录：我是从 (2, 4) 走到 (2, 5) 的 (到达终点！)

            #     # === 探索过的岔路（A* 尝试过但也记录下来了） ===
            #     (3, 3): (2, 2),   # 记录：(2, 2) 当时也尝试过往右下角走
            #     (1, 2): (2, 2),   # 记录：(2, 2) 当时也尝试过往上走
            #     (3, 4): (2, 3),   # 记录：(2, 3) 当时也尝试过往右下走
            # }
            raw_path = _reconstruct_path(came_from, current_node)
            # 根据 A* 的特性，第一次弹出的终点，一定是最短路径
            break

        # 8 方向邻居 (上下左右 + 4个对角线)
        # (行号的变化, 列号的变化, 移动代价(距离))
        neighbors = [
            (0, 1, 1.0),  # 右
            (0, -1, 1.0),  # 左
            (1, 0, 1.0),  # 下
            (-1, 0, 1.0),  # 上
            (1, 1, 1.414),  # 右下
            (1, -1, 1.414),  # 左下
            (-1, 1, 1.414),  # 右上
            (-1, -1, 1.414),  # 左上
        ]

        for dr, dc, move_cost in neighbors:
            # 遍历8个方向的邻居,获取邻居的当前坐标:(nr,nc),(行号,列号)
            nr, nc = current_node[0] + dr, current_node[1] + dc

            # 4.1 边界与障碍物检查
            # 如果邻居越界了
            if not (0 <= nr < rows and 0 <= nc < cols):
                # 跳过
                continue
            # 如果邻居在障碍物里
            if grid[nr][nc] == 1:
                # 跳过
                continue

            # 4.2 对角线移动的"切角"检查 (Corner Cutting Prevention)
            # 如果想斜着走，必须保证两侧的格子都不是障碍物，否则会卡住
            # 如果代价大于1,表示在遍历斜角的邻居
            if move_cost > 1.0:
                # 例如看向右上角的邻居,必须检查右侧和上侧是否允许通行
                if (
                    grid[current_node[0] + dr][current_node[1]] == 1
                    or grid[current_node[0]][current_node[1] + dc] == 1
                ):
                    # 如果右侧或上侧有障碍物,跳过
                    # 跳过表示不更新代价,即为初始的无穷大
                    continue

            # 4.3 更新代价
            # g_cost 实际代价; f_cost 估算代价; tentative_g 尝试的实际代价
            tentative_g = g_cost[current_node] + move_cost
            # 获取实际代价字典中(nr,nc)坐标的代价,如果没有,返回无穷大
            if tentative_g < g_cost.get((nr, nc), float("inf")):
                # came_from 路径树, key=邻居点((nr, nc)), value=父节点(当前点)
                came_from[(nr, nc)] = current_node
                g_cost[(nr, nc)] = tentative_g
                # new_f 某个节点的总评分,等于实际代价和估算代价,估算代价由启发函数计算
                new_f = tentative_g + _heuristic((nr, nc), end_pos)
                f_cost[(nr, nc)] = new_f
                if (nr, nc) not in open_set_hash:
                    # 把新的评分加入优先队列
                    heapq.heappush(open_set, (new_f, (nr, nc)))
                    open_set_hash.add((nr, nc))

    if not raw_path:
        logger.debug(f"A* 搜索结束: 未找到路径 (探索节点数: {nodes_explored})")
        return None

    logger.debug(
        f"A* 搜索成功: 原始路径长度 {len(raw_path)}, 探索节点数 {nodes_explored}"
    )

    # 5. 阶段二优化：捷径算法 (可选)
    if shortcut_optimizer:
        return shortcut_optimizer.optimize(raw_path, grid)

    return raw_path


# =========================================================================
# === 部分 2: 智能起重机寻路架构 (Architecture)
# =========================================================================


class WorkshopMapManager:
    """
    车间地图管理器 (Thread-Safe Workshop Map Manager)

    职责:
    1. 维护静态和动态障碍物列表。
    2. 管理 2D 网格状态缓存 (避免每次请求都重新栅格化)。
    3. 提供 物理坐标(米) <-> 网格坐标(Grid) 的转换。
    4. 线程安全地处理读写操作。
    """

    def __init__(
        self,
        width_m: float,
        length_m: float,
        resolution_m: float,
        logger: Optional[LoggerFunc] = None,
    ):
        self.width_m = width_m
        self.length_m = length_m
        self.resolution_m = resolution_m
        self.logger = logger or print

        # 线程锁：防止多个请求同时修改障碍物导致数据不一致
        self._lock = threading.RLock()

        # 计算网格尺寸
        self.cols = int(math.ceil(width_m / resolution_m))
        self.rows = int(math.ceil(length_m / resolution_m))

        self.logger(
            f"[MapMgr] 初始化: {self.rows}行 x {self.cols}列 (分辨率: {resolution_m}m/格)"
        )

        # 数据存储
        self.static_obstacles = {}
        self.dynamic_obstacles = {}

        # 缓存系统
        # _base_grid_cache: 仅包含障碍物的原始网格 (无膨胀)
        self._base_grid_cache: Optional[Grid] = None
        # _inflated_grid_caches: 针对不同安全半径的膨胀网格缓存 {margin_float: grid}
        self._inflated_grid_caches: Dict[float, Grid] = {}

    def get_full_state(self) -> Dict:
        """获取当前地图的完整状态快照 (线程安全)。"""
        with self._lock:
            return {
                "width_m": self.width_m,
                "length_m": self.length_m,
                "resolution_m": self.resolution_m,
                "static_obstacles": copy.deepcopy(self.static_obstacles),
                "dynamic_obstacles": copy.deepcopy(self.dynamic_obstacles),
            }

    def _invalidate_cache(self) -> None:
        """清空所有缓存。当障碍物发生变动时调用。"""
        self._base_grid_cache = None
        self._inflated_grid_caches.clear()

    def world_to_grid(self, x_m: float, y_m: float) -> Point2D:
        """物理坐标(米) -> 网格坐标(Row, Col)"""
        col = int(x_m / self.resolution_m)
        row = int(y_m / self.resolution_m)
        # 限制在地图范围内
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return (row, col)

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """网格坐标(Row, Col) -> 物理中心坐标(米)"""
        x_m = (col + 0.5) * self.resolution_m
        y_m = (row + 0.5) * self.resolution_m
        return (x_m, y_m)

    def _mark_obstacle_area(
        self, grid: Grid, x_m: float, y_m: float, w_m: float, h_m: float
    ) -> None:
        """在网格上标记一个矩形障碍物区域。"""
        start_row, start_col = self.world_to_grid(x_m, y_m)
        end_row, end_col = self.world_to_grid(x_m + w_m, y_m + h_m)

        for r in range(start_row, end_row + 1):
            for c in range(start_col, end_col + 1):
                if 0 <= r < self.rows and 0 <= c < self.cols:
                    grid[r][c] = 1

    def add_static_obstacle(
        self, obs_id: str, x_m: float, y_m: float, w_m: float, h_m: float
    ) -> None:
        with self._lock:
            self.logger(f"[MapMgr] 添加静态障碍物: {obs_id}")
            self.static_obstacles[obs_id] = {
                "x_m": x_m,
                "y_m": y_m,
                "w_m": w_m,
                "h_m": h_m,
            }
            self._invalidate_cache()

    def remove_static_obstacle(self, obs_id: str) -> None:
        with self._lock:
            if obs_id in self.static_obstacles:
                del self.static_obstacles[obs_id]
                self._invalidate_cache()

    def update_dynamic_obstacle(
        self, obs_id: str, x_m: float, y_m: float, w_m: float, h_m: float
    ) -> None:
        with self._lock:
            self.dynamic_obstacles[obs_id] = {
                "x_m": x_m,
                "y_m": y_m,
                "w_m": w_m,
                "h_m": h_m,
            }
            self._invalidate_cache()

    def remove_dynamic_obstacle(self, obs_id: str) -> None:
        with self._lock:
            if obs_id in self.dynamic_obstacles:
                del self.dynamic_obstacles[obs_id]
                self._invalidate_cache()

    def find_obstacle_near(self, x_m: float, y_m: float) -> Optional[Tuple[str, str]]:
        """查找指定坐标点内的障碍物 ID 和类型。"""
        with self._lock:
            for obs_id, obs in self.dynamic_obstacles.items():
                if (obs["x_m"] <= x_m <= obs["x_m"] + obs["w_m"]) and (
                    obs["y_m"] <= y_m <= obs["y_m"] + obs["h_m"]
                ):
                    return (obs_id, "dynamic")
            for obs_id, obs in self.static_obstacles.items():
                if (obs["x_m"] <= x_m <= obs["x_m"] + obs["w_m"]) and (
                    obs["y_m"] <= y_m <= obs["y_m"] + obs["h_m"]
                ):
                    return (obs_id, "static")
            return None

    def _get_base_grid(self) -> Grid:
        """获取或重建基础网格 (仅包含原始障碍物)。"""
        if self._base_grid_cache is not None:
            return self._base_grid_cache

        # 初始化全 0 矩阵
        grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

        # 栅格化所有障碍物
        for obs in self.static_obstacles.values():
            self._mark_obstacle_area(
                grid, obs["x_m"], obs["y_m"], obs["w_m"], obs["h_m"]
            )
        for obs in self.dynamic_obstacles.values():
            self._mark_obstacle_area(
                grid, obs["x_m"], obs["y_m"], obs["w_m"], obs["h_m"]
            )

        self._base_grid_cache = grid
        return grid

    def get_inflated_grid(self, safety_margin: Union[int, float]) -> Grid:
        """
        获取膨胀后的网格 (支持缓存)。

        参数:
            safety_margin: 膨胀半径 (格子数)
        """
        with self._lock:
            # 检查是否有对应 margin 的缓存
            if safety_margin in self._inflated_grid_caches:
                return self._inflated_grid_caches[safety_margin]

            self.logger(f"[MapMgr] 计算膨胀网格 (Margin={safety_margin:.2f})...")
            base_grid = self._get_base_grid()

            # 执行耗时的膨胀计算
            inflated = _create_inflated_grid(base_grid, safety_margin)

            # 存入缓存
            self._inflated_grid_caches[safety_margin] = inflated
            return inflated


class IntelligentCranePlanner:
    """
    智能起重机规划器 (High-Level Planner)

    职责:
    1. 协调 MapManager 获取环境数据。
    2. 计算机器人的安全膨胀半径。
    3. 执行 3D -> 2D -> 3D 的规划管线。
    4. 集成路径优化器 (Shortcut + Bezier)。
    """

    def __init__(
        self,
        map_manager: WorkshopMapManager,
        crane_footprint_m: float,
        safe_travel_z_m: float = 5.0,
        enable_shortcut: bool = True,
        enable_bezier: bool = True,
        bezier_smoothness: float = 0.3,
        bezier_segments: int = 10,
        logger: Optional[LoggerFunc] = None,
    ):
        self.map_mgr = map_manager
        self.footprint = crane_footprint_m
        self.safe_z = safe_travel_z_m
        self.logger = logger or print

        # 初始化优化器组件
        self.shortcut_opt = GreedyShortcutOptimizer() if enable_shortcut else None
        self.bezier_opt = (
            BezierCurveSmoother(bezier_smoothness, bezier_segments)
            if enable_bezier
            else None
        )

        # 计算安全半径 (格数)
        # 半径 = 直径 / 2
        # 除以分辨率得到格子数
        # +1e-6 是为了处理浮点数边界误差，确保稍微大一点点
        radius_m = self.footprint / 2
        self.safety_cells_float = (radius_m / self.map_mgr.resolution_m) + 1e-6

        self.logger(
            f"[Planner] 就绪 | 起重机半径: {radius_m}m -> 膨胀安全距: {self.safety_cells_float:.2f} 格"
        )

    def find_path_3d(
        self, start_xyz: Point3D_M, end_xyz: Point3D_M
    ) -> Optional[List[Point3D_M]]:
        """
        执行完整的 3D 路径规划。

        策略:
        1. 投影: 将 3D 起终点投影到 2D 平面上。
        2. 寻路: 在 2D 平面上规划避障路径 (A* + 优化)。
        3. 提升: 将路径点提升到安全高度 (safe_travel_z)。
        4. 衔接: 添加垂直起降动作 (Start -> Lift -> Move -> Drop -> End)。
        """
        sx, sy, sz = start_xyz
        ex, ey, ez = end_xyz

        # 坐标转换: 物理 -> 网格
        start_grid = self.map_mgr.world_to_grid(sx, sy)
        end_grid = self.map_mgr.world_to_grid(ex, ey)

        # 1. 获取膨胀网格 (Configuration Space)
        # 这一步是核心：它确保后续 A* 找到的路径，对于有体积的起重机来说是安全的
        inflated_grid = self.map_mgr.get_inflated_grid(self.safety_cells_float)

        # 2. A* 寻路 + 捷径优化
        path_grid = find_crane_path(
            grid=inflated_grid,
            start_pos=start_grid,
            end_pos=end_grid,
            shortcut_optimizer=self.shortcut_opt,
        )

        if not path_grid:
            self.logger("[Planner] 失败: 未找到可行路径 (A* 返回空)")
            return None

        # 3. 坐标还原: 网格 -> 物理 (Path Digitization)
        path_2d_m = []
        path_2d_m.append((sx, sy))  # 强制保留精确的起点物理坐标
        for r, c in path_grid[1:-1]:
            xm, ym = self.map_mgr.grid_to_world(r, c)
            path_2d_m.append((xm, ym))
        path_2d_m.append((ex, ey))  # 强制保留精确的终点物理坐标

        # 4. 贝塞尔平滑 (带物理碰撞检测)
        if self.bezier_opt:
            # 定义一个闭包函数，用于在贝塞尔生成过程中检查任意物理点是否安全
            def is_safe_point(x_m: float, y_m: float) -> bool:
                r, c = self.map_mgr.world_to_grid(x_m, y_m)
                # 检查越界
                if not (0 <= r < self.map_mgr.rows and 0 <= c < self.map_mgr.cols):
                    return False
                # 检查是否落入膨胀后的障碍区
                # 如果 inflated_grid[r][c] == 1，说明该位置距离障碍物小于安全半径
                if inflated_grid[r][c] == 1:
                    return False
                return True

            # 执行平滑，如果不安全会自动回退到尖角
            path_2d_m = self.bezier_opt.smooth(
                path_2d_m, collision_check_fn=is_safe_point
            )

        # 5. 2.5D 动作合成 (Motion Synthesis)
        # 逻辑: 起点 -> 垂直上升 -> 平面移动(路径) -> 垂直下降 -> 终点
        final_path_3d = []

        # P1: 起点
        final_path_3d.append(start_xyz)

        # P2: 提升点 (仅当当前高度低于安全高度时)
        if sz < self.safe_z:
            final_path_3d.append((sx, sy, self.safe_z))

        # P3...Pn: 巡航路径 (Z = safe_z)
        for px, py in path_2d_m[1:]:
            final_path_3d.append((px, py, self.safe_z))

        # Pn+1: 下降点 (到达终点上方)
        # 如果路径最后一个点已经是 (ex, ey)，我们只需要处理高度
        # 如果终点高度低于安全高度，则需要下降动作
        if ez < self.safe_z:
            # 实际上 path_2d_m[-1] 已经是 (ex, ey)
            # 所以 final_path_3d 目前最后一个点是 (ex, ey, safe_z)
            # 我们只需要追加最终点即可
            final_path_3d.append((ex, ey, ez))
        elif final_path_3d[-1] != end_xyz:
            # 如果终点高度很高(比如在高台上)，直接连过去
            final_path_3d.append(end_xyz)

        return final_path_3d


if __name__ == "__main__":
    # 简单的单元测试入口
    logging.basicConfig(level=logging.DEBUG)
    pass
