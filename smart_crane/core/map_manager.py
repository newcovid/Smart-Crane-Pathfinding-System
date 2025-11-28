import math
import threading
import copy
import logging
from typing import List, Tuple, Optional, Dict, Union, Any

# 尝试导入科学计算库 numpy 和 scipy
# 作用: 这些库处理矩阵运算非常快。如果没有安装，我们会降级使用 Python 原生列表，但速度会慢很多。
try:
    import numpy as np
    from scipy.ndimage import distance_transform_edt, maximum_filter

    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# --- 类型别名定义 (方便阅读) ---
# 2D 网格: 一个由 0 和 1 组成的二维列表 (0=空地, 1=障碍)
Grid2D = List[List[int]]
# 3D 网格: 一个三维列表 (Z轴/高度 是第三维)
Grid3D = List[List[List[int]]]


class WorkshopMapManager:
    """
    【车间地图管理器 (Workshop Map Manager)】

    角色定位:
    它是整个系统的"地理数据库" (Ground Truth)。
    它不关心具体的寻路算法怎么跑 (怎么走)，它只负责维护"世界长什么样"。

    核心职责:
    1. **数据存储**: 记录场地大小、分辨率，以及所有障碍物的位置和尺寸。
    2. **坐标翻译**: 在"物理坐标 (米)"和"网格坐标 (格子索引)"之间进行转换。
    3. **碰撞裁判**: 提供最底层的几何碰撞检测，判断某点是否撞到了东西。
    4. **网格生成**: 为寻路算法生成 2D 平面图或 3D 体素图，并负责处理"障碍物膨胀"。

    关键特性:
    - **线程安全**: 使用锁 (Lock)，允许多个线程同时读取，但修改时必须排队，防止数据错乱。
    - **缓存机制**: 生成网格计算量大，所以会把结果存起来 (Cache)，除非障碍物变了，否则直接用旧的。
    """

    def __init__(
        self,
        width_m: float,
        length_m: float,
        resolution_m: float,
        height_m: float = 20.0,
        logger: Optional[logging.Logger] = None,
    ):
        """
        初始化地图管理器。

        Args:
            width_m (float): 场地的物理宽度 (X轴)，单位: 米。
            length_m (float): 场地的物理长度 (Y轴)，单位: 米。
            resolution_m (float): 网格分辨率。例如 0.5 表示 1个格子代表 0.5米。
                                  分辨率越小，精度越高，但计算量呈指数级增长。
            height_m (float): 场地的物理高度 (Z轴)，单位: 米。
            logger (logging.Logger, optional): 日志记录器。如果不传，自动创建一个名为 "MapManager" 的记录器。
        """
        self.width_m = width_m
        self.length_m = length_m
        self.height_m = height_m
        self.resolution_m = resolution_m

        # [规范化日志] 统一日志初始化逻辑
        # 如果外部没传 logger，就自己创建一个名为 "MapManager" 的 logger
        # 这样在控制台日志中可以清晰地区分这是来自 [MapManager] 的消息
        self.logger = logger or logging.getLogger("MapManager")

        # [线程锁] Reentrant Lock (可重入锁)
        # 就像一个房间的钥匙，保证同一时间只有一个人能修改障碍物数据，防止数据冲突。
        self._lock = threading.RLock()

        # 计算网格的维度 (行数、列数、层数)
        # math.ceil 是向上取整，确保网格能完全包住物理空间 (例如 10.1米 需要 11个 1米的格子)
        self.cols = int(math.ceil(width_m / resolution_m))  # X轴方向的格子数
        self.rows = int(math.ceil(length_m / resolution_m))  # Y轴方向的格子数
        self.layers = int(math.ceil(height_m / resolution_m))  # Z轴方向的层数

        # 障碍物字典: 用 ID 存储具体的尺寸信息
        self.static_obstacles: Dict[str, dict] = {}  # 静态障碍 (如墙壁、柱子)
        self.dynamic_obstacles: Dict[str, dict] = {}  # 动态障碍 (如移动的货物)

        # 缓存容器: 存储计算好的网格，避免重复计算
        # Key 是 (膨胀半径, 高度限制等参数), Value 是网格数据
        self._inflated_grid_caches: Dict[Tuple, Grid2D] = {}
        self._3d_grid_caches: Dict[Tuple, Grid3D] = {}

        self.logger.info(
            f"[MapMgr] 地图初始化完成. "
            f"物理尺寸: {self.width_m}x{self.length_m}x{self.height_m}m, "
            f"网格尺寸: {self.cols}x{self.rows}x{self.layers} "
            f"(精度: {self.resolution_m}m/格)"
        )

    def get_full_state(self) -> Dict[str, Any]:
        """
        获取地图的全量状态快照 (线程安全)。
        通常用于传给前端进行渲染，让用户看到当前的地图状态。
        """
        with self._lock:
            return {
                "width_m": self.width_m,
                "length_m": self.length_m,
                "height_m": self.height_m,
                "resolution_m": self.resolution_m,
                # 使用 deepcopy 防止外部修改影响内部数据 (防御性编程)
                "static_obstacles": copy.deepcopy(self.static_obstacles),
                "dynamic_obstacles": copy.deepcopy(self.dynamic_obstacles),
            }

    def _invalidate_cache(self) -> None:
        """
        [内部方法] 清空所有缓存。
        当添加、删除或修改障碍物时必须调用，强制下次获取网格时重新计算。
        """
        self._inflated_grid_caches.clear()
        self._3d_grid_caches.clear()
        self.logger.debug(
            "[MapMgr] 障碍物发生变更，所有网格缓存已清空 (Cache Invalidated)"
        )

    # =========================================================================
    # 1. 坐标转换 (Coordinate Transformation)
    # =========================================================================

    def world_to_grid(
        self, x_m: float, y_m: float, z_m: float = 0.0
    ) -> Tuple[int, int, int]:
        """
        将物理坐标 (米) 转换为网格索引 (第几个格子)。

        原理: 索引 = 物理距离 / 分辨率
        例如: 5.5米 / 0.5分辨率 = 第11个格子
        """
        col = int(x_m / self.resolution_m)
        row = int(y_m / self.resolution_m)
        layer = int(z_m / self.resolution_m)

        # 限制范围 (Clamp)，防止索引越界导致程序崩溃
        # 比如算出 -1 或者超过最大列数，强行拉回到有效范围内
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        layer = max(0, min(layer, self.layers - 1))

        return (row, col, layer)

    def grid_to_world(
        self, row: int, col: int, layer: int = 0
    ) -> Tuple[float, float, float]:
        """
        将网格索引还原为物理坐标 (取网格中心点)。

        原理: 物理距离 = (索引 + 0.5) * 分辨率
        为什么要 +0.5？因为我们希望得到的坐标位于格子的正中心，而不是左上角。
        """
        x_m = (col + 0.5) * self.resolution_m
        y_m = (row + 0.5) * self.resolution_m
        z_m = (layer + 0.5) * self.resolution_m
        return (x_m, y_m, z_m)

    # =========================================================================
    # 2. 障碍物管理 (Obstacle Management)
    # =========================================================================

    def add_static_obstacle(
        self, obs_id: str, x: float, y: float, w: float, h: float, z: float = 100.0
    ):
        """
        添加静态障碍物 (如墙壁、柱子)。
        静态障碍物通常是不动的，变动频率低。
        """
        with self._lock:
            self.static_obstacles[obs_id] = {
                "x_m": x,
                "y_m": y,  # 左下角坐标
                "w_m": w,
                "h_m": h,  # 长宽
                "z_m": z,  # 高度 (默认为 100米，视为无限高)
            }
            self._invalidate_cache()  # 数据变了，缓存作废
            self.logger.info(
                f"[MapMgr] 新增静态障碍: ID={obs_id}, Pos=({x},{y}), Size={w}x{h}x{z}"
            )

    def remove_static_obstacle(self, obs_id: str):
        """移除静态障碍物。"""
        with self._lock:
            if obs_id in self.static_obstacles:
                del self.static_obstacles[obs_id]
                self._invalidate_cache()
                self.logger.info(f"[MapMgr] 移除静态障碍: ID={obs_id}")

    def update_dynamic_obstacle(
        self, obs_id: str, x: float, y: float, w: float, h: float, z: float = 100.0
    ):
        """
        更新动态障碍物 (如移动的货物)。
        如果 ID 不存在则自动创建，如果存在则更新位置。
        """
        with self._lock:
            self.dynamic_obstacles[obs_id] = {
                "x_m": x,
                "y_m": y,
                "w_m": w,
                "h_m": h,
                "z_m": z,
            }
            self._invalidate_cache()
            # 动态障碍物更新频率很高，通常只打 debug 日志，避免刷屏
            self.logger.debug(f"[MapMgr] 更新动态障碍: {obs_id}")

    def remove_dynamic_obstacle(self, obs_id: str):
        """移除动态障碍物。"""
        with self._lock:
            if obs_id in self.dynamic_obstacles:
                del self.dynamic_obstacles[obs_id]
                self._invalidate_cache()
                self.logger.info(f"[MapMgr] 移除动态障碍: {obs_id}")

    def find_obstacle_near(self, x_m: float, y_m: float) -> Optional[Tuple[str, str]]:
        """
        在指定位置查找障碍物 (用于鼠标点击移除功能)。
        优先查找动态障碍物，因为它们通常叠在静态障碍物上面。

        Returns:
            (obs_id, obs_type) 或 None
        """
        with self._lock:
            # 1. 先找动态的
            for oid, o in self.dynamic_obstacles.items():
                if (
                    o["x_m"] <= x_m <= o["x_m"] + o["w_m"]
                    and o["y_m"] <= y_m <= o["y_m"] + o["h_m"]
                ):
                    return (oid, "dynamic")
            # 2. 再找静态的
            for oid, o in self.static_obstacles.items():
                if (
                    o["x_m"] <= x_m <= o["x_m"] + o["w_m"]
                    and o["y_m"] <= y_m <= o["y_m"] + o["h_m"]
                ):
                    return (oid, "static")
            return None

    # =========================================================================
    # 3. 几何碰撞检测 (Geometric Collision Detection - Truth Layer)
    # =========================================================================

    def check_collision_raw(
        self,
        x: float,
        y: float,
        z: float,
        xy_margin: float,
        z_margin: float,
        ignore_z: bool = False,
    ) -> bool:
        """
        [真理层] 基于连续 3D 几何的精确碰撞检测。

        更新说明:
        从"AABB包围盒检测"升级为"点到矩形的欧几里得距离检测"。
        这允许安全区域在角落处呈现圆角 (Rounded Rectangle)，从而允许路径更贴近拐角，
        极大地提高了捷径优化 (Greedy Shortcut) 的成功率。

        Args:
            x, y, z: 待检查点的物理坐标。
            xy_margin: 水平安全距离 (膨胀半径)。通常是吊具的旋转半径。
            z_margin: 垂直安全距离。
            ignore_z: 是否忽略高度。

        Returns:
            bool: True 表示**发生碰撞** (不安全)，False 表示安全。
        """
        # 1. 边界检测 (Wall Collision)
        if x - xy_margin < 0 or x + xy_margin > self.width_m:
            return True
        if y - xy_margin < 0 or y + xy_margin > self.length_m:
            return True

        if z + z_margin > self.height_m or z - z_margin < 0:
            pass  # 暂时不处理天地碰撞

        xy_margin_sq = xy_margin * xy_margin

        with self._lock:
            all_obs = list(self.static_obstacles.values()) + list(
                self.dynamic_obstacles.values()
            )

            for o in all_obs:
                # --- 优化后的 XY 平面检测 (Rounded Corner Check) ---
                # 寻找障碍物矩形上距离测试点 (x, y) 最近的点 (closest_x, closest_y)
                obs_min_x, obs_max_x = o["x_m"], o["x_m"] + o["w_m"]
                obs_min_y, obs_max_y = o["y_m"], o["y_m"] + o["h_m"]

                # Clamp 函数：把点拉回到矩形范围内
                closest_x = max(obs_min_x, min(x, obs_max_x))
                closest_y = max(obs_min_y, min(y, obs_max_y))

                # 计算距离平方
                dist_x = x - closest_x
                dist_y = y - closest_y
                dist_sq = dist_x * dist_x + dist_y * dist_y

                # 如果点在障碍物内部，dist_sq 为 0。
                # 碰撞条件: 距离 < 安全半径
                if dist_sq < xy_margin_sq:
                    # XY 平面发生碰撞 (或处于危险半径内)

                    # 如果忽略 Z 轴，直接判定为撞车
                    if ignore_z:
                        return True

                    # 否则检查 Z 轴高度
                    obs_z = o.get("z_m", 100.0)
                    if z - z_margin <= obs_z:
                        return True

            return False  # 安全

    # =========================================================================
    # 4. 网格生成与处理 (Grid Generation & Inflation)
    # =========================================================================

    def _mark_obstacle_area(self, grid: Grid2D, x: float, y: float, w: float, h: float):
        """
        [辅助方法] 在 2D 网格上把一个矩形区域涂黑 (设为 1)。
        将物理世界的矩形映射到网格矩阵中。
        """
        r_s, c_s, _ = self.world_to_grid(x, y)
        # -0.01 是为了处理边界精度问题
        r_e, c_e, _ = self.world_to_grid(x + w - 0.01, y + h - 0.01)

        r_s, r_e = max(0, r_s), min(self.rows - 1, r_e)
        c_s, c_e = max(0, c_s), min(self.cols - 1, c_e)

        # 双重循环填格子
        for r in range(r_s, r_e + 1):
            for c in range(c_s, c_e + 1):
                grid[r][c] = 1

    def _get_base_grid_2d(
        self, check_z_height: Optional[float], z_safety_margin: float
    ) -> Grid2D:
        """
        获取基础的 2D 投影网格 (未膨胀)。
        只包含那些高度超过 check_z_height 的障碍物。
        """
        # 初始化全 0 (空) 网格
        grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        all_obs = list(self.static_obstacles.values()) + list(
            self.dynamic_obstacles.values()
        )
        z_threshold = None
        if check_z_height is not None:
            z_threshold = check_z_height - z_safety_margin

        for o in all_obs:
            obs_h = o.get("z_m", 100.0)
            # 如果障碍物够高，或者我们不关心高度，就把它画上去
            if z_threshold is None or obs_h > z_threshold:
                self._mark_obstacle_area(grid, o["x_m"], o["y_m"], o["w_m"], o["h_m"])
        return grid

    def get_2d_projection_grid(
        self, xy_margin: float, check_z: Optional[float] = None, z_margin: float = 0.0
    ) -> Grid2D:
        """
        [核心接口] 获取带有膨胀层的 2D 投影网格。

        逻辑:
        1. 查缓存 (如果算过就直接给)。
        2. 拿原始网格。
        3. 做数学变换 (膨胀)。
        4. 存缓存并返回。

        Args:
            xy_margin: 水平膨胀格数 (例如 2.5 格)。
            check_z: 巡航高度。只考虑比这个高度高的障碍物。
            z_margin: 垂直安全余量。
        """
        with self._lock:
            # 1. 查缓存 (Memoization)
            # 将参数作为 Key，如果之前算过一模一样的，直接返回，极快！
            key = (round(xy_margin, 3), check_z, round(z_margin, 3))
            if key in self._inflated_grid_caches:
                return self._inflated_grid_caches[key]

            # 2. 计算
            base = self._get_base_grid_2d(check_z, z_margin)
            inflated = _create_inflated_grid_2d(base, xy_margin)

            # 3. 存缓存
            self._inflated_grid_caches[key] = inflated
            return inflated

    def get_3d_voxel_grid(
        self,
        xy_margin: float,
        z_margin_obs: float,
        z_margin_ceil: float,
        is_infinite: bool = False,
    ) -> Grid3D:
        """
        [核心接口] 获取 3D 体素网格 (Voxel Grid)。
        用于全 3D 自由度规划。
        """
        with self._lock:
            key = (
                round(xy_margin, 3),
                round(z_margin_obs, 3),
                round(z_margin_ceil, 3),
                is_infinite,
            )
            if key in self._3d_grid_caches:
                return self._3d_grid_caches[key]

            if not HAS_SCIPY:
                self.logger.warning(
                    "[MapMgr] 未安装 SciPy，无法生成 3D 膨胀网格！返回空网格。"
                )
                return []

            # 1. 构建高度图 (Height Map)
            # 这是一个 2D 数组，每个点的值代表该位置障碍物的高度
            height_map = np.zeros((self.rows, self.cols), dtype=np.float32)
            all_obs = list(self.static_obstacles.values()) + list(
                self.dynamic_obstacles.values()
            )

            for o in all_obs:
                r_s, c_s, _ = self.world_to_grid(o["x_m"], o["y_m"])

                # [修复] 精度边界处理
                # 问题描述: 在2D投影逻辑(_mark_obstacle_area)中，我们使用了 -0.01 的偏移量
                # 来处理正好落在网格线上的坐标。但在原先的 3D 逻辑中缺少这个偏移，导致 3D 障碍物
                # 往往比 2D 障碍物多占一格(0.5m)。对于窄路，这一格就是致命的。
                # 修复: 引入 -1e-4 的 epsilon，保持与 2D 逻辑一致。
                r_e, c_e, _ = self.world_to_grid(
                    o["x_m"] + o["w_m"] - 1e-4, o["y_m"] + o["h_m"] - 1e-4
                )

                # 确定障碍物的"阻挡高度"
                if is_infinite:
                    obs_occupy_z = self.height_m + 1.0  # 无限高
                else:
                    obs_occupy_z = o.get("z_m", 100.0) + z_margin_obs

                # [修复] 范围切片逻辑修正
                # Python 的 slice 是左闭右开的 (start:end 不包含 end)。
                # world_to_grid 返回的是索引。
                # 如果 r_s=0, r_e=1 (跨度2格)，我们希望填 0 和 1。
                # 原逻辑: r_e + 1 -> 2. height_map[0:2] -> 填 0, 1。正确。
                r_s, r_e = max(0, r_s), min(self.rows, r_e + 1)
                c_s, c_e = max(0, c_s), min(self.cols, c_e + 1)

                if r_s < r_e and c_s < c_e:
                    # 在范围内取最大值 (叠加障碍物)
                    height_map[r_s:r_e, c_s:c_e] = np.maximum(
                        height_map[r_s:r_e, c_s:c_e], obs_occupy_z
                    )

            # 2. 高度图膨胀 (XY Plane Inflation)
            if xy_margin > 0:
                # [核心修正] 膨胀卷积核半径补偿
                # 问题: 原逻辑 `radius = ceil(xy_margin)` 并直接用作 mask 半径。
                # EDT 计算的是中心到中心的距离，而安全碰撞需要计算边缘到中心的距离。
                # 差值为 0.5 个格子。
                # 修复: 在 mask 计算中引入 +0.5 的补偿。
                search_radius = int(math.ceil(xy_margin + 0.5))

                # 创建卷积核 (圆形)
                y, x = np.ogrid[
                    -search_radius : search_radius + 1,
                    -search_radius : search_radius + 1,
                ]
                # 判定条件: 距离 <= (安全半径 + 0.5格补偿)
                mask = x**2 + y**2 <= (xy_margin + 0.5) ** 2

                # 使用 maximum_filter 让柱子变粗
                height_map = maximum_filter(
                    height_map,
                    footprint=mask,
                    mode="constant",
                    cval=self.height_m + 1.0,
                )

            # 3. 转换为 3D 体素 (Voxelization)
            # 利用 numpy 广播机制，快速生成 3D 布尔矩阵
            z_coords = (np.arange(self.layers) + 0.5) * self.resolution_m
            is_obstacle = z_coords.reshape(1, 1, -1) < height_map.reshape(
                self.rows, self.cols, 1
            )

            # 检查天花板和地板限制
            is_ceiling_hit = (z_coords + z_margin_ceil) > self.height_m
            is_floor_hit = (z_coords - z_margin_obs) < 0

            final_grid_mask = (
                is_obstacle
                | is_ceiling_hit.reshape(1, 1, -1)
                | is_floor_hit.reshape(1, 1, -1)
            )

            grid_3d = final_grid_mask.astype(np.int8).tolist()
            self._3d_grid_caches[key] = grid_3d
            return grid_3d


def _create_inflated_grid_2d(grid: Grid2D, safety_margin: float) -> Grid2D:
    """
    [独立函数] 创建膨胀后的 2D 网格。
    使用 SciPy 的欧几里得距离变换 (EDT) 算法，这是计算膨胀最快、最科学的方法。
    """
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    if rows == 0:
        return []

    if HAS_SCIPY:
        np_grid = np.array(grid, dtype=np.int8)

        # 墙壁膨胀处理
        # 我们在原始网格外面手动加一圈 "1" (Padding)。
        # 这样算法在计算距离时，就会把边界也视为障碍物，从而实现墙壁的自动膨胀。
        np_grid_padded = np.pad(
            np_grid, pad_width=1, mode="constant", constant_values=1
        )

        # 计算距离场: 算出每个点距离最近障碍物的距离
        # 距离单位: 格子数 (float)
        # 注意: EDT 计算的是 "当前点中心" 到 "障碍物点中心" 的欧氏距离
        feature_mask = (np_grid_padded == 0).astype(int)
        dist_map = distance_transform_edt(feature_mask)

        # 切掉 Padding，还原大小
        dist_map = dist_map[1:-1, 1:-1]

        # [核心修正] 膨胀阈值补偿 (+0.5 Grid)
        # 现象: 分辨率越粗，障碍物边缘的不确定性越大。
        # 原逻辑: dist <= safety_margin
        # 问题: 假设 safety_margin=0.8, 相邻格距离=1.0。1.0 > 0.8 -> 安全。
        #       但物理上，障碍物占据了半个格子，实际净空 = 1.0 - 0.5 = 0.5。
        #       0.5 < 0.8 -> 应该是不安全的！
        # 修复: 判定距离 <= (安全半径 + 0.5格补偿)
        # 效果: 强制保证物理上的边缘净空满足要求，无论分辨率多粗。
        #       这将解决"分辨率1.0时路径存在，0.1时路径消失"的假阴性问题。
        inflated_np = (dist_map <= (safety_margin + 0.5)).astype(int)
        inflated_np = np.maximum(inflated_np, np_grid)
        return inflated_np.tolist()

    return grid
