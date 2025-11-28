import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Set, Union, Any

# 导入基类和类型定义
from .base import PathPlannerBase, NodeType

# --- 全局常量定义 ---
# 浮点数比较容差 (Epsilon)
# 用于判断两个浮点数是否"相等"或比较大小，解决计算机浮点数精度丢失问题。
EPSILON = 1e-4
# 无穷大常量
INF = float("inf")


class AStarPlanner(PathPlannerBase[NodeType]):
    """
    【A* 路径规划器 (通用 2D/3D 版)】
    (A* Path Planner - Generic 2D/3D Implementation)

    算法简介:
    A* (A-Star) 是一种启发式搜索算法，它通过评估函数 f(n) = g(n) + h(n) 来选择路径。
    - g(n): 从起点到当前节点 n 的实际移动代价。
    - h(n): 从当前节点 n 到终点的预估代价 (启发式)。
    - f(n): 经过节点 n 到达终点的总预估代价。

    设计理念:
    - **维度无关性**: 本类不硬编码 2D 或 3D 逻辑，而是根据传入节点的坐标维度自动适配。
    - **加权 A***: 支持 heuristic_weight 参数，权重 > 1.0 时变成加权 A*，牺牲最优性换取速度。
    """

    def __init__(
        self,
        grid: Any,
        width_m: float,
        length_m: float,
        height_m: float = 0.0,
        resolution: float = 0.5,
        logger: Optional[logging.Logger] = None,
        grid_lock=None,
        use_octile_3d: bool = False,
        heuristic_weight: float = 1.0,
    ):
        """
        初始化 A* 规划器。

        Args:
            grid: 网格数据引用 (0=空, 1=障碍)。
            width_m, length_m, height_m: 场地的物理尺寸(米)。
            resolution: 网格分辨率(米)。
            logger: 日志记录器。
            grid_lock: 线程锁，用于多线程环境。
            use_octile_3d: 是否在 3D 下使用 Octile 距离 (更精确但稍慢)，False 则用欧几里得距离。
            heuristic_weight: 启发式权重 (默认为 1.0，即标准 A*)。
        """
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )
        self.use_octile_3d = use_octile_3d
        # 确保权重至少为 1.0，否则 A* 无法保证最优性 (虽然在这个工程应用中我们更关心速度)
        self.heuristic_weight = max(1.0, heuristic_weight)

        # 记录起点和终点，用于 initialize 和后续重规划
        self.start_node: Optional[NodeType] = None
        self.goal_node: Optional[NodeType] = None

        # 预定义移动代价常量，避免重复计算开方
        self.COST_1 = 1.0  # 直线移动代价 (1格)
        self.COST_2 = 1.41421356  # 2D 对角线移动代价 (√2)
        self.COST_3 = 1.73205081  # 3D 对角线移动代价 (√3)

        # 计算总格点数 (Total Voxels)
        total_grid_size = self.rows * self.cols * self.layers

        # 设定动态阈值
        # 逻辑：至少允许遍历全图 5 次，且最低不少于 5000 次 (防止微型地图瞬间报错)
        self.max_nodes_expanded = max(5000, total_grid_size * 5)

        # 打印日志告知当前的熔断限制
        self.logger.info(
            f"[A* Config] 动态熔断阈值已设定: {self.max_nodes_expanded} (地图大小: {total_grid_size})"
        )

        self.logger.info(
            f"[A* Init] 初始化完成. 模式: {'3D' if self.layers > 0 else '2D'}, "
            f"启发式权重: W={self.heuristic_weight:.2f}, "
            f"距离算法: {'Octile' if self.use_octile_3d else 'Euclidean'}"
        )

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """
        初始化规划任务。检查起终点是否合法。

        Args:
            start: 起点坐标 (x, y) 或 (x, y, z)
            goal: 终点坐标

        Returns:
            bool: 如果起终点有效返回 True，否则 False。
        """
        # 基础非空检查
        if start is None or goal is None:
            self.logger.error("[A* Init] 初始化失败: 起点或终点为 None。")
            return False

        # 1. 检查坐标是否越界 (Out of Bounds)
        if not self.is_valid(start):
            self.logger.error(f"[A* Init] 初始化失败: 起点 {start} 超出地图边界。")
            return False
        if not self.is_valid(goal):
            self.logger.error(f"[A* Init] 初始化失败: 终点 {goal} 超出地图边界。")
            return False

        # 2. 检查是否在障碍物内 (Collision Check)
        if self.is_obstacle(start):
            self.logger.warning(
                f"[A* Init] 警告: 起点 {start} 位于障碍物内部 (可能导致无解)。"
            )
            # 这里我们只警告，不强制返回 False，以便后续支持“脱困”逻辑
            # return False

        if self.is_obstacle(goal):
            self.logger.error(
                f"[A* Init] 初始化失败: 终点 {goal} 位于障碍物内部，无法到达。"
            )
            return False

        self.start_node = start
        self.goal_node = goal

        self.logger.info(f"[A* Init] 任务设定成功: {start} -> {goal}")
        return True

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """
        处理障碍物更新。

        注意:
        A* 是静态重规划算法 (Static Re-planning)。
        这意味着每次障碍物变化，都需要从头重新计算整个路径，无法像 D* Lite 那样复用上次的搜索树。
        因此，此函数在 A* 中通常不执行具体逻辑，或者仅仅是打印一条日志。
        """
        if changes:
            self.logger.debug(
                f"[A* Update] 检测到 {len(changes)} 个环境变化，下次 plan() 将基于新地图计算。"
            )
        pass

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """
        执行 A* 寻路算法。

        Args:
            current_pos: 机器人当前位置 (通常等于 self.start_node，但在动态过程中可能不同)。

        Returns:
            List[NodeType]: 包含从 current_pos 到 goal_node 的完整路径点列表。
            None: 如果无法到达终点。
        """
        start_node = current_pos
        goal_node = self.goal_node

        # 如果没有初始化终点，无法规划
        if goal_node is None:
            self.logger.error("[A* Core] 失败: 未设置终点 (goal_node is None)。")
            return None

        self.logger.debug(f"[A* Start] 开始搜寻: {start_node} -> {goal_node}")

        # --- 1. 数据结构初始化 ---

        # 开放列表 (Open Set): 存储待检查的节点。
        # 使用优先队列 (Min-Heap)，按 f_score 排序，确保每次弹出的都是预估代价最小的节点。
        # 元素格式: (f_score, node_coordinate)
        open_set: List[Tuple[float, NodeType]] = []
        heapq.heappush(open_set, (0.0, start_node))

        # 来源记录 (Came From): 用于在找到终点后回溯路径。
        # Key: 当前节点, Value: 父节点 (从哪个节点走过来的)
        came_from: Dict[NodeType, NodeType] = {}

        # G值表 (G Score): 从起点到当前节点的已知最小代价。
        # 默认值为无穷大常量 INF
        g_score: Dict[NodeType, float] = {start_node: 0.0}

        # 开放列表哈希表 (Open Set Hash): 用于 O(1) 快速检查节点是否在 open_set 中。
        # 注意: heapq 只能快速 push/pop，检查元素存在性是 O(N) 的，所以需要这个辅助集合。
        open_set_hash: Set[NodeType] = {start_node}

        # --- 2. 搜索循环 ---

        # 安全限制: 防止在无解或地图过大时无限死循环卡死系统
        MAX_NODES_EXPANDED = self.max_nodes_expanded
        nodes_expanded = 0

        while open_set:
            # 2.1 弹出 F 值最小的节点 (贪心策略)
            # current_f 是 f(n)，current 是节点坐标
            current_f, current = heapq.heappop(open_set)

            # 因为是 Lazy Deletion (懒惰删除)，堆中可能存在同一个节点的旧记录
            # 通过检查 open_set_hash 来确保处理的是有效记录
            if current in open_set_hash:
                open_set_hash.remove(current)
            else:
                # 这是一个过期的条目 (该节点已经以更小的代价被处理过了)，直接跳过
                continue

            nodes_expanded += 1

            # 2.2 检查是否到达终点
            if current == goal_node:
                self.logger.info(
                    f"[A* Success] 路径规划成功! 耗时节点数: {nodes_expanded}, "
                    f"最终代价(G): {g_score[current]:.2f}"
                )
                self.stats["nodes_expanded"] = nodes_expanded
                return self._reconstruct_path(came_from, current)

            # 2.3 安全熔断
            if nodes_expanded > MAX_NODES_EXPANDED:
                self.logger.error(
                    f"[A* Fail] 搜索节点数超过上限 ({MAX_NODES_EXPANDED})，强制停止。可能路径被完全封死。"
                )
                return None

            # 2.4 扩展邻居 (Expand Neighbors)
            # 获取当前节点的所有合法邻居及其移动代价
            for neighbor, move_cost in self._get_neighbors(current):
                # tentative_g: 经由当前节点到达邻居的 G 值
                tentative_g = g_score[current] + move_cost

                # 如果这条路径比已知到达该邻居的路径更短 (Relaxation / 松弛操作)
                # 使用 EPSILON 进行浮点数比较，确保数值稳定性
                if tentative_g < g_score.get(neighbor, INF) - EPSILON:
                    # 更新记录
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    # 计算 F 值 = 新 G 值 + 启发式 H 值
                    f = tentative_g + self._heuristic(neighbor, goal_node)

                    # 如果邻居不在待检查列表中，加入
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f, neighbor))
                        open_set_hash.add(neighbor)

        # --- 3. 循环结束仍未找到终点 ---
        self.logger.warning(
            f"[A* Fail] Open Set 耗尽，未找到路径。搜索节点数: {nodes_expanded}。起点或终点可能被障碍物完全包围。"
        )
        self.stats["nodes_expanded"] = nodes_expanded
        return None

    def _heuristic(self, a: NodeType, b: NodeType) -> float:
        """
        启发式函数 h(n): 估算从节点 a 到节点 b 的代价。

        策略:
        - 2D: 使用 Octile 距离 (允许对角线) 或 Manhattan (只允许直角)。
        - 3D: 使用 Octile 3D 或 Euclidean (欧几里得)。
        """
        h = 0.0
        dims = len(a)

        if dims == 2:
            # 2D 启发式
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            # Octile Distance (八方向距离):
            # 假设直行代价 1，斜行代价 √2
            # 公式: (dx + dy) + (sqrt(2) - 2) * min(dx, dy)
            h = (dx + dy) + (self.COST_2 - 2) * min(dx, dy)
        else:
            # 3D 启发式
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            dz = abs(a[2] - b[2])

            if self.use_octile_3d:
                # Octile 3D (26方向距离估算):
                # 将 dx, dy, dz 从小到大排序，分别对应三个维度的移动分量
                delta = sorted([dx, dy, dz])
                # 最小的 delta[0] 部分使用 3D 对角线 (√3)
                # 中间的 (delta[1] - delta[0]) 部分使用 2D 对角线 (√2)
                # 最大的 (delta[2] - delta[1]) 部分使用直线 (1)
                h = (
                    delta[0] * self.COST_3
                    + (delta[1] - delta[0]) * self.COST_2
                    + (delta[2] - delta[1]) * self.COST_1
                )
            else:
                # Euclidean Distance (欧几里得距离):
                # 直接计算直线距离。通常比 Octile 稍小，保证 admissible，但扩展节点可能更多。
                h = math.sqrt(dx * dx + dy * dy + dz * dz)

        # 应用启发式权重 (Weighted A*)
        # 权重 > 1 会让算法更倾向于向着目标冲(Greedy)，从而搜索更快，但路径可能不是最短。
        return h * self.heuristic_weight

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
        """
        获取当前节点的合法邻居。

        包含:
        1. 边界检查 (is_valid)。
        2. 障碍物检查 (is_safe)。
        3. 防切角检查 (Corner Cutting Prevention): 防止穿过两个对角障碍物的缝隙。

        Returns:
            List[(邻居坐标, 移动代价)]
        """
        neighbors = []
        dims = len(node)

        if dims == 2:
            # --- 2D 邻居生成 (8邻域) ---
            # 强制转换坐标为 int，防止 float 坐标传入导致索引错误
            r, c = int(node[0]), int(node[1])
            # 8个方向的偏移量和基础代价
            moves = [
                (0, 1, self.COST_1),  # 右
                (0, -1, self.COST_1),  # 左
                (1, 0, self.COST_1),  # 下
                (-1, 0, self.COST_1),  # 上
                (1, 1, self.COST_2),  # 右下
                (1, -1, self.COST_2),  # 左下
                (-1, 1, self.COST_2),  # 右上
                (-1, -1, self.COST_2),  # 左上
            ]
            for dr, dc, cost in moves:
                nr, nc = r + dr, c + dc

                # 1. 基础合法性检查 (越界或撞墙)
                if not self.is_safe((nr, nc)):
                    continue

                # 2. 严格防切角 (Strict Corner Check)
                # 如果是斜向移动 (dr, dc 都不为0)，需要检查相邻的两个直角格是否阻挡
                # 例如：从 (0,0) 走到 (1,1)，如果 (0,1) 或 (1,0) 是障碍物，则物理上无法穿过
                if dr != 0 and dc != 0:
                    if self.is_obstacle((r + dr, c)) or self.is_obstacle((r, c + dc)):
                        continue

                neighbors.append(((nr, nc), cost))  # type: ignore
        else:
            # --- 3D 邻居生成 (26邻域) ---
            # 强制转换坐标为 int，防止 float 坐标传入导致索引错误
            x, y, z = int(node[0]), int(node[1]), int(node[2])
            # 遍历 x, y, z 三个维度的 -1, 0, 1 偏移
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        # 跳过自己
                        if dx == 0 and dy == 0 and dz == 0:
                            continue

                        nx, ny, nz = x + dx, y + dy, z + dz

                        # 1. 基础合法性检查
                        if not self.is_safe((nx, ny, nz)):
                            continue

                        # 2. 3D 防切角 (简化版: 仅检查 XY 平面切角)
                        # 在起重机场景中，Z轴通常是垂直起降，不太会出现复杂的空间斜穿切角。
                        # 这里主要防止在同一高度层(Z不变)或斜向爬升时穿墙。
                        if dx != 0 and dy != 0:
                            # 检查水平对角阻挡
                            if self.is_obstacle((x + dx, y, z)) or self.is_obstacle(
                                (x, y + dy, z)
                            ):
                                continue

                        # 计算移动代价
                        dist_sq = dx * dx + dy * dy + dz * dz
                        cost = (
                            self.COST_1
                            if dist_sq == 1
                            else (self.COST_2 if dist_sq == 2 else self.COST_3)
                        )
                        neighbors.append(((nx, ny, nz), cost))  # type: ignore

        return neighbors

    def _reconstruct_path(
        self, came_from: Dict[NodeType, NodeType], current: NodeType
    ) -> List[NodeType]:
        """
        从终点回溯到起点，重建路径。

        Args:
            came_from: 路径回溯字典。
            current: 终点节点。

        Returns:
            List[NodeType]: 从起点到终点的正序路径。
        """
        path = [current]

        # 设置最大回溯步数，防止因数据结构异常导致的死循环
        # 使用 came_from 的大小作为合理上限的参考
        max_steps = len(came_from) + 100
        steps = 0

        # 不断查找父节点，直到回溯到起点 (起点不在 came_from 中)
        while current in came_from:
            current = came_from[current]
            path.append(current)

            # 死循环检查
            steps += 1
            if steps > max_steps:
                self.logger.error(
                    "[A* Path] 严重错误: 路径重建检测到死循环，强制截断。"
                )
                break

        # 翻转路径，使其变为 Start -> Goal
        return path[::-1]
