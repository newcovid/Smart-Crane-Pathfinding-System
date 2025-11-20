import heapq
import math
from typing import List, Tuple, Dict, Optional, Set
from .base import PathPlannerBase, NodeType, Point2D, Point3D


class AStarPlanner(PathPlannerBase[NodeType]):
    """
    【L1 层 - A* 路径规划器 (加速版)】
    (A* Path Planner - Weighted & 3D Optimized)

    更新说明:
    1. 引入 Heuristic Weight (加权 A*): 大幅减少 3D 空间下的节点搜索量。
    2. 保持 2D/3D 兼容性。
    """

    def __init__(
        self,
        grid,
        width_m,
        length_m,
        height_m=0.0,
        resolution=0.5,
        logger=None,
        grid_lock=None,
        use_octile_3d=False,
        heuristic_weight=1.0,  # [新增] 默认 1.0 为标准 A*
    ):
        """
        Args:
            use_octile_3d (bool): 是否使用 Octile 距离。
            heuristic_weight (float): 启发式权重。
                                      1.0 = 最短路径保证。
                                      >1.0 (如 1.5) = 贪婪搜索，速度大幅提升，路径可能略长。
        """
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )

        self.use_octile_3d = use_octile_3d
        self.heuristic_weight = max(1.0, heuristic_weight)

        self.start_node: Optional[NodeType] = None
        self.goal_node: Optional[NodeType] = None

        # 预计算移动代价
        self.COST_1_AXIS = 1.0
        self.COST_2_AXIS = 1.4142
        self.COST_3_AXIS = 1.7320

        if self.heuristic_weight > 1.01:
            self.logger.info(f"[A*] 启用加权加速 (Weight={self.heuristic_weight})")

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """初始化规划任务。"""
        # 1. 基础合法性检查
        if not self.is_valid(start) or not self.is_valid(goal):
            self.logger.error(f"初始化失败: 起点 {start} 或 终点 {goal} 越界")
            return False

        # 2. 安全性检查
        if self.is_obstacle(start):
            self.logger.warning(f"初始化警告: 起点 {start} 位于障碍物内")
            return False
        if self.is_obstacle(goal):
            self.logger.warning(f"初始化警告: 终点 {goal} 位于障碍物内")
            return False

        self.start_node = start
        self.goal_node = goal

        self.logger.info(
            f"A* 任务初始化: {start} -> {goal} (Mode: {'3D Octile' if self.use_octile_3d else 'Euclidean'}, W={self.heuristic_weight})"
        )
        return True

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """A* 无需增量更新，只需记录"""
        if changes:
            self.logger.debug(
                f"收到 {len(changes)} 个环境变更通知 (A* 将在下次计算应用)。"
            )

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """[核心实现] 执行 A* 搜索主循环。"""
        if not self.goal_node:
            self.logger.error("未初始化 Goal，无法规划")
            return None

        start = current_pos
        goal = self.goal_node

        # --- A* 数据结构 ---
        open_set = []
        # Priority Queue 存储 (F-Score, Node)
        heapq.heappush(open_set, (0.0, start))

        came_from: Dict[NodeType, NodeType] = {}
        g_score: Dict[NodeType, float] = {start: 0.0}

        # F = G + H * Weight
        h_start = self._heuristic(start, goal)
        f_score: Dict[NodeType, float] = {start: h_start}

        open_set_hash: Set[NodeType] = {start}

        nodes_expanded = 0
        max_nodes = 500000  # 防止死机兜底

        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current in open_set_hash:
                open_set_hash.remove(current)
            else:
                continue

            nodes_expanded += 1
            if nodes_expanded > max_nodes:
                self.logger.error(f"A* 搜索超时: 超过 {max_nodes} 个节点")
                return None

            # 1. 终止条件
            if current == goal:
                self.stats["nodes_expanded"] = nodes_expanded
                return self._reconstruct_path(came_from, current)

            # 2. 扩展邻居
            for neighbor, move_cost in self._get_neighbors(current):
                tentative_g = g_score[current] + move_cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    # 关键: 加权启发式
                    f = tentative_g + self._heuristic(neighbor, goal)
                    f_score[neighbor] = f

                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f, neighbor))
                        open_set_hash.add(neighbor)

        self.logger.warning(f"A* 搜索失败: 未找到路径 (Nodes: {nodes_expanded})")
        self.stats["nodes_expanded"] = nodes_expanded
        return None

    # =========================================================================
    # 辅助方法
    # =========================================================================

    def _heuristic(self, a: NodeType, b: NodeType) -> float:
        """
        计算启发式代价 H(n) * Weight。
        """
        h_val = 0.0
        dims = len(a)

        # --- 2D ---
        if dims == 2:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            h_val = (dx + dy) + (self.COST_2_AXIS - 2) * min(dx, dy)

        # --- 3D ---
        elif dims == 3:
            if not self.use_octile_3d:
                # 欧几里得 (Euclidean)
                h_val = math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])
            else:
                # 3D Octile
                dx = abs(a[0] - b[0])
                dy = abs(a[1] - b[1])
                dz = abs(a[2] - b[2])
                delta = sorted([dx, dy, dz])
                h_val = (
                    delta[0] * self.COST_3_AXIS
                    + (delta[1] - delta[0]) * self.COST_2_AXIS
                    + (delta[2] - delta[1]) * self.COST_1_AXIS
                )

        # 应用权重加速
        return h_val * self.heuristic_weight

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
        """生成合法邻居 (26-Connectivity for 3D)"""
        neighbors = []
        dims = len(node)

        if dims == 2:
            r, c = node
            directions = [
                (0, 1, self.COST_1_AXIS),
                (0, -1, self.COST_1_AXIS),
                (1, 0, self.COST_1_AXIS),
                (-1, 0, self.COST_1_AXIS),
                (1, 1, self.COST_2_AXIS),
                (1, -1, self.COST_2_AXIS),
                (-1, 1, self.COST_2_AXIS),
                (-1, -1, self.COST_2_AXIS),
            ]
            for dr, dc, cost in directions:
                nr, nc = r + dr, c + dc
                if self.is_safe((nr, nc)):
                    # 2D 防穿墙简化检查
                    if cost > 1.0 and (
                        self.is_obstacle((r + dr, c)) and self.is_obstacle((r, c + dc))
                    ):
                        continue
                    neighbors.append(((nr, nc), cost))

        elif dims == 3:
            x, y, z = node
            # 3D 26-Neighborhood
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue

                        nx, ny, nz = x + dx, y + dy, z + dz

                        if self.is_safe((nx, ny, nz)):
                            dist_sq = dx * dx + dy * dy + dz * dz
                            if dist_sq == 1:
                                cost = self.COST_1_AXIS
                            elif dist_sq == 2:
                                cost = self.COST_2_AXIS
                            else:
                                cost = self.COST_3_AXIS
                            neighbors.append(((nx, ny, nz), cost))

        return neighbors

    def _reconstruct_path(
        self, came_from: Dict[NodeType, NodeType], current: NodeType
    ) -> List[NodeType]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
