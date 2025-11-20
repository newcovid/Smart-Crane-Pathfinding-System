import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Set, Union, Any
from .base import PathPlannerBase, NodeType

# 浮点数比较容差，防止 Key 值震荡
EPSILON = 1e-5


class DLitePlanner(PathPlannerBase[NodeType]):
    """
    【L1 层 - D* Lite 路径规划器 (最终修复版)】
    (D* Lite Path Planner - Incremental & 3D Ready)

    修复记录:
    - Fix: 修复了处理堆中陈旧节点时错误重置已一致节点导致的死循环问题。
    - Opt: 优化 initialize，终点不变时不重置搜索树。
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
    ):
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )

        self.use_octile_3d = use_octile_3d

        # --- D* Lite 核心数据结构 ---
        # g: 当前节点到 Goal 的已知最短路径代价
        self.g: Dict[NodeType, float] = {}

        # rhs: 一步前瞻代价
        self.rhs: Dict[NodeType, float] = {}

        # U: 优先队列 (Key, Node)
        self.U: List[Tuple[Tuple[float, float], NodeType]] = []

        # km: Key Modifier (机器人移动导致的启发式偏差累加)
        self.km = 0.0

        self.start_node: Optional[NodeType] = None
        self.goal_node: Optional[NodeType] = None
        self.last_start_node: Optional[NodeType] = None

        self.COST_1_AXIS = 1.0
        self.COST_2_AXIS = 1.4142
        self.COST_3_AXIS = 1.7320
        self.INF = float("inf")

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """
        初始化规划任务。
        优化：如果 Goal 未变且搜索树存在，则复用（Incremental），避免全量重算。
        """
        # 1. 基础检查
        if not self.is_valid(start) or not self.is_valid(goal):
            self.logger.error(f"初始化失败: 起点 {start} 或 终点 {goal} 越界")
            return False

        if self.is_obstacle(goal):
            self.logger.warning(f"初始化警告: 终点 {goal} 位于障碍物内")
            return False

        # 2. 智能初始化 (Smart Init)
        # 如果终点没变，且我们有之前的搜索数据，则不需要重置整个图
        # 只需要更新起点，D* Lite 会通过 km 自动处理起点的移动
        if self.goal_node == goal and self.rhs:
            # 仅更新起点记录，搜索树保留
            self.start_node = start
            # 注意：last_start_node 不要在这里更新，它会在 compute_path_core 中
            # 用于计算与当前 start 的距离差来更新 km
            return True

        # 3. 全量初始化 (Full Reset)
        self.start_node = start
        self.goal_node = goal
        self.last_start_node = start

        self.U = []
        self.km = 0.0
        self.rhs.clear()
        self.g.clear()

        # 设定 Goal 为根 (RHS=0)
        self.rhs[goal] = 0.0
        heapq.heappush(self.U, (self._calculate_key(goal), goal))

        self.logger.info(f"D* Lite 全量初始化: Reverse Search {goal} -> {start}")

        # 首次搜索
        success = self._compute_shortest_path()
        if not success:
            self.logger.warning("D* Lite 首次搜索未收敛")

        return True

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """处理环境变化 (增量更新)"""
        if not self.goal_node or not self.start_node:
            return

        if not changes:
            return

        # self.logger.debug(f"D* Lite: 处理 {len(changes)} 个网格变更...")

        affected_nodes: Set[NodeType] = set()

        for change in changes:
            coords = change[:-1]
            u: NodeType = tuple(coords)  # type: ignore

            if not self.is_valid(u):
                continue

            affected_nodes.add(u)
            for neighbor, _ in self._get_neighbors(u):
                affected_nodes.add(neighbor)

        for u in affected_nodes:
            self._update_vertex(u)

        self.stats["replanning_count"] = self.stats.get("replanning_count", 0) + 1
        self._compute_shortest_path()

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """运行时路径提取"""
        if not self.goal_node:
            return None

        # 1. 处理机器人移动 (km 更新)
        if current_pos != self.last_start_node:
            self.km += self._heuristic(self.last_start_node, current_pos)
            self.last_start_node = current_pos

        # 2. 修复路径
        self._compute_shortest_path()

        # 3. 梯度下降提取路径
        if self.g.get(current_pos, self.INF) == self.INF:
            # 尝试最后的挣扎：如果当前点不可达，搜寻周围最近的可达点
            # self.logger.warning(f"D* Lite: 起点 {current_pos} 不可达，尝试搜索邻域...")
            pass  # 暂不实现复杂脱困

            self.logger.warning(f"D* Lite: 无法规划 (Start G=INF)")
            return None

        path = [current_pos]
        curr = current_pos
        max_steps = self.rows * self.cols * 2
        steps = 0

        while curr != self.goal_node and steps < max_steps:
            min_cost = self.INF
            best_next = None

            for neighbor, move_cost in self._get_neighbors(curr):
                if self.is_obstacle(neighbor):
                    continue

                c = move_cost + self.g.get(neighbor, self.INF)
                if c < min_cost - EPSILON:
                    min_cost = c
                    best_next = neighbor

            if best_next:
                path.append(best_next)
                curr = best_next
                steps += 1
            else:
                self.logger.error(f"D* Lite: 路径中断于 {curr}")
                return None

        return path

    # =========================================================================
    # D* Lite 核心逻辑
    # =========================================================================

    def _calculate_key(self, u: NodeType) -> Tuple[float, float]:
        g_val = self.g.get(u, self.INF)
        rhs_val = self.rhs.get(u, self.INF)
        min_val = min(g_val, rhs_val)

        if min_val == self.INF:
            return (self.INF, self.INF)

        k1 = min_val + self._heuristic(self.start_node, u) + self.km
        k2 = min_val
        return (k1, k2)

    def _update_vertex(self, u: NodeType):
        if u != self.goal_node:
            min_rhs = self.INF
            for neighbor, move_cost in self._get_neighbors(u):
                if self.is_obstacle(neighbor) or self.is_obstacle(u):
                    continue

                neighbor_g = self.g.get(neighbor, self.INF)
                if neighbor_g != self.INF:
                    temp = move_cost + neighbor_g
                    if temp < min_rhs:
                        min_rhs = temp
            self.rhs[u] = min_rhs

        g_val = self.g.get(u, self.INF)
        rhs_val = self.rhs.get(u, self.INF)

        if abs(g_val - rhs_val) > EPSILON:
            heapq.heappush(self.U, (self._calculate_key(u), u))

    def _compute_shortest_path(self) -> bool:
        """主循环"""
        if not self.start_node:
            return False

        MAX_ITERATIONS = 200000
        iterations = 0

        k_start = self._calculate_key(self.start_node)

        while self.U:
            iterations += 1
            if iterations >= MAX_ITERATIONS:
                self._dump_debug_info(iterations, k_start)
                self.logger.error(f"[D* Lite] 死循环保护触发 ({MAX_ITERATIONS})")
                return False

            # Peek 堆顶
            k_old, u = self.U[0]
            k_start = self._calculate_key(self.start_node)

            # 检查是否结束
            start_g = self.g.get(self.start_node, self.INF)
            start_rhs = self.rhs.get(self.start_node, self.INF)
            start_consistent = abs(start_g - start_rhs) < EPSILON

            if k_old >= k_start and start_consistent:
                self.stats["nodes_expanded"] = (
                    self.stats.get("nodes_expanded", 0) + iterations
                )
                return True

            # Pop
            heapq.heappop(self.U)
            k_new = self._calculate_key(u)

            # --- 核心分支 ---

            # 1. Key 过期 (Lazy Removal)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))

            # 2. Overconsistent (路径变短，传播更优值)
            elif self.g.get(u, self.INF) > self.rhs.get(u, self.INF):
                self.g[u] = self.rhs[u]
                for s, _ in self._get_neighbors(u):
                    self._update_vertex(s)

            # 3. Underconsistent (路径变长/阻断) 或 已一致 (Stale Consistent)
            else:
                g_val = self.g.get(u, self.INF)
                rhs_val = self.rhs.get(u, self.INF)

                # [关键修复] 如果节点实际上已经一致 (g == rhs)，说明这是一个陈旧的堆条目
                # 此时绝对不能重置 g，否则会摧毁已建立的搜索树，导致死循环！
                if abs(g_val - rhs_val) < EPSILON:
                    continue

                # 真正的 Underconsistent：重置 g 并触发重算
                self.g[u] = self.INF
                self._update_vertex(u)  # 自己重算 rhs
                for s, _ in self._get_neighbors(u):
                    self._update_vertex(s)

        return True

    def _dump_debug_info(self, iterations, k_start):
        top = self.U[0] if self.U else "Empty"
        msg = (
            f"\n=== D* Lite Crash Dump ===\n"
            f"Iter: {iterations} | Heap: {len(self.U)}\n"
            f"Start: {self.start_node} (Key: {k_start})\n"
            f"Top: {top}\n"
            f"=========================="
        )
        self.logger.error(msg)

    def _heuristic(self, a: NodeType, b: NodeType) -> float:
        if not a or not b:
            return 0.0
        dims = len(a)
        if dims == 2:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return (dx + dy) + (self.COST_2_AXIS - 2) * min(dx, dy)
        elif dims == 3:
            if not self.use_octile_3d:
                return math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])
            else:
                d = sorted([abs(a[i] - b[i]) for i in range(3)])
                return (
                    d[0] * self.COST_3_AXIS
                    + (d[1] - d[0]) * self.COST_2_AXIS
                    + (d[2] - d[1]) * self.COST_1_AXIS
                )
        return 0.0

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
        neighbors = []
        dims = len(node)
        if dims == 2:
            r, c = node  # type: ignore
            directions = [
                (0, 1, 1.0),
                (0, -1, 1.0),
                (1, 0, 1.0),
                (-1, 0, 1.0),
                (1, 1, 1.4142),
                (1, -1, 1.4142),
                (-1, 1, 1.4142),
                (-1, -1, 1.4142),
            ]
            for dr, dc, cost in directions:
                nr, nc = r + dr, c + dc
                if self.is_valid((nr, nc)):
                    neighbors.append(((nr, nc), cost))  # type: ignore
        elif dims == 3:
            x, y, z = node  # type: ignore
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        nx, ny, nz = x + dx, y + dy, z + dz
                        if self.is_valid((nx, ny, nz)):
                            dist = dx * dx + dy * dy + dz * dz
                            cost = (
                                1.0 if dist == 1 else (1.4142 if dist == 2 else 1.7320)
                            )
                            neighbors.append(((nx, ny, nz), cost))  # type: ignore
        return neighbors
