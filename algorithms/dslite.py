import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Set, Union, Any
from .base import PathPlannerBase, NodeType

# 浮点数比较容差
EPSILON = 1e-5
# 无穷大常量
INF = float("inf")


class DLitePlanner(PathPlannerBase[NodeType]):
    """
    【L1 层 - D* Lite 路径规划器 (严格防切角版)】
    (D* Lite Path Planner - Strict Corner Checking)

    核心改进:
    - Fix: _get_neighbors 实现了与 A* 一致的严格防切角逻辑 (OR check)。
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
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )

        self.use_octile_3d = use_octile_3d
        self.heuristic_weight = max(1.0, heuristic_weight)

        # --- D* Lite 核心状态 ---
        self.g: Dict[NodeType, float] = {}
        self.rhs: Dict[NodeType, float] = {}
        self.U: List[Tuple[Tuple[float, float], NodeType]] = []
        self.km = 0.0

        self.start_node: Optional[NodeType] = None
        self.goal_node: Optional[NodeType] = None
        self.last_start_node: Optional[NodeType] = None

        # 预计算代价常量
        self.COST_1 = 1.0
        self.COST_2 = 1.41421356
        self.COST_3 = 1.73205081

        if self.heuristic_weight > 1.01:
            self.logger.info(f"[D* Lite] 加权模式 W={self.heuristic_weight}")

    # ... (initialize, update_obstacles, _compute_path_core, _calculate_key, _update_vertex, _compute_shortest_path, _heuristic_inline 保持不变，省略以节省篇幅，请保留原逻辑) ...

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        if not self.is_valid(start) or not self.is_valid(goal):
            self.logger.error(f"D* Init 失败: 坐标越界 {start}->{goal}")
            return False
        if self.is_obstacle(goal):
            self.logger.warning(f"D* Init 警告: 终点 {goal} 在障碍物内")
            return False

        if self.goal_node == goal and self.rhs:
            self.start_node = start
            return True

        self.start_node = start
        self.goal_node = goal
        self.last_start_node = start
        self.km = 0.0

        self.U = []
        self.g.clear()
        self.rhs.clear()

        self.rhs[goal] = 0.0
        heapq.heappush(self.U, (self._calculate_key(goal), goal))

        self.logger.info(f"D* Lite 全量重置: {goal} -> {start}")

        if not self._compute_shortest_path():
            self.logger.warning("D* Lite 初始搜索未能找到路径")

        return True

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        if not self.goal_node or not self.start_node:
            return
        if not changes:
            return

        for change in changes:
            coords = change[:-1]
            u: NodeType = tuple(coords)  # type: ignore
            if not self.is_valid(u):
                continue
            self._update_vertex(u)
            for neighbor, cost in self._get_neighbors(u):
                self._update_vertex(neighbor)

        self.stats["replanning_count"] = self.stats.get("replanning_count", 0) + 1
        self._compute_shortest_path()

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        if not self.goal_node:
            return None

        if current_pos != self.last_start_node:
            self.km += self._heuristic_inline(self.last_start_node, current_pos)
            self.last_start_node = current_pos

        self.start_node = current_pos
        self._compute_shortest_path()

        if self.g.get(current_pos, INF) == INF:
            return None

        path = [current_pos]
        curr = current_pos
        max_steps = 5000

        while curr != self.goal_node and len(path) < max_steps:
            min_cost = INF
            best_next = None

            for neighbor, move_cost in self._get_neighbors(curr):
                if self.is_obstacle(neighbor):
                    continue
                c = move_cost + self.g.get(neighbor, INF)
                if c < min_cost:
                    min_cost = c
                    best_next = neighbor

            if best_next:
                path.append(best_next)
                curr = best_next
            else:
                break

        return path

    def _calculate_key(self, u: NodeType) -> Tuple[float, float]:
        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)
        min_val = g_val if g_val < rhs_val else rhs_val
        if min_val == INF:
            return (INF, INF)
        k1 = min_val + self._heuristic_inline(self.start_node, u) + self.km
        return (k1, min_val)

    def _update_vertex(self, u: NodeType):
        if u != self.goal_node:
            min_rhs = INF
            for neighbor, move_cost in self._get_neighbors(u):
                if self.is_obstacle(neighbor):
                    continue
                g_neighbor = self.g.get(neighbor, INF)
                if g_neighbor != INF:
                    temp = move_cost + g_neighbor
                    if temp < min_rhs:
                        min_rhs = temp
            self.rhs[u] = min_rhs

        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)
        if abs(g_val - rhs_val) > EPSILON:
            heapq.heappush(self.U, (self._calculate_key(u), u))

    def _compute_shortest_path(self) -> bool:
        if not self.start_node:
            return False
        heappush = heapq.heappush
        heappop = heapq.heappop
        calc_key = self._calculate_key
        get_g = self.g.get
        get_rhs = self.rhs.get
        max_iter = 50000000
        iters = 0

        while self.U:
            iters += 1
            if iters > max_iter:
                self.logger.error(f"D* Lite 超出迭代上限 ({max_iter})，强制终止")
                return False

            k_old, u = self.U[0]
            k_start = calc_key(self.start_node)
            start_g = get_g(self.start_node, INF)
            start_rhs = get_rhs(self.start_node, INF)

            if k_old >= k_start and abs(start_g - start_rhs) < EPSILON:
                self.stats["nodes_expanded"] = (
                    self.stats.get("nodes_expanded", 0) + iters
                )
                return True

            heappop(self.U)
            k_new = calc_key(u)

            if k_old < k_new:
                heappush(self.U, (k_new, u))
                continue

            g_u = get_g(u, INF)
            rhs_u = get_rhs(u, INF)

            if g_u > rhs_u:
                self.g[u] = rhs_u
                new_g = rhs_u
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    new_rhs_s = new_g + cost
                    curr_rhs_s = get_rhs(s, INF)
                    if new_rhs_s < curr_rhs_s:
                        self.rhs[s] = new_rhs_s
                        heappush(self.U, (calc_key(s), s))
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    self._update_vertex(s)
        return False

    def _heuristic_inline(self, a: Optional[NodeType], b: Optional[NodeType]) -> float:
        if not a or not b:
            return 0.0
        if len(a) == 3:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            dz = abs(a[2] - b[2])
            if dx < dy:
                dx, dy = dy, dx
            if dx < dz:
                dx, dz = dz, dx
            if dy < dz:
                dy, dz = dz, dy
            return (
                dz * self.COST_3 + (dy - dz) * self.COST_2 + (dx - dy) * self.COST_1
            ) * self.heuristic_weight
        else:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            if dx < dy:
                dx, dy = dy, dx
            return (dy * self.COST_2 + (dx - dy) * self.COST_1) * self.heuristic_weight

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
        """
        获取邻居 (Strict Corner Checking)
        """
        res = []
        if len(node) == 3:
            x, y, z = node  # type: ignore
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        nx, ny, nz = x + dx, y + dy, z + dz

                        # 1. 基础检查 (is_valid)
                        # D* 外部调用 update_vertex 时需要 valid 检查，这里内联一部分
                        if (
                            0 <= nx < self.rows
                            and 0 <= ny < self.cols
                            and 0 <= nz < self.layers
                        ):

                            # 2. [Strict] 3D XY 切角检查
                            # 如果水平方向是斜着走
                            if dx != 0 and dy != 0:
                                # 检查 (x+dx, y) 和 (x, y+dy)
                                # 注意：这里我们假设 D* 的 _get_neighbors 可以在此处访问 grid
                                # 如果 is_obstacle(neighbor) 为真，在外部循环中会被跳过
                                # 但切角检查需要检查 *非* 目标邻居的中间节点
                                if self.is_obstacle((x + dx, y, z)) or self.is_obstacle(
                                    (x, y + dy, z)
                                ):
                                    continue

                            dist_sq = dx * dx + dy * dy + dz * dz
                            cost = (
                                self.COST_1
                                if dist_sq == 1
                                else (self.COST_2 if dist_sq == 2 else self.COST_3)
                            )
                            res.append(((nx, ny, nz), cost))  # type: ignore
        else:
            r, c = node  # type: ignore
            # 2D 8-neighbor Strict
            moves = [
                (0, 1, self.COST_1),
                (0, -1, self.COST_1),
                (1, 0, self.COST_1),
                (-1, 0, self.COST_1),
                (1, 1, self.COST_2),
                (1, -1, self.COST_2),
                (-1, 1, self.COST_2),
                (-1, -1, self.COST_2),
            ]
            for dr, dc, cost in moves:
                nr, nc = r + dr, c + dc
                # 1. 基础越界检查
                if 0 <= nr < self.rows and 0 <= nc < self.cols:

                    # 2. [Strict] 对角线检查
                    if dr != 0 and dc != 0:
                        # 只要任意一边有障碍物，就跳过 (OR 逻辑)
                        if self.is_obstacle((r + dr, c)) or self.is_obstacle(
                            (r, c + dc)
                        ):
                            continue

                    res.append(((nr, nc), cost))  # type: ignore
        return res
