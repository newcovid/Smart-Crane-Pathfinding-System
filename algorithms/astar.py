import heapq
import math
from typing import List, Tuple, Dict, Optional, Set
from .base import PathPlannerBase, NodeType


class AStarPlanner(PathPlannerBase[NodeType]):
    """
    【A* 路径规划器 (Generic 2D/3D)】

    核心理念:
    - 算法本身是维数无关的 (Dimension Agnostic)。
    - 它只关心节点(Node)和邻居(Neighbor)。
    - 2D 或 3D 的具体逻辑由 _get_neighbors 和 _heuristic 里的分支处理。
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
        heuristic_weight=1.0,
    ):
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )
        self.use_octile_3d = use_octile_3d
        self.heuristic_weight = max(1.0, heuristic_weight)

        self.start_node: Optional[NodeType] = None
        self.goal_node: Optional[NodeType] = None

        self.COST_1 = 1.0
        self.COST_2 = 1.4142
        self.COST_3 = 1.7320

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        if not self.is_valid(start) or not self.is_valid(goal):
            return False
        if self.is_obstacle(start) or self.is_obstacle(goal):
            self.logger.warning(f"Start {start} or Goal {goal} is in obstacle.")
            return False

        self.start_node = start
        self.goal_node = goal
        return True

    def update_obstacles(self, changes):
        pass  # A* is static re-plan, ignores incremental updates

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        start, goal = current_pos, self.goal_node

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}

        # 节点访问去重优化
        open_set_hash = {start}
        max_nodes = 500000
        nodes_expanded = 0

        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_hash.remove(current)
            nodes_expanded += 1

            if current == goal:
                self.stats["nodes_expanded"] = nodes_expanded
                return self._reconstruct_path(came_from, current)

            if nodes_expanded > max_nodes:
                self.logger.error("A* Max nodes limit reached.")
                return None

            for neighbor, move_cost in self._get_neighbors(current):
                tentative_g = g_score[current] + move_cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal)

                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f, neighbor))
                        open_set_hash.add(neighbor)

        self.stats["nodes_expanded"] = nodes_expanded
        return None

    def _heuristic(self, a: NodeType, b: NodeType) -> float:
        h = 0.0
        dims = len(a)
        if dims == 2:
            dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
            # Octile distance for 2D
            h = (dx + dy) + (self.COST_2 - 2) * min(dx, dy)
        else:
            dx, dy, dz = abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2])
            if self.use_octile_3d:
                # Octile distance for 3D
                delta = sorted([dx, dy, dz])
                h = (
                    delta[0] * self.COST_3
                    + (delta[1] - delta[0]) * self.COST_2
                    + (delta[2] - delta[1]) * self.COST_1
                )
            else:
                # Euclidean
                h = math.sqrt(dx * dx + dy * dy + dz * dz)
        return h * self.heuristic_weight

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
        """生成合法邻居 (严格防切角)"""
        neighbors = []
        dims = len(node)

        if dims == 2:
            r, c = node
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
                if not self.is_safe((nr, nc)):
                    continue
                # 2D Strict Corner Check
                if dr != 0 and dc != 0:
                    if self.is_obstacle((r + dr, c)) or self.is_obstacle((r, c + dc)):
                        continue
                neighbors.append(((nr, nc), cost))
        else:
            # 3D Neighbors
            x, y, z = node
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        nx, ny, nz = x + dx, y + dy, z + dz
                        if not self.is_safe((nx, ny, nz)):
                            continue

                        # 3D Strict Corner Check (Simplified: XY plane)
                        if dx != 0 and dy != 0:
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
                        neighbors.append(((nx, ny, nz), cost))
        return neighbors

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
