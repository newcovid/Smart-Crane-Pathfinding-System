import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Any

from .base import PathPlannerBase, NodeType
from smart_crane.core.rust_bridge import RustBackend
from smart_crane.core.constants import SQRT_2, SQRT_3, EPSILON, FLOAT_TOLERANCE, INF


class DLitePlanner(PathPlannerBase[NodeType]):
    """D* Lite 增量路径规划器。

    支持动态环境下的快速重规划。
    实现了 Koening & Likhachev 的 D* Lite 算法逻辑。
    支持 Rust 加速后端。
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
        enable_rust: bool = True,
    ):
        """初始化 D* Lite 规划器。"""
        # 调用基类初始化，自动处理 logger 嵌套
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )

        self.rust_planner = None
        self.enable_rust = enable_rust
        # 用于累积 Rust 端报告的节点扩展数
        self.pending_expansions = 0

        self.use_octile_3d = use_octile_3d
        self.heuristic_weight = max(1.0, heuristic_weight)

        # D* Lite 核心数据结构 (仅 Python 模式使用)
        self.g: Dict[NodeType, float] = {}
        self.rhs: Dict[NodeType, float] = {}
        # U 是优先队列，存储 (Key, Node)
        self.U: List[Tuple[Tuple[float, float], NodeType]] = []
        # 辅助字典，用于 O(1) 查找节点是否在堆中及其 Key
        self.open_keys: Dict[NodeType, Tuple[float, float]] = {}

        self.km = 0.0
        self.start_node: Optional[NodeType] = None
        self.goal_node: Optional[NodeType] = None
        self.last_start_node: Optional[NodeType] = None

        self.max_nodes_expanded = 5000
        # 成本阈值，防止在无解情况下无限搜索
        self.cost_threshold = 100000.0

        self.COST_1 = 1.0
        self.COST_2 = SQRT_2
        self.COST_3 = SQRT_3

        # 加载 Rust 核心
        if RustBackend.is_available() and self.enable_rust:
            RustClass = RustBackend.get_dslite_class()
            if RustClass:
                try:
                    self.rust_planner = RustClass(
                        self.grid,
                        self.width_m,
                        self.length_m,
                        self.height_m,
                        self.resolution,
                        self.use_octile_3d,
                        self.heuristic_weight,
                    )
                    self.logger.info("Rust 核心加载成功!")
                except Exception as e:
                    self.logger.error(f"Rust 初始化失败: {e}")
                    self.rust_planner = None
        else:
            self.logger.info("使用 Python 原生模式运行。")

    @property
    def grid(self):
        return self._grid

    @grid.setter
    def grid(self, value):
        self._grid = value
        if self.rust_planner:
            try:
                self.rust_planner.update_grid(value)
            except Exception as e:
                self.logger.error(f"网格同步错误: {e}")

    @property
    def supports_incremental(self) -> bool:
        """D* Lite 支持增量更新。"""
        return True

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """初始化 D* Lite 搜索状态。

        Args:
            start: 起点。
            goal: 终点。

        Returns:
            bool: 是否成功。
        """
        self.stats["nodes_expanded"] = self.pending_expansions
        self.pending_expansions = 0
        self.stats["replanning_count"] = 0

        if start is None or goal is None:
            return False
        if not self.is_valid(start) or not self.is_valid(goal):
            return False
        if self.is_obstacle(goal):
            self.logger.error(f"终点 {goal} 不可用。")
            return False

        # 1. Rust 模式初始化
        if self.rust_planner:
            s_tuple = (
                int(start[0]),
                int(start[1]),
                int(start[2]) if len(start) > 2 else 0,
            )
            e_tuple = (int(goal[0]), int(goal[1]), int(goal[2]) if len(goal) > 2 else 0)
            self.start_node = start
            self.goal_node = goal

            # Rust initialize 返回 (success, nodes_expanded_in_init)
            success, nodes = self.rust_planner.initialize(s_tuple, e_tuple)
            self.stats["nodes_expanded"] += nodes
            return success

        # 2. Python 模式初始化
        # 如果起点或终点没变，尝试复用上次的搜索树
        if self.goal_node == goal and self.g:
            if self.start_node != start:
                # 机器人移动了，需要更新 km
                self.km += self._heuristic_inline(self.last_start_node, start)
                self.last_start_node = start
                self.start_node = start
            return True

        # 全量重置
        self.start_node = start
        self.goal_node = goal
        self.last_start_node = start
        self.km = 0.0
        self.U = []
        self.open_keys.clear()
        self.g.clear()
        self.rhs.clear()

        # D* 是反向搜索，rhs(goal) = 0
        self.rhs[goal] = 0.0
        self._insert_to_open(goal, self._calculate_key(goal))

        return self._compute_shortest_path()

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """处理动态障碍物更新。

        Args:
            changes: 变化列表。
        """
        if not changes:
            return

        # 1. Rust 更新
        if self.rust_planner:
            rust_changes = []
            for item in changes:
                if len(item) == 3:
                    rust_changes.append((item[0], item[1], 0, item[2]))
                elif len(item) == 4:
                    rust_changes.append(item)
            try:
                # Rust 返回由此更新引发的节点扩展数
                nodes = self.rust_planner.update_obstacles(rust_changes)
                self.pending_expansions += nodes
                return
            except Exception as e:
                self.logger.error(f"Rust 增量更新异常: {e}")

        # 2. Python 更新
        if not self.goal_node or not self.start_node:
            return

        for change in changes:
            coords = change[:-1]
            u: NodeType = tuple(coords)
            if not self.is_valid(u):
                continue

            # 更新受影响的节点及其邻居
            self._update_vertex(u)
            for neighbor, _ in self._get_neighbors(u):
                self._update_vertex(neighbor)

        self.stats["replanning_count"] = self.stats.get("replanning_count", 0) + 1

        # 触发重规划并统计扩展数
        pre = self.stats.get("nodes_expanded", 0)
        self._compute_shortest_path()
        post = self.stats.get("nodes_expanded", 0)
        self.pending_expansions += post - pre
        self.stats["nodes_expanded"] = pre

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """计算路径（核心逻辑）。"""

        # 1. Rust 计算
        if self.enable_rust and self.rust_planner:
            s_tuple = (
                int(current_pos[0]),
                int(current_pos[1]),
                int(current_pos[2]) if len(current_pos) > 2 else 0,
            )
            try:
                # Rust: compute_path(current) -> (path, nodes, replans)
                result = self.rust_planner.compute_path(s_tuple)
                rust_path, nodes_expanded, replan_count = result

                self.stats["nodes_expanded"] += nodes_expanded
                self.stats["replanning_count"] = replan_count

                if rust_path:
                    py_path = []
                    for p in rust_path:
                        if self.layers <= 1 and len(p) == 3:
                            py_path.append((p[0], p[1]))
                        else:
                            py_path.append(tuple(p))
                    return py_path
                else:
                    return None
            except Exception as e:
                self.logger.error(f"Rust 核心崩溃: {e}", exc_info=True)
                return None

        # 2. Python 计算
        if not self.goal_node:
            return None
        if current_pos == self.goal_node:
            return [current_pos]

        # 如果机器人移动了，更新 km 并重新计算优先队列
        if current_pos != self.last_start_node:
            self.km += self._heuristic_inline(self.last_start_node, current_pos)
            self.last_start_node = current_pos

        self.start_node = current_pos

        # 确保路径一致性
        if not self._compute_shortest_path():
            self.logger.debug("路径计算未完全收敛或无解。")

        if self.g.get(current_pos, INF) == INF:
            self.logger.info("当前点 g 值无穷大，无法到达目标。")
            return None

        # 贪婪回溯路径 (Gradient Descent)
        path = [current_pos]
        curr = current_pos
        MAX_STEPS = self.max_nodes_expanded

        while curr != self.goal_node and len(path) < MAX_STEPS:
            min_cost = INF
            best_next = None

            for neighbor, move_cost in self._get_neighbors(curr):
                if self.is_obstacle(neighbor):
                    continue
                # c = cost(curr, next) + g(next)
                c = move_cost + self.g.get(neighbor, INF)
                if c < min_cost - EPSILON:
                    min_cost = c
                    best_next = neighbor

            if best_next:
                if best_next in path:
                    self.logger.warning("检测到路径环路，停止回溯。")
                    return None
                path.append(best_next)
                curr = best_next
            else:
                break

        return path

    # --- D* Lite 内部辅助函数 ---

    def _insert_to_open(self, u, key):
        """插入或更新优先队列。"""
        self.open_keys[u] = key
        heapq.heappush(self.U, (key, u))

    def _calculate_key(self, u):
        """计算节点的 Key = [k1, k2]。
        k1 = min(g, rhs) + h + km
        k2 = min(g, rhs)
        """
        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)
        min_val = min(g_val, rhs_val)
        if min_val == INF:
            return (INF, INF)
        k1 = min_val + self._heuristic_inline(self.start_node, u) + self.km
        return (k1, min_val)

    def _update_vertex(self, u):
        """更新节点状态 (Consistent / Inconsistent)。"""
        if self.is_obstacle(u):
            self.rhs[u] = INF
        elif u != self.goal_node:
            # rhs(u) = min(cost(u, s') + g(s')) for all neighbors s'
            min_rhs = INF
            for neighbor, move_cost in self._get_neighbors(u):
                if self.is_obstacle(neighbor):
                    continue
                temp = move_cost + self.g.get(neighbor, INF)
                if temp < min_rhs:
                    min_rhs = temp
            self.rhs[u] = min_rhs

        # 检查是否需要加入 OpenSet
        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)

        if abs(g_val - rhs_val) <= EPSILON:
            # Consistent: 移除
            if u in self.open_keys:
                del self.open_keys[u]
        else:
            # Inconsistent: 更新 Key 并加入
            self._insert_to_open(u, self._calculate_key(u))

    def _compute_shortest_path(self) -> bool:
        """D* Lite 的核心循环：处理不一致节点直到起点收敛。"""
        if not self.start_node:
            return False

        expansions = 0
        while self.U:
            if expansions > self.max_nodes_expanded:
                return False

            # 取出堆顶
            k_old, u = self.U[0]

            # Lazy Removal: 如果取出的 key 与实际存储的 key 不符，说明是旧数据，丢弃
            if u not in self.open_keys or self.open_keys[u] != k_old:
                heapq.heappop(self.U)
                continue

            # 终止条件检查
            start_g = self.g.get(self.start_node, INF)
            start_rhs = self.rhs.get(self.start_node, INF)
            k_start = self._calculate_key(self.start_node)

            # 比较 Key(u) < Key(start) OR rhs(start) != g(start)
            term1 = k_old[0] > k_start[0] + FLOAT_TOLERANCE
            term2 = (
                abs(k_old[0] - k_start[0]) < EPSILON
                and k_old[1] > k_start[1] + FLOAT_TOLERANCE
            )

            # 如果堆顶键值已经大于起点键值，且起点已经收敛 (g==rhs)，则搜索完成
            if abs(start_g - start_rhs) < EPSILON and (term1 or term2):
                return True

            # 提前退出：如果起点不可达且搜索代价过大
            if start_g == INF and start_rhs == INF and k_old[0] > self.cost_threshold:
                heapq.heappop(self.U)
                if u in self.open_keys:
                    del self.open_keys[u]
                continue

            heapq.heappop(self.U)
            expansions += 1
            self.stats["nodes_expanded"] = self.stats.get("nodes_expanded", 0) + 1

            k_new = self._calculate_key(u)

            if k_old < k_new:
                # 节点的启发值变大了 (km 增加)，重新推入堆
                self._insert_to_open(u, k_new)
                continue

            g_u = self.g.get(u, INF)
            rhs_u = self.rhs.get(u, INF)

            if g_u > rhs_u + EPSILON:
                # Overconsistent: g > rhs -> 令 g = rhs，传播给邻居
                self.g[u] = rhs_u
                if u in self.open_keys:
                    del self.open_keys[u]
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    # 更新前驱节点的 rhs
                    if self.rhs.get(s, INF) > rhs_u + cost:
                        self.rhs[s] = rhs_u + cost
                        self._update_vertex(s)
            else:
                # Underconsistent: g < rhs -> 令 g = INF，变为非常不一致，迫使邻居重算
                self.g[u] = INF
                self._update_vertex(u)
                for s, _ in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    self._update_vertex(s)
        return False

    def _heuristic_inline(self, a, b):
        """内联启发式函数。"""
        if not a or not b:
            return 0.0
        if len(a) == 3:
            dx, dy, dz = abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2])
            if self.use_octile_3d:
                delta = sorted([dx, dy, dz])
                return (
                    delta[0] * self.COST_3
                    + (delta[1] - delta[0]) * self.COST_2
                    + (delta[2] - delta[1]) * self.COST_1
                )
            return math.sqrt(dx * dx + dy * dy + dz * dz)
        else:
            dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
            return (dx + dy) + (self.COST_2 - 2) * min(dx, dy)

    def _get_neighbors(self, node):
        """获取邻居节点。"""
        res = []
        if len(node) == 3:
            x, y, z = int(node[0]), int(node[1]), int(node[2])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        nx, ny, nz = x + dx, y + dy, z + dz
                        if not self.is_safe((nx, ny, nz)):
                            continue
                        # 简化对角线碰撞检查
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
                        res.append(((nx, ny, nz), cost))
        else:
            r, c = int(node[0]), int(node[1])
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
                if dr != 0 and dc != 0:
                    if self.is_obstacle((r + dr, c)) or self.is_obstacle((r, c + dc)):
                        continue
                res.append(((nr, nc), cost))
        return res
