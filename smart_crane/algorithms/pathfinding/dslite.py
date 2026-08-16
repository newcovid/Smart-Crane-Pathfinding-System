import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Any

from .base import PathPlannerBase, NodeType
from smart_crane.core.rust_bridge import RustBackend
from smart_crane.core.constants import (
    A_STAR_DEFAULT_RESOLUTION,
    DEFAULT_HEURISTIC_WEIGHT,
    SQRT_2,
    SQRT_3,
    EPSILON,
    FLOAT_TOLERANCE,
    INF,
    DSLITE_DEFAULT_MAX_NODES,
    DSLITE_MAX_NODES_MULTIPLIER,
    DSLITE_COST_THRESHOLD_MULTIPLIER,
    DSLITE_DEFAULT_HEURISTIC_MIN_WEIGHT,
)


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
        resolution: float = A_STAR_DEFAULT_RESOLUTION,
        logger: Optional[logging.Logger] = None,
        grid_lock=None,
        use_octile_3d: bool = False,
        heuristic_weight: float = DEFAULT_HEURISTIC_WEIGHT,
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

        # D* Lite 的正确性依赖启发式的【一致性】(consistency)：
        # 对任意相邻 u、v 需满足 h(u) <= cost(u,v) + h(v)。给 h 乘上 >1 的权重
        # 会破坏这个前提，增量修复得到的 g 值不再是最短代价，路径可能次优甚至错误。
        # 因此这里强制为 1.0——Rust 实现（dslite.rs 的 `_heuristic_weight: 1.0`）
        # 本来就是这么做的，两个引擎必须一致，否则同一份配置会给出不同长度的路径。
        # 加权只对 A* 有意义（weighted A*），见 AStarPlanner。
        requested_weight = max(DSLITE_DEFAULT_HEURISTIC_MIN_WEIGHT, heuristic_weight)
        if abs(requested_weight - 1.0) > EPSILON:
            self.logger.warning(
                f"D* Lite 忽略 heuristic_weight={requested_weight}，强制使用 1.0。"
                f"加权启发式会破坏 D* Lite 所需的一致性前提；如需加权搜索请改用 A*。"
            )
        self.heuristic_weight = 1.0

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

        # 动态计算最大扩展节点数，防止复杂地图下过早熔断
        layers_count = self.layers if self.layers > 0 else 1
        total_grid_size = self.rows * self.cols * layers_count
        self.max_nodes_expanded = max(
            DSLITE_DEFAULT_MAX_NODES, total_grid_size * DSLITE_MAX_NODES_MULTIPLIER
        )

        # 成本阈值，防止在无解情况下无限搜索
        self.cost_threshold = float(total_grid_size) * DSLITE_COST_THRESHOLD_MULTIPLIER

        self.COST_1 = 1.0
        self.COST_2 = SQRT_2
        self.COST_3 = SQRT_3

        self.logger.info(
            f"D* Lite（Python）限制：最大扩展节点 = {self.max_nodes_expanded}, W = {self.heuristic_weight}"
        )

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
        """初始化 D* Lite 搜索状态。"""
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

        # 新任务：全量重置
        self._full_reset(start, goal)
        return self._compute_shortest_path()

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """处理动态障碍物更新。"""
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
            # 入参契约：2D 为 (r, c, 新值)，3D 为 (r, c, l, 新值)。末位是值不是坐标。
            expected_len = 4 if self.layers > 0 else 3
            if len(change) != expected_len:
                self.logger.warning(
                    f"忽略格式非法的变更项 {change}："
                    f"当前为 {'3D' if self.layers > 0 else '2D'} 地图，"
                    f"期望 {expected_len} 元组 "
                    f"{'(r, c, l, 新值)' if self.layers > 0 else '(r, c, 新值)'}。"
                )
                continue

            coords = change[:-1]
            u: NodeType = tuple(coords)
            if not self.is_valid(u):
                self.logger.warning(f"忽略越界的变更点 {u}（地图 {self.rows}x{self.cols}）。")
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

        # 严格的失败回退机制 (Reset Fallback)
        # 如果增量修复失败（通常因为扩展节点数超限未收敛），必须全量重算
        # 否则会带着不一致的 G 值进入回溯，导致死循环或非法路径
        if not self._compute_shortest_path():
            self.logger.warning("D* Lite 增量修复未收敛，执行全量重置 (Full Reset)...")
            self._full_reset(current_pos, self.goal_node)

            if not self._compute_shortest_path():
                self.logger.error("D* Lite 全量重算仍无解！")
                return None

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
                    self.logger.warning(f"检测到路径环路，停止回溯。Node: {best_next}")
                    return None
                path.append(best_next)
                curr = best_next
            else:
                self.logger.debug(
                    f"回溯中断：无更好邻居 (Curr: {curr}, MinCost: {min_cost})"
                )
                break

        return path

    # --- D* Lite 内部辅助函数 ---

    def _full_reset(self, start: NodeType, goal: NodeType):
        """[新增] 全量重置搜索状态（相当于全新的 A*）。"""
        self.start_node = start
        self.goal_node = goal
        self.last_start_node = start
        self.km = 0.0

        # 清空所有状态
        self.U = []
        self.open_keys.clear()
        self.g.clear()
        self.rhs.clear()

        # D* 是反向搜索，rhs(goal) = 0
        self.rhs[goal] = 0.0
        self._insert_to_open(goal, self._calculate_key(goal))

    def _insert_to_open(self, u, key):
        """插入或更新优先队列。"""
        self.open_keys[u] = key
        heapq.heappush(self.U, (key, u))

    def _calculate_key(self, u):
        """计算节点的 Key = [k1, k2]。"""
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

        # 必须先做 `==` 判等再做 EPSILON 比较：两者同为 INF 时
        # `abs(inf - inf)` 是 NaN，而 NaN 参与的任何比较都为 False，
        # 会把"两侧都不可达"这种一致状态误判为不一致，从而把
        # (INF, INF) 键反复塞回优先队列，推高扩展数并更早触发熔断重置。
        # Rust 侧 `dslite.rs` 用的是 `> EPSILON` 判不一致，NaN 同样为 False，
        # 因此天然落在"一致"分支——此处对齐 Rust 的行为。
        if g_val == rhs_val or abs(g_val - rhs_val) <= EPSILON:
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
                # 超过限定的最大扩展节点数，中止搜索以防阻塞。
                # （这里原有一层 `if expansions % max == 0` 的取模判断，但循环每轮
                #   只加 1，首次越界时 expansions 恒为 max+1，取模恒为 1，
                #   那条 warning 永远打印不出来。已去掉。）
                self.logger.warning(
                    f"扩展节点超出限制（{self.max_nodes_expanded}），增量搜索中止，将触发全量重置。"
                )
                return False

            # 取出堆顶项
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
                    # 仅当 u 能让前驱 s 变得更优时才需要重算 s。
                    # 注意这里不再预先写入 self.rhs[s]——_update_vertex(s) 会
                    # 从 s 的全部邻居重新求一遍 min，预写的值必然被覆盖，是死代码。
                    # （Rust 侧 dslite.rs 就没有这一句，两边逻辑本就等价。）
                    if self.rhs.get(s, INF) > rhs_u + cost:
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
        val = 0.0
        if len(a) == 3:
            dx, dy, dz = abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2])
            if self.use_octile_3d:
                delta = sorted([dx, dy, dz])
                val = (
                    delta[0] * self.COST_3
                    + (delta[1] - delta[0]) * self.COST_2
                    + (delta[2] - delta[1]) * self.COST_1
                )
            else:
                val = math.sqrt(dx * dx + dy * dy + dz * dz)
        else:
            dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
            val = (dx + dy) + (self.COST_2 - 2) * min(dx, dy)

        # [关键修复] 应用权重 (heuristic_weight)
        # 如果不乘权重，H 值偏小，会导致节点优先级计算错误，在复杂地图下引发大量无效扩展甚至死循环
        return val * self.heuristic_weight

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
