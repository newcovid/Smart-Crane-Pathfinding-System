import heapq
import math
import logging
import time
from typing import List, Tuple, Dict, Optional, Set, Union, Any

# 导入基类和类型定义
# PathPlannerBase: 规划器抽象基类，提供通用的接口和锁机制
# NodeType: 泛型坐标类型，适配 (row, col) 或 (row, col, layer)
from .base import PathPlannerBase, NodeType

# 尝试导入 Rust 核心库 (加速引擎)
try:
    import smart_crane_core

    HAS_RUST_CORE = True
except ImportError:
    HAS_RUST_CORE = False

# --- 全局常量定义 ---

# 浮点数比较容差 (Epsilon)
# 用于判断两个浮点数是否"相等"，解决计算机浮点数精度丢失问题。
# 例如: abs(a - b) < EPSILON 视作 a == b
EPSILON = 1e-4

# 堆排序键值比较容差 (Float Tolerance)
#  D* Lite 在判断终止条件 (k_min >= k_start) 时，
# 由于路径累积代价计算存在微小误差，可能导致 k_min 比 k_start 仅仅大 0.000001 就提前退出了。
# 这会导致"路断了"的消息没能传回起点。因此我们需要一个稍大的容差来强制算法"多算一点"。
FLOAT_TOLERANCE = 1e-3

# 无穷大常量 (Infinity)
# 代表节点不可达、未探索或障碍物的代价。
INF = float("inf")


class DLitePlanner(PathPlannerBase[NodeType]):
    """
    【L1 层 - D* Lite 路径规划器 (Rust/Python 混合双核版)】
    (D* Lite Path Planner - Hybrid Rust/Python Engine)

    算法原理 (Algorithm Theory):
    D* Lite 是基于 LPA* (Lifelong Planning A*) 的反向增量搜索算法。

    1. **反向搜索 (Reverse Search)**:
       - 搜索从【终点 Goal】向【起点 Start】进行。
       - 优势: 当机器人移动（Start 变化）时，只要 Goal 不变，搜索树的大部分结构（即各节点到 Goal 的路径）依然有效。

    2. **RHS 值 (Lookahead Value)**:
       - rhs(u) = min(g(s) + c(s, u))，其中 s 是 u 的邻居。
       - 代表基于邻居信息推导出的"理论最优代价"。

    3. **增量更新 (Incremental Update)**:
       - 算法维护一个优先队列 (U)，仅包含"不一致" (g != rhs) 的节点。
       - 每次环境变化或机器人移动，只需处理队列中的节点，直到起点恢复一致性。

    **架构设计 (Architecture)**:
    - **Rust Core (优先)**: 利用 Rust 的强类型系统和零开销抽象，提供比 Python 快 10-50 倍的计算速度。
    - **Python Fallback (兜底)**: 保留完整的 Python 原生实现，用于调试、算法验证或 Rust 扩展未安装的环境。
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
        enable_rust: bool = True,  # [Config] 是否启用 Rust 加速
    ):
        """
        初始化 D* Lite 规划器。

        Args:
            grid: 网格数据引用 (0=空, 1=障碍)。
            width_m, length_m, height_m: 场地物理尺寸。
            resolution: 网格分辨率。
            logger: 日志记录器。
            grid_lock: 线程锁。
            use_octile_3d: 是否使用 3D Octile 距离。
            heuristic_weight: 启发式权重 (D* Lite 强制要求为 1.0)。
            enable_rust: 是否尝试使用 Rust 核心进行计算。
        """
        # [CRITICAL] 提前初始化 rust_planner 占位符，防止父类 __init__ 触发 property setter 报错
        self.rust_planner = None
        self.enable_rust = enable_rust

        # 调用父类初始化，设置基础属性 (会触发 self.grid = grid)
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )

        self.use_octile_3d = use_octile_3d

        # [权重一致性检查]
        # D* Lite 算法严重依赖启发式的一致性 (Consistency)。
        # 加权启发式 (Weighted Heuristic, w > 1) 会破坏三角不等式，导致优先级队列排序错乱。
        if heuristic_weight > 1.0:
            if self.logger:
                self.logger.warning(
                    f"[D* Init] ⚠️ 警告: 检测到加权启发式 (W={heuristic_weight})。"
                    f"D* Lite 要求严格一致性，为防止增量更新死锁，已强制重置 W=1.0。"
                )
            self.heuristic_weight = 1.0
        else:
            self.heuristic_weight = max(1.0, heuristic_weight)

        # --- Python 原生实现的数据结构 (Fallback) ---
        self.g: Dict[NodeType, float] = {}
        self.rhs: Dict[NodeType, float] = {}
        self.U: List[Tuple[Tuple[float, float], NodeType]] = []
        self.open_keys: Dict[NodeType, Tuple[float, float]] = {}
        self.km = 0.0
        self.start_node: Optional[NodeType] = None
        self.goal_node: Optional[NodeType] = None
        self.last_start_node: Optional[NodeType] = None

        # --- 动态阈值配置 ---
        rows_int = int(self.rows) if self.rows else 100
        cols_int = int(self.cols) if self.cols else 100
        layers_int = int(self.layers) if self.layers else 1
        total_voxels = max(1000, rows_int * cols_int * max(1, layers_int))

        # 熔断参数
        self.max_nodes_expanded = max(5000, total_voxels * 5)
        self.cost_threshold = float(total_voxels) * 1.74 * 10.0

        # 预计算移动代价
        self.COST_1 = 1.0
        self.COST_2 = 1.41421356
        self.COST_3 = 1.73205081

        # --- Rust 核心引擎初始化 ---
        if HAS_RUST_CORE and self.enable_rust:
            try:
                self.logger.info("[D* Lite] 正在尝试加载 Rust 高性能核心...")
                self.rust_planner = smart_crane_core.RustDLitePlanner(
                    self.grid,
                    self.width_m,
                    self.length_m,
                    self.height_m,
                    self.resolution,
                    self.use_octile_3d,
                    self.heuristic_weight,
                )
                self.logger.info(
                    f"[D* Lite] 🚀 Rust 核心加载成功! (熔断阈值: {self.max_nodes_expanded} 节点)"
                )
            except Exception as e:
                self.logger.error(
                    f"[D* Lite] ❌ Rust 核心初始化失败，回退到 Python 模式: {e}",
                    exc_info=True,
                )
                self.rust_planner = None
        else:
            mode = "用户禁用" if not self.enable_rust else "扩展未安装"
            self.logger.info(f"[D* Lite] 使用 Python 原生模式运行 ({mode})。")

    # --- 核心拦截器: 网格同步 ---
    @property
    def grid(self):
        return self._grid

    @grid.setter
    def grid(self, value):
        """
        拦截网格更新，确保 Rust 核心拥有最新的内存数据。
        """
        self._grid = value
        # 如果 Rust 核心存在，必须同步底层数据结构
        if self.rust_planner:
            try:
                self.rust_planner.update_grid(value)
                # 注意：这里只同步了数据，并未触发 rhs 的更新。
                # rhs 的更新需要调用 update_obstacles 并传入具体的变更列表。
            except Exception as e:
                self.logger.error(f"[D* Sync] ❌ 网格同步严重错误: {e}")

    def _reset_stats(self):
        """重置单次规划的统计数据。"""
        self.stats["nodes_expanded"] = 0
        self.stats["replanning_count"] = 0

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """
        [生命周期] 初始化规划任务。
        """
        self._reset_stats()

        if start is None or goal is None:
            self.logger.warning("[D* Init] ❌ 失败: 起点或终点为 None。")
            return False

        if not self.is_valid(start) or not self.is_valid(goal):
            self.logger.error(f"[D* Init] ❌ 失败: 坐标越界 {start} -> {goal}。")
            return False

        if self.is_obstacle(goal):
            self.logger.warning(f"[D* Init] ⚠️ 警告: 终点 {goal} 位于障碍物内。")
            return False

        # --- 分支 A: Rust 核心 ---
        if self.rust_planner:
            # 转换坐标为 (x, y, z) 元组
            s_tuple = (
                int(start[0]),
                int(start[1]),
                int(start[2]) if len(start) > 2 else 0,
            )
            e_tuple = (int(goal[0]), int(goal[1]), int(goal[2]) if len(goal) > 2 else 0)

            # 同时记录 Python 侧状态，以备万一需要回退或调试
            self.start_node = start
            self.goal_node = goal

            if self.rust_planner.initialize(s_tuple, e_tuple):
                self.logger.info(f"[D* Rust] 初始化成功: {start} -> {goal}")
                return True
            else:
                self.logger.warning(f"[D* Rust] 初始化被拒绝 (可能是不可达)。")
                return False

        # --- 分支 B: Python 原生 ---
        # 智能复用: 目标点没变且有历史数据
        if self.goal_node == goal and self.g:
            if self.start_node != start:
                # 更新 km
                self.km += self._heuristic_inline(self.last_start_node, start)
                self.last_start_node = start
                self.start_node = start
            self.logger.info(f"[D* Py] 🔄 增量模式: 复用搜索树 (Km={self.km:.2f})")
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

        self.rhs[goal] = 0.0
        self._insert_to_open(goal, self._calculate_key(goal))

        self.logger.info(f"[D* Py] 🆕 全量初始化: {start} -> {goal}")

        if not self._compute_shortest_path():
            self.logger.warning("[D* Py] 初次计算无解。")
            return False

        return True

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """
        [生命周期] 处理环境变化 (增量更新入口)。
        """
        if not changes:
            return

        # --- Rust 核心 ---
        if self.rust_planner:
            # 数据格式转换: List[Tuple] -> List[(r,c,l,val)]
            rust_changes = []
            for item in changes:
                if len(item) == 3:  # (r, c, val)
                    rust_changes.append((item[0], item[1], 0, item[2]))
                elif len(item) == 4:  # (r, c, l, val)
                    rust_changes.append(item)

            try:
                t_start = time.perf_counter()
                self.rust_planner.update_obstacles(rust_changes)
                dt = (time.perf_counter() - t_start) * 1000
                self.logger.info(
                    f"[D* Rust] 增量更新完成: 处理 {len(changes)} 处变化, 耗时 {dt:.2f}ms"
                )
                return
            except Exception as e:
                self.logger.error(f"[D* Rust] 增量更新崩溃: {e}")
                # Rust 状态已脏，这里不做复杂回退，依靠下一次 Plan 的错误恢复机制

        # --- Python 原生 ---
        if not self.goal_node or not self.start_node:
            return

        self.logger.debug(f"[D* Py] 处理 {len(changes)} 个环境变化...")

        for change in changes:
            coords = change[:-1]
            u: NodeType = tuple(coords)  # type: ignore
            if not self.is_valid(u):
                continue

            # 更新自身和邻居
            self._update_vertex(u)
            for neighbor, _ in self._get_neighbors(u):
                self._update_vertex(neighbor)

        self.stats["replanning_count"] = self.stats.get("replanning_count", 0) + 1
        # 触发增量传播
        self._compute_shortest_path()

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """
        [生命周期] 路径提取与修复 (Query Phase)。
        """
        # --- 分支 A: Rust 核心 ---
        if self.enable_rust and self.rust_planner:
            s_tuple = (
                int(current_pos[0]),
                int(current_pos[1]),
                int(current_pos[2]) if len(current_pos) > 2 else 0,
            )

            try:
                # 调用 Rust 计算
                # 返回值: (路径列表, 本次扩展节点数, 累计重规划次数)
                result = self.rust_planner.compute_path(s_tuple)
                rust_path, nodes_expanded, replan_count = result

                # 更新统计面板数据
                self.stats["nodes_expanded"] = nodes_expanded
                self.stats["replanning_count"] = replan_count

                if rust_path:
                    # 将 Rust 的 List[PyObject] 转换回 Python 的 List[Tuple]
                    py_path = []
                    for p in rust_path:
                        # 2D 模式下 Rust 可能会返回 (x, y, 0)，我们需要切掉 z
                        if self.layers <= 1 and len(p) == 3:
                            py_path.append((p[0], p[1]))
                        else:
                            py_path.append(tuple(p))

                    self.logger.info(
                        f"[D* Rust] 路径生成成功. 长度: {len(py_path)}, "
                        f"扩展节点: {nodes_expanded}, 重规划: {replan_count}"
                    )
                    return py_path
                else:
                    self.logger.warning(f"[D* Rust] 无解 (Nodes: {nodes_expanded})。")
                    return None

            except Exception as e:
                self.logger.error(
                    f"[D* Rust] 运行时发生 Panic: {e}。建议检查地图数据一致性。",
                    exc_info=True,
                )
                return None

        # --- 分支 B: Python 原生 ---

        if not self.goal_node:
            return None
        if current_pos == self.goal_node:
            return [current_pos]

        # 1. 位置同步与 Km 更新
        if current_pos != self.last_start_node:
            self.km += self._heuristic_inline(self.last_start_node, current_pos)
            self.last_start_node = current_pos
            self.logger.debug(f"[D* Move] 起点移动 -> Km更新为 {self.km:.2f}")

        self.start_node = current_pos

        # 2. 路径修复 (尝试增量计算)
        if not self._compute_shortest_path():
            self.logger.warning(
                "[D* Repair] ⚠️ 增量修复失败，尝试全量重置 (Fallback)..."
            )

            # 兜底: 全量重置
            self.U = []
            self.open_keys.clear()
            self.g.clear()
            self.rhs.clear()
            self.km = 0.0

            self.rhs[self.goal_node] = 0.0
            self._insert_to_open(self.goal_node, self._calculate_key(self.goal_node))

            if not self._compute_shortest_path():
                self.logger.error("[D* Repair] ❌ 全量重规划失败，确认为不可达。")
                return None

        # 3. 梯度下降提取路径
        if self.g.get(current_pos, INF) == INF:
            self.logger.warning(f"[D* Path] 起点 g=INF，不可达。")
            return None

        path = [current_pos]
        curr = current_pos
        MAX_STEPS = self.max_nodes_expanded

        while curr != self.goal_node and len(path) < MAX_STEPS:
            min_cost = INF
            best_next = None

            # 寻找梯度下降方向
            for neighbor, move_cost in self._get_neighbors(curr):
                if self.is_obstacle(neighbor):
                    continue

                c = move_cost + self.g.get(neighbor, INF)
                if c < min_cost - EPSILON:
                    min_cost = c
                    best_next = neighbor

            if best_next:
                if best_next in path:
                    self.logger.error(f"[D* Path] ⛔️ 检测到死循环环路 at {best_next}。")
                    return None
                path.append(best_next)
                curr = best_next
            else:
                self.logger.error(f"[D* Path] 陷入局部死胡同 at {curr}。")
                break

        if len(path) >= MAX_STEPS:
            self.logger.error("[D* Path] 路径过长，触发最大步数熔断。")
            return None

        self.logger.info(
            f"[D* Py] 路径生成完毕. 长度: {len(path)}, 扩展: {self.stats.get('nodes_expanded',0)}"
        )
        return path

    # =========================================================================
    # Python 核心逻辑 (保持原有结构，作为 Rust 的镜像实现)
    # =========================================================================

    def _insert_to_open(self, u: NodeType, key: Tuple[float, float]):
        self.open_keys[u] = key
        heapq.heappush(self.U, (key, u))

    def _calculate_key(self, u: NodeType) -> Tuple[float, float]:
        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)
        min_val = min(g_val, rhs_val)

        if min_val == INF:
            return (INF, INF)

        k1 = min_val + self._heuristic_inline(self.start_node, u) + self.km
        return (k1, min_val)

    def _update_vertex(self, u: NodeType):
        if self.is_obstacle(u):
            self.rhs[u] = INF
        elif u != self.goal_node:
            min_rhs = INF
            for neighbor, move_cost in self._get_neighbors(u):
                if self.is_obstacle(neighbor):
                    continue
                temp = move_cost + self.g.get(neighbor, INF)
                if temp < min_rhs:
                    min_rhs = temp
            self.rhs[u] = min_rhs

        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)

        if abs(g_val - rhs_val) <= EPSILON:
            if u in self.open_keys:
                del self.open_keys[u]
        else:
            self._insert_to_open(u, self._calculate_key(u))

    def _compute_shortest_path(self) -> bool:
        """
        [核心] D* Lite 主循环 (Wavefront Propagation)。
        """
        if not self.start_node:
            return False

        expansions = 0

        while self.U:
            # 熔断保护
            if expansions > self.max_nodes_expanded:
                self.logger.error(
                    f"[D* Loop] ⛔️ 触发熔断保护 (>{self.max_nodes_expanded} nodes)。"
                )
                return False

            k_old, u = self.U[0]

            # Lazy Removal
            if u not in self.open_keys or self.open_keys[u] != k_old:
                heapq.heappop(self.U)
                continue

            start_g = self.g.get(self.start_node, INF)
            start_rhs = self.rhs.get(self.start_node, INF)
            k_start = self._calculate_key(self.start_node)

            # 终止条件: k_old >= k_start 且 start 一致
            term1 = k_old[0] > k_start[0] + FLOAT_TOLERANCE
            term2 = (
                abs(k_old[0] - k_start[0]) < EPSILON
                and k_old[1] > k_start[1] + FLOAT_TOLERANCE
            )

            if abs(start_g - start_rhs) < EPSILON and (term1 or term2):
                return True

            # 启发式剪枝 (Cost Threshold)
            if start_g == INF and start_rhs == INF and k_old[0] > self.cost_threshold:
                heapq.heappop(self.U)
                if u in self.open_keys:
                    del self.open_keys[u]
                continue

            heapq.heappop(self.U)
            expansions += 1
            self.stats["nodes_expanded"] = self.stats.get("nodes_expanded", 0) + 1

            k_new = self._calculate_key(u)

            # 情况 A: Key 变小
            if k_old < k_new:
                self._insert_to_open(u, k_new)
                continue

            g_u = self.g.get(u, INF)
            rhs_u = self.rhs.get(u, INF)

            # 情况 B: Overconsistent (g > rhs)
            if g_u > rhs_u + EPSILON:
                self.g[u] = rhs_u
                if u in self.open_keys:
                    del self.open_keys[u]
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    # 传播更新
                    if self.rhs.get(s, INF) > rhs_u + cost:
                        self.rhs[s] = rhs_u + cost
                        self._update_vertex(s)
            # 情况 C: Underconsistent (g < rhs)
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for s, _ in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    self._update_vertex(s)

        return False

    def _heuristic_inline(self, a: Optional[NodeType], b: Optional[NodeType]) -> float:
        if not a or not b:
            return 0.0

        # 3D
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
        # 2D
        else:
            dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
            return (dx + dy) + (self.COST_2 - 2) * min(dx, dy)

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
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

                        # 3D 简单防切角
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
                        res.append(((nx, ny, nz), cost))  # type: ignore
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
                res.append(((nr, nc), cost))  # type: ignore
        return res
