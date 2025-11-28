import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Set, Union, Any

# 导入基类和类型定义
# PathPlannerBase: 规划器抽象基类，提供通用的接口和锁机制
# NodeType: 泛型坐标类型，适配 (row, col) 或 (row, col, layer)
from .base import PathPlannerBase, NodeType

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
    【L1 层 - D* Lite 路径规划器】
    (D* Lite Path Planner - Production Grade)

    算法原理 (Algorithm Theory):
    D* Lite 是基于 LPA* (Lifelong Planning A*) 的反向增量搜索算法。

    1. **反向搜索 (Reverse Search)**:
       - 搜索从【终点 Goal】向【起点 Start】进行。
       - 此时 g(x) 代表 x 到 Goal 的实际代价。
       - 优势: 当机器人移动（Start 变化）时，只要 Goal 不变，搜索树的大部分结构（即各节点到 Goal 的路径）依然有效，无需重算。

    2. **RHS 值 (Lookahead Value)**:
       - rhs(u) = min(g(s) + c(s, u))，其中 s 是 u 的邻居。
       - 它代表基于邻居信息推导出的"理论最优代价"。
       - g(u): 当前节点存储的"旧代价"。

    3. **一致性 (Consistency)**:
       - **局部一致 (Consistent)**: g(u) == rhs(u)。节点状态稳定，无需更新。
       - **局部过一致 (Overconsistent)**: g(u) > rhs(u)。通常意味着发现了一条更短的路径（如障碍物移除）。
       - **局部欠一致 (Underconsistent)**: g(u) < rhs(u)。通常意味着路径被阻断（如障碍物出现），需要将 g(u) 重置为 INF 并重新传播。

    4. **增量更新 (Incremental Update)**:
       - 算法维护一个优先队列 (U)，仅包含"不一致"的节点。
       - 每次环境变化或机器人移动，只需处理队列中的节点，直到起点恢复一致性。
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
        初始化 D* Lite 规划器。

        Args:
            grid: 网格数据引用 (0=空, 1=障碍)。
            width_m, length_m, height_m: 场地物理尺寸。
            resolution: 网格分辨率。
            logger: 日志记录器。
            grid_lock: 线程锁。
            use_octile_3d: 是否使用 3D Octile 距离。
            heuristic_weight: 启发式权重 (D* Lite 强制要求为 1.0)。
        """
        # 调用父类初始化，设置基础属性
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )

        self.use_octile_3d = use_octile_3d

        # [权重一致性]
        # D* Lite 算法严重依赖启发式的一致性 (Consistency)。
        # 加权启发式 (Weighted Heuristic, w > 1) 会破坏三角不等式，导致优先级队列 (Priority Queue) 排序错乱。
        # 具体表现为: 增量更新时，队列中远处的节点 Key 被人为放大，导致算法误判"起点已经最优"，
        # 从而过早终止 (Premature Termination)，留下未更新的"死循环"节点。
        if heuristic_weight > 1.0:
            if self.logger:
                self.logger.warning(
                    f"[D* Lite Init] ⚠️ 警告: 检测到加权启发式 (W={heuristic_weight})。"
                    f"D* Lite 要求严格一致性，为防止增量更新死锁，已强制重置 W=1.0。"
                )
            self.heuristic_weight = 1.0
        else:
            self.heuristic_weight = max(1.0, heuristic_weight)

        # --- D* Lite 核心数据结构 ---

        # g 值表: 记录从当前节点到终点的已知代价
        # g[u] = Cost(u -> Goal)
        self.g: Dict[NodeType, float] = {}

        # rhs 值表: 基于邻居推算出的理论最小代价
        # rhs[u] = min(Cost(u -> s) + g[s])
        self.rhs: Dict[NodeType, float] = {}

        # 优先队列 (Min-Heap): 存放待处理的“不一致”节点
        # 元素格式: (Key, Node)，其中 Key 是一个二元组 (k1, k2)
        # Python 的 heapq 是最小堆，Key 越小越优先出队。
        self.U: List[Tuple[Tuple[float, float], NodeType]] = []

        # 索引字典: 映射 Node -> 最新合法的 Key
        # 作用: 配合 heapq 实现 Lazy Removal (懒惰删除)。
        # 因为 heapq 不支持高效的 update/remove 操作，我们在更新节点 key 时直接 push 新的。
        # 弹出时，对比堆顶元素的 key 和 open_keys 中记录的 key，如果不一致则丢弃。
        self.open_keys: Dict[NodeType, Tuple[float, float]] = {}

        # km (Key Modifier): 关键修饰值
        # 当起重机移动（起点变化）时，heuristic 会发生变化。
        # 为了避免重新计算全图中所有节点的 Key，我们引入 km 来累积 heuristic 的差值。
        self.km = 0.0

        # 状态记录
        self.start_node: Optional[NodeType] = None  # 当前起重机的位置 (作为搜索目标)
        self.goal_node: Optional[NodeType] = None  # 目标位置 (作为搜索树的根)
        self.last_start_node: Optional[NodeType] = None  # 上一次计算时的起重机位置

        # --- 动态阈值配置 ---

        # 估算体素总数
        # 确保行列层为整数，防止 float 计算导致 total_voxels 异常
        rows_int = int(self.rows) if self.rows else 100
        cols_int = int(self.cols) if self.cols else 100
        layers_int = int(self.layers) if self.layers else 1
        total_voxels = max(1000, rows_int * cols_int * max(1, layers_int))

        # 最大迭代次数，防止死循环 (Safety Brake)
        self.max_iter_limit = total_voxels * 20

        # 代价熔断阈值
        # 如果路径代价超过这个值，认为不可达。
        # 计算逻辑: 全图格子数 * 最大单步代价(sqrt(3)) * 安全系数(10)
        max_step_cost = 1.74
        self.cost_threshold = float(total_voxels) * max_step_cost * 10.0
        self.cost_threshold = max(100000.0, self.cost_threshold)

        # 预计算移动代价，避免重复开方运算 (Optimization)
        self.COST_1 = 1.0  # 直线
        self.COST_2 = 1.41421356  # 2D 对角线 (sqrt 2)
        self.COST_3 = 1.73205081  # 3D 对角线 (sqrt 3)

        # 扩展节点数熔断阈值
        self.max_nodes_expanded = max(5000, total_voxels * 5)

        self.logger.info(
            f"[D* Lite Init] 初始化就绪. "
            f"启发式权重 (Heuristic Weight)={self.heuristic_weight:.2f}, "
            f"地图尺寸 (Map Size)={rows_int}x{cols_int}x{layers_int}"
        )

    def _reset_stats(self):
        """重置单次规划的统计数据。"""
        self.stats["nodes_expanded"] = 0
        self.stats["replanning_count"] = 0

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """
        [生命周期] 初始化规划任务。
        根据起终点是否变化，智能决定是【复用】之前的搜索树，还是【全量重置】。

        Args:
            start: 起点坐标 (机器人当前位置)。
            goal: 终点坐标。

        Returns:
            bool: 初始化是否成功。
        """
        self._reset_stats()

        # 1. 基础合法性校验
        if start is None or goal is None:
            self.logger.warning("[D* Init] ❌ 失败: 起点或终点为 None。")
            return False

        # 快速检查起点终点重合
        if start == goal:
            self.logger.info("[D* Init] 起点与终点重合，无需规划。")
            self.start_node = start
            self.goal_node = goal
            # 伪造一个简单的状态以便 _compute_path_core 能返回路径
            self.g.clear()
            self.rhs.clear()
            self.g[start] = 0.0
            self.rhs[start] = 0.0
            return True

        if not self.is_valid(start):
            self.logger.error(f"[D* Init] ❌ 失败: 起点 {start} 越界。")
            return False
        if not self.is_valid(goal):
            self.logger.error(f"[D* Init] ❌ 失败: 终点 {goal} 越界。")
            return False
        if self.is_obstacle(goal):
            self.logger.warning(f"[D* Init] ⚠️ 警告: 终点 {goal} 位于障碍物内。")
            return False

        # 2. [智能复用] 检查是否可以进行增量规划
        # 条件: 目标点没变 (搜索树的根没变) 且 g 表已有数据
        if self.goal_node == goal and self.g:
            # 如果起重机移动了，需要更新 km
            if self.start_node != start:
                # 累加 heuristic 的变化量
                # km += h(last_start, new_start)
                self.km += self._heuristic_inline(self.last_start_node, start)
                self.last_start_node = start
                self.start_node = start

            self.logger.info(
                f"[D* Init] 🔄 增量模式激活: 复用搜索树，当前 Km={self.km:.2f}"
            )
            return True

        # 3. [全量重置] 否则，清空所有数据重新开始
        self.start_node = start
        self.goal_node = goal
        self.last_start_node = start
        self.km = 0.0

        # 清空核心数据结构
        self.U = []
        self.open_keys.clear()
        self.g.clear()
        self.rhs.clear()

        # 设定终点初始状态 (反向搜索，Goal 的 rhs 设为 0)
        self.rhs[goal] = 0.0

        # 将终点加入优先队列，作为搜索波浪的源头
        # 计算 Key 时使用初始起点作为 heuristic 的目标
        self._insert_to_open(goal, self._calculate_key(goal))

        self.logger.info(f"[D* Init] 🆕 全量初始化: {start} -> {goal} (反向搜索模式)")

        # 执行首次计算
        if not self._compute_shortest_path():
            self.logger.warning("[D* Init] ⚠️ 初次计算未找到路径 (可能不可达)。")
            return False

        return True

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """
        [生命周期] 处理环境变化 (增量更新入口)。

        当 MapManager 发现障碍物变化时调用。

        Args:
            changes: 变化列表。
                     格式: [(row, col, new_val), ...] 或 [(row, col, layer, new_val), ...]
        """
        if not self.goal_node or not self.start_node:
            self.logger.warning("[D* Update] ⚠️ 未初始化起终点，跳过更新。")
            return
        if not changes:
            return

        self.logger.debug(
            f"[D* Update] 收到 {len(changes)} 个环境变化，准备更新 Rhs 值..."
        )

        # 遍历所有变化的节点
        for change in changes:
            # 提取坐标 (切片去掉最后一个 value)
            coords = change[:-1]
            # 确保坐标为元组
            u: NodeType = tuple(coords)  # type: ignore

            if not self.is_valid(u):
                continue

            # 1. 更新节点 u 自身的状态
            # 如果 u 变成了障碍物，它的 rhs 应该变 INF
            # 如果 u 变空了，它的 rhs 需要根据邻居重算
            self._update_vertex(u)

            # 2. 更新 u 的所有邻居
            # 因为 u 的阻挡状态变了，经过 u 到达 Goal 的邻居们的代价也会变
            for neighbor, cost in self._get_neighbors(u):
                self._update_vertex(neighbor)

        # 统计重规划次数
        self.stats["replanning_count"] = self.stats.get("replanning_count", 0) + 1

        # 触发增量修复循环
        self._compute_shortest_path()

    def _insert_to_open(self, u: NodeType, key: Tuple[float, float]):
        """
        [内部方法] 将节点插入优先队列，并维护索引。
        """
        self.open_keys[u] = key
        heapq.heappush(self.U, (key, u))

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """
        [生命周期] 路径提取与修复 (Query Phase)。

        这是每帧调用的主入口。它负责：
        1. 检查起重机移动，更新 km。
        2. 触发路径修复 (_compute_shortest_path)。
        3. 如果修复失败，尝试全量重置作为兜底。
        4. 使用梯度下降 (Gradient Descent) 提取路径。

        Args:
            current_pos: 起重机当前位置。
        """
        if not self.goal_node:
            return None

        # 起点终点重合直接返回
        if current_pos == self.goal_node:
            return [current_pos]

        # --- 1. 位置同步与 Km 更新 ---
        if current_pos != self.last_start_node:
            # 起重机移动了，所有节点相对于起重机的 heuristic 都变了
            # 我们不更新全图 Key，而是更新全局偏移量 km
            self.km += self._heuristic_inline(self.last_start_node, current_pos)
            self.last_start_node = current_pos
            self.logger.debug(
                f"[D* Move] 🤖 起重机移动到 {current_pos}, 更新 km={self.km:.2f}"
            )

        self.start_node = current_pos

        # --- 2. 路径修复 (含自动回退机制) ---
        # 尝试增量修复
        if not self._compute_shortest_path():
            self.logger.warning(
                "[D* Repair] ⚠️ 增量修复失败或超时，触发全量重规划兜底 (Fallback)..."
            )

            # 兜底方案: 全量重置状态
            self.U = []
            self.open_keys.clear()
            self.g.clear()
            self.rhs.clear()
            self.km = 0.0

            # 重新初始化终点
            self.rhs[self.goal_node] = 0.0
            self._insert_to_open(self.goal_node, self._calculate_key(self.goal_node))

            if not self._compute_shortest_path():
                self.logger.error("[D* Repair] ❌ 全量重规划也失败，确认为不可达。")
                return None
            else:
                self.logger.info("[D* Repair] ✅ 全量重规划成功挽救了路径。")

        # --- 3. 检查起点可达性 ---
        if self.g.get(current_pos, INF) == INF:
            self.logger.warning(
                f"[D* Path] ⚠️ 起点 {current_pos} 的 g 值为 INF，路径不可达。"
            )
            return None

        # --- 4. 梯度下降提取路径 (Gradient Descent) ---
        # D* Lite 生成的是一个"代价场" (Cost Field)，我们需要沿着代价下降最快的方向走
        path = [current_pos]
        curr = current_pos
        MAX_STEPS = self.max_nodes_expanded

        # 环路检测集合
        visited_set = {curr}

        while curr != self.goal_node and len(path) < MAX_STEPS:
            min_cost = INF
            best_next = None

            # Debug Log 收集邻居详情，用于诊断死循环
            debug_neighbors_info = []

            # 遍历邻居，寻找 Cost = c(curr, next) + g(next) 最小的节点
            for neighbor, move_cost in self._get_neighbors(curr):
                if self.is_obstacle(neighbor):
                    continue

                g_n = self.g.get(neighbor, INF)
                c = move_cost + g_n

                # 仅在疑似问题时记录详细信息 (例如已经在路径中，或者是候选节点)
                if len(path) < 10:
                    debug_neighbors_info.append(f"{neighbor}:g={g_n:.2f},c={c:.2f}")
                elif neighbor in visited_set:
                    debug_neighbors_info.append(
                        f"{neighbor}[LOOP]:g={g_n:.2f},c={c:.2f}"
                    )

                # 浮点比较建议使用 EPSILON，防止两点代价极度接近时反复跳跃
                if c < min_cost - EPSILON:
                    min_cost = c
                    best_next = neighbor
                # 如果没有找到显著更优的，但正好等于 min_cost (或者在误差范围内)，
                # 则保持现有的 best_next (通常第一个找到的就行)，避免抖动。

            # [详细日志] 打印前几步的决策过程
            if len(path) < 10:
                self.logger.debug(
                    f"[PathTrace] Step {len(path)}: {curr} (g={self.g.get(curr, INF):.2f}) "
                    f"-> Select {best_next} (cost={min_cost:.2f}). Candidates: {debug_neighbors_info}"
                )

            # [严重错误检测: 局部极小值陷阱]
            if best_next and best_next in visited_set:
                self.logger.error(
                    f"[D* LOOP TRAP] ⛔️ 死循环检测!\n"
                    f"    Current Node: {curr} (g={self.g.get(curr, INF):.2f})\n"
                    f"    Target Node : {best_next} (g={self.g.get(best_next, INF):.2f})\n"
                    f"    Calc Cost   : {min_cost:.2f}\n"
                    f"    Context     : 邻居的组合代价比当前更优，但它已经在路径历史中。\n"
                    f"                  这说明梯度场更新不完整，存在未传播的低 g 值孤岛。\n"
                    f"                  建议检查 update_obstacles 的传播逻辑或 heuristic 一致性。"
                )
                return None

            if best_next:
                path.append(best_next)
                visited_set.add(best_next)
                curr = best_next
            else:
                self.logger.error(f"[D* Path] ❌ 在 {curr} 处陷入死胡同 (无路可走)。")
                break

        if len(path) >= MAX_STEPS:
            self.logger.error("[D* Path] ❌ 路径生成超过最大步数，可能存在环路。")
            return None

        return path

    def _calculate_key(self, u: NodeType) -> Tuple[float, float]:
        """
        计算节点的排序键值 (Key)。

        Key = [k1, k2]
        - k1 = min(g, rhs) + h + km   (预估总代价，用于主排序)
        - k2 = min(g, rhs)            (当前实际代价，用于打破平局)
        """
        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)

        # 取 g 和 rhs 中的较小值作为基础代价
        min_val = g_val if g_val < rhs_val else rhs_val

        if min_val == INF:
            return (INF, INF)

        # 加上 heuristic 和 km
        # 注意: h 这里必须使用 Consistent Heuristic (即 weight=1.0)
        k1 = min_val + self._heuristic_inline(self.start_node, u) + self.km
        return (k1, min_val)

    def _update_vertex(self, u: NodeType):
        """
        [核心] 节点状态更新与一致性检查。

        逻辑:
        1. 根据邻居重新计算 u 的 rhs 值。
        2. 比较 g(u) 和 rhs(u)。
        3. 如果一致 (g==rhs)，从队列移除。
        4. 如果不一致 (g!=rhs)，插入队列重新排队。
        """
        # 情况 1: 如果是障碍物，代价无限大
        if self.is_obstacle(u):
            self.rhs[u] = INF
        # 情况 2: 普通节点 (非 Goal)，rhs = min(g(neighbor) + cost)
        elif u != self.goal_node:
            min_rhs = INF
            for neighbor, move_cost in self._get_neighbors(u):
                if self.is_obstacle(neighbor):
                    continue
                g_n = self.g.get(neighbor, INF)
                if g_n != INF:
                    temp = move_cost + g_n
                    if temp < min_rhs:
                        min_rhs = temp
            self.rhs[u] = min_rhs

        # 检查一致性
        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)

        if abs(g_val - rhs_val) <= EPSILON:
            # 局部一致: 无需处理，从队列移除索引 (Heap 懒惰删除)
            if u in self.open_keys:
                del self.open_keys[u]
        else:
            # 局部不一致: 插入队列
            self._insert_to_open(u, self._calculate_key(u))

    def _compute_shortest_path(self) -> bool:
        """
        [核心] D* Lite 主循环 (Wavefront Propagation)。

        只要队列不为空，且 (堆顶 Key < 起点 Key OR 起点不一致)，就持续循环。
        这个过程会将代价的变化像波浪一样传播开来。
        """
        if not self.start_node:
            return False

        # 缓存函数引用，减少循环内的查找开销 (Optimization)
        heappush = heapq.heappush
        heappop = heapq.heappop
        calc_key = self._calculate_key
        get_g = self.g.get
        get_rhs = self.rhs.get

        loops = 0
        valid_expansions = 0
        current_expansion = 0

        while self.U:
            loops += 1
            # --- 熔断保护 ---
            if valid_expansions > self.max_nodes_expanded:
                self.logger.error(
                    f"[D* Loop] ⛔️ 超过扩展上限 {self.max_nodes_expanded}，强制中断防止卡死。"
                )
                self.stats["nodes_expanded"] = (
                    self.stats.get("nodes_expanded", 0) + current_expansion
                )
                return False

            # 获取堆顶 (Key, Node)
            k_old, u = self.U[0]

            # --- Lazy Removal (懒惰删除) ---
            # 检查堆顶元素是否是过期的脏数据
            if u not in self.open_keys or self.open_keys[u] != k_old:
                heappop(self.U)
                continue

            # --- 终结条件检查 (Termination Check) ---
            start_g = get_g(self.start_node, INF)
            start_rhs = get_rhs(self.start_node, INF)

            # 只有当 Start Node 是一致的 (g == rhs)
            if abs(start_g - start_rhs) < EPSILON:
                k_start = calc_key(self.start_node)

                # 使用带容差的比较: k_old > k_start + tolerance
                # 原理: 由于浮点数累加误差，k_old 可能仅仅比 k_start 大 1e-10。
                # 如果此时退出，会导致微小的代价差异没有传播到位。
                # 只有当堆顶 Key 显著大于 Start Key 时，我们才确信"所有重要的更新都处理完了"。
                if k_old[0] > k_start[0] + FLOAT_TOLERANCE:
                    # Key 确实大很多，安全退出
                    self.stats["nodes_expanded"] = (
                        self.stats.get("nodes_expanded", 0) + current_expansion
                    )
                    self.logger.debug(
                        f"[D* Loop] ✅ 正常收敛 (Key满足). Loops={loops}, Valid={valid_expansions}"
                    )
                    return True
                elif (
                    abs(k_old[0] - k_start[0]) < EPSILON
                    and k_old[1] > k_start[1] + FLOAT_TOLERANCE
                ):
                    # 第一关键字相等，第二关键字大很多，安全退出
                    self.stats["nodes_expanded"] = (
                        self.stats.get("nodes_expanded", 0) + current_expansion
                    )
                    self.logger.debug(
                        f"[D* Loop] ✅ 正常收敛 (Key2满足). Loops={loops}"
                    )
                    return True

                # 如果代码走到这里，说明虽然 start 是一致的，但堆里还有代价非常接近的节点
                # 策略: 继续扩展，哪怕稍微多算一点，也要保证正确性。

            # 启发式剪枝 (不可达判断)
            # 如果起点已经是 INF 且 堆顶代价极大，说明搜索到了天边也没找到路
            if start_g == INF and start_rhs == INF:
                if k_old[0] > self.cost_threshold:
                    heappop(self.U)
                    if u in self.open_keys:
                        del self.open_keys[u]
                    continue

            # --- 节点扩展 (Expansion) ---
            heappop(self.U)
            valid_expansions += 1
            current_expansion += 1

            # 计算当前最新的 Key
            k_new = calc_key(u)

            # 情况 A: 节点的 Key 已经过时 (例如 km 变化导致)
            # Key 是元组，Python 的元组比较是逐元严格比较。
            # D* Lite 中这里判断的是 k_old 是否小于 k_new。
            # 如果 k_old < k_new，说明在等待处理期间，节点的优先级降低了（代价变大了），需要重新入队。
            if k_old < k_new:
                self._insert_to_open(u, k_new)
                continue

            g_u = get_g(u, INF)
            rhs_u = get_rhs(u, INF)

            # 情况 B: Overconsistent (g > rhs)
            # 意味着发现了一条更短的路径 (通常是障碍物移除或初次搜索)
            # 显式使用 EPSILON 防止浮点相等被误判为 >
            if g_u > rhs_u + EPSILON:
                # 1. 让 g 逼近 rhs (变小)
                if u in self.open_keys:
                    del self.open_keys[u]
                self.g[u] = rhs_u
                new_g = rhs_u

                # 2. 传播这个"好消息"给邻居
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    # 如果经由 u 能让邻居 s 变好，更新 s
                    if new_g + cost < get_rhs(s, INF):
                        self.rhs[s] = new_g + cost
                        self._update_vertex(s)

            # 情况 C: Underconsistent (g < rhs)
            # 意味着路径被阻断 (g 值偏低，是无效的旧值)，或者节点本身变成了障碍物
            # 显式处理 < 状态
            elif g_u < rhs_u - EPSILON:
                # 1. 强制将 g 重置为 INF (清理旧状态)
                self.g[u] = INF
                self._update_vertex(
                    u
                )  # 将自己加回 Queue 重新评估 (因为 g 变了，rhs 可能还没变)

                # 2. 强制通知所有邻居 "u 变坏了"
                # 原有 BUG: 试图判断 if rhs[s] == g_u + cost。
                # 修复逻辑: 无论邻居之前是否依赖 u，都强制触发一次 _update_vertex(s)。
                # _update_vertex 内部会重新扫描 s 的所有邻居。如果 s 能找到其他路，它的 rhs 不变；
                # 如果 s 只能走 u，它的 rhs 就会变大。
                # 这保证了"阻断"消息能无条件传播，避免死循环。
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    self._update_vertex(s)

            else:
                # 剩下的是 abs(g-rhs) <= EPSILON 的情况。
                # 理论上这些节点不应该在 U 中被处理（它们是一致的），
                # 但由于 Lazy Removal 或浮点误差可能到达这里。
                # 视为一致，清理索引，不做任何操作。
                if u in self.open_keys:
                    del self.open_keys[u]
                continue

        # 堆空了
        self.stats["nodes_expanded"] = (
            self.stats.get("nodes_expanded", 0) + current_expansion
        )
        return False

    def _heuristic_inline(self, a: Optional[NodeType], b: Optional[NodeType]) -> float:
        """
        启发式函数计算 (H值)。
        根据模式选择 Euclidean (欧氏) 或 Octile (八方向/二十六方向) 距离。
        """
        if not a or not b:
            return 0.0
        h = 0.0

        # 3D Heuristic
        if len(a) == 3:
            dx, dy, dz = abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2])
            if self.use_octile_3d:
                # Octile 3D: 对角线移动优先
                delta = sorted([dx, dy, dz])
                h = (
                    delta[0] * self.COST_3
                    + (delta[1] - delta[0]) * self.COST_2
                    + (delta[2] - delta[1]) * self.COST_1
                )
            else:
                # Euclidean
                h = math.sqrt(dx * dx + dy * dy + dz * dz)
        # 2D Heuristic
        else:
            dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
            # Octile 2D: max(dx, dy) + (sqrt2 - 1) * min(dx, dy)
            h = (dx + dy) + (self.COST_2 - 2) * min(dx, dy)

        # 注意: 这里虽然乘了权重，但在 __init__ 中已被强制修正为 1.0
        return h * self.heuristic_weight

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
        """
        获取合法邻居。
        包含: 越界检查、障碍物检查、3D切角检查。
        """
        res = []
        # --- 3D 邻居生成 (26 邻域) ---
        if len(node) == 3:
            # 强制转换为 int，防止外部传入 float 坐标导致数组索引错误
            x, y, z = int(node[0]), int(node[1]), int(node[2])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        nx, ny, nz = x + dx, y + dy, z + dz

                        # 1. 越界检查
                        if not (
                            0 <= nx < self.rows
                            and 0 <= ny < self.cols
                            and 0 <= nz < self.layers
                        ):
                            continue

                        # 2. 障碍物检查
                        if self.grid[nx][ny][nz] == 1:
                            continue

                        # 3. 切角检查 (防止穿模)
                        # 如果是斜向移动，检查涉及的几个基础方块是否有障碍
                        is_corner = False
                        if (
                            dx != 0
                            and dy != 0
                            and (
                                self.grid[x + dx][y][z] == 1
                                or self.grid[x][y + dy][z] == 1
                            )
                        ):
                            is_corner = True
                        elif (
                            dx != 0
                            and dz != 0
                            and (
                                self.grid[x + dx][y][z] == 1
                                or self.grid[x][y][z + dz] == 1
                            )
                        ):
                            is_corner = True
                        elif (
                            dy != 0
                            and dz != 0
                            and (
                                self.grid[x][y + dy][z] == 1
                                or self.grid[x][y][z + dz] == 1
                            )
                        ):
                            is_corner = True
                        if is_corner:
                            continue

                        # 计算代价
                        dist_sq = dx * dx + dy * dy + dz * dz
                        cost = (
                            self.COST_1
                            if dist_sq == 1
                            else (self.COST_2 if dist_sq == 2 else self.COST_3)
                        )
                        res.append(((nx, ny, nz), cost))  # type: ignore

        # --- 2D 邻居生成 (8 邻域) ---
        else:
            # 强制转换为 int，防止外部传入 float 坐标导致数组索引错误
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
                # 越界
                if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                    continue
                # 障碍
                if self.grid[nr][nc] == 1:
                    continue
                # 切角 (Corner Cutting)
                if dr != 0 and dc != 0:
                    if self.grid[r + dr][c] == 1 or self.grid[r][c + dc] == 1:
                        continue
                res.append(((nr, nc), cost))  # type: ignore
        return res
