import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Set, Union, Any

# 导入基类和类型定义
from .base import PathPlannerBase, NodeType

# 浮点数比较容差 (用于判断两个浮点数是否相等)
EPSILON = 1e-4
# 无穷大常量 (表示不可达或尚未探索)
INF = float("inf")


class DLitePlanner(PathPlannerBase[NodeType]):
    """
    【L1 层 - D* Lite 路径规划器】
    (D* Lite Path Planner - Optimized with Dictionary Indexing)

    算法简介:
    D* Lite 是一种【增量式】启发式搜索算法，非常适合动态环境。
    与 A* 每次必须从头计算不同，D* Lite 可以复用上一次的搜索结果，
    只针对地图中发生变化（如新增障碍物）的部分进行局部更新，从而极大提高效率。

    核心机制:
    1. **反向搜索**: 从【终点】向【起点】搜索。这样当机器人移动时（起点变了），
       也就是目标树的根节点没变（终点没变），大部分路径信息依然有效。
    2. **RHS 值**: One-step lookahead value。基于邻居节点的 g 值计算出的“理论最优代价”。
       - g(u): 节点 u 当前存储的代价。
       - rhs(u): min(g(s) + c(s, u))，其中 s 是 u 的邻居。
    3. **一致性**:
       - g == rhs: 局部一致 (Consistent)，状态稳定。
       - g != rhs: 局部不一致 (Inconsistent)，该节点需要被加入优先队列进行处理。
    4. **优先队列 (Min-Heap)**: 存储所有不一致节点，按照 Key 排序。
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
        """
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )

        self.use_octile_3d = use_octile_3d
        self.heuristic_weight = max(1.0, heuristic_weight)

        # --- D* Lite 核心数据结构 ---

        # g 值表: 记录从当前节点到终点的已知代价
        self.g: Dict[NodeType, float] = {}

        # rhs 值表: 基于邻居推算出的理论最小代价
        self.rhs: Dict[NodeType, float] = {}

        # 优先队列 (Min-Heap): 存放待处理的“不一致”节点
        # 元素格式: (Key, Node)，其中 Key 是一个二元组 (k1, k2)
        self.U: List[Tuple[Tuple[float, float], NodeType]] = []

        # 索引字典: 映射 Node -> 最新合法的 Key
        # 作用: 配合 heapq 实现 Lazy Removal (懒惰删除)。
        # 当从堆中弹出一个节点时，检查它的 Key 是否与字典中的一致，不一致说明是过期的旧数据，直接丢弃。
        self.open_keys: Dict[NodeType, Tuple[float, float]] = {}

        # km (Key Modifier): 关键修饰值
        # 当机器人移动（起点变化）时， heuristic 会发生变化。
        # 为了不重算全图 Key，我们引入 km 来补偿 heuristic 的差值。
        self.km = 0.0

        # 状态记录
        self.start_node: Optional[NodeType] = None  # 当前机器人的位置
        self.goal_node: Optional[NodeType] = None  # 目标位置
        self.last_start_node: Optional[NodeType] = None  # 上一次计算时的机器人位置

        # 动态计算最大迭代次数，防止死循环
        # 估算体素总数
        total_voxels = max(1000, self.rows * self.cols * max(1, self.layers))
        self.max_iter_limit = total_voxels * 20

        # [优化] 动态计算不可达代价阈值 (De-hardcoding 1,000,000)
        # 逻辑: 即使是全图最长路径 (遍历所有格子)，其代价也不应超过 total_voxels * max_step_cost
        # 我们乘以一个安全系数 (如 10.0) 来作为判定"不可达"的软上限
        # 这比硬编码的 1,000,000 更适应不同尺寸的地图
        max_step_cost = 1.74  # sqrt(3) 约等于 1.732
        self.cost_threshold = float(total_voxels) * max_step_cost * 10.0
        # 确保阈值不低于基础值，防止微型地图误判
        self.cost_threshold = max(100000.0, self.cost_threshold)

        # 预计算移动代价，避免重复开方
        self.COST_1 = 1.0
        self.COST_2 = 1.41421356
        self.COST_3 = 1.73205081

        # 设定动态阈值
        self.max_nodes_expanded = max(5000, total_voxels * 5)

        self.logger.info(
            f"[D* Config] 熔断阈值: {self.max_nodes_expanded} nodes, "
            f"代价阈值: {self.cost_threshold:.1f} (地图体素: {total_voxels})"
        )

        self.logger.info(
            f"[D* Lite Init] 初始化完成. W={self.heuristic_weight:.2f}, "
            f"MaxIter={self.max_iter_limit}, 模式={'Octile' if use_octile_3d else 'Euclidean'}"
        )

    def _reset_stats(self):
        """重置单次规划的统计数据"""
        self.stats["nodes_expanded"] = 0
        self.stats["replanning_count"] = 0

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """
        初始化规划任务。
        优化：如果起点/终点与上次一致，则保留搜索树，仅更新起点位置，实现增量规划。
        """
        self._reset_stats()

        # 1. 基础校验
        if start is None or goal is None:
            self.logger.warning("[D* Lite Init] 失败: 起点或终点为 None。")
            return False

        if not self.is_valid(start):
            self.logger.error(f"[D* Lite Init] 失败: 起点 {start} 越界。")
            return False
        if not self.is_valid(goal):
            self.logger.error(f"[D* Lite Init] 失败: 终点 {goal} 越界。")
            return False
        if self.is_obstacle(goal):
            self.logger.warning(f"[D* Lite Init] 警告: 终点 {goal} 位于障碍物内。")
            return False

        # 2. 【核心修改】检查是否可以复用状态
        # 如果目标点没变，且 g 表不为空，说明是基于旧环境的增量更新
        if self.goal_node == goal and self.g:
            # 仅更新起点 (机器人移动了，或者只是障碍物变了但起点没变)
            if self.start_node != start:
                # 机器人移动导致的 heuristic 变化由 km 处理，这里只需更新记录
                self.km += self._heuristic_inline(self.last_start_node, start)
                self.last_start_node = start
                self.start_node = start

            self.logger.info(f"[D* Lite] 增量模式激活: 复用搜索树，Km={self.km:.2f}")
            return True

        # 3. 如果是全新的任务 (终点变了)，则执行全量重置 (保持原有逻辑)
        self.start_node = start
        self.goal_node = goal
        self.last_start_node = start
        self.km = 0.0

        # 清空数据结构
        self.U = []
        self.open_keys.clear()  # [优化] 清空索引
        self.g.clear()
        self.rhs.clear()

        # 3. 设定终点初始状态 (反向搜索)
        # 终点到终点的代价为 0
        self.rhs[goal] = 0.0

        # 将终点加入优先队列，作为搜索的种子
        # 计算 Key 时使用初始起点作为 heuristic 的目标
        self._insert_to_open(goal, self._calculate_key(goal))

        self.logger.info(f"[D* Lite] 全量初始化: {start} -> {goal} (反向搜索模式)")

        if not self._compute_shortest_path():
            self.logger.warning("[D* Lite Init] 初次计算未找到路径。")
            return False

        return True

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """
        [步骤 2] 处理环境变化 (增量更新)。
        当 MapManager 发现障碍物变化时调用此函数。

        Args:
            changes: 变化列表，格式如 [(x, y, 1), (x, y, 0)...]
        """
        if not self.goal_node or not self.start_node:
            self.logger.warning("[D* Update] 未初始化起终点，跳过更新。")
            return
        if not changes:
            return

        # [DEBUG 1] 打印变更详情
        self.logger.debug(
            f"[D* DEBUG] update_obstacles 触发. 变更点数量: {len(changes)}"
        )
        if len(changes) > 0:
            first = changes[0]
            u = tuple(first[:-1])
            is_obs = self.is_obstacle(u)
            self.logger.debug(
                f"[D* DEBUG] 首个变更点: {u}, IsObstacleNow={is_obs}, RawChange={first}"
            )

        self.logger.debug(
            f"[D* Update] 收到 {len(changes)} 个环境变化，开始增量更新..."
        )

        # 遍历所有变化的节点
        for change in changes:
            # 提取坐标 (移除最后一位的值，只取坐标部分)
            coords = change[:-1]
            u: NodeType = tuple(coords)  # type: ignore

            if not self.is_valid(u):
                continue

            # 障碍物状态改变了，u 的代价估计(rhs)可能发生变化
            self._update_vertex(u)

            # u 的邻居通过 u 到达终点的代价也可能变化，因此也要更新邻居
            for neighbor, cost in self._get_neighbors(u):
                self._update_vertex(neighbor)

        # 记录重规划次数
        self.stats["replanning_count"] = self.stats.get("replanning_count", 0) + 1

        # 触发增量修复
        self._compute_shortest_path()

    def _insert_to_open(self, u: NodeType, key: Tuple[float, float]):
        """
        [优化] 封装插入逻辑，同时更新 heap 和 index dict。
        """
        # 更新字典索引，标记该节点最新的 Key
        self.open_keys[u] = key
        # 推入堆中
        heapq.heappush(self.U, (key, u))

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """
        [步骤 3] 获取当前最优路径 (路径查询)。
        D* Lite 维护的是从各个网格到终点的梯度场 (g值)。
        这里需要通过梯度下降 (Gradient Descent) 实时生成路径。

        Args:
            current_pos: 机器人当前实际位置。
        """
        if not self.goal_node:
            return None

        # 1. 检测机器人是否移动了 (起点变化)
        if current_pos != self.last_start_node:
            # 机器人移动了， heuristic 发生变化，累加 km
            self.km += self._heuristic_inline(self.last_start_node, current_pos)
            self.last_start_node = current_pos
            self.logger.debug(
                f"[D* Move] 机器人位置更新: {current_pos}, km 更新为 {self.km:.2f}"
            )

        self.start_node = current_pos

        # 2. 再次确保路径是最新的
        # 如果机器人偏离了原定路线，或者世界发生了变化，这里会进行必要的修复
        # 尝试增量计算，并包含自动降级机制
        # 如果增量修复失败（比如超过最大迭代次数），则回退到全量重算
        if not self._compute_shortest_path():
            self.logger.warning(
                "[D* Lite] 增量修复耗时过长或失败，触发自动全量重规划 (Auto-Fallback)..."
            )

            # --- Fallback: 全量重置 ---
            # 1. 清空所有搜索树状态
            self.U = []
            self.open_keys.clear()
            self.g.clear()
            self.rhs.clear()
            self.km = 0.0

            # 2. 重新初始化终点 (反向搜索)
            self.rhs[self.goal_node] = 0.0
            self._insert_to_open(self.goal_node, self._calculate_key(self.goal_node))

            # 3. 再次尝试计算 (此时等价于 A*)
            if not self._compute_shortest_path():
                self.logger.error("[D* Lite] 全量重规划失败，确实无解。")
                return None
            else:
                self.logger.info("[D* Lite] 全量重规划成功挽救了路径。")

        # 3. 检查起点是否可达
        if self.g.get(current_pos, INF) == INF:
            self.logger.warning("[D* Path] 起点 g 值为 INF，路径不可达。")
            return None

        # 4. 梯度下降生成路径 (Greedy Descent)
        path = [current_pos]
        curr = current_pos
        MAX_STEPS = self.max_nodes_expanded  # 防止死循环

        # [DEBUG Fix] 1. 添加访问记录集合，用于检测死循环
        visited_set = {curr}

        while curr != self.goal_node and len(path) < MAX_STEPS:
            min_cost = INF
            best_next = None

            # [DEBUG Logger] 2. 仅在路径长度较短时打印，避免刷屏
            debug_candidates = []

            # 遍历邻居，找 g 值最小的那个 (g值代表该邻居到终点的代价)
            for neighbor, move_cost in self._get_neighbors(curr):
                if self.is_obstacle(neighbor):
                    continue

                # 预期总代价 = 移动到邻居的代价 + 邻居到终点的 g 值
                g_neighbor = self.g.get(neighbor, INF)
                c = move_cost + g_neighbor

                # [DEBUG Fix] 收集邻居信息用于调试
                if len(path) < 500:
                    debug_candidates.append(f"{neighbor}:g={g_neighbor:.2f},c={c:.2f}")

                if c < min_cost:
                    min_cost = c
                    best_next = neighbor

            # [DEBUG Logger] 打印当前的决策情况
            if len(path) < 500:
                self.logger.debug(
                    f"[PathTrace] Step {len(path)}: {curr} (g={self.g.get(curr, INF):.2f}) -> 选择 {best_next} (cost={min_cost:.2f}). 候选项: {debug_candidates}"
                )

            if best_next:
                # [DEBUG Fix] 3. 关键修复: 回环检测
                if best_next in visited_set:
                    self.logger.error(
                        f"[D* Path Error] 检测到死循环! 节点 {best_next} 已在路径中。"
                        f"当前节点: {curr}, 目标g: {self.g.get(best_next, INF)}。"
                        "这通常意味着 g 值梯度场存在局部极小值陷阱 (Local Minimum Trap)。"
                    )
                    # 遇到死循环直接认为无解，不要返回错误路径给优化器去"穿模"
                    return None

                path.append(best_next)
                visited_set.add(best_next)
                curr = best_next
            else:
                self.logger.error(
                    f"[D* Path] 在 {curr} 处陷入局部死胡同 (无可行邻居)。"
                )
                break

        if len(path) >= MAX_STEPS:
            self.logger.error("[D* Path] 路径生成超过最大步数，可能存在环路。")

        return path

    def _calculate_key(self, u: NodeType) -> Tuple[float, float]:
        """
        计算节点的排序键值 (Key)。
        Key = [k1, k2]
        k1 = min(g, rhs) + h + km  (估计总代价)
        k2 = min(g, rhs)           (已知最小代价)
        通常优先比较 k1 (总代价)，如果 k1 相等，比较 k2 (越靠近终点越优先)。
        """
        g_val = self.g.get(u, INF)
        rhs_val = self.rhs.get(u, INF)

        # 取 g 和 rhs 中的较小值作为基础代价
        min_val = g_val if g_val < rhs_val else rhs_val

        if min_val == INF:
            return (INF, INF)

        # 加上 heuristic 和 km
        k1 = min_val + self._heuristic_inline(self.start_node, u) + self.km
        return (k1, min_val)

    def _update_vertex(self, u: NodeType):
        """
        [核心] 节点状态更新函数。
        检查节点是否一致 (Consistent)，如果不一致则更新其在优先队列中的状态。
        """

        # =========================================================================
        # 如果当前节点 u 是障碍物，它到终点的代价(rhs)必须是无穷大。
        # 否则节点会变成“幽灵”，导致增量计算无法感知路断了。
        # =========================================================================
        if self.is_obstacle(u):
            self.rhs[u] = INF
        # 1. 如果不是终点，根据邻居重新计算 rhs 值
        # rhs(u) = min(g(s) + c(u,s)) for all s in neighbors
        elif u != self.goal_node:
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

        # [DEBUG 2] 监控关键节点的状态变化 (可选: 仅针对前几个变更点或特定坐标打印)
        # if u == (特定坐标):
        # self.logger.debug(
        #     f"[D* DEBUG] Update {u}: g={g_val}, rhs={rhs_val}, IsObs={self.is_obstacle(u)}"
        # )

        # 2. 检查一致性并操作优先队列
        # 如果节点变得一致 (Consistent)，从 Open Set 逻辑移除
        if abs(g_val - rhs_val) <= EPSILON:
            if u in self.open_keys:
                del self.open_keys[u]
                # 注意：我们只删除了索引，堆中的旧数据通过 Lazy Removal 清理
        else:
            # 否则更新 Key 并重新插入/更新
            # 注意：这里我们 push 了一个新副本，旧副本还在 heap 里
            # 但旧副本的 key 与 self.open_keys[u] 不一致，会被 _compute_shortest_path 过滤
            self._insert_to_open(u, self._calculate_key(u))

    def _compute_shortest_path(self) -> bool:
        """
        [D* Lite 核心循环]
        持续处理优先队列，直到起点的一致性得到满足，或者证明无解。

        算法原理解读:
        D* Lite 维护一个优先队列 (Open List)，里面存放的是"不一致"的节点 (g != rhs)。
        - g: 我们当前认为的到终点的代价。
        - rhs: 根据邻居推算出的到终点的理论最小代价。

        循环不仅要处理堆里的节点，还要时刻关注"起点"的状态。
        只要堆里还有比"起点当前Key"更小的节点，或者起点本身就是"不一致"的，
        我们就必须继续传播代价波浪，修补地图变化带来的影响。
        """
        if not self.start_node:
            return False

        heappush = heapq.heappush
        heappop = heapq.heappop
        calc_key = self._calculate_key
        get_g = self.g.get
        get_rhs = self.rhs.get

        # [DEBUG 3] 初始化单次搜索的访问计数器
        visit_counts = {}
        # 限制循环次数，防止死锁 (基于配置的限制)
        max_iter = self.max_iter_limit * 5

        # 性能统计
        loops = 0  # 总循环次数 (含无效操作)
        valid_expansions = 0  # 有效扩展次数 (真实计算量)
        current_expansion = 0

        # 循环条件: 只要堆不为空，我们就持续尝试修复路径
        while self.U:
            loops += 1

            # --- 1. 安全熔断 (Safety Cutoff) ---
            # 使用 valid_expansions 作为熔断依据，忽略 lazy removal 带来的计数虚高
            if valid_expansions > self.max_nodes_expanded:
                efficiency = (valid_expansions / loops * 100) if loops > 0 else 0
                self.logger.error(
                    f"[D* Loop] 超过有效扩展上限 ({self.max_nodes_expanded})，强制中断。\n"
                    f"    - 堆操作总数 (Heap Pops): {loops}\n"
                    f"    - 有效扩展数 (Valid Expansions): {valid_expansions}\n"
                    f"    - 操作效率 (Efficiency): {efficiency:.1f}%"
                )
                self.stats["nodes_expanded"] = (
                    self.stats.get("nodes_expanded", 0) + current_expansion
                )
                return False

            # 获取堆顶 Key 和 节点 (但不立即弹出，因为要先做判断)
            k_old, u = self.U[0]

            # --- 2. 懒惰删除检查 (Lazy Removal) ---
            # 堆中的节点可能是旧状态的残留。我们需要检查它是否有效。
            # 如果 self.open_keys[u] 记录的新 Key 不等于堆里的 k_old，说明这个节点已经更新过了，
            # 堆里这个是旧的垃圾数据，直接丢弃。
            if u not in self.open_keys or self.open_keys[u] != k_old:
                heappop(self.U)
                continue

            # [DEBUG 3] 记录并检查单点重访率 (检测死循环或震荡)
            visit_counts[u] = visit_counts.get(u, 0) + 1
            if visit_counts[u] == 10:  # 阈值设为10，一旦超过说明有异常震荡
                g_u_debug = get_g(u, INF)
                rhs_u_debug = get_rhs(u, INF)
                self.logger.warning(
                    f"[D* DEBUG] 警告: 节点 {u} 在单次搜索中已扩展 10 次! "
                    f"g={g_u_debug}, rhs={rhs_u_debug}, key={k_old}, IsObs={self.is_obstacle(u)}"
                )

            # --- 3. 终结条件检查 (Termination Condition) ---
            # [Fix Logic] 这里采用了更严格的退出判定
            # 我们需要检查是否已经找到了通往起点的最优路径。
            start_g = get_g(self.start_node, INF)
            start_rhs = get_rhs(self.start_node, INF)

            # 检查起点的一致性 (g == rhs)
            is_start_consistent = abs(start_g - start_rhs) < EPSILON

            if is_start_consistent:
                k_start = calc_key(self.start_node)

                # [核心逻辑修复] 从 >= 改为 >
                # 只有当堆顶元素的 Key *严格大于* 起点 Key 时，才安全停止。
                # 这保证了所有与 Start 代价相同的波前（Wavefront）都被处理完毕。
                should_terminate = False

                if k_old > k_start:
                    should_terminate = True
                # 针对浮点数精度的额外保护：如果 f-score 确实大出 EPSILON，也停止
                elif k_old[0] > k_start[0] + EPSILON:
                    should_terminate = True

                if should_terminate:
                    self.stats["nodes_expanded"] = (
                        self.stats.get("nodes_expanded", 0) + current_expansion
                    )
                    efficiency = (valid_expansions / loops * 100) if loops > 0 else 0
                    self.logger.debug(
                        f"[D* Loop] 搜索完成。\n"
                        f"    - 有效扩展: {current_expansion} (累计本次: {valid_expansions})\n"
                        f"    - 堆操作数: {loops}\n"
                        f"    - 堆脏数据率: {100 - efficiency:.1f}% (效率: {efficiency:.1f}%)"
                    )
                    return True

            # [启发式剪枝] 检查孤立点
            # 如果起点的 g 和 rhs 都是 INF (不可达)，并且堆顶元素的代价已经非常巨大
            # 说明我们已经搜索了很远很远，但依然没能连通起点。
            if start_g == INF and start_rhs == INF:
                if k_old[0] > self.cost_threshold:
                    heappop(self.U)
                    if u in self.open_keys:
                        del self.open_keys[u]
                    continue

            # --- 4. 节点扩展 (Expansion) ---
            # 如果还没结束，正式弹出堆顶节点进行处理
            heappop(self.U)
            # 既然已经处理，暂时从索引移除 (update_vertex 会根据情况决定是否加回)
            # 此时我们认定这是一个有效扩展
            valid_expansions += 1
            current_expansion += 1

            k_new = calc_key(u)

            # 情况 A: 节点的 Key 已经过时 (例如 km 发生变化导致 heuristic 变化)
            # 这时我们需要用新的 Key 把它重新塞回堆里
            if k_old < k_new:
                self._insert_to_open(u, k_new)
                continue

            g_u = get_g(u, INF)
            rhs_u = get_rhs(u, INF)

            # 情况 B: 过一致 (Overconsistent, g > rhs)
            # 这通常发生在发现了一条更短的路径时 (例如障碍物移除)。
            if g_u > rhs_u:
                # 显式移除 open_keys 记录 (如果存在)
                if u in self.open_keys:
                    del self.open_keys[u]

                self.g[u] = rhs_u
                new_g = rhs_u

                # 传播这个好消息给邻居
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue

                    new_rhs_s = new_g + cost
                    # 如果经由 u 到达 s 的代价比 s 原有的 rhs 更小，更新 s
                    if new_rhs_s < get_rhs(s, INF):
                        self.rhs[s] = new_rhs_s
                        self._update_vertex(s)

            # 情况 C: 欠一致 (Underconsistent, g < rhs)
            # 这通常发生在路径被阻断时 (例如障碍物添加)。
            # 该节点的代价变大了，我们需要先把 g 设为 INF (强制重置)
            else:
                self.g[u] = INF  # 先设为无穷大，强制重算
                self._update_vertex(u)  # 把自己加回去重新评估

                # 通知邻居：我这里路断了，你们要重新找路
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue

                    # [Fix Logic] 仅当邻居 s 原本的 rhs 是通过 (u + cost) 得到时，才需要更新 s
                    # 这里的判断使用 EPSILON 容差处理浮点数对比
                    # 如果 s 的 rhs 等于 old_g_u + cost，说明 s 依赖于 u，现在 u 变了，s 也得变
                    if abs(get_rhs(s, INF) - (g_u + cost)) < EPSILON:
                        self._update_vertex(s)

        # 堆空了还没找到路径，说明无解
        efficiency = (valid_expansions / loops * 100) if loops > 0 else 0
        self.stats["nodes_expanded"] = (
            self.stats.get("nodes_expanded", 0) + current_expansion
        )
        self.logger.debug(
            f"[D* Loop] 堆空退出 (无更多节点可扩展)。\n"
            f"    - 有效扩展: {valid_expansions}\n"
            f"    - 总堆操作: {loops}\n"
            f"    - 效率: {efficiency:.1f}%"
        )
        return False

    def _heuristic_inline(self, a: Optional[NodeType], b: Optional[NodeType]) -> float:
        """
        内联启发式函数计算。
        为了性能，直接在这里实现，减少函数调用开销。
        """
        if not a or not b:
            return 0.0
        h = 0.0

        # 3D 启发式
        if len(a) == 3:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            dz = abs(a[2] - b[2])
            if self.use_octile_3d:
                # Octile 3D: max_delta + (sqrt2-1)*mid_delta + (sqrt3-sqrt2)*min_delta
                # 排序成本较低，适合 3D
                delta = sorted([dx, dy, dz])
                h = (
                    delta[0] * self.COST_3
                    + (delta[1] - delta[0]) * self.COST_2
                    + (delta[2] - delta[1]) * self.COST_1
                )
            else:
                # Euclidean
                h = math.sqrt(dx * dx + dy * dy + dz * dz)
        # 2D 启发式
        else:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            # Octile 2D: max(dx, dy) + (sqrt2 - 1) * min(dx, dy)
            # 等价于 (dx + dy) + (sqrt2 - 2) * min(dx, dy)
            h = (dx + dy) + (self.COST_2 - 2) * min(dx, dy)

        return h * self.heuristic_weight

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
        """
        获取邻居节点。
        包含了：越界检查、障碍物检查、切角检查。
        """
        res = []
        if len(node) == 3:
            x, y, z = node  # type: ignore
            # 3D 26-Connectivity
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue

                        nx, ny, nz = x + dx, y + dy, z + dz

                        # 1. 越界检查 (Inline is_valid for speed)
                        if not (
                            0 <= nx < self.rows
                            and 0 <= ny < self.cols
                            and 0 <= nz < self.layers
                        ):
                            continue

                        # 2. 障碍物检查 (Inline is_obstacle for speed)
                        # self.grid[nx][ny][nz] lookup is fast
                        if self.grid[nx][ny][nz] == 1:
                            continue

                        # 3. 切角检查 (Strict 3D)
                        is_corner_hit = False
                        if (
                            dx != 0
                            and dy != 0
                            and (
                                self.grid[x + dx][y][z] == 1
                                or self.grid[x][y + dy][z] == 1
                            )
                        ):
                            is_corner_hit = True
                        elif (
                            dx != 0
                            and dz != 0
                            and (
                                self.grid[x + dx][y][z] == 1
                                or self.grid[x][y][z + dz] == 1
                            )
                        ):
                            is_corner_hit = True
                        elif (
                            dy != 0
                            and dz != 0
                            and (
                                self.grid[x][y + dy][z] == 1
                                or self.grid[x][y][z + dz] == 1
                            )
                        ):
                            is_corner_hit = True

                        if is_corner_hit:
                            continue

                        # 计算代价
                        dist_sq = dx * dx + dy * dy + dz * dz
                        # 查表法代替 if-else
                        cost = (
                            self.COST_1
                            if dist_sq == 1
                            else (self.COST_2 if dist_sq == 2 else self.COST_3)
                        )
                        res.append(((nx, ny, nz), cost))  # type: ignore
        else:
            r, c = node  # type: ignore
            # 2D 8-Connectivity
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

                # 1. 越界检查
                if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                    continue

                # 2. 障碍物检查
                if self.grid[nr][nc] == 1:  # Inline obstacle check
                    continue

                # 3. 切角检查
                if dr != 0 and dc != 0:
                    if self.grid[r + dr][c] == 1 or self.grid[r][c + dc] == 1:
                        continue

                res.append(((nr, nc), cost))  # type: ignore
        return res
