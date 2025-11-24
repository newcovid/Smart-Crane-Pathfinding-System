import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Set, Union, Any

# 导入基类和类型定义
from .base import PathPlannerBase, NodeType

# 浮点数比较容差 (用于判断两个浮点数是否相等)
EPSILON = 1e-5
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

        # 预计算移动代价，避免重复开方
        self.COST_1 = 1.0
        self.COST_2 = 1.41421356
        self.COST_3 = 1.73205081

        # 计算总格点数 (Total Voxels)
        total_grid_size = self.rows * self.cols * self.layers

        # 设定动态阈值
        # 逻辑：至少允许遍历全图 5 次，且最低不少于 5000 次 (防止微型地图瞬间报错)
        self.max_nodes_expanded = max(5000, total_grid_size * 5)

        # 打印日志告知当前的熔断限制
        self.logger.info(
            f"[D* Config] 动态熔断阈值已设定: {self.max_nodes_expanded} (地图大小: {total_grid_size})"
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
        [步骤 1] 初始化规划任务 (全量重置)。
        通常只在第一次任务开始时调用。
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

        # 2. 强制重置所有状态
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

        self.logger.info(f"[D* Lite Init] 任务重置: {start} -> {goal} (反向搜索模式)")

        # 4. 初次计算路径
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
        self._compute_shortest_path()

        # 3. 检查起点是否可达
        if self.g.get(current_pos, INF) == INF:
            self.logger.warning("[D* Path] 起点 g 值为 INF，路径不可达。")
            return None

        # 4. 梯度下降生成路径 (Greedy Descent)
        path = [current_pos]
        curr = current_pos
        MAX_STEPS = self.max_nodes_expanded  # 防止死循环

        while curr != self.goal_node and len(path) < MAX_STEPS:
            min_cost = INF
            best_next = None

            # 遍历邻居，找 g 值最小的那个 (g值代表该邻居到终点的代价)
            for neighbor, move_cost in self._get_neighbors(curr):
                if self.is_obstacle(neighbor):
                    continue

                # 预期总代价 = 移动到邻居的代价 + 邻居到终点的 g 值
                c = move_cost + self.g.get(neighbor, INF)
                if c < min_cost:
                    min_cost = c
                    best_next = neighbor

            if best_next:
                path.append(best_next)
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
        # 1. 如果不是终点，根据邻居重新计算 rhs 值
        # rhs(u) = min(g(s) + c(u,s)) for all s in neighbors
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
        [核心循环] 持续处理优先队列，直到起点达到一致状态。
        """
        if not self.start_node:
            return False

        heappush = heapq.heappush
        heappop = heapq.heappop
        calc_key = self._calculate_key
        get_g = self.g.get
        get_rhs = self.rhs.get

        max_iter = self.max_iter_limit
        iters = 0
        current_expansion = 0

        # 循环条件:
        # 1. 堆不为空
        # 2. 堆顶元素的 Key 小于起点的 Key (说明还有比当前起点路径更有潜力的节点需要处理)
        # 3. 或者 起点的 rhs != g (起点本身状态不一致，需要修复)
        while self.U:
            iters += 1
            if iters > max_iter:
                self.logger.error(
                    f"[D* Loop] 超过动态上限 ({max_iter} nodes)，强制中断防死锁。"
                )
                self.stats["nodes_expanded"] = (
                    self.stats.get("nodes_expanded", 0) + current_expansion
                )
                return False

            # 取出堆顶 (Key 最小的节点)
            k_old, u = self.U[0]

            # [优化] 核心过滤逻辑 (Lazy Removal):
            # 如果 u 不在 open_keys 中，或者堆顶的 key 不是最新的 key
            # 说明这是一个过期节点 (Stale Node)，直接丢弃，不消耗计算资源
            if u not in self.open_keys or self.open_keys[u] != k_old:
                heappop(self.U)
                continue

            # 再次检查起点一致性 (退出条件的另一半)
            start_g = get_g(self.start_node, INF)
            start_rhs = get_rhs(self.start_node, INF)

            # 检查起点是否被孤立 (g和rhs都是INF)
            if start_g == INF and start_rhs == INF:
                # 如果堆顶代价已经非常大，说明可能无解，提前剪枝
                if k_old[0] > 1000000:
                    heappop(self.U)
                    if u in self.open_keys:
                        del self.open_keys[u]
                    continue

            # 如果起点已经一致，且当前的堆顶 Key 已经不小于起点的 Key
            # 说明剩下的节点只会导致更大的代价，无需继续搜索
            if abs(start_g - start_rhs) < EPSILON:
                k_start = calc_key(self.start_node)
                if k_old >= k_start:
                    self.stats["nodes_expanded"] = (
                        self.stats.get("nodes_expanded", 0) + current_expansion
                    )
                    self.logger.debug(
                        f"[D* Loop] 搜索完成。Expanded: {current_expansion}"
                    )
                    return True

            # --- 正式扩展节点 ---
            # 真正的弹出操作
            heappop(self.U)
            # 既然已经处理，暂时从索引移除
            # 如果后续处理发现它仍不一致，会再次加入
            # 如果不移除，u 会一直保留在 open_keys 中直到 g==rhs
            # 但这里我们采用 update_vertex 负责管理的策略，不需要手动 del，update_vertex 会覆盖

            current_expansion += 1
            k_new = calc_key(u)

            # 情况 1: 节点的 Key 已经过时 (比如 km 变了)，更新 Key 后重新塞回去
            if k_old < k_new:
                self._insert_to_open(u, k_new)
                continue

            g_u = get_g(u, INF)
            rhs_u = get_rhs(u, INF)

            # 情况 2: Overconsistent (g > rhs)
            # 说明找到了更短的路径，通常是因为某个障碍物移除了
            if g_u > rhs_u:
                self.g[u] = rhs_u  # 更新 g 值
                new_g = rhs_u
                # 传播这个好消息给邻居
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue

                    new_rhs_s = new_g + cost
                    curr_rhs_s = get_rhs(s, INF)
                    if new_rhs_s < curr_rhs_s:
                        self.rhs[s] = new_rhs_s
                        self._update_vertex(s)  # 使用 update_vertex 统一管理插入

            # 情况 3: Underconsistent (g < rhs)
            # 说明原路径被阻断了 (障碍物增加)，该节点的代价变大了
            else:
                self.g[u] = INF  # 先设为无穷大，强制重算
                self._update_vertex(u)  # 把自己加回去重新评估
                # 通知邻居：我这里路断了，你们要重新找路
                for s, cost in self._get_neighbors(u):
                    if self.is_obstacle(s):
                        continue
                    self._update_vertex(s)

        self.stats["nodes_expanded"] = (
            self.stats.get("nodes_expanded", 0) + current_expansion
        )
        self.logger.debug(f"[D* Loop] 堆空退出。Expanded: {current_expansion}")
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

        为了性能，这里依然保持返回 List，但在内部做了内联检查 (Inline Check)。
        包含了：
        1. 越界检查
        2. 障碍物检查
        3. 切角检查 (Strict Corner Check)
        """
        # 为了极致性能，这里不再生成复杂的列表对象
        # 而是直接 yield 或者在一个固定流程里返回
        # 考虑到 Python yield 也有开销，且 PathPlannerBase 接口定义返回 List，保持 List

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
