use crate::common::{parse_python_grid, FlatGrid, Node};
// 引入公共常量
use crate::common::constants::{
    DSLITE_COST_THRESHOLD_MULTIPLIER, DSLITE_DEFAULT_MAX_NODES, DSLITE_MAX_NODES_MULTIPLIER,
};
use crate::common::{COST_1, COST_2, COST_3, EPSILON, FLOAT_TOLERANCE, INF};
use ordered_float::NotNan;
use pyo3::prelude::*;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};
// 引入日志宏
use log::{debug, info, warn};
use std::time::Instant;

/// D* Lite 的优先队列键值 (Keys)。
///
/// 包含两个分量: k1 = min(g, rhs) + h + km, k2 = min(g, rhs)。
/// D* Lite 使用字典序比较这两个键值。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DKey(NotNan<f32>, NotNan<f32>);

impl Ord for DKey {
    fn cmp(&self, other: &Self) -> Ordering {
        // 先比较 k1 (综合优先级)，如果相等则比较 k2 (距离目标的实际代价)
        match self.0.cmp(&other.0) {
            Ordering::Equal => self.1.cmp(&other.1),
            ord => ord,
        }
    }
}

impl PartialOrd for DKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// 优先队列中的元素。
///
/// 注意：BinaryHeap 是最大堆，而我们需要最小 Key 优先。
/// 因此在 Ord 实现中反转了 Key 的比较顺序。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PriorityItem {
    key: DKey,
    node: Node,
}

impl Ord for PriorityItem {
    fn cmp(&self, other: &Self) -> Ordering {
        // 反转 Key 的比较顺序，实现最小堆效果
        match other.key.cmp(&self.key) {
            Ordering::Equal => other.node.cmp(&self.node), // 节点坐标作为 Tie-breaker
            ord => ord,
        }
    }
}

impl PartialOrd for PriorityItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Rust 实现的 D* Lite 路径规划器。
///
/// D* Lite 是一种增量式启发式搜索算法，适用于动态环境。
/// 它反向搜索（从目标到起点），并在起点移动或障碍物变化时利用已知信息快速重规划。
#[pyclass]
pub struct RustDLitePlanner {
    grid: FlatGrid,
    use_octile_3d: bool,
    _heuristic_weight: f32,

    // --- D* Lite 核心状态 ---
    /// g(u): 从 u 到目标节点的当前已知最短路径代价。
    g: HashMap<Node, f32>,
    /// rhs(u): 基于 u 的后继节点（即靠近目标的邻居）计算出的一步前瞻代价。
    /// 如果 g(u) != rhs(u)，则该节点是“局部不一致”的，需要更新。
    rhs: HashMap<Node, f32>,
    /// 优先队列 U，存储不一致的节点。
    u_queue: BinaryHeap<PriorityItem>,
    /// 辅助映射，记录节点当前在队列中的 Key，用于去重和懒惰删除检查。
    open_keys: HashMap<Node, (f32, f32)>,
    /// Key Modifier (km): 累积的启发式偏移量。
    /// 当机器人（起点）移动时，为了避免更新整个堆中所有节点的 H 值，我们通过增加 km 来隐式调整。
    km: f32,

    // --- 导航状态 ---
    start_node: Option<Node>,
    goal_node: Option<Node>,
    /// 上一次计算时的起点位置，用于计算位移产生的 km 增量。
    last_start_node: Option<Node>,

    /// 搜索限制参数
    max_nodes_expanded: usize,
    _cost_threshold: f32,

    // --- 统计 ---
    nodes_expanded_stat: AtomicUsize,
    replanning_count_stat: AtomicUsize,
}

#[pymethods]
impl RustDLitePlanner {
    #[new]
    #[pyo3(signature = (grid, _width_m, _length_m, _height_m, _resolution, use_octile_3d, _heuristic_weight))]
    pub fn new(
        grid: &Bound<PyAny>,
        _width_m: f32,
        _length_m: f32,
        _height_m: f32,
        _resolution: f32,
        use_octile_3d: bool,
        _heuristic_weight: f32,
    ) -> PyResult<Self> {
        debug!("[Rust D*] 正在初始化...");
        let flat_grid = parse_python_grid(grid)?;
        let total_voxels = (flat_grid.rows * flat_grid.cols * flat_grid.layers.max(1)) as usize;
        let max_nodes = std::cmp::max(
            DSLITE_DEFAULT_MAX_NODES,
            total_voxels * DSLITE_MAX_NODES_MULTIPLIER,
        );
        let cost_thresh = (total_voxels as f32) * DSLITE_COST_THRESHOLD_MULTIPLIER;

        info!(
            "[Rust D*] 初始化完毕. Grid: {}x{}x{}, MaxNodes: {}",
            flat_grid.rows, flat_grid.cols, flat_grid.layers, max_nodes
        );

        Ok(Self {
            grid: flat_grid,
            use_octile_3d,
            // 刻意忽略入参：D* Lite 的正确性依赖启发式的一致性
            // (h(u) <= cost(u,v) + h(v))，加权会破坏该前提。
            // Python 侧 DLitePlanner 同样强制 1.0 并在权重 != 1 时告警，
            // 两个引擎必须保持一致，否则同一配置会产出不同长度的路径。
            _heuristic_weight: 1.0,
            g: HashMap::new(),
            rhs: HashMap::new(),
            u_queue: BinaryHeap::new(),
            open_keys: HashMap::new(),
            km: 0.0,
            start_node: None,
            goal_node: None,
            last_start_node: None,
            max_nodes_expanded: max_nodes,
            _cost_threshold: cost_thresh,
            nodes_expanded_stat: AtomicUsize::new(0),
            replanning_count_stat: AtomicUsize::new(0),
        })
    }

    /// 更新地图数据（全量）。通常在初始化或地图完全改变时调用。
    pub fn update_grid(&mut self, grid: &Bound<PyAny>) -> PyResult<()> {
        debug!("[Rust D*] 网格全量刷新");
        let flat_grid: FlatGrid = parse_python_grid(grid)?;
        self.grid = flat_grid;
        Ok(())
    }

    /// 初始化搜索任务。设置起点终点，并执行首次搜索。
    ///
    /// # Returns
    /// (bool success, usize nodes_expanded_in_this_step)
    pub fn initialize(&mut self, start: (i32, i32, i32), goal: (i32, i32, i32)) -> (bool, usize) {
        let start_nodes_count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);
        let s_node = Node::new(start.0, start.1, start.2);
        let g_node = Node::new(goal.0, goal.1, goal.2);

        // 基础有效性检查
        if !self.grid.is_valid(&s_node) || !self.grid.is_valid(&g_node) {
            warn!("[Rust D*] 起点或终点无效");
            return (false, 0);
        }
        if self.grid.is_obstacle_unsafe(&g_node) {
            warn!("[Rust D*] 终点位于障碍物内");
            return (false, 0);
        }

        // 复用逻辑：如果终点没变，且已有搜索状态
        if Some(g_node) == self.goal_node && !self.g.is_empty() {
            // 如果机器人位置（起点）变了
            if Some(s_node) != self.start_node {
                if let Some(last_start) = self.last_start_node {
                    // 核心逻辑：累加 km，而不是重新计算所有节点的 H
                    let h_diff = self.heuristic(&last_start, &s_node);
                    self.km += h_diff;
                    debug!("[Rust D*] 机器人移动，更新 km += {:.2}", h_diff);
                }
                self.last_start_node = Some(s_node);
                self.start_node = Some(s_node);
            }
            let end_nodes = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);
            return (true, end_nodes - start_nodes_count);
        }

        // 新任务：全量重置
        info!("[Rust D*] 全量重置搜索状态 (New Mission)");
        self.full_reset(s_node, g_node);
        let success = self.compute_shortest_path();

        let end_nodes = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);
        (success, end_nodes - start_nodes_count)
    }

    /// 处理动态障碍物更新。
    ///
    /// # Arguments
    /// * `changes` - 变化点列表 [(r, c, l, val), ...]
    pub fn update_obstacles(&mut self, changes: Vec<(i32, i32, i32, i32)>) -> PyResult<usize> {
        let start_time = Instant::now();
        let start_nodes_count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);

        if self.start_node.is_none() || self.goal_node.is_none() {
            return Ok(0);
        }

        debug!("[Rust D*] 处理 {} 个增量更新...", changes.len());

        for (r, c, l, val) in changes {
            let u = Node::new(r, c, l);
            if !self.grid.is_valid(&u) {
                warn!("[Rust D*] 忽略越界的变更点 ({}, {}, {})", r, c, l);
                continue;
            }

            // 先把新状态写进自己持有的网格，再重算代价。
            // 本规划器在构造时拷贝了网格，不与 Python 侧共享内存；
            // 早期实现把入参里的新状态丢弃（`_val`），只更新 g/rhs，
            // 导致 is_obstacle_unsafe 和梯度回溯一直读的是旧网格，
            // 最终可能返回一条穿过障碍物的路径。
            self.grid.set_occupancy(&u, val == 1);

            // 障碍物状态改变，影响该节点及其邻居的代价值
            self.update_vertex(&u);
            for (neighbor, _) in self.get_neighbors(&u) {
                self.update_vertex(&neighbor);
            }
        }

        self.replanning_count_stat
            .fetch_add(1, AtomicOrdering::Relaxed);
        self.compute_shortest_path();

        let end_nodes = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);
        let expanded = end_nodes - start_nodes_count;
        let duration = start_time.elapsed();

        // 注入 Logger
        debug!(
            "[Rust D*] update_obstacles 耗时: {:.2}ms, 扩展节点: {}",
            duration.as_secs_f64() * 1000.0,
            expanded
        );

        Ok(expanded)
    }

    /// 获取当前路径（通常每帧调用）。
    ///
    /// # Returns
    /// (Option<path>, nodes_expanded, replanning_count)
    pub fn compute_path(
        &mut self,
        current_pos: (i32, i32, i32),
    ) -> (Option<Vec<Py<PyAny>>>, usize, usize) {
        let curr_node = Node::new(current_pos.0, current_pos.1, current_pos.2);
        let start_nodes_count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);

        if self.goal_node.is_none() {
            return (None, 0, 0);
        }
        let goal_node = self.goal_node.unwrap();

        if curr_node == goal_node {
            let is_3d = self.grid.layers > 1;
            return (Some(vec![curr_node.to_tuple(is_3d)]), 0, 0);
        }

        // 机器人移动检测
        if Some(curr_node) != self.last_start_node {
            if let Some(last) = self.last_start_node {
                self.km += self.heuristic(&last, &curr_node);
            }
            self.last_start_node = Some(curr_node);
        }
        self.start_node = Some(curr_node);

        // 尝试增量修复路径
        if !self.compute_shortest_path() {
            warn!("[Rust D*] 增量修复失败，尝试全量重算...");
            self.full_reset(curr_node, goal_node);
            if !self.compute_shortest_path() {
                warn!("[Rust D*] 全量重算仍无解!");
                let count =
                    self.nodes_expanded_stat.load(AtomicOrdering::Relaxed) - start_nodes_count;
                return (None, count, 0);
            }
        }

        if self.get_g(&curr_node) == INF {
            warn!("[Rust D*] 当前点 g=INF，不可达");
            let count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed) - start_nodes_count;
            return (None, count, 0);
        }

        // --- 路径回溯 (Gradient Descent) ---
        // 从当前点开始，总是走向 g 值最小的邻居（minimize: cost(u,v) + g(v)）
        let mut path = Vec::with_capacity(100);
        path.push(curr_node);
        let mut curr = curr_node;
        let mut visited = HashSet::new();
        visited.insert(curr);

        while curr != goal_node && path.len() < self.max_nodes_expanded {
            let mut min_cost = INF;
            let mut best_next = None;

            for (neighbor, move_cost) in self.get_neighbors(&curr) {
                if self.grid.is_obstacle_unsafe(&neighbor) {
                    continue;
                }
                // D* Lite 反向搜索，g(neighbor) 代表 neighbor 到 goal 的代价
                let c = move_cost + self.get_g(&neighbor);
                if c < min_cost - EPSILON {
                    min_cost = c;
                    best_next = Some(neighbor);
                }
            }

            if let Some(next) = best_next {
                if visited.contains(&next) {
                    warn!("[Rust D*] 路径回溯发现环路!");
                    break;
                }
                path.push(next);
                visited.insert(next);
                curr = next;
            } else {
                break; // 陷入局部死胡同
            }
        }

        let count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed) - start_nodes_count;
        let is_3d = self.grid.layers > 1;
        let py_path = path.iter().map(|n| n.to_tuple(is_3d)).collect();

        info!("[Rust D*] 路径生成成功 (Len: {})", path.len());
        (
            Some(py_path),
            count,
            self.replanning_count_stat.load(AtomicOrdering::Relaxed),
        )
    }
}

// --- 内部辅助方法 ---
impl RustDLitePlanner {
    /// 全量重置状态（相当于全新的 Dijkstra/A* 初始化）。
    /// 注意：D* Lite 从 Goal 开始反向搜索。
    fn full_reset(&mut self, start: Node, goal: Node) {
        self.start_node = Some(start);
        self.goal_node = Some(goal);
        self.last_start_node = Some(start);
        self.km = 0.0;
        self.g.clear();
        self.rhs.clear();
        self.u_queue.clear();
        self.open_keys.clear();

        // 终点 rhs = 0，触发首次扩展
        self.rhs.insert(goal, 0.0);
        let k = self.calculate_key(&goal);
        self.insert_to_open(goal, k);
    }

    /// 获取 g 值 (current cost to goal)，默认为无穷大。
    fn get_g(&self, u: &Node) -> f32 {
        *self.g.get(u).unwrap_or(&INF)
    }

    /// 获取 rhs 值 (lookahead cost)，默认为无穷大。
    fn get_rhs(&self, u: &Node) -> f32 {
        *self.rhs.get(u).unwrap_or(&INF)
    }

    /// 启发式函数 H(a, b)。
    fn heuristic(&self, a: &Node, b: &Node) -> f32 {
        let dx = (a.x - b.x).abs() as f32;
        let dy = (a.y - b.y).abs() as f32;
        let dz = (a.z - b.z).abs() as f32;
        if self.grid.layers <= 1 {
            let min_d = dx.min(dy);
            (dx + dy) + (COST_2 - 2.0) * min_d
        } else {
            if self.use_octile_3d {
                let mut arr = [dx, dy, dz];
                arr.sort_by(|a, b| a.partial_cmp(b).unwrap());
                arr[0] * COST_3 + (arr[1] - arr[0]) * COST_2 + (arr[2] - arr[1]) * COST_1
            } else {
                (dx * dx + dy * dy + dz * dz).sqrt()
            }
        }
    }

    /// 计算节点的 Key 值 (k1, k2)。
    /// Key 用于在优先队列中排序。
    /// k1 = min(g, rhs) + h + km
    /// k2 = min(g, rhs)
    fn calculate_key(&self, u: &Node) -> (f32, f32) {
        let g_val = self.get_g(u);
        let rhs_val = self.get_rhs(u);
        let min_val = g_val.min(rhs_val);

        if min_val == INF {
            return (INF, INF);
        }

        // H 是从当前 Start 到 u 的估计距离
        let h = self.heuristic(self.start_node.as_ref().unwrap(), u);
        (min_val + h + self.km, min_val)
    }

    /// 将节点插入优先队列（Open Set）。
    fn insert_to_open(&mut self, u: Node, key: (f32, f32)) {
        self.open_keys.insert(u, key);
        let k = DKey(NotNan::new(key.0).unwrap(), NotNan::new(key.1).unwrap());
        self.u_queue.push(PriorityItem { key: k, node: u });
    }

    /// 更新节点的一致性状态 (Vertex Update)。
    /// 如果 g != rhs，节点被认为是不一致的，需要放入队列更新。
    /// 如果 g == rhs，节点一致，可以从队列移除。
    fn update_vertex(&mut self, u: &Node) {
        if self.goal_node.is_none() {
            return;
        }
        // 除了 Goal 节点外，所有节点的 rhs 基于其邻居计算
        if Some(*u) != self.goal_node {
            let mut min_rhs = INF;
            if self.grid.is_obstacle_unsafe(u) {
                min_rhs = INF;
            } else {
                // rhs(u) = min(cost(u, s) + g(s)) for all s in succ(u)
                for (neighbor, cost) in self.get_neighbors(u) {
                    if self.grid.is_obstacle_unsafe(&neighbor) {
                        continue;
                    }
                    let temp = self.get_g(&neighbor) + cost;
                    if temp < min_rhs {
                        min_rhs = temp;
                    }
                }
            }
            self.rhs.insert(*u, min_rhs);
        }

        let g_val = self.get_g(u);
        let rhs_val = self.get_rhs(u);

        // 检查一致性
        if (g_val - rhs_val).abs() > EPSILON {
            self.insert_to_open(*u, self.calculate_key(u));
        } else {
            self.open_keys.remove(u); // 移除 (Rust BinaryHeap 不支持直接移除，需配合 lazy removal)
        }
    }

    /// D* Lite 核心循环：计算最短路径。
    /// 持续处理优先队列，直到 Start 节点的一致性得到满足，或者堆顶 Key 超过 Start Key。
    fn compute_shortest_path(&mut self) -> bool {
        if self.start_node.is_none() {
            return false;
        }
        let start = self.start_node.unwrap();
        let mut valid_expansions = 0;

        while let Some(item) = self.u_queue.peek() {
            if valid_expansions > self.max_nodes_expanded {
                warn!(
                    "[Rust D*] 扩展节点超过阈值 {}，强制中断",
                    self.max_nodes_expanded
                );
                return false;
            }

            // 获取堆顶 Key (Old Key)
            let k_old_raw = item.key;
            let u = item.node;
            let k_old = (k_old_raw.0.into_inner(), k_old_raw.1.into_inner());

            // 懒惰删除检查 (Lazy Removal):
            // 检查堆中的 Key 是否与 open_keys 中存储的最新 Key 一致。如果不一致，说明是过期的条目。
            if let Some(k_curr) = self.open_keys.get(&u) {
                if (k_curr.0 - k_old.0).abs() > EPSILON || (k_curr.1 - k_old.1).abs() > EPSILON {
                    self.u_queue.pop();
                    continue;
                }
            } else {
                // 已经在 update_vertex 中被逻辑移除了
                self.u_queue.pop();
                continue;
            }

            // 终止条件检查
            let k_start = self.calculate_key(&start);
            let start_g = self.get_g(&start);
            let start_rhs = self.get_rhs(&start);

            // 恢复原逻辑：使用 FLOAT_TOLERANCE 进行严格的不等式判断，防止震荡
            // term1: k_old.k1 > k_start.k1 + tol
            let term1 = k_old.0 > k_start.0 + FLOAT_TOLERANCE;
            // term2: k_old.k1 == k_start.k1 (近似) AND k_old.k2 > k_start.k2 + tol
            let term2 =
                (k_old.0 - k_start.0).abs() < EPSILON && k_old.1 > k_start.1 + FLOAT_TOLERANCE;

            // 终止条件：Start 节点局部一致 (g == rhs) 且 堆顶 Key 严格大于 Start Key (超出容差)
            if (start_g - start_rhs).abs() < EPSILON && (term1 || term2) {
                return true;
            }

            // 优化：如果 Start 不可达 (INF) 且搜索代价过大，提前退出
            if start_g == INF && start_rhs == INF && k_old.0 > self._cost_threshold {
                self.u_queue.pop();
                self.open_keys.remove(&u);
                continue;
            }

            // 弹出堆顶，正式扩展
            self.u_queue.pop();
            valid_expansions += 1;
            self.nodes_expanded_stat
                .fetch_add(1, AtomicOrdering::Relaxed);

            let k_new = self.calculate_key(&u);

            // 情况 1: 节点的 Key 已经过时 (由于 km 变化等)，需要重新插入
            // 这里的比较逻辑：如果 k_old < k_new，说明它在堆里的权重偏小了，需要更新
            let old_smaller_new = if (k_old.0 - k_new.0).abs() < EPSILON {
                k_old.1 < k_new.1 - EPSILON
            } else {
                k_old.0 < k_new.0 - EPSILON
            };

            if old_smaller_new {
                self.insert_to_open(u, k_new);
                continue;
            }

            let g_u = self.get_g(&u);
            let rhs_u = self.get_rhs(&u);

            // 情况 2: Overconsistent (g > rhs) - 通常发生在发现更短路径时
            if g_u > rhs_u {
                self.g.insert(u, rhs_u); // 令 g = rhs
                self.open_keys.remove(&u); // 变得一致了，移出队列
                                           // 传播变化给邻居
                for (s, _) in self.get_neighbors(&u) {
                    self.update_vertex(&s);
                }
            }
            // 情况 3: Underconsistent (g < rhs) - 通常发生在障碍物出现，原路径被阻断
            else {
                self.g.insert(u, INF); // 设为无穷大
                self.update_vertex(&u); // 自身重新入队 (因为 g=INF != rhs)
                                        // 传播变化给邻居
                for (s, _) in self.get_neighbors(&u) {
                    self.update_vertex(&s);
                }
            }
        }
        false
    }

    fn get_neighbors(&self, node: &Node) -> Vec<(Node, f32)> {
        // 与 A* 相同的邻居生成逻辑，此处略去重复注释
        let mut successors = Vec::with_capacity(26);
        let is_2d = self.grid.layers <= 1;
        if is_2d {
            let moves = [
                (0, 1, COST_1),
                (0, -1, COST_1),
                (1, 0, COST_1),
                (-1, 0, COST_1),
                (1, 1, COST_2),
                (1, -1, COST_2),
                (-1, 1, COST_2),
                (-1, -1, COST_2),
            ];
            for &(dr, dc, cost) in &moves {
                let nr = node.x + dr;
                let nc = node.y + dc;
                let next = Node::new(nr, nc, 0);
                if !self.grid.is_safe(&next) {
                    continue;
                }
                if dr != 0 && dc != 0 {
                    let c1 = Node::new(node.x + dr, node.y, 0);
                    let c2 = Node::new(node.x, node.y + dc, 0);
                    if self.grid.is_obstacle_unsafe(&c1) || self.grid.is_obstacle_unsafe(&c2) {
                        continue;
                    }
                }
                successors.push((next, cost));
            }
        } else {
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if dx == 0 && dy == 0 && dz == 0 {
                            continue;
                        }
                        let nx = node.x + dx;
                        let ny = node.y + dy;
                        let nz = node.z + dz;
                        let next = Node::new(nx, ny, nz);
                        if !self.grid.is_safe(&next) {
                            continue;
                        }
                        if dx != 0 && dy != 0 {
                            let c1 = Node::new(node.x + dx, node.y, node.z);
                            let c2 = Node::new(node.x, node.y + dy, node.z);
                            if !self.grid.is_safe(&c1) || !self.grid.is_safe(&c2) {
                                continue;
                            }
                        }
                        let dist_sq = dx * dx + dy * dy + dz * dz;
                        let cost = if dist_sq == 1 {
                            COST_1
                        } else if dist_sq == 2 {
                            COST_2
                        } else {
                            COST_3
                        };
                        successors.push((next, cost));
                    }
                }
            }
        }
        successors
    }
}
