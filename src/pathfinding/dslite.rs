use crate::common::{parse_python_grid, FlatGrid, Node};
use ordered_float::NotNan;
use pyo3::prelude::*;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};

// --- 常量定义 ---
const COST_1: f32 = 1.0;
const COST_2: f32 = 1.41421356; // sqrt(2)
const COST_3: f32 = 1.73205081; // sqrt(3)
const INF: f32 = f32::INFINITY;
const EPSILON: f32 = 1e-4;
const FLOAT_TOLERANCE: f32 = 1e-3;

/// D* Lite 的排序键值 Key(k1, k2)
/// k1 = min(g, rhs) + h + km
/// k2 = min(g, rhs)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DKey(NotNan<f32>, NotNan<f32>);

// 实现反向比较以适配 Rust 的 Max-Heap -> Min-Heap
// Rust 的 BinaryHeap 是最大堆，我们需要最小堆，所以这里反转比较逻辑
impl Ord for DKey {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.0.cmp(&self.0) {
            Ordering::Equal => other.1.cmp(&self.1),
            ord => ord,
        }
    }
}

impl PartialOrd for DKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// 优先队列元素
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PriorityItem {
    key: DKey,
    node: Node,
}

impl Ord for PriorityItem {
    fn cmp(&self, other: &Self) -> Ordering {
        self.key.cmp(&other.key)
    }
}

impl PartialOrd for PriorityItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[pyclass]
pub struct RustDLitePlanner {
    // 地图数据
    grid: FlatGrid,
    use_octile_3d: bool,

    // [Fix] 使用下划线前缀抑制未使用警告，但在逻辑中保留它们以备将来扩展
    _heuristic_weight: f32,

    // D* Lite 核心状态
    g: HashMap<Node, f32>,
    rhs: HashMap<Node, f32>,
    u_queue: BinaryHeap<PriorityItem>,
    // 辅助索引，用于 Lazy Removal (懒惰删除)
    // 记录节点当前在堆中有效的最新 Key，处理堆中过期的条目
    open_keys: HashMap<Node, (f32, f32)>,

    // Key Modifier (km): 累积起点的移动距离
    km: f32,

    // 位置记录
    start_node: Option<Node>,
    goal_node: Option<Node>,
    last_start_node: Option<Node>,

    // 熔断与安全阈值
    max_nodes_expanded: usize,

    // [Fix] 使用下划线前缀抑制未使用警告
    _cost_threshold: f32,

    // 统计指标 (原子操作以支持多线程/GIL释放场景)
    nodes_expanded_stat: AtomicUsize,
    replanning_count_stat: AtomicUsize,
}

#[pymethods]
impl RustDLitePlanner {
    #[new]
    #[pyo3(signature = (grid, _width_m, _length_m, _height_m, _resolution, use_octile_3d, heuristic_weight))]
    pub fn new(
        grid: &Bound<PyAny>,
        _width_m: f32,
        _length_m: f32,
        _height_m: f32,
        _resolution: f32,
        use_octile_3d: bool,
        heuristic_weight: f32,
    ) -> PyResult<Self> {
        let flat_grid = parse_python_grid(grid)?;
        let total_voxels = (flat_grid.rows * flat_grid.cols * flat_grid.layers.max(1)) as usize;

        // 设置合理的动态阈值
        let max_nodes = std::cmp::max(5000, total_voxels * 10);
        let cost_thresh = (total_voxels as f32) * 1.74 * 10.0;

        // D* Lite 在动态更新时要求 heuristic 严格一致 (Consistent)，
        // 加权 heuristic (w > 1) 会破坏三角不等式，导致增量更新死锁或错误。
        // 因此这里我们在内部强制使用 1.0。
        let safe_weight = 1.0;
        if heuristic_weight > 1.0 {
            // 在 Python 端应该已经打过日志了，这里静默修正
        }

        Ok(Self {
            grid: flat_grid,
            use_octile_3d,
            _heuristic_weight: safe_weight,
            g: HashMap::new(),
            rhs: HashMap::new(),
            u_queue: BinaryHeap::new(),
            open_keys: HashMap::new(),
            km: 0.0,
            start_node: None,
            goal_node: None,
            last_start_node: None,
            max_nodes_expanded: max_nodes,
            _cost_threshold: cost_thresh.max(100000.0),
            nodes_expanded_stat: AtomicUsize::new(0),
            replanning_count_stat: AtomicUsize::new(0),
        })
    }

    /// [新增] 同步网格数据
    /// 类似于 A*，当 Python 端发生 grid = new_grid 时调用。
    /// 注意：对于 D* Lite，仅仅替换 grid 是不够的，还需要随后调用 update_obstacles
    /// 来触发受影响区域的 rhs 更新。此方法仅负责底层数据的内存同步。
    pub fn update_grid(&mut self, grid: &Bound<PyAny>) -> PyResult<()> {
        let flat_grid = parse_python_grid(grid)?;
        // 只有当尺寸发生变化或者这是初次加载时可能需要特殊处理
        // 但通常 TrajectoryPlanner 会处理好这些。
        // 我们直接替换 grid。
        self.grid = flat_grid;
        Ok(())
    }

    /// 初始化规划任务
    pub fn initialize(&mut self, start: (i32, i32, i32), goal: (i32, i32, i32)) -> bool {
        let s_node = Node::new(start.0, start.1, start.2);
        let g_node = Node::new(goal.0, goal.1, goal.2);

        // 重置统计
        self.nodes_expanded_stat.store(0, AtomicOrdering::Relaxed);

        if !self.grid.is_valid(&s_node) || !self.grid.is_valid(&g_node) {
            return false;
        }
        if self.grid.is_obstacle_unsafe(&g_node) {
            return false;
        }

        // 增量复用判断:
        // 如果终点没变且 g 表不为空，尝试复用
        if Some(g_node) == self.goal_node && !self.g.is_empty() {
            if Some(s_node) != self.start_node {
                if let Some(last_start) = self.last_start_node {
                    // 更新 km += h(last_start, new_start)
                    self.km += self.heuristic(&last_start, &s_node);
                }
                self.last_start_node = Some(s_node);
                self.start_node = Some(s_node);
            }
            return true;
        }

        // 全量重置
        self.full_reset(s_node, g_node);
        self.compute_shortest_path()
    }

    /// 处理障碍物更新
    /// changes: List[(r, c, l, val), ...]
    pub fn update_obstacles(&mut self, changes: Vec<(i32, i32, i32, i32)>) -> PyResult<()> {
        // 注意：底层 grid 数据应该已经在调用此方法前通过 update_grid 更新了，
        // 或者 Python 端直接修改了引用。这里我们假设 self.grid 已经是最新的。

        if self.start_node.is_none() || self.goal_node.is_none() {
            return Ok(());
        }

        for (r, c, l, _val) in changes {
            let u = Node::new(r, c, l);
            if !self.grid.is_valid(&u) {
                continue;
            }

            // 1. 更新节点 u 自身的状态
            self.update_vertex(&u);

            // 2. 更新 u 的所有邻居
            // 因为 u 的阻挡状态变了，经过 u 的代价也会变
            for (neighbor, _) in self.get_neighbors(&u) {
                self.update_vertex(&neighbor);
            }
        }

        self.replanning_count_stat
            .fetch_add(1, AtomicOrdering::Relaxed);

        // 触发路径修复
        self.compute_shortest_path();
        Ok(())
    }

    /// 主入口：计算路径
    /// 返回: (路径列表, 扩展节点数, 重规划次数)
    pub fn compute_path(
        &mut self,
        current_pos: (i32, i32, i32),
    ) -> (Option<Vec<PyObject>>, usize, usize) {
        let curr_node = Node::new(current_pos.0, current_pos.1, current_pos.2);

        // 记录开始时的节点数，用于计算本次增量
        let start_nodes_count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);

        if self.goal_node.is_none() {
            return (
                None,
                0,
                self.replanning_count_stat.load(AtomicOrdering::Relaxed),
            );
        }
        let goal_node = self.goal_node.unwrap();

        if curr_node == goal_node {
            let is_3d = self.grid.layers > 1;
            return (
                Some(vec![curr_node.to_tuple(is_3d)]),
                0,
                self.replanning_count_stat.load(AtomicOrdering::Relaxed),
            );
        }

        // 1. 位置同步与 Km 更新
        if Some(curr_node) != self.last_start_node {
            if let Some(last) = self.last_start_node {
                self.km += self.heuristic(&last, &curr_node);
            }
            self.last_start_node = Some(curr_node);
        }
        self.start_node = Some(curr_node);

        // 2. 尝试修复路径
        if !self.compute_shortest_path() {
            // 修复失败，尝试全量重置 (Fallback)
            self.full_reset(curr_node, goal_node);
            if !self.compute_shortest_path() {
                // 彻底失败
                let count =
                    self.nodes_expanded_stat.load(AtomicOrdering::Relaxed) - start_nodes_count;
                return (
                    None,
                    count,
                    self.replanning_count_stat.load(AtomicOrdering::Relaxed),
                );
            }
        }

        // 3. 提取路径 (Gradient Descent)
        // 检查起点可达性
        if self.get_g(&curr_node) == INF {
            let count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed) - start_nodes_count;
            return (
                None,
                count,
                self.replanning_count_stat.load(AtomicOrdering::Relaxed),
            );
        }

        let mut path = Vec::with_capacity(100);
        path.push(curr_node);
        let mut curr = curr_node;
        let mut visited = HashSet::new();
        visited.insert(curr);

        let max_steps = self.max_nodes_expanded; // 借用一下这个阈值

        while curr != goal_node && path.len() < max_steps {
            let mut min_cost = INF;
            let mut best_next = None;

            // 寻找 Cost = c(curr, next) + g(next) 最小的邻居
            for (neighbor, move_cost) in self.get_neighbors(&curr) {
                // 提取路径时必须避开障碍物
                if self.grid.is_obstacle_unsafe(&neighbor) {
                    continue;
                }

                let g_n = self.get_g(&neighbor);
                let c = move_cost + g_n;

                if c < min_cost - EPSILON {
                    min_cost = c;
                    best_next = Some(neighbor);
                }
            }

            if let Some(next) = best_next {
                if visited.contains(&next) {
                    // 环路检测
                    break;
                }
                path.push(next);
                visited.insert(next);
                curr = next;
            } else {
                // 死胡同
                break;
            }
        }

        let count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed) - start_nodes_count;
        let is_3d = self.grid.layers > 1;
        let py_path = path.iter().map(|n| n.to_tuple(is_3d)).collect();

        (
            Some(py_path),
            count,
            self.replanning_count_stat.load(AtomicOrdering::Relaxed),
        )
    }
}

// 内部辅助方法实现
impl RustDLitePlanner {
    fn full_reset(&mut self, start: Node, goal: Node) {
        self.start_node = Some(start);
        self.goal_node = Some(goal);
        self.last_start_node = Some(start);
        self.km = 0.0;

        self.g.clear();
        self.rhs.clear();
        self.u_queue.clear();
        self.open_keys.clear();

        // 反向搜索：Goal 为根，rhs = 0
        self.rhs.insert(goal, 0.0);
        let k = self.calculate_key(&goal);
        self.insert_to_open(goal, k);
    }

    fn get_g(&self, u: &Node) -> f32 {
        *self.g.get(u).unwrap_or(&INF)
    }

    fn get_rhs(&self, u: &Node) -> f32 {
        *self.rhs.get(u).unwrap_or(&INF)
    }

    fn heuristic(&self, a: &Node, b: &Node) -> f32 {
        let dx = (a.x - b.x).abs() as f32;
        let dy = (a.y - b.y).abs() as f32;
        let dz = (a.z - b.z).abs() as f32;

        // [Note] D* Lite 要求 heuristic 一致，因此不使用 _heuristic_weight (始终乘 1.0)

        if self.grid.layers <= 1 {
            // 2D Heuristic
            let min_d = dx.min(dy);
            (dx + dy) + (COST_2 - 2.0) * min_d
        } else {
            // 3D Heuristic
            if self.use_octile_3d {
                let mut arr = [dx, dy, dz];
                // 排序以应用 Octile 逻辑
                arr.sort_by(|a, b| a.partial_cmp(b).unwrap());
                arr[0] * COST_3 + (arr[1] - arr[0]) * COST_2 + (arr[2] - arr[1]) * COST_1
            } else {
                (dx * dx + dy * dy + dz * dz).sqrt()
            }
        }
    }

    fn calculate_key(&self, u: &Node) -> (f32, f32) {
        let g_val = self.get_g(u);
        let rhs_val = self.get_rhs(u);
        let min_val = g_val.min(rhs_val);

        if min_val == INF {
            return (INF, INF);
        }

        // k1 = min_val + h + km
        let h = self.heuristic(self.start_node.as_ref().unwrap(), u);
        let k1 = min_val + h + self.km;
        (k1, min_val)
    }

    fn insert_to_open(&mut self, u: Node, key: (f32, f32)) {
        self.open_keys.insert(u, key);
        // 使用 NotNan 保证排序安全性
        let k = DKey(NotNan::new(key.0).unwrap(), NotNan::new(key.1).unwrap());
        self.u_queue.push(PriorityItem { key: k, node: u });
    }

    /// 更新节点的一致性状态
    fn update_vertex(&mut self, u: &Node) {
        if self.goal_node.is_none() {
            return;
        }

        // 只有非目标点才需要根据邻居更新 rhs
        if Some(*u) != self.goal_node {
            let mut min_rhs = INF;

            // 如果 u 本身是障碍物，rhs 必须是 INF
            if self.grid.is_obstacle_unsafe(u) {
                min_rhs = INF;
            } else {
                // rhs(u) = min(g(s) + c(u, s))
                for (neighbor, cost) in self.get_neighbors(u) {
                    if self.grid.is_obstacle_unsafe(&neighbor) {
                        continue;
                    }
                    let g_n = self.get_g(&neighbor);
                    let temp = g_n + cost;
                    if temp < min_rhs {
                        min_rhs = temp;
                    }
                }
            }
            self.rhs.insert(*u, min_rhs);
        }

        // 检查一致性
        let g_val = self.get_g(u);
        let rhs_val = self.get_rhs(u);

        if (g_val - rhs_val).abs() > EPSILON {
            // 不一致 -> 插入队列
            let k = self.calculate_key(u);
            self.insert_to_open(*u, k);
        } else {
            // 一致 -> 从队列索引中移除 (Lazy Removal)
            self.open_keys.remove(u);
        }
    }

    /// D* Lite 核心循环
    fn compute_shortest_path(&mut self) -> bool {
        if self.start_node.is_none() {
            return false;
        }
        let start = self.start_node.unwrap();

        let mut valid_expansions = 0;

        while let Some(item) = self.u_queue.peek() {
            // 熔断保护
            if valid_expansions > self.max_nodes_expanded {
                return false;
            }

            let k_old_raw = item.key;
            let u = item.node;
            let k_old = (k_old_raw.0.into_inner(), k_old_raw.1.into_inner());

            // Lazy Removal 检查
            if let Some(k_curr) = self.open_keys.get(&u) {
                // 如果堆里的 key 和当前记录的不一致，说明是过期的
                if (k_curr.0 - k_old.0).abs() > EPSILON || (k_curr.1 - k_old.1).abs() > EPSILON {
                    self.u_queue.pop();
                    continue;
                }
            } else {
                // 如果 open_keys 里没有，说明已经达到一致被移除了
                self.u_queue.pop();
                continue;
            }

            // 终止条件检查
            let k_start = self.calculate_key(&start);
            let start_g = self.get_g(&start);
            let start_rhs = self.get_rhs(&start);

            // k_old >= k_start + tolerance 且 start 一致
            let term1 = k_old.0 > k_start.0 + FLOAT_TOLERANCE;
            let term2 =
                (k_old.0 - k_start.0).abs() < EPSILON && k_old.1 > k_start.1 + FLOAT_TOLERANCE;

            // 如果堆顶 Key 已经比起点 Key 大了，且起点是一致的，说明传播完成
            if (start_g - start_rhs).abs() < EPSILON && (term1 || term2) {
                return true;
            }

            // 启发式剪枝 (参考 _cost_threshold)
            // 如果起点不可达 (g=INF, rhs=INF) 且堆顶代价已经非常大，可以提前退出
            // 这里使用了 _cost_threshold 避免无限搜索
            if start_g == INF && start_rhs == INF {
                if k_old.0 > self._cost_threshold {
                    self.u_queue.pop();
                    self.open_keys.remove(&u);
                    continue;
                }
            }

            // 弹出并扩展
            self.u_queue.pop();
            valid_expansions += 1;
            self.nodes_expanded_stat
                .fetch_add(1, AtomicOrdering::Relaxed);

            let k_new = self.calculate_key(&u);

            // 情况 1: 节点 Key 变小了 (需更新)
            // 比较 k_old < k_new (DKey 是反向的 Ord，但这里我们用 float 直接比)
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

            // 情况 2: Overconsistent (g > rhs) -> 变好了
            if g_u > rhs_u {
                self.g.insert(u, rhs_u);
                self.open_keys.remove(&u); // 局部一致
                                           // 传播给邻居
                for (s, _) in self.get_neighbors(&u) {
                    self.update_vertex(&s);
                }
            }
            // 情况 3: Underconsistent (g < rhs) -> 变坏了 (阻断)
            else {
                self.g.insert(u, INF);
                self.update_vertex(&u); // 自己变回不一致重新入队
                                        // 强制通知所有邻居
                for (s, _) in self.get_neighbors(&u) {
                    self.update_vertex(&s);
                }
            }
        }

        false
    }

    /// 获取邻居节点 (包含严格的防切角逻辑)
    fn get_neighbors(&self, node: &Node) -> Vec<(Node, f32)> {
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

                // 2D 防切角
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
            // 3D 26-邻域
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

                        // 3D 防切角 (简化版：检查涉及的任何一个基础方块)
                        if dx != 0 && dy != 0 {
                            let c1 = Node::new(node.x + dx, node.y, node.z);
                            let c2 = Node::new(node.x, node.y + dy, node.z);
                            if !self.grid.is_safe(&c1) || !self.grid.is_safe(&c2) {
                                continue;
                            }
                        }
                        // 如果有 Z 轴移动，这里还可以添加更严格的检查，但根据 Python 版逻辑，
                        // 主要防止 XY 平面穿模即可。

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
