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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DKey(NotNan<f32>, NotNan<f32>);

// 实现反向比较以适配 Rust 的 Max-Heap -> Min-Heap
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
        match self.key.cmp(&other.key) {
            Ordering::Equal => other.node.cmp(&self.node),
            ord => ord,
        }
    }
}

impl PartialOrd for PriorityItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[pyclass]
pub struct RustDLitePlanner {
    grid: FlatGrid,
    use_octile_3d: bool,
    _heuristic_weight: f32,
    g: HashMap<Node, f32>,
    rhs: HashMap<Node, f32>,
    u_queue: BinaryHeap<PriorityItem>,
    open_keys: HashMap<Node, (f32, f32)>,
    km: f32,
    start_node: Option<Node>,
    goal_node: Option<Node>,
    last_start_node: Option<Node>,
    max_nodes_expanded: usize,
    _cost_threshold: f32,
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
        let flat_grid = parse_python_grid(grid)?;
        let total_voxels = (flat_grid.rows * flat_grid.cols * flat_grid.layers.max(1)) as usize;

        // 设置合理的动态阈值 (对齐 Python 版参数)
        let max_nodes = std::cmp::max(5000, total_voxels * 5);
        let cost_thresh = (total_voxels as f32) * 1.74 * 10.0;

        let safe_weight = 1.0;

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

    pub fn update_grid(&mut self, grid: &Bound<PyAny>) -> PyResult<()> {
        let flat_grid = parse_python_grid(grid)?;
        self.grid = flat_grid;
        Ok(())
    }

    // [Fix] 返回 (bool, usize) 以显式传递扩展节点数
    pub fn initialize(&mut self, start: (i32, i32, i32), goal: (i32, i32, i32)) -> (bool, usize) {
        let start_nodes_count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);

        let s_node = Node::new(start.0, start.1, start.2);
        let g_node = Node::new(goal.0, goal.1, goal.2);

        // 有效性检查失败，开销为 0
        if !self.grid.is_valid(&s_node) || !self.grid.is_valid(&g_node) {
            return (false, 0);
        }
        if self.grid.is_obstacle_unsafe(&g_node) {
            return (false, 0);
        }

        // 复用检查
        if Some(g_node) == self.goal_node && !self.g.is_empty() {
            if Some(s_node) != self.start_node {
                if let Some(last_start) = self.last_start_node {
                    self.km += self.heuristic(&last_start, &s_node);
                }
                self.last_start_node = Some(s_node);
                self.start_node = Some(s_node);
            }
            // 即使复用，也可能产生微小开销，这里计算差值
            let end_nodes = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);
            return (true, end_nodes - start_nodes_count);
        }

        self.full_reset(s_node, g_node);
        let success = self.compute_shortest_path();

        let end_nodes = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);
        (success, end_nodes - start_nodes_count)
    }

    // [Fix] 返回 PyResult<usize> 以显式传递扩展节点数
    pub fn update_obstacles(&mut self, changes: Vec<(i32, i32, i32, i32)>) -> PyResult<usize> {
        let start_nodes_count = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);

        if self.start_node.is_none() || self.goal_node.is_none() {
            return Ok(0);
        }

        for (r, c, l, _val) in changes {
            let u = Node::new(r, c, l);
            if !self.grid.is_valid(&u) {
                continue;
            }
            self.update_vertex(&u);
            for (neighbor, _) in self.get_neighbors(&u) {
                self.update_vertex(&neighbor);
            }
        }

        self.replanning_count_stat
            .fetch_add(1, AtomicOrdering::Relaxed);

        self.compute_shortest_path();

        let end_nodes = self.nodes_expanded_stat.load(AtomicOrdering::Relaxed);
        Ok(end_nodes - start_nodes_count)
    }

    pub fn compute_path(
        &mut self,
        current_pos: (i32, i32, i32),
    ) -> (Option<Vec<PyObject>>, usize, usize) {
        let curr_node = Node::new(current_pos.0, current_pos.1, current_pos.2);
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

        if Some(curr_node) != self.last_start_node {
            if let Some(last) = self.last_start_node {
                self.km += self.heuristic(&last, &curr_node);
            }
            self.last_start_node = Some(curr_node);
        }
        self.start_node = Some(curr_node);

        if !self.compute_shortest_path() {
            self.full_reset(curr_node, goal_node);
            if !self.compute_shortest_path() {
                let count =
                    self.nodes_expanded_stat.load(AtomicOrdering::Relaxed) - start_nodes_count;
                return (
                    None,
                    count,
                    self.replanning_count_stat.load(AtomicOrdering::Relaxed),
                );
            }
        }

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

        let max_steps = self.max_nodes_expanded;

        while curr != goal_node && path.len() < max_steps {
            let mut min_cost = INF;
            let mut best_next = None;

            for (neighbor, move_cost) in self.get_neighbors(&curr) {
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
                    break;
                }
                path.push(next);
                visited.insert(next);
                curr = next;
            } else {
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

// 内部方法 impl RustDLitePlanner { ... } 保持不变
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

    fn calculate_key(&self, u: &Node) -> (f32, f32) {
        let g_val = self.get_g(u);
        let rhs_val = self.get_rhs(u);
        let min_val = g_val.min(rhs_val);

        if min_val == INF {
            return (INF, INF);
        }

        let h = self.heuristic(self.start_node.as_ref().unwrap(), u);
        let k1 = min_val + h + self.km;
        (k1, min_val)
    }

    fn insert_to_open(&mut self, u: Node, key: (f32, f32)) {
        self.open_keys.insert(u, key);
        let k = DKey(NotNan::new(key.0).unwrap(), NotNan::new(key.1).unwrap());
        self.u_queue.push(PriorityItem { key: k, node: u });
    }

    fn update_vertex(&mut self, u: &Node) {
        if self.goal_node.is_none() {
            return;
        }

        if Some(*u) != self.goal_node {
            let mut min_rhs = INF;

            if self.grid.is_obstacle_unsafe(u) {
                min_rhs = INF;
            } else {
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

        let g_val = self.get_g(u);
        let rhs_val = self.get_rhs(u);

        if (g_val - rhs_val).abs() > EPSILON {
            let k = self.calculate_key(u);
            self.insert_to_open(*u, k);
        } else {
            self.open_keys.remove(u);
        }
    }

    fn compute_shortest_path(&mut self) -> bool {
        if self.start_node.is_none() {
            return false;
        }
        let start = self.start_node.unwrap();

        let mut valid_expansions = 0;

        while let Some(item) = self.u_queue.peek() {
            if valid_expansions > self.max_nodes_expanded {
                return false;
            }

            let k_old_raw = item.key;
            let u = item.node;
            let k_old = (k_old_raw.0.into_inner(), k_old_raw.1.into_inner());

            if let Some(k_curr) = self.open_keys.get(&u) {
                if (k_curr.0 - k_old.0).abs() > EPSILON || (k_curr.1 - k_old.1).abs() > EPSILON {
                    self.u_queue.pop();
                    continue;
                }
            } else {
                self.u_queue.pop();
                continue;
            }

            let k_start = self.calculate_key(&start);
            let start_g = self.get_g(&start);
            let start_rhs = self.get_rhs(&start);

            let term1 = k_old.0 > k_start.0 + FLOAT_TOLERANCE;
            let term2 =
                (k_old.0 - k_start.0).abs() < EPSILON && k_old.1 > k_start.1 + FLOAT_TOLERANCE;

            if (start_g - start_rhs).abs() < EPSILON && (term1 || term2) {
                return true;
            }

            if start_g == INF && start_rhs == INF {
                if k_old.0 > self._cost_threshold {
                    self.u_queue.pop();
                    self.open_keys.remove(&u);
                    continue;
                }
            }

            self.u_queue.pop();
            valid_expansions += 1;
            self.nodes_expanded_stat
                .fetch_add(1, AtomicOrdering::Relaxed);

            let k_new = self.calculate_key(&u);

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

            if g_u > rhs_u {
                self.g.insert(u, rhs_u);
                self.open_keys.remove(&u);
                for (s, _) in self.get_neighbors(&u) {
                    self.update_vertex(&s);
                }
            } else {
                self.g.insert(u, INF);
                self.update_vertex(&u);
                for (s, _) in self.get_neighbors(&u) {
                    self.update_vertex(&s);
                }
            }
        }

        false
    }

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
