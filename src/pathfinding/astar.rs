use crate::common::{parse_python_grid, FlatGrid, Node};
use ordered_float::NotNan;
use pyo3::prelude::*;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};

// --- 常量定义 ---
const COST_1: f32 = 1.0;
const COST_2: f32 = 1.41421356; // sqrt(2)
const COST_3: f32 = 1.73205081; // sqrt(3)
const EPSILON: f32 = 1e-4;

/// A* 优先队列元素
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AStarItem {
    f_score: NotNan<f32>, // f = g + h
    node: Node,
}

// [重要] 实现确定性比较
// Rust 的 BinaryHeap 是最大堆，我们需要最小堆的效果。
// 1. 优先比较 f_score (越小越好 -> other.cmp(self))
// 2. 如果 f_score 相同，比较 node 坐标 (越小越好 -> other.cmp(self))
//    这模仿了 Python heapq (Min-Heap) 比较 tuple (f, node) 的行为：
//    Python: (1.0, (0,0)) < (1.0, (1,1)) -> pop (0,0)
//    Rust MaxHeap: pop 最大的。如果不反转 node 比较，(1,1) 会被认为比 (0,0) 大从而先 pop。
//    为了对齐 Python 的 "坐标小者优先"，我们需要让 (0,0) 在 MaxHeap 中显得"更大"。
impl Ord for AStarItem {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.f_score.cmp(&self.f_score) {
            Ordering::Equal => other.node.cmp(&self.node), // 平局打破：坐标小的优先
            ord => ord,
        }
    }
}

impl PartialOrd for AStarItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[pyclass]
pub struct RustAStarPlanner {
    grid: FlatGrid,
    use_octile_3d: bool,
    heuristic_weight: f32,
    max_nodes_expanded: usize,
    nodes_expanded_stat: AtomicUsize,
}

#[pymethods]
impl RustAStarPlanner {
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

        // 动态阈值: total_grid_size * 5
        let max_nodes = std::cmp::max(5000, total_voxels * 5);

        Ok(Self {
            grid: flat_grid,
            use_octile_3d,
            heuristic_weight: if heuristic_weight < 1.0 {
                1.0
            } else {
                heuristic_weight
            },
            max_nodes_expanded: max_nodes,
            nodes_expanded_stat: AtomicUsize::new(0),
        })
    }

    pub fn update_grid(&mut self, grid: &Bound<PyAny>) -> PyResult<()> {
        let flat_grid = parse_python_grid(grid)?;
        self.grid = flat_grid;
        Ok(())
    }

    pub fn initialize(&self, start: (i32, i32, i32), goal: (i32, i32, i32)) -> bool {
        let s_node = Node::new(start.0, start.1, start.2);
        let g_node = Node::new(goal.0, goal.1, goal.2);

        if !self.grid.is_valid(&s_node) || !self.grid.is_valid(&g_node) {
            return false;
        }
        if self.grid.is_obstacle_unsafe(&g_node) {
            return false;
        }
        true
    }

    /// 执行 A* 计算
    pub fn compute_path(
        &self,
        current_pos: (i32, i32, i32),
        goal_pos: (i32, i32, i32),
    ) -> (Option<Vec<PyObject>>, usize) {
        let start = Node::new(current_pos.0, current_pos.1, current_pos.2);
        let goal = Node::new(goal_pos.0, goal_pos.1, goal_pos.2);

        self.nodes_expanded_stat.store(0, AtomicOrdering::Relaxed);

        let mut open_set = BinaryHeap::new();
        let mut g_score: HashMap<Node, f32> = HashMap::new();
        let mut came_from: HashMap<Node, Node> = HashMap::new();

        g_score.insert(start, 0.0);
        open_set.push(AStarItem {
            f_score: NotNan::new(0.0).unwrap(),
            node: start,
        });

        let mut nodes_expanded = 0;

        while let Some(item) = open_set.pop() {
            let current = item.node;

            // 如果当前路径比已知最短路径差，跳过 (Lazy Deletion)
            // 注意：因为 f = g + h，h 是固定的，所以比较 f 和 g 是一样的效果
            let current_g = *g_score.get(&current).unwrap_or(&f32::INFINITY);

            // 简单的有效性检查：如果弹出的节点的 f 值严重大于预期，可能需要跳过
            // 但标准做法是检查 g 值。这里为了性能，我们假设堆中的冗余是可以接受的，
            // 只要我们在处理邻居时进行更严格的检查。

            nodes_expanded += 1;

            // 熔断保护
            if nodes_expanded > self.max_nodes_expanded {
                return (None, nodes_expanded);
            }

            if current == goal {
                let path = self.reconstruct_path(came_from, current);
                return (Some(path), nodes_expanded);
            }

            for (neighbor, move_cost) in self.get_neighbors(&current) {
                let tentative_g = current_g + move_cost;
                let neighbor_g = *g_score.get(&neighbor).unwrap_or(&f32::INFINITY);

                if tentative_g < neighbor_g - EPSILON {
                    came_from.insert(neighbor, current);
                    g_score.insert(neighbor, tentative_g);

                    let h = self.heuristic(&neighbor, &goal);
                    let f = tentative_g + h;

                    open_set.push(AStarItem {
                        f_score: NotNan::new(f).unwrap(),
                        node: neighbor,
                    });
                }
            }
        }

        (None, nodes_expanded)
    }
}

// 内部方法
impl RustAStarPlanner {
    fn heuristic(&self, a: &Node, b: &Node) -> f32 {
        let dx = (a.x - b.x).abs() as f32;
        let dy = (a.y - b.y).abs() as f32;
        let h_val;

        if self.grid.layers <= 1 {
            let min_delta = dx.min(dy);
            h_val = (dx + dy) + (COST_2 - 2.0) * min_delta;
        } else {
            let dz = (a.z - b.z).abs() as f32;
            if self.use_octile_3d {
                let mut delta = [dx, dy, dz];
                delta.sort_by(|a, b| a.partial_cmp(b).unwrap());
                h_val = delta[0] * COST_3
                    + (delta[1] - delta[0]) * COST_2
                    + (delta[2] - delta[1]) * COST_1;
            } else {
                h_val = (dx * dx + dy * dy + dz * dz).sqrt();
            }
        }
        h_val * self.heuristic_weight
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
                let next_node = Node::new(nr, nc, 0);

                if !self.grid.is_safe(&next_node) {
                    continue;
                }

                if dr != 0 && dc != 0 {
                    let c1 = Node::new(node.x + dr, node.y, 0);
                    let c2 = Node::new(node.x, node.y + dc, 0);
                    if self.grid.is_obstacle_unsafe(&c1) || self.grid.is_obstacle_unsafe(&c2) {
                        continue;
                    }
                }
                successors.push((next_node, cost));
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
                        let next_node = Node::new(nx, ny, nz);

                        if !self.grid.is_safe(&next_node) {
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

                        successors.push((next_node, cost));
                    }
                }
            }
        }
        successors
    }

    fn reconstruct_path(&self, came_from: HashMap<Node, Node>, current: Node) -> Vec<PyObject> {
        let mut path = Vec::new();
        let mut curr = current;
        path.push(curr);

        while let Some(&parent) = came_from.get(&curr) {
            curr = parent;
            path.push(curr);
        }
        path.reverse();

        let is_3d = self.grid.layers > 1;
        path.iter().map(|n| n.to_tuple(is_3d)).collect()
    }
}
