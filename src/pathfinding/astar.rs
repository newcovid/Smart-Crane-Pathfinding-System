use crate::common::{parse_python_grid, FlatGrid, Node};
use ordered_float::NotNan;
use pathfinding::prelude::astar;
use pyo3::prelude::*;
use std::sync::atomic::{AtomicUsize, Ordering}; // [修复] 使用原子类型替代 Cell 以确保线程安全 (Sync)

// 预定义代价常量
const COST_1: f32 = 1.0;
const COST_2: f32 = 1.41421356; // sqrt(2)
const COST_3: f32 = 1.73205081; // sqrt(3)

#[pyclass]
pub struct RustAStarPlanner {
    grid: FlatGrid,
    use_octile_3d: bool,
    heuristic_weight: f32,
    start_node: Option<Node>,
    goal_node: Option<Node>,

    // [修复] 性能统计器
    // 使用 AtomicUsize 替代 Cell<usize>。
    // Cell 只能在单线程内部可变，不满足 Sync trait，无法在 PyO3 类中安全共享。
    // AtomicUsize 提供了线程安全的内部可变性。
    nodes_expanded: AtomicUsize,
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

        Ok(Self {
            grid: flat_grid,
            use_octile_3d,
            heuristic_weight: if heuristic_weight < 1.0 {
                1.0
            } else {
                heuristic_weight
            },
            start_node: None,
            goal_node: None,
            // 初始化原子计数器
            nodes_expanded: AtomicUsize::new(0),
        })
    }

    /// [新增] 同步网格数据
    /// 当 Python 端的网格发生变化时（如添加障碍物），必须调用此方法更新 Rust 内部数据
    pub fn update_grid(&mut self, grid: &Bound<PyAny>) -> PyResult<()> {
        let flat_grid = parse_python_grid(grid)?;
        self.grid = flat_grid;
        Ok(())
    }

    pub fn initialize(&mut self, start: (i32, i32, i32), goal: (i32, i32, i32)) -> bool {
        let s_node = Node::new(start.0, start.1, start.2);
        let g_node = Node::new(goal.0, goal.1, goal.2);

        if !self.grid.is_valid(&s_node) || !self.grid.is_valid(&g_node) {
            return false;
        }

        if self.grid.is_obstacle_unsafe(&g_node) {
            return false;
        }

        self.start_node = Some(s_node);
        self.goal_node = Some(g_node);
        true
    }

    /// 执行计算
    /// 返回值: (路径列表, 扩展节点数)
    pub fn compute_path(&self, current_pos: (i32, i32, i32)) -> (Option<Vec<PyObject>>, usize) {
        let start = Node::new(current_pos.0, current_pos.1, current_pos.2);

        // 重置计数器 (使用 Relaxed 顺序即可，这里不涉及复杂的内存同步)
        self.nodes_expanded.store(0, Ordering::Relaxed);

        if let Some(goal) = self.goal_node {
            let result = astar(
                &start,
                |p| self.get_successors(p),
                |p| self.heuristic(p, &goal),
                |p| *p == goal,
            );

            // 获取最终的统计值
            let nodes_count = self.nodes_expanded.load(Ordering::Relaxed);

            match result {
                Some((path, _cost)) => {
                    let is_3d = self.grid.layers > 1;
                    let py_path = path.iter().map(|n| n.to_tuple(is_3d)).collect();
                    (Some(py_path), nodes_count)
                }
                None => (None, nodes_count),
            }
        } else {
            (None, 0)
        }
    }
}

impl RustAStarPlanner {
    fn get_successors(&self, node: &Node) -> Vec<(Node, NotNan<f32>)> {
        // [统计] 每次请求邻居，意味着当前节点被展开了
        // fetch_add 返回的是修改前的值，我们这里只关心副作用
        self.nodes_expanded.fetch_add(1, Ordering::Relaxed);

        let mut successors = Vec::with_capacity(26);
        let is_2d = self.grid.layers == 1;

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
                successors.push((next_node, NotNan::new(cost).unwrap()));
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

                        successors.push((next_node, NotNan::new(cost).unwrap()));
                    }
                }
            }
        }
        successors
    }

    fn heuristic(&self, a: &Node, b: &Node) -> NotNan<f32> {
        let h_val: f32;
        let dx = (a.x - b.x).abs() as f32;
        let dy = (a.y - b.y).abs() as f32;

        if self.grid.layers == 1 {
            let min_delta = if dx < dy { dx } else { dy };
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
        NotNan::new(h_val * self.heuristic_weight).unwrap()
    }
}
