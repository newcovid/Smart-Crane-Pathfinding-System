use crate::common::{parse_python_grid, FlatGrid, Node};
// 引入公共常量
use crate::common::{COST_1, COST_2, COST_3, EPSILON};
use ordered_float::NotNan;
use pyo3::prelude::*;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};
// 引入日志宏
use log::{debug, info, warn};

/// A* 算法优先队列中的元素。
///
/// 包含节点本身以及该节点的综合评分 `f_score` (f = g + h)。
/// 实现了 `Ord` trait 以便在 `BinaryHeap` 中排序。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AStarItem {
    /// 综合评分 f(n) = g(n) + h(n)。使用 NotNan 保证浮点数可以进行全序比较。
    f_score: NotNan<f32>,
    /// 网格中的节点坐标
    node: Node,
}

// 为 AStarItem 实现 Ord，使其在二叉堆中按 f_score 从小到大排序（最小堆）。
impl Ord for AStarItem {
    fn cmp(&self, other: &Self) -> Ordering {
        // Rust 的 BinaryHeap 默认为最大堆 (pop 取最大值)。
        // 我们需要最小 f_score 优先，因此反转比较顺序 (other.cmp(self))。
        // 当 f_score 相同时，比较 node 坐标 (模拟 Python 的稳定排序行为，通常选坐标较小的)。
        match other.f_score.cmp(&self.f_score) {
            Ordering::Equal => other.node.cmp(&self.node),
            ord => ord,
        }
    }
}

impl PartialOrd for AStarItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Rust 实现的 A* 路径规划器。
///
/// 该结构体导出为 Python 类 `RustAStarPlanner`。
/// 它维护网格地图数据并提供静态路径规划能力。
#[pyclass]
pub struct RustAStarPlanner {
    /// 扁平化的网格地图数据
    grid: FlatGrid,
    /// 是否使用 Octile 距离作为 3D 启发式 (true) 还是欧几里得距离 (false)
    use_octile_3d: bool,
    /// 启发式函数的权重 (通常 >= 1.0，值越大搜索越贪婪但路径可能非最优)
    heuristic_weight: f32,
    /// 最大允许扩展的节点数量，防止死循环或耗时过长
    max_nodes_expanded: usize,
    /// 统计指标：记录扩展的节点总数 (原子操作，线程安全)
    nodes_expanded_stat: AtomicUsize,
}

#[pymethods]
impl RustAStarPlanner {
    /// 构造一个新的 A* 规划器实例。
    ///
    /// # Arguments
    /// * `grid` - Python 侧传入的网格对象 (需包含 dim, obstacles 等属性)
    /// * `_width_m`, `_length_m`, `_height_m`, `_resolution` - 物理尺寸参数 (当前实现未使用，保留接口兼容性)
    /// * `use_octile_3d` - 是否使用 Octile 距离计算启发式
    /// * `heuristic_weight` - A* 的 H 值权重 (Weighted A*)
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
        debug!("[Rust A*] 开始初始化实例...");

        // 解析 Python 网格数据
        let flat_grid: FlatGrid = parse_python_grid(grid)?;

        // 动态计算最大扩展节点数限制
        let total_voxels: usize =
            (flat_grid.rows * flat_grid.cols * flat_grid.layers.max(1)) as usize;
        // 限制至少 5000，或体素总数的 5 倍
        let max_nodes: usize = std::cmp::max(5000, total_voxels * 5);

        // 确保 heuristic_weight 不小于 1.0，否则 A* 变种特性不可控
        let safe_weight: f32 = if heuristic_weight < 1.0 {
            1.0
        } else {
            heuristic_weight
        };

        info!(
            "[Rust A*] 初始化就绪. 地图尺寸: {}x{}x{}, 最大搜索节点: {}, H权重: {:.1}",
            flat_grid.rows, flat_grid.cols, flat_grid.layers, max_nodes, safe_weight
        );

        Ok(Self {
            grid: flat_grid,
            use_octile_3d,
            heuristic_weight: safe_weight,
            max_nodes_expanded: max_nodes,
            nodes_expanded_stat: AtomicUsize::new(0),
        })
    }

    /// 更新规划器内部的网格数据。
    ///
    /// 当环境发生变化（例如障碍物增加）时调用此方法。
    pub fn update_grid(&mut self, grid: &Bound<PyAny>) -> PyResult<()> {
        debug!("[Rust A*] 接收到新的网格数据，正在解析...");
        let flat_grid: FlatGrid = parse_python_grid(grid)?;
        self.grid = flat_grid;
        debug!("[Rust A*] 网格更新完成.");
        Ok(())
    }

    /// 验证起点和终点的有效性。
    ///
    /// 检查点是否在地图边界内，以及终点是否位于障碍物中。
    pub fn initialize(&self, start: (i32, i32, i32), goal: (i32, i32, i32)) -> bool {
        let s_node = Node::new(start.0, start.1, start.2);
        let g_node = Node::new(goal.0, goal.1, goal.2);

        if !self.grid.is_valid(&s_node) {
            warn!("[Rust A*] 初始化失败: 起点越界 {:?}", s_node);
            return false;
        }
        if !self.grid.is_valid(&g_node) {
            warn!("[Rust A*] 初始化失败: 终点越界 {:?}", g_node);
            return false;
        }
        if self.grid.is_obstacle_unsafe(&g_node) {
            warn!("[Rust A*] 初始化失败: 终点位于障碍物内 {:?}", g_node);
            return false;
        }
        true
    }

    /// 执行 A* 路径搜索。
    ///
    /// # Returns
    /// 返回元组 `(Option<Vec<Py<PyAny>>>, usize)`:
    /// * `Option<Vec<Py<PyAny>>>`: 成功时包含路径点列表（Python对象），失败时为 None。
    /// * `usize`: 本次搜索扩展的节点总数。
    pub fn compute_path(
        &self,
        current_pos: (i32, i32, i32),
        goal_pos: (i32, i32, i32),
    ) -> (Option<Vec<Py<PyAny>>>, usize) {
        let start = Node::new(current_pos.0, current_pos.1, current_pos.2);
        let goal = Node::new(goal_pos.0, goal_pos.1, goal_pos.2);

        // 重置统计计数器
        self.nodes_expanded_stat.store(0, AtomicOrdering::Relaxed);

        // Open Set: 待探索的节点，使用二叉堆优化最小值提取
        let mut open_set: BinaryHeap<AStarItem> = BinaryHeap::new();
        // G Score: 从起点到当前点的实际已知代价
        let mut g_score: HashMap<Node, f32> = HashMap::new();
        // Came From: 记录路径来源，用于回溯路径
        let mut came_from: HashMap<Node, Node> = HashMap::new();

        // 初始化起点
        g_score.insert(start, 0.0);
        open_set.push(AStarItem {
            f_score: NotNan::new(0.0).unwrap(), // 起点 f = g(0) + h(start, goal) ? 此处简化为0，因为马上会弹出
            node: start,
        });

        let mut nodes_expanded: usize = 0;

        // --- 主循环 ---
        while let Some(item) = open_set.pop() {
            let current = item.node;

            // 如果堆中的节点代价已经劣于已知的 g_score (懒惰删除策略)，则跳过
            // 注意：Rust 的 BinaryHeap 不支持 decrease_key，通常通过 push 新值并在此处检查来处理
            // 但此处我们直接取值，不严格检查 stale entry 也是一种常见做法，
            // 若需严格 correctness: if item.f_score > f_score_of_current ...

            let current_g: f32 = *g_score.get(&current).unwrap_or(&f32::INFINITY);

            nodes_expanded += 1;

            // 安全熔断机制
            if nodes_expanded > self.max_nodes_expanded {
                warn!(
                    "[Rust A*] 搜索超时! 已扩展 {} 节点，超过阈值 {}",
                    nodes_expanded, self.max_nodes_expanded
                );
                return (None, nodes_expanded);
            }

            // 找到目标
            if current == goal {
                info!(
                    "[Rust A*] 路径规划成功! 扩展节点: {}, 总代价: {:.2}",
                    nodes_expanded, current_g
                );
                let path = self.reconstruct_path(came_from, current);
                return (Some(path), nodes_expanded);
            }

            // 扩展邻居
            for (neighbor, move_cost) in self.get_neighbors(&current) {
                let tentative_g: f32 = current_g + move_cost;
                let neighbor_g: f32 = *g_score.get(&neighbor).unwrap_or(&f32::INFINITY);

                // 发现更优路径
                if tentative_g < neighbor_g - EPSILON {
                    came_from.insert(neighbor, current);
                    g_score.insert(neighbor, tentative_g);

                    let h: f32 = self.heuristic(&neighbor, &goal);
                    let f: f32 = tentative_g + h;

                    open_set.push(AStarItem {
                        f_score: NotNan::new(f).unwrap(),
                        node: neighbor,
                    });
                }
            }
        }

        warn!(
            "[Rust A*] OpenSet 耗尽，无解。共扩展: {} 节点",
            nodes_expanded
        );
        (None, nodes_expanded)
    }
}

// --- 内部辅助方法 ---
impl RustAStarPlanner {
    /// 计算启发式代价 H(n)。
    ///
    /// 根据配置使用欧几里得距离或 Octile 距离。
    fn heuristic(&self, a: &Node, b: &Node) -> f32 {
        let dx = (a.x - b.x).abs() as f32;
        let dy = (a.y - b.y).abs() as f32;
        let h_val: f32;

        if self.grid.layers <= 1 {
            // 2D 情况：Octile 距离 (对角线移动优化)
            let min_delta = dx.min(dy);
            let _max_delta = dx.max(dy);
            // 代价 = 对角线步数 * sqrt(2) + 剩余直线步数 * 1
            // 简化公式: (dx + dy) + (sqrt(2) - 2) * min(dx, dy)
            h_val = (dx + dy) + (COST_2 - 2.0) * min_delta;
        } else {
            // 3D 情况
            let dz = (a.z - b.z).abs() as f32;
            if self.use_octile_3d {
                let mut delta = [dx, dy, dz];
                // 排序以找到最小、中间和最大差值
                delta.sort_by(|x, y| x.partial_cmp(y).unwrap());
                // 3D Octile 公式：
                // 最小差值部分走 3D 对角线 (COST_3)
                // 中间差值减去最小差值部分走 2D 对角线 (COST_2)
                // 最大差值减去中间差值部分走直线 (COST_1)
                h_val = delta[0] * COST_3
                    + (delta[1] - delta[0]) * COST_2
                    + (delta[2] - delta[1]) * COST_1;
            } else {
                // 标准欧几里得距离
                h_val = (dx * dx + dy * dy + dz * dz).sqrt();
            }
        }
        // 应用加权 A* 的权重
        h_val * self.heuristic_weight
    }

    /// 获取当前节点的所有合法邻居。
    ///
    /// # Returns
    /// 返回元组向量 `Vec<(Node, move_cost)>`。
    /// 包含碰撞检测逻辑：不仅检查目标点是否安全，还检查对角线移动是否会“穿墙”（Corner Cutting）。
    fn get_neighbors(&self, node: &Node) -> Vec<(Node, f32)> {
        // 预分配容量，2D最多8个，3D最多26个
        let mut successors = Vec::with_capacity(26);
        let is_2d = self.grid.layers <= 1;

        if is_2d {
            // 2D 移动向量表: (dr, dc, cost)
            let moves = [
                (0, 1, COST_1),
                (0, -1, COST_1),
                (1, 0, COST_1),
                (-1, 0, COST_1), // 直线
                (1, 1, COST_2),
                (1, -1, COST_2),
                (-1, 1, COST_2),
                (-1, -1, COST_2), // 对角线
            ];

            for &(dr, dc, cost) in &moves {
                let nr = node.x + dr;
                let nc = node.y + dc;
                let next = Node::new(nr, nc, 0);

                // 1. 基础边界和障碍物检查
                if !self.grid.is_safe(&next) {
                    continue;
                }

                // 2. 对角线移动的额外碰撞检测 (防止穿过两个障碍物的夹角)
                if dr != 0 && dc != 0 {
                    let c1 = Node::new(node.x + dr, node.y, 0);
                    let c2 = Node::new(node.x, node.y + dc, 0);
                    // 如果两个分量方向都是障碍物，则不能走对角线
                    if self.grid.is_obstacle_unsafe(&c1) || self.grid.is_obstacle_unsafe(&c2) {
                        continue;
                    }
                }
                successors.push((next, cost));
            }
        } else {
            // 3D 邻居生成 (三重循环)
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

                        // 3D 对角线检查 (简化版：仅检查水平面对角线，如果 dz!=0 且 dx,dy!=0 需要更严谨逻辑，此处沿用 2D 逻辑的扩展)
                        if dx != 0 && dy != 0 {
                            let c1 = Node::new(node.x + dx, node.y, node.z);
                            let c2 = Node::new(node.x, node.y + dy, node.z);
                            if !self.grid.is_safe(&c1) || !self.grid.is_safe(&c2) {
                                continue;
                            }
                        }

                        // 计算移动代价
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

    /// 从 came_from 映射中重建路径。
    fn reconstruct_path(&self, came_from: HashMap<Node, Node>, current: Node) -> Vec<Py<PyAny>> {
        let mut path = Vec::new();
        let mut curr = current;
        path.push(curr);

        while let Some(&parent) = came_from.get(&curr) {
            curr = parent;
            path.push(curr);
        }

        // 路径是从终点回溯的，需要反转
        path.reverse();

        let is_3d = self.grid.layers > 1;
        // 转换为 Python 元组对象
        path.iter().map(|n| n.to_tuple(is_3d)).collect()
    }
}
