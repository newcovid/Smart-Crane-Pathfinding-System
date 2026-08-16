use crate::common::Node;
use crate::map::RustMapManager;
use log::{debug, info};
use pyo3::prelude::*;

/// Rust 实现的高性能安全守卫。
///
/// 主要负责高频的搜索和校验逻辑，例如智能脱困 (Smart Escape)。
#[pyclass]
pub struct RustSafetyGuard;

#[pymethods]
impl RustSafetyGuard {
    /// 智能脱困搜索 (Smart Escape)。
    ///
    /// 当起点位于不可行区域（如膨胀层）时，以同心壳层向外搜索最近的安全节点。
    ///
    /// # Arguments
    /// * `start_node` - 原始起点网格坐标 (x, y, z)。
    /// * `ref_goal` - 参考目标点网格坐标 (用于启发式选择方向)。
    /// * `map_manager` - 地图管理器引用。
    /// * `check_radius` - 碰撞检测半径 (m)。
    /// * `check_z_margin` - 垂直安全边距 (m)。
    /// * `max_radius_grid` - 最大搜索半径 (格)。
    /// * `ignore_z` - 是否忽略高度检测。
    ///
    /// # Returns
    /// * `Option<(i32, i32, i32)>` - 找到的安全节点坐标，失败返回 None。
    #[staticmethod]
    // 参数均为彼此独立的量（起终点、地图、半径、边距、策略开关），
    // 打包成 struct 只会增加一层间接，不提升可读性。
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (start_node, ref_goal, map_manager, check_radius, check_z_margin, max_radius_grid, ignore_z, search_3d))]
    pub fn smart_escape(
        start_node: (i32, i32, i32),
        ref_goal: (i32, i32, i32),
        map_manager: &RustMapManager,
        check_radius: f32,
        check_z_margin: f32,
        max_radius_grid: i32,
        ignore_z: bool,
        search_3d: bool,
    ) -> Option<(i32, i32, i32)> {
        // 1. 快速检查原点
        // RustMapManager 内部没有缓存 Grid 数组，所以直接用 check_collision_raw 进行几何检测
        // 这在障碍物数量不是极多时，性能通常优于 Python 的 Grid 数组访问 + 解释器开销
        let (wx, wy, wz) = map_manager.grid_to_world(start_node.0, start_node.1, start_node.2);
        if !map_manager.check_collision_raw(wx, wy, wz, check_radius, check_z_margin, ignore_z) {
            return Some(start_node);
        }

        debug!(
            "[Rust Safety] 启动脱困搜索: Start={:?}, MaxR={} (Res={:.2}m)",
            start_node, max_radius_grid, map_manager.resolution_m
        );

        // 规划维度由调用方显式传入，不能从 map_manager.layers 推断：
        // layers = ceil(height_m / resolution) 恒 > 1（默认 20/1.0 = 20），
        // 于是 2.5D 定高巡航模式下也会做 3D 壳层搜索。那样的候选点包含
        // (0, 0, ±r) 这类 XY 未移动、只抬高了 Z 的位置；当 ignore_z=false 时
        // 它们因高度足够而通过碰撞检测被当作脱困点返回，Python 侧再把 z 截掉，
        // 结果脱困点等于原点，起点仍在膨胀层内。
        let dims_are_3d = search_3d;

        // 2. 同心壳层搜索 (Concentric Shell Search)
        for r in 1..=max_radius_grid {
            let mut candidates = Vec::new();

            if !dims_are_3d {
                // 2D 搜索: 仅遍历 XY 平面的外壳
                // 外壳定义：切比雪夫距离 max(|dx|, |dy|) == r
                for dx in -r..=r {
                    for dy in -r..=r {
                        if dx.abs().max(dy.abs()) == r {
                            let nx = start_node.0 + dx;
                            let ny = start_node.1 + dy;
                            let nz = start_node.2; // 2D 模式保持 Z 不变

                            // 转换坐标并检查碰撞
                            let (wx, wy, wz) = map_manager.grid_to_world(nx, ny, nz);
                            if !map_manager.check_collision_raw(
                                wx,
                                wy,
                                wz,
                                check_radius,
                                check_z_margin,
                                ignore_z,
                            ) {
                                candidates.push(Node::new(nx, ny, nz));
                            }
                        }
                    }
                }
            } else {
                // 3D 搜索: 遍历 XYZ 立方体外壳
                for dx in -r..=r {
                    for dy in -r..=r {
                        for dz in -r..=r {
                            if dx.abs().max(dy.abs()).max(dz.abs()) == r {
                                let nx = start_node.0 + dx;
                                let ny = start_node.1 + dy;
                                let nz = start_node.2 + dz;

                                let (wx, wy, wz) = map_manager.grid_to_world(nx, ny, nz);
                                if !map_manager.check_collision_raw(
                                    wx,
                                    wy,
                                    wz,
                                    check_radius,
                                    check_z_margin,
                                    ignore_z,
                                ) {
                                    candidates.push(Node::new(nx, ny, nz));
                                }
                            }
                        }
                    }
                }
            }

            // 如果当前层找到了候选点
            if !candidates.is_empty() {
                // 贪婪策略：选择离目标欧氏距离最近的点
                // 避免 float 比较的烦恼，直接用距离平方 (i32 足够，因为是 Grid 坐标)
                let best_node = candidates
                    .iter()
                    .min_by_key(|n| {
                        let dx = n.row - ref_goal.0;
                        let dy = n.col - ref_goal.1;
                        let dz = n.layer - ref_goal.2;
                        dx * dx + dy * dy + dz * dz
                    })
                    .unwrap();

                info!(
                    "[Rust Safety] 脱困成功! 半径: {}, 新起点: {:?}",
                    r, best_node
                );
                return Some((best_node.row, best_node.col, best_node.layer));
            }
        }

        debug!("[Rust Safety] 脱困失败: 范围耗尽");
        None
    }
}
