use crate::map::RustMapManager;
use log::debug;

/// 贪婪捷径优化器实现 (Greedy Shortcut Implementation)。
///
/// 逻辑与 Python 版 `GreedyShortcutProcessor` 保持一致。
pub struct GreedyOptimizer;

impl GreedyOptimizer {
    /// 执行路径简化。
    ///
    /// # Arguments
    /// * `path` - 原始路径点列表 (x, y, z)。
    /// * `map` - 地图管理器引用，用于碰撞检测。
    /// * `check_radius` - 碰撞检测半径 (包含吊具尺寸和安全边距)。
    /// * `check_z_margin` - 垂直安全边距。
    /// * `ignore_z` - 是否忽略高度检测 (无限高模式)。
    /// * `grace_start` - 豁免区起点 (允许碰撞的区域中心)。
    /// * `grace_end` - 豁免区终点。
    pub fn process(
        path: Vec<(f32, f32, f32)>,
        map: &RustMapManager,
        check_radius: f32,
        check_z_margin: f32,
        ignore_z: bool,
        grace_start: (f32, f32, f32),
        grace_end: (f32, f32, f32),
    ) -> Vec<(f32, f32, f32)> {
        if path.len() < 3 {
            debug!("[Rust Greedy] 路径太短，无需优化。");
            return path;
        }

        let mut optimized_path = Vec::new();
        optimized_path.push(path[0]);

        let mut current_idx = 0;
        let total_nodes = path.len();
        let mut shortcut_count = 0;

        // 主循环：从当前锚点向后寻找最远的可达点
        while current_idx < total_nodes - 1 {
            let mut check_idx = total_nodes - 1;
            let mut found_shortcut = false;

            // 内循环：尝试连接 current_idx 和 check_idx
            // 至少跳过一个中间点 (check_idx > current_idx + 1)
            while check_idx > current_idx + 1 {
                let start_node = path[current_idx];
                let target_node = path[check_idx];

                // 视线检查
                if Self::has_line_of_sight(
                    start_node,
                    target_node,
                    map,
                    check_radius,
                    check_z_margin,
                    ignore_z,
                    grace_start,
                    grace_end,
                ) {
                    optimized_path.push(target_node);
                    debug!(
                        "[Rust Greedy] 发现捷径: Node[{}] -> Node[{}] (Skip {})",
                        current_idx,
                        check_idx,
                        check_idx - current_idx - 1
                    );

                    current_idx = check_idx;
                    found_shortcut = true;
                    shortcut_count += 1;
                    break;
                }
                check_idx -= 1;
            }

            // 兜底：如果无法跳过任何点，则前进一步
            if !found_shortcut {
                current_idx += 1;
                if current_idx < total_nodes {
                    optimized_path.push(path[current_idx]);
                }
            }
        }

        debug!(
            "[Rust Greedy] 优化统计: {} 次捷径操作，压缩比: {:.1}%",
            shortcut_count,
            (1.0 - optimized_path.len() as f32 / path.len() as f32) * 100.0
        );

        optimized_path
    }

    /// 检查两点之间是否存在无障碍视线 (Line of Sight)。
    #[allow(clippy::too_many_arguments)]
    fn has_line_of_sight(
        start: (f32, f32, f32),
        end: (f32, f32, f32),
        map: &RustMapManager,
        radius: f32,
        z_margin: f32,
        ignore_z: bool,
        grace_start: (f32, f32, f32),
        grace_end: (f32, f32, f32),
    ) -> bool {
        let dist_sq =
            (start.0 - end.0).powi(2) + (start.1 - end.1).powi(2) + (start.2 - end.2).powi(2);
        let distance = dist_sq.sqrt();

        // 极短距离直接通过
        if distance < 1e-6 {
            return true;
        }

        // 采样密度：每米至少 5 个点 (0.2m 精度)，至少 2 个点
        let steps = (distance * 5.0) as i32;
        let steps = steps.max(2);

        let delta_x = end.0 - start.0;
        let delta_y = end.1 - start.1;
        let delta_z = end.2 - start.2;

        // 采样检测 (跳过起点和终点，因为它们通常是已知的可行点或在豁免区)
        for i in 1..steps {
            let t = i as f32 / steps as f32;
            let cx = start.0 + delta_x * t;
            let cy = start.1 + delta_y * t;
            let cz = start.2 + delta_z * t;

            // 豁免区检测 (Grace Zone Check)
            // 如果点在起点或终点的豁免半径内 (0.5m)，则跳过碰撞检测
            // 0.25 = 0.5^2
            let d_start_sq = (cx - grace_start.0).powi(2)
                + (cy - grace_start.1).powi(2)
                + (cz - grace_start.2).powi(2);
            if d_start_sq < 0.25 {
                continue;
            }

            let d_end_sq = (cx - grace_end.0).powi(2)
                + (cy - grace_end.1).powi(2)
                + (cz - grace_end.2).powi(2);
            if d_end_sq < 0.25 {
                continue;
            }

            // 调用地图管理器的原生检测
            if map.check_collision_raw(cx, cy, cz, radius, z_margin, ignore_z) {
                return false;
            }
        }

        true
    }
}
