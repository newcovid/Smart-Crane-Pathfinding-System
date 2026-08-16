use crate::map::RustMapManager;
use log::debug;

/// 贝塞尔平滑处理器实现 (Bezier Curve Smoother Implementation)。
///
/// 逻辑与 Python 版 `BezierSmoothProcessor` 保持一致。
pub struct BezierSmoother;

impl BezierSmoother {
    /// 执行贝塞尔平滑。
    ///
    /// # Arguments
    /// * `path` - 原始路径点列表。
    /// * `map` - 地图管理器引用。
    /// * `smoothness` - 初始平滑度 (0.0 - 0.5)。
    /// * `segments` - 曲线插值段数。
    /// * `check_radius` - 碰撞检测半径。
    /// * `check_z_margin` - 垂直安全边距。
    /// * `ignore_z` - 是否忽略高度检测。
    /// * `grace_start` - 豁免区起点。
    /// * `grace_end` - 豁免区终点。
    // 参数均为彼此独立的物理量（尺寸、分辨率、边距、阈值），
    // 打包成 struct 只会增加一层间接，不提升可读性。
    #[allow(clippy::too_many_arguments)]
    pub fn process(
        path: Vec<(f32, f32, f32)>,
        map: &RustMapManager,
        smoothness: f32,
        segments: usize,
        check_radius: f32,
        check_z_margin: f32,
        ignore_z: bool,
        grace_start: (f32, f32, f32),
        grace_end: (f32, f32, f32),
    ) -> Vec<(f32, f32, f32)> {
        if path.len() < 3 {
            return path;
        }

        let default_smoothness = smoothness.clamp(0.0, 0.5);
        let mut smoothed_path = Vec::new();
        smoothed_path.push(path[0]);

        let input_nodes_count = path.len();
        let mut curved_corners = 0;

        // 遍历每一个拐角点 P1 (从第2个点到倒数第2个点)
        for i in 1..path.len() - 1 {
            let p0 = path[i - 1];
            let p1 = path[i];
            let p2 = path[i + 1];

            let mut current_smoothness = default_smoothness;
            let mut best_segment: Option<Vec<(f32, f32, f32)>> = None;

            // 自适应回退机制 (Adaptive Fallback)
            // 尝试生成曲线，如果碰撞则减半平滑度重试，最多 5 次
            for attempt in 0..5 {
                if Self::check_curve_safety(
                    p0,
                    p1,
                    p2,
                    current_smoothness,
                    map,
                    check_radius,
                    check_z_margin,
                    ignore_z,
                    grace_start,
                    grace_end,
                ) {
                    best_segment = Some(Self::generate_curve(
                        p0,
                        p1,
                        p2,
                        current_smoothness,
                        segments,
                    ));

                    if attempt > 0 {
                        debug!(
                            "[Rust Bezier] 拐角 {} 触发自适应回退: 尝试 {} 次后成功 (S: {:.4} -> {:.4})",
                            i,
                            attempt,
                            default_smoothness,
                            current_smoothness
                        );
                    }
                    break;
                } else {
                    current_smoothness *= 0.5;
                }
            }

            if let Some(segment) = best_segment {
                smoothed_path.extend(segment);
                curved_corners += 1;
            } else {
                debug!("[Rust Bezier] 拐角 {} 空间狭窄无法平滑，保留硬连接。", i);
                smoothed_path.push(p1);
            }
        }

        smoothed_path.push(path[path.len() - 1]);

        debug!(
            "[Rust Bezier] 平滑统计: 处理 {}/{} 个拐角",
            curved_corners,
            input_nodes_count - 2
        );

        smoothed_path
    }

    /// 检查生成的贝塞尔曲线是否安全。
    #[allow(clippy::too_many_arguments)]
    fn check_curve_safety(
        p0: (f32, f32, f32),
        p1: (f32, f32, f32),
        p2: (f32, f32, f32),
        s: f32,
        map: &RustMapManager,
        radius: f32,
        z_margin: f32,
        ignore_z: bool,
        grace_start: (f32, f32, f32),
        grace_end: (f32, f32, f32),
    ) -> bool {
        // 计算切点 (Anchor Points)
        let q0 = (
            p1.0 + (p0.0 - p1.0) * s,
            p1.1 + (p0.1 - p1.1) * s,
            p1.2 + (p0.2 - p1.2) * s,
        );
        let q2 = (
            p1.0 + (p2.0 - p1.0) * s,
            p1.1 + (p2.1 - p1.1) * s,
            p1.2 + (p2.2 - p1.2) * s,
        );

        // 估算曲线弧长
        let d1 = ((p1.0 - p0.0).powi(2) + (p1.1 - p0.1).powi(2) + (p1.2 - p0.2).powi(2)).sqrt();
        let d2 = ((p2.0 - p1.0).powi(2) + (p2.1 - p1.1).powi(2) + (p2.2 - p1.2).powi(2)).sqrt();
        let total_len = (d1 + d2) * s;

        // 采样密度: 每 0.2m 检测一次
        let steps = (total_len * 5.0) as i32;
        let steps = steps.max(5);

        for k in 0..=steps {
            let t = k as f32 / steps as f32;
            let t_sq = t * t;
            let inv_t = 1.0 - t;
            let inv_t_sq = inv_t * inv_t;

            // 二阶贝塞尔公式: B(t) = (1-t)^2 * q0 + 2t(1-t) * p1 + t^2 * q2
            let cx = inv_t_sq * q0.0 + 2.0 * t * inv_t * p1.0 + t_sq * q2.0;
            let cy = inv_t_sq * q0.1 + 2.0 * t * inv_t * p1.1 + t_sq * q2.1;
            let cz = inv_t_sq * q0.2 + 2.0 * t * inv_t * p1.2 + t_sq * q2.2;

            // 豁免区检测
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

            if map.check_collision_raw(cx, cy, cz, radius, z_margin, ignore_z) {
                return false;
            }
        }
        true
    }

    /// 生成贝塞尔曲线点集。
    fn generate_curve(
        p0: (f32, f32, f32),
        p1: (f32, f32, f32),
        p2: (f32, f32, f32),
        s: f32,
        segments: usize,
    ) -> Vec<(f32, f32, f32)> {
        let q0 = (
            p1.0 + (p0.0 - p1.0) * s,
            p1.1 + (p0.1 - p1.1) * s,
            p1.2 + (p0.2 - p1.2) * s,
        );
        let q2 = (
            p1.0 + (p2.0 - p1.0) * s,
            p1.1 + (p2.1 - p1.1) * s,
            p1.2 + (p2.2 - p1.2) * s,
        );

        let mut points = Vec::with_capacity(segments + 1);

        // 从 i=0（t=0，即 q0）开始输出，包含终点 q2。
        //
        // 早期实现从 1..=segments 起，理由是"q0 是上一段的终点，避免重复"——
        // 但这不成立：q0 位于 P0→P1 线段上、距 P1 为 s 的位置，与上一段的
        // 终点（上一拐角的 q2）并不重合。跳过它会让实际路径变成
        // "上一点 → curve(1/n)"这条弦，它比碰撞检查覆盖的
        // "q0 → curve(1/n)"更贴近拐角内侧，且从未被验证。
        // 碰撞检查是从 t=0 起做的，输出必须与之一致。
        for i in 0..=segments {
            let t = i as f32 / segments as f32;
            let t_sq = t * t;
            let inv_t = 1.0 - t;
            let inv_t_sq = inv_t * inv_t;

            let px = inv_t_sq * q0.0 + 2.0 * t * inv_t * p1.0 + t_sq * q2.0;
            let py = inv_t_sq * q0.1 + 2.0 * t * inv_t * p1.1 + t_sq * q2.1;
            let pz = inv_t_sq * q0.2 + 2.0 * t * inv_t * p1.2 + t_sq * q2.2;

            points.push((px, py, pz));
        }

        points
    }
}
