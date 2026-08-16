use crate::map::RustMapManager;
use crate::post_processing::bezier::BezierSmoother;
use crate::post_processing::greedy::GreedyOptimizer;
use pyo3::prelude::*;

/// Rust 实现的高性能后处理器工厂。
///
/// 该类作为一个无状态的工具类暴露给 Python，提供静态方法来调用底层的优化算法。
/// 这样设计是为了让 Python 侧的 `PathPostProcessor` 仍能管理生命周期和配置，
/// 仅将繁重的计算委托给 Rust。
#[pyclass]
pub struct RustPostProcessor;

#[pymethods]
impl RustPostProcessor {
    /// 执行贪婪捷径优化。
    ///
    /// # Arguments
    /// * `path` - 路径点列表 [(x, y, z), ...]。
    /// * `map_manager` - RustMapManager 的实例引用。
    /// * `check_radius` - 碰撞检测半径 (m)。
    /// * `check_z_margin` - 垂直安全边距 (m)。
    /// * `ignore_z` - 是否忽略高度检测。
    /// * `grace_start` - 豁免检测的起始点 (x, y, z)。
    /// * `grace_end` - 豁免检测的终点 (x, y, z)。
    #[staticmethod]
    #[pyo3(signature = (path, map_manager, check_radius, check_z_margin, ignore_z, grace_start, grace_end))]
    pub fn greedy_shortcut(
        path: Vec<(f32, f32, f32)>,
        map_manager: &RustMapManager,
        check_radius: f32,
        check_z_margin: f32,
        ignore_z: bool,
        grace_start: (f32, f32, f32),
        grace_end: (f32, f32, f32),
    ) -> PyResult<Vec<(f32, f32, f32)>> {
        // 直接调用内部实现，无需获取 GIL，因为数据已经转换
        let optimized = GreedyOptimizer::process(
            path,
            map_manager,
            check_radius,
            check_z_margin,
            ignore_z,
            grace_start,
            grace_end,
        );
        Ok(optimized)
    }

    /// 执行贝塞尔平滑优化。
    ///
    /// # Arguments
    /// * `path` - 路径点列表。
    /// * `map_manager` - RustMapManager 实例。
    /// * `smoothness` - 平滑度 (0.0 - 0.5)。
    /// * `segments` - 插值段数。
    /// * `check_radius` - 碰撞检测半径。
    /// * `check_z_margin` - 垂直安全边距。
    /// * `ignore_z` - 忽略高度。
    /// * `grace_start` - 豁免起点。
    /// * `grace_end` - 豁免终点。
    #[staticmethod]
    #[pyo3(signature = (path, map_manager, smoothness, segments, check_radius, check_z_margin, ignore_z, grace_start, grace_end))]
    #[allow(clippy::too_many_arguments)]
    pub fn bezier_smooth(
        path: Vec<(f32, f32, f32)>,
        map_manager: &RustMapManager,
        smoothness: f32,
        segments: usize,
        check_radius: f32,
        check_z_margin: f32,
        ignore_z: bool,
        grace_start: (f32, f32, f32),
        grace_end: (f32, f32, f32),
    ) -> PyResult<Vec<(f32, f32, f32)>> {
        let smoothed = BezierSmoother::process(
            path,
            map_manager,
            smoothness,
            segments,
            check_radius,
            check_z_margin,
            ignore_z,
            grace_start,
            grace_end,
        );
        Ok(smoothed)
    }
}
