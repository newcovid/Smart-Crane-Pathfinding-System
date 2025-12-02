use pyo3::prelude::*;

mod common;
mod components;
mod map;
mod pathfinding;
mod post_processing;

use components::grid_adapter::RustGridAdapter; // 新增
use components::safety_guard::RustSafetyGuard;
use map::RustMapManager;
use pathfinding::astar::RustAStarPlanner;
use pathfinding::dslite::RustDLitePlanner;
use post_processing::RustPostProcessor;

/// Rust 核心寻路模块入口
#[pymodule]
fn smart_crane_core(m: &Bound<PyModule>) -> PyResult<()> {
    // 初始化日志桥接
    pyo3_log::init();

    m.add_class::<RustAStarPlanner>()?;
    m.add_class::<RustDLitePlanner>()?;
    m.add_class::<RustMapManager>()?;
    m.add_class::<RustPostProcessor>()?;
    // Components
    m.add_class::<RustSafetyGuard>()?;
    m.add_class::<RustGridAdapter>()?; // 注册
    Ok(())
}
