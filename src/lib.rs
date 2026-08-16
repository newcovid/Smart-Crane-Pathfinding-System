use pyo3::prelude::*;

mod common;
mod components;
mod map;
mod pathfinding;
mod post_processing;

use crate::components::grid_adapter::RustGridAdapter;
use crate::components::safety_guard::RustSafetyGuard;
use crate::map::RustMapManager;
use crate::pathfinding::astar::RustAStarPlanner;
use crate::pathfinding::dslite::RustDLitePlanner;
use crate::post_processing::RustPostProcessor;

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
    m.add_class::<RustGridAdapter>()?;
    Ok(())
}
