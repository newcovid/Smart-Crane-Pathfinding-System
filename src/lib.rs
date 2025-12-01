use pyo3::prelude::*;

mod common;
mod pathfinding;

use pathfinding::astar::RustAStarPlanner;
use pathfinding::dslite::RustDLitePlanner;

/// Rust 核心寻路模块入口
#[pymodule]
fn smart_crane_core(m: &Bound<PyModule>) -> PyResult<()> {
    // 初始化日志桥接
    // 这会将 Rust 的 log::* 宏转发给 Python 的 logging 模块
    // 必须在模块加载时第一时间调用
    pyo3_log::init();

    m.add_class::<RustAStarPlanner>()?;
    m.add_class::<RustDLitePlanner>()?;
    Ok(())
}
