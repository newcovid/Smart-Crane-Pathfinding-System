use pyo3::prelude::*;

mod common;
mod pathfinding;

use pathfinding::astar::RustAStarPlanner;
use pathfinding::dslite::RustDLitePlanner;

/// Rust 核心寻路模块
#[pymodule]
fn smart_crane_core(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<RustAStarPlanner>()?;
    m.add_class::<RustDLitePlanner>()?;
    Ok(())
}
