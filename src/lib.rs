use pyo3::prelude::*;

mod common;
// 注册 map 模块
mod components;
mod map;
mod pathfinding;
mod post_processing; // 新增模块

use components::safety_guard::RustSafetyGuard; // 引入新类
use map::RustMapManager;
use pathfinding::astar::RustAStarPlanner;
use pathfinding::dslite::RustDLitePlanner;
use post_processing::RustPostProcessor;

/// Rust 核心寻路模块入口
#[pymodule]
fn smart_crane_core(m: &Bound<PyModule>) -> PyResult<()> {
    // 初始化日志桥接
    // 这会将 Rust 的 log::* 宏转发给 Python 的 logging 模块
    // 必须在模块加载时第一时间调用
    pyo3_log::init();

    m.add_class::<RustAStarPlanner>()?;
    m.add_class::<RustDLitePlanner>()?;
    // 导出地图管理器
    m.add_class::<RustMapManager>()?;
    // 导出后处理器
    m.add_class::<RustPostProcessor>()?;
    // Components
    m.add_class::<RustSafetyGuard>()?;
    Ok(())
}
