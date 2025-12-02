pub mod grid_factory;
pub mod manager;

pub use manager::RustMapManager;

// 定义障碍物结构体，供 manager 和 grid_factory 共享使用
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub x_m: f32,
    pub y_m: f32,
    pub w_m: f32,
    pub h_m: f32,
    pub z_m: f32,
}
