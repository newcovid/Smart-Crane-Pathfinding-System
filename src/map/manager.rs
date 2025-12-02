use crate::common::constants::DEFAULT_Z_HIGH;
use crate::map::grid_factory::GridFactory;
use crate::map::Obstacle;
use log::{debug, info};
use pyo3::prelude::*;
use std::collections::HashMap;

/// Rust 实现的高性能车间地图管理器。
///
/// 负责维护车间的物理属性、障碍物数据（静态/动态）以及坐标转换逻辑。
/// 它整合了 Python 版 `WorkshopMapManager` 的功能，并将网格生成逻辑委托给 `GridFactory`。
#[pyclass]
pub struct RustMapManager {
    // --- 物理属性 ---
    width_m: f32,
    length_m: f32,
    height_m: f32,
    resolution_m: f32,

    // --- 网格属性 ---
    rows: i32,
    cols: i32,
    layers: i32,

    // --- 障碍物数据库 ---
    static_obstacles: HashMap<String, Obstacle>,
    dynamic_obstacles: HashMap<String, Obstacle>,
}

#[pymethods]
impl RustMapManager {
    #[new]
    #[pyo3(signature = (width_m, length_m, height_m, resolution_m))]
    pub fn new(width_m: f32, length_m: f32, height_m: f32, resolution_m: f32) -> Self {
        let cols = (width_m / resolution_m).ceil() as i32;
        let rows = (length_m / resolution_m).ceil() as i32;
        let layers = (height_m / resolution_m).ceil() as i32;

        info!(
            "[RustMap] 初始化完成. 尺寸: {}x{}x{}, 分辨率: {:.2}m",
            cols, rows, layers, resolution_m
        );

        Self {
            width_m,
            length_m,
            height_m,
            resolution_m,
            rows,
            cols,
            layers,
            static_obstacles: HashMap::new(),
            dynamic_obstacles: HashMap::new(),
        }
    }

    // =========================================================================
    // 障碍物管理 (Obstacle Management)
    // =========================================================================

    pub fn add_static_obstacle(&mut self, oid: String, x: f32, y: f32, w: f32, h: f32, z: f32) {
        let obs = Obstacle {
            x_m: x,
            y_m: y,
            w_m: w,
            h_m: h,
            z_m: z,
        };
        self.static_obstacles.insert(oid.clone(), obs);
        debug!("[RustMap] 更新静态障碍: {}", oid);
    }

    pub fn remove_static_obstacle(&mut self, oid: String) {
        if self.static_obstacles.remove(&oid).is_some() {
            debug!("[RustMap] 移除静态障碍: {}", oid);
        }
    }

    pub fn update_dynamic_obstacle(&mut self, oid: String, x: f32, y: f32, w: f32, h: f32, z: f32) {
        let obs = Obstacle {
            x_m: x,
            y_m: y,
            w_m: w,
            h_m: h,
            z_m: z,
        };
        self.dynamic_obstacles.insert(oid, obs);
    }

    pub fn remove_dynamic_obstacle(&mut self, oid: String) {
        self.dynamic_obstacles.remove(&oid);
    }

    pub fn clear_dynamic_obstacles(&mut self) {
        self.dynamic_obstacles.clear();
    }

    // =========================================================================
    // 坐标转换 (Coordinate Conversion)
    // =========================================================================

    pub fn world_to_grid(&self, x_m: f32, y_m: f32, z_m: f32) -> (i32, i32, i32) {
        let col = (x_m / self.resolution_m) as i32;
        let row = (y_m / self.resolution_m) as i32;
        let layer = (z_m / self.resolution_m) as i32;

        (
            row.clamp(0, self.rows - 1),
            col.clamp(0, self.cols - 1),
            layer.clamp(0, self.layers - 1),
        )
    }

    pub fn grid_to_world(&self, row: i32, col: i32, layer: i32) -> (f32, f32, f32) {
        let x = (col as f32 + 0.5) * self.resolution_m;
        let y = (row as f32 + 0.5) * self.resolution_m;
        let z = (layer as f32 + 0.5) * self.resolution_m;
        (x, y, z)
    }

    // =========================================================================
    // 碰撞检测 (Collision Detection)
    // =========================================================================

    pub fn check_collision_raw(
        &self,
        x: f32,
        y: f32,
        z: f32,
        xy_margin: f32,
        z_margin: f32,
        ignore_z: bool,
    ) -> bool {
        if x - xy_margin < 0.0 || x + xy_margin > self.width_m {
            return true;
        }
        if y - xy_margin < 0.0 || y + xy_margin > self.length_m {
            return true;
        }

        let xy_margin_sq = xy_margin * xy_margin;

        let iter = self
            .static_obstacles
            .values()
            .chain(self.dynamic_obstacles.values());

        for o in iter {
            let closest_x = x.clamp(o.x_m, o.x_m + o.w_m);
            let closest_y = y.clamp(o.y_m, o.y_m + o.h_m);
            let dist_sq = (x - closest_x).powi(2) + (y - closest_y).powi(2);

            if dist_sq < xy_margin_sq {
                if ignore_z {
                    return true;
                }
                let obs_z = if o.z_m == 0.0 { DEFAULT_Z_HIGH } else { o.z_m };
                if obs_z >= z - z_margin {
                    return true;
                }
            }
        }
        false
    }

    // =========================================================================
    // 网格生成 (Delegate to GridFactory)
    // =========================================================================

    /// 获取带膨胀的 2D 投影网格。
    ///
    /// [API 调整] 为了符合 Python 语法 (必选参数在前，可选参数在后)，
    /// 这里将 Rust 接口参数顺序调整为 `(xy_margin, z_margin, check_z)`.
    /// `check_z` 默认为 `None`。
    #[pyo3(signature = (xy_margin, z_margin, check_z=None))]
    pub fn get_2d_projection_grid(
        &self,
        xy_margin: f32,
        z_margin: f32,
        check_z: Option<f32>,
    ) -> Vec<Vec<u8>> {
        // 收集所有障碍物的引用
        let obstacles: Vec<&Obstacle> = self
            .static_obstacles
            .values()
            .chain(self.dynamic_obstacles.values())
            .collect();

        // 委托给 GridFactory 处理
        // 注意：GridFactory 保持原内部逻辑参数顺序，这里做适配
        GridFactory::create_inflated_grid_2d(
            self.rows,
            self.cols,
            self.resolution_m,
            &obstacles,
            xy_margin,
            check_z,
            z_margin,
        )
    }

    /// 获取 3D 体素网格。
    pub fn get_3d_voxel_grid(
        &self,
        xy_margin: f32,
        z_margin_obs: f32,
        z_margin_ceil: f32,
        is_infinite: bool,
    ) -> Vec<Vec<Vec<u8>>> {
        let obstacles: Vec<&Obstacle> = self
            .static_obstacles
            .values()
            .chain(self.dynamic_obstacles.values())
            .collect();

        GridFactory::create_3d_voxel_grid(
            self.rows,
            self.cols,
            self.layers,
            self.height_m,
            self.resolution_m,
            &obstacles,
            xy_margin,
            z_margin_obs,
            z_margin_ceil,
            is_infinite,
        )
    }
}
