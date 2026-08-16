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
    // [Fix] 将字段设为 pub 以便 components 模块访问
    pub width_m: f32,
    pub length_m: f32,
    pub height_m: f32,
    pub resolution_m: f32,

    // --- 网格属性 ---
    pub rows: i32,
    pub cols: i32,
    pub layers: i32,

    // --- 障碍物数据库 ---
    // 保持私有，通过方法操作
    static_obstacles: HashMap<String, Obstacle>,
    dynamic_obstacles: HashMap<String, Obstacle>,

    // --- 静态层膨胀缓存 ---
    //
    // 膨胀对障碍物集合可分配：dist(A∪B) = min(dist(A), dist(B))，
    // 故 inflate(A∪B) == inflate(A) OR inflate(B)。据此把静态部分的膨胀
    // 结果缓存下来，动态障碍物只需在其副本上叠加，无需每次重算全图。
    //
    // 意义在于障碍物数量越过 OBSTACLE_COUNT_THRESHOLD 之后会切换到
    // 复杂度 O(网格面积) 的全局 EDT：此时静态部分正是开销的主要来源，
    // 而它在两次动态变更之间根本没有变化。
    static_cache: Option<StaticGridCache>,
}

/// 静态障碍物膨胀结果的单条缓存。
///
/// 参数按千分位量化后比较，避免浮点直接判等。
struct StaticGridCache {
    xy_margin_q: i64,
    z_margin_q: i64,
    check_z_q: Option<i64>,
    grid: Vec<Vec<u8>>,
}

#[inline]
fn quantize(v: f32) -> i64 {
    (v as f64 * 1000.0).round() as i64
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
            static_cache: None,
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
        self.static_cache = None; // 静态层变了，缓存作废
        debug!("[RustMap] 更新静态障碍: {}", oid);
    }

    pub fn remove_static_obstacle(&mut self, oid: String) {
        if self.static_obstacles.remove(&oid).is_some() {
            self.static_cache = None; // 静态层变了，缓存作废
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

            // 注意：Rust 这里使用严格小于 (<)。
            // 如果 xy_margin 为 0 (硬碰撞检测) 且点在内部 (dist_sq 为 0)，0 < 0 为 False。
            // 因此，调用者如果想检测硬碰撞，必须传入一个微小的 xy_margin (如 1e-4)。
            // 对于安全脱困 (Smart Escape) 场景，xy_margin 包含了安全半径，远大于 0，所以没有问题。
            if dist_sq < xy_margin_sq {
                if ignore_z {
                    return true;
                }
                let obs_z = o.z_m;
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
        &mut self,
        xy_margin: f32,
        z_margin: f32,
        check_z: Option<f32>,
    ) -> Vec<Vec<u8>> {
        let (xq, zq, cq) = (
            quantize(xy_margin),
            quantize(z_margin),
            check_z.map(quantize),
        );

        // 1. 静态层：命中缓存则直接复用，否则只对静态障碍物做一次膨胀
        let hit = matches!(
            &self.static_cache,
            Some(c) if c.xy_margin_q == xq && c.z_margin_q == zq && c.check_z_q == cq
        );
        if !hit {
            let static_obs: Vec<&Obstacle> = self.static_obstacles.values().collect();
            let grid = GridFactory::create_inflated_grid_2d(
                self.rows,
                self.cols,
                self.resolution_m,
                &static_obs,
                xy_margin,
                check_z,
                z_margin,
            );
            debug!(
                "[RustMap] 静态层膨胀已重算并缓存（{} 个静态障碍）",
                static_obs.len()
            );
            self.static_cache = Some(StaticGridCache {
                xy_margin_q: xq,
                z_margin_q: zq,
                check_z_q: cq,
                grid,
            });
        }

        let static_grid = &self.static_cache.as_ref().unwrap().grid;

        // 无动态障碍物时，静态层就是最终结果
        if self.dynamic_obstacles.is_empty() {
            return static_grid.clone();
        }

        // 2. 动态层：单独膨胀后与静态层求并集
        //    依据 inflate(A∪B) == inflate(A) OR inflate(B)。
        //    动态障碍物通常只有个位数，走的是廉价的逐障碍绘制分支。
        let dyn_obs: Vec<&Obstacle> = self.dynamic_obstacles.values().collect();
        let dyn_grid = GridFactory::create_inflated_grid_2d(
            self.rows,
            self.cols,
            self.resolution_m,
            &dyn_obs,
            xy_margin,
            check_z,
            z_margin,
        );

        let mut out = static_grid.clone();
        for (row_out, row_dyn) in out.iter_mut().zip(dyn_grid.iter()) {
            for (cell, d) in row_out.iter_mut().zip(row_dyn.iter()) {
                *cell |= *d;
            }
        }
        out
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
