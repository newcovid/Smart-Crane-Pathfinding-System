use crate::common::constants::{DEFAULT_Z_HIGH, OBSTACLE_COUNT_THRESHOLD};
use crate::map::Obstacle;
use log::{debug, info};
use std::f32;
use std::time::Instant;

/// 网格计算工厂 (Grid Computation Factory)。
///
/// 负责处理高密度的网格生成和数学运算，特别是涉及障碍物膨胀（Inflation）
/// 和 3D 体素化（Voxelization）的繁重计算任务。
/// 此类设计为无状态工具类，核心算法采用 Rust 原生实现以提升性能。
pub struct GridFactory;

impl GridFactory {
    // =========================================================================
    // 2D 网格生成 (2D Grid Generation)
    // =========================================================================

    pub fn create_inflated_grid_2d(
        rows: i32,
        cols: i32,
        resolution_m: f32,
        obstacles: &[&Obstacle],
        xy_margin: f32,
        check_z: Option<f32>,
        z_margin: f32,
    ) -> Vec<Vec<u8>> {
        let start_time = Instant::now();

        // 过滤有效障碍物
        let z_threshold = check_z.map(|z| z - z_margin);
        let active_obstacles: Vec<&Obstacle> = obstacles
            .iter()
            .filter(|&&o| {
                let obs_h = if o.z_m == 0.0 { DEFAULT_Z_HIGH } else { o.z_m };
                match z_threshold {
                    Some(th) => obs_h > th,
                    None => true,
                }
            })
            .map(|&o| o)
            .collect();

        let result = if active_obstacles.len() > OBSTACLE_COUNT_THRESHOLD {
            debug!(
                "[Rust/GridFactory] 障碍物数量 ({}) > 阈值，使用 EDT 算法优化。",
                active_obstacles.len()
            );
            Self::create_2d_via_edt(rows, cols, resolution_m, &active_obstacles, xy_margin)
        } else {
            Self::create_2d_via_painting(rows, cols, resolution_m, &active_obstacles, xy_margin)
        };

        let duration = start_time.elapsed();
        // [Fix] 移除阈值限制，强制打印耗时
        info!(
            "[Rust/GridFactory] 2D 网格生成耗时: {:.2}ms (Size: {}x{}, Obs: {})",
            duration.as_secs_f64() * 1000.0,
            rows,
            cols,
            active_obstacles.len()
        );

        result
    }

    // ... (create_2d_via_painting 和 create_2d_via_edt 保持不变) ...
    fn create_2d_via_painting(
        rows: i32,
        cols: i32,
        resolution_m: f32,
        obstacles: &[&Obstacle],
        xy_margin: f32,
    ) -> Vec<Vec<u8>> {
        let mut grid = vec![vec![0u8; cols as usize]; rows as usize];

        // `xy_margin` 的单位是【网格数】，与 Python 侧 grid_adapter 的
        // `xy_margin = radius_m / resolution_m` 及 grid_factory.py 的文档一致。
        // 此处的距离计算在物理米下进行，所以先换算成米；再补 0.5 格的离散补偿，
        // 与 Python 的 `dist_map <= xy_margin + 0.5` 和下面 EDT 分支的判据对齐。
        // （旧代码把 xy_margin 当米用，分辨率恰为 1.0 m 时两者数值相同，
        //   所以一直没暴露；分辨率 0.5 m 时膨胀量翻倍，2.0 m 时只剩一半——
        //   后者是往不安全的方向错。）
        let margin_m = (xy_margin + 0.5) * resolution_m;
        let xy_margin_sq = margin_m.powi(2);
        let margin_grid_cells = xy_margin.ceil() as i32 + 1;

        let threshold = xy_margin - 0.5;
        if threshold >= 0.0 {
            // 上界必须是 rows-1 / cols-1：循环同时索引 grid[r] 与 grid[rows-1-r]，
            // 一旦 border_cells 取到 rows，(rows-1-r) 会算出 -1，
            // 转成 usize 后回绕成巨大下标并 panic。
            let border_cells = (threshold.floor() as i32)
                .min(cols - 1)
                .min(rows - 1)
                .max(0);
            for r in 0..=border_cells {
                for c in 0..cols {
                    grid[r as usize][c as usize] = 1;
                    grid[(rows - 1 - r) as usize][c as usize] = 1;
                }
            }
            for c in 0..=border_cells {
                for r in 0..rows {
                    grid[r as usize][c as usize] = 1;
                    grid[r as usize][(cols - 1 - c) as usize] = 1;
                }
            }
        }

        for o in obstacles {
            let r_s = (o.y_m / resolution_m) as i32;
            let c_s = (o.x_m / resolution_m) as i32;
            let r_e = ((o.y_m + o.h_m - 0.01) / resolution_m) as i32;
            let c_e = ((o.x_m + o.w_m - 0.01) / resolution_m) as i32;

            let search_r_start = (r_s - margin_grid_cells).max(0);
            let search_r_end = (r_e + margin_grid_cells).min(rows - 1);
            let search_c_start = (c_s - margin_grid_cells).max(0);
            let search_c_end = (c_e + margin_grid_cells).min(cols - 1);

            for r in search_r_start..=search_r_end {
                for c in search_c_start..=search_c_end {
                    if grid[r as usize][c as usize] == 1 {
                        continue;
                    }
                    let px = (c as f32 + 0.5) * resolution_m;
                    let py = (r as f32 + 0.5) * resolution_m;
                    let closest_x = px.clamp(o.x_m, o.x_m + o.w_m);
                    let closest_y = py.clamp(o.y_m, o.y_m + o.h_m);
                    let dist_sq = (px - closest_x).powi(2) + (py - closest_y).powi(2);
                    if dist_sq <= xy_margin_sq {
                        grid[r as usize][c as usize] = 1;
                    }
                }
            }
        }
        grid
    }

    fn create_2d_via_edt(
        rows: i32,
        cols: i32,
        resolution_m: f32,
        obstacles: &[&Obstacle],
        xy_margin: f32,
    ) -> Vec<Vec<u8>> {
        let rows_usize = rows as usize;
        let cols_usize = cols as usize;

        let mut base_grid = vec![vec![false; cols_usize]; rows_usize];

        for o in obstacles {
            let r_s = ((o.y_m / resolution_m) as i32).max(0);
            let c_s = ((o.x_m / resolution_m) as i32).max(0);
            let r_e = (((o.y_m + o.h_m - 0.01) / resolution_m) as i32).min(rows - 1);
            let c_e = (((o.x_m + o.w_m - 0.01) / resolution_m) as i32).min(cols - 1);

            for r in r_s..=r_e {
                for c in c_s..=c_e {
                    base_grid[r as usize][c as usize] = true;
                }
            }
        }

        let dist_sq_map = Self::compute_edt_squared(&base_grid, rows_usize, cols_usize);
        // EDT 输出的距离单位就是【网格数】，而入参 xy_margin 也已经是网格数，
        // 不能再除以 resolution_m（旧代码多除了一次）。
        let margin_grid = xy_margin;
        let threshold_sq = (margin_grid + 0.5).powi(2);
        let mut final_grid = vec![vec![0u8; cols_usize]; rows_usize];

        for r in 0..rows_usize {
            for c in 0..cols_usize {
                let d_r = (r as f32 + 1.0).min((rows_usize - r) as f32);
                let d_c = (c as f32 + 1.0).min((cols_usize - c) as f32);
                let wall_dist_sq = d_r.min(d_c).powi(2);
                let final_dist_sq = dist_sq_map[r][c].min(wall_dist_sq);
                if final_dist_sq <= threshold_sq {
                    final_grid[r][c] = 1;
                }
            }
        }
        final_grid
    }

    // =========================================================================
    // 3D 网格生成 (3D Voxel Grid Generation)
    // =========================================================================

    pub fn create_3d_voxel_grid(
        rows: i32,
        cols: i32,
        layers: i32,
        height_m: f32,
        resolution_m: f32,
        obstacles: &[&Obstacle],
        xy_margin: f32,
        z_margin_obs: f32,
        z_margin_ceil: f32,
        is_infinite: bool,
    ) -> Vec<Vec<Vec<u8>>> {
        let start_time = Instant::now();

        // 1. 生成高度图 (Height Map)
        let height_map = Self::create_height_map_painting(
            rows,
            cols,
            resolution_m,
            obstacles,
            xy_margin,
            z_margin_obs,
            is_infinite,
            height_m,
        );

        // 2. 体素化 (Voxelization)
        let mut grid_3d = vec![vec![vec![0u8; layers as usize]; cols as usize]; rows as usize];

        for r in 0..rows {
            for c in 0..cols {
                let obstacle_h = height_map[r as usize][c as usize];

                for l in 0..layers {
                    let z_center = (l as f32 + 0.5) * resolution_m;
                    let is_obstacle = z_center < obstacle_h;
                    let is_ceiling_hit = (z_center + z_margin_ceil) > height_m;
                    let is_floor_hit = (z_center - z_margin_obs) < 0.0;

                    if is_obstacle || is_ceiling_hit || is_floor_hit {
                        grid_3d[r as usize][c as usize][l as usize] = 1;
                    }
                }
            }
        }

        let duration = start_time.elapsed();
        // [Fix] 强制打印耗时
        info!(
            "[Rust/GridFactory] 3D 体素生成耗时: {:.2}ms",
            duration.as_secs_f64() * 1000.0
        );

        grid_3d
    }

    // ... (create_height_map_painting, compute_edt_squared, compute_1d_parabolic_lower_envelope 保持不变) ...
    fn create_height_map_painting(
        rows: i32,
        cols: i32,
        resolution_m: f32,
        obstacles: &[&Obstacle],
        xy_margin: f32,
        z_margin_obs: f32,
        is_infinite: bool,
        map_height_m: f32,
    ) -> Vec<Vec<f32>> {
        let mut height_map = vec![vec![0.0f32; cols as usize]; rows as usize];

        // 单位约定同 create_2d_via_painting：xy_margin 为【网格数】。
        let margin_m = (xy_margin + 0.5) * resolution_m;
        let xy_margin_sq = margin_m.powi(2);
        let margin_grid_cells = xy_margin.ceil() as i32 + 1;

        let threshold = xy_margin - 0.5;
        if threshold >= 0.0 {
            // 上界取 rows-1 / cols-1，避免 (rows - 1 - r) 出现 -1 后转 usize 回绕。
            let border_cells = (threshold.floor() as i32)
                .min(cols - 1)
                .min(rows - 1)
                .max(0);
            let infinite_h = map_height_m + 10.0;
            for r in 0..=border_cells {
                for c in 0..cols {
                    height_map[r as usize][c as usize] = infinite_h;
                    height_map[(rows - 1 - r) as usize][c as usize] = infinite_h;
                }
            }
            for c in 0..=border_cells {
                for r in 0..rows {
                    height_map[r as usize][c as usize] = infinite_h;
                    height_map[r as usize][(cols - 1 - c) as usize] = infinite_h;
                }
            }
        }

        for o in obstacles {
            let obs_occupy_z = if is_infinite {
                map_height_m + 1.0
            } else {
                let z = if o.z_m == 0.0 { DEFAULT_Z_HIGH } else { o.z_m };
                z + z_margin_obs
            };

            let r_s = (o.y_m / resolution_m) as i32;
            let c_s = (o.x_m / resolution_m) as i32;
            let r_e = ((o.y_m + o.h_m - 0.01) / resolution_m) as i32;
            let c_e = ((o.x_m + o.w_m - 0.01) / resolution_m) as i32;

            let search_r_start = (r_s - margin_grid_cells).max(0);
            let search_r_end = (r_e + margin_grid_cells).min(rows - 1);
            let search_c_start = (c_s - margin_grid_cells).max(0);
            let search_c_end = (c_e + margin_grid_cells).min(cols - 1);

            for r in search_r_start..=search_r_end {
                for c in search_c_start..=search_c_end {
                    let px = (c as f32 + 0.5) * resolution_m;
                    let py = (r as f32 + 0.5) * resolution_m;
                    let closest_x = px.clamp(o.x_m, o.x_m + o.w_m);
                    let closest_y = py.clamp(o.y_m, o.y_m + o.h_m);
                    let dist_sq = (px - closest_x).powi(2) + (py - closest_y).powi(2);
                    if dist_sq <= xy_margin_sq {
                        height_map[r as usize][c as usize] =
                            height_map[r as usize][c as usize].max(obs_occupy_z);
                    }
                }
            }
        }
        height_map
    }

    fn compute_edt_squared(grid: &Vec<Vec<bool>>, rows: usize, cols: usize) -> Vec<Vec<f32>> {
        let inf = 1e9_f32;
        let mut g = vec![vec![inf; cols]; rows];
        for r in 0..rows {
            let mut last_obstacle_col = -1i32;
            for c in 0..cols {
                if grid[r][c] {
                    last_obstacle_col = c as i32;
                    g[r][c] = 0.0;
                } else if last_obstacle_col != -1 {
                    g[r][c] = (c as i32 - last_obstacle_col).pow(2) as f32;
                }
            }
            last_obstacle_col = -1;
            for c in (0..cols).rev() {
                if grid[r][c] {
                    last_obstacle_col = c as i32;
                } else if last_obstacle_col != -1 {
                    let dist = (last_obstacle_col - c as i32).pow(2) as f32;
                    if dist < g[r][c] {
                        g[r][c] = dist;
                    }
                }
            }
        }
        let mut dt = vec![vec![0.0; cols]; rows];
        for c in 0..cols {
            let col_g: Vec<f32> = (0..rows).map(|r| g[r][c]).collect();
            let col_dt = Self::compute_1d_parabolic_lower_envelope(&col_g);
            for r in 0..rows {
                dt[r][c] = col_dt[r];
            }
        }
        dt
    }

    fn compute_1d_parabolic_lower_envelope(input: &[f32]) -> Vec<f32> {
        let n = input.len();
        let mut output = vec![0.0; n];
        for i in 0..n {
            let mut min_val = input[i];
            for k in (0..i).rev() {
                let dist_sq = ((i - k) as f32).powi(2);
                if dist_sq >= min_val {
                    break;
                }
                let val = input[k] + dist_sq;
                if val < min_val {
                    min_val = val;
                }
            }
            for k in (i + 1)..n {
                let dist_sq = ((k - i) as f32).powi(2);
                if dist_sq >= min_val {
                    break;
                }
                let val = input[k] + dist_sq;
                if val < min_val {
                    min_val = val;
                }
            }
            output[i] = min_val;
        }
        output
    }
}
