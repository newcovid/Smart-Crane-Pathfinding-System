use crate::common::constants::{DEFAULT_Z_HIGH, OBSTACLE_COUNT_THRESHOLD};
use crate::map::Obstacle;
use log::debug;
use std::f32;

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

    /// 获取带膨胀的 2D 投影网格。
    ///
    /// 根据障碍物数量自动选择算法：
    /// 1. 数量少时：使用几何绘制法 (Geometric Painting)。
    /// 2. 数量多时：使用欧几里得距离变换 (EDT)。
    pub fn create_inflated_grid_2d(
        rows: i32,
        cols: i32,
        resolution_m: f32,
        obstacles: &[&Obstacle],
        xy_margin: f32,
        check_z: Option<f32>,
        z_margin: f32,
    ) -> Vec<Vec<u8>> {
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

        // 策略分支
        if active_obstacles.len() > OBSTACLE_COUNT_THRESHOLD {
            debug!(
                "[GridFactory] 障碍物数量 ({}) > 阈值，使用 EDT 算法优化。",
                active_obstacles.len()
            );
            Self::create_2d_via_edt(rows, cols, resolution_m, &active_obstacles, xy_margin)
        } else {
            Self::create_2d_via_painting(rows, cols, resolution_m, &active_obstacles, xy_margin)
        }
    }

    /// [算法A] 几何绘制法：遍历障碍物并在网格上“画”出膨胀区域。
    /// 修复：增加了对地图四面墙壁的膨胀处理。
    fn create_2d_via_painting(
        rows: i32,
        cols: i32,
        resolution_m: f32,
        obstacles: &[&Obstacle],
        xy_margin: f32,
    ) -> Vec<Vec<u8>> {
        let mut grid = vec![vec![0u8; cols as usize]; rows as usize];
        let xy_margin_sq = xy_margin.powi(2);
        let margin_grid_cells = (xy_margin / resolution_m).ceil() as i32 + 1;

        // [Fix] 1. 先绘制四周墙壁的膨胀层
        // 计算受墙壁影响的网格范围。
        // 原理：Python 的 pad_width=1 相当于在 index -1 处有墙。
        // 距离公式匹配 EDT：dist <= xy_margin + 0.5
        let threshold = xy_margin - 0.5;
        if threshold >= 0.0 {
            // 需要填充的层数
            let border_cells = (threshold.floor() as i32).min(cols).min(rows);

            // 上下边界 (Y轴)
            for r in 0..=border_cells {
                for c in 0..cols {
                    grid[r as usize][c as usize] = 1; // Bottom
                    grid[(rows - 1 - r) as usize][c as usize] = 1; // Top
                }
            }
            // 左右边界 (X轴)
            for c in 0..=border_cells {
                for r in 0..rows {
                    grid[r as usize][c as usize] = 1; // Left
                    grid[r as usize][(cols - 1 - c) as usize] = 1; // Right
                }
            }
        }

        // 2. 绘制障碍物
        for o in obstacles {
            // 计算原始包围盒索引
            let r_s = (o.y_m / resolution_m) as i32;
            let c_s = (o.x_m / resolution_m) as i32;
            let r_e = ((o.y_m + o.h_m - 0.01) / resolution_m) as i32;
            let c_e = ((o.x_m + o.w_m - 0.01) / resolution_m) as i32;

            // 确定搜索范围 (Bound Clamp)
            let search_r_start = (r_s - margin_grid_cells).max(0);
            let search_r_end = (r_e + margin_grid_cells).min(rows - 1);
            let search_c_start = (c_s - margin_grid_cells).max(0);
            let search_c_end = (c_e + margin_grid_cells).min(cols - 1);

            // 像素级遍历
            for r in search_r_start..=search_r_end {
                for c in search_c_start..=search_c_end {
                    // 如果已经标记，跳过
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

    /// [算法B] EDT 法：生成基础二值图 -> 计算距离场 -> 阈值截断。
    /// 修复：在距离计算阶段融入了到墙壁的距离。
    fn create_2d_via_edt(
        rows: i32,
        cols: i32,
        resolution_m: f32,
        obstacles: &[&Obstacle],
        xy_margin: f32,
    ) -> Vec<Vec<u8>> {
        let rows_usize = rows as usize;
        let cols_usize = cols as usize;

        // 1. 生成基础二值图 (Binary Grid)
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

        // 2. 计算距离变换 (Squared EDT) - 仅针对内部障碍物
        let dist_sq_map = Self::compute_edt_squared(&base_grid, rows_usize, cols_usize);

        // 3. 应用膨胀阈值 (并合并墙壁距离)
        let margin_grid = xy_margin / resolution_m;
        // 增加 0.5 的容差以匹配 Python behavior (pad_width=1, distance from center to padded wall)
        let threshold_sq = (margin_grid + 0.5).powi(2);

        let mut final_grid = vec![vec![0u8; cols_usize]; rows_usize];

        for r in 0..rows_usize {
            for c in 0..cols_usize {
                // 计算到四面墙壁的最近距离 (Grid Units)
                // 墙壁位置相当于在 -1 和 Size 处
                // r+1 是到下墙(-1)的距离, rows-r 是到上墙(Size)的距离
                let d_r = (r as f32 + 1.0).min((rows_usize - r) as f32);
                let d_c = (c as f32 + 1.0).min((cols_usize - c) as f32);
                let wall_dist_sq = d_r.min(d_c).powi(2);

                // 最终距离是 内部障碍物距离 和 墙壁距离 的最小值
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

    /// 获取 3D 体素网格。
    /// 修复：在高度图中填充了四周墙壁为无限高。
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

        grid_3d
    }

    /// 高度图生成：绘制法
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
        let xy_margin_sq = xy_margin.powi(2);
        let margin_grid_cells = (xy_margin / resolution_m).ceil() as i32 + 1;

        // [Fix] 1. 填充墙壁为无限高
        let threshold = xy_margin - 0.5;
        if threshold >= 0.0 {
            let border_cells = (threshold.floor() as i32).min(cols).min(rows);
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

        // 2. 绘制障碍物
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

    // =========================================================================
    // 内部算法实现 (Internal Algorithms)
    // =========================================================================

    /// 计算二维欧几里得距离变换的平方 (Squared EDT)。
    ///
    /// 实现了 Meijster 算法的简化版 (Separable transform)。
    fn compute_edt_squared(grid: &Vec<Vec<bool>>, rows: usize, cols: usize) -> Vec<Vec<f32>> {
        let inf = 1e9_f32;

        // g[r][c] 存储点 (r,c) 到当前行最近障碍物的距离平方
        let mut g = vec![vec![inf; cols]; rows];

        // 1. Pass 1: Horizontal Scan
        for r in 0..rows {
            // Forward pass
            let mut last_obstacle_col = -1i32;
            for c in 0..cols {
                if grid[r][c] {
                    last_obstacle_col = c as i32;
                    g[r][c] = 0.0;
                } else if last_obstacle_col != -1 {
                    g[r][c] = (c as i32 - last_obstacle_col).pow(2) as f32;
                }
            }

            // Backward pass
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

        // 2. Pass 2: Vertical Scan
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

    /// 计算 1D 数组的下包络线距离变换。
    fn compute_1d_parabolic_lower_envelope(input: &[f32]) -> Vec<f32> {
        let n = input.len();
        let mut output = vec![0.0; n];

        for i in 0..n {
            let mut min_val = input[i];

            // 向前搜索
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
            // 向后搜索
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
