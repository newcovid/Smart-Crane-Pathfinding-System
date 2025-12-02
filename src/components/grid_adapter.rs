use crate::map::RustMapManager;
use log::debug;
use pyo3::prelude::*;
use pyo3::types::{PyList, PyTuple};
use std::time::Instant;

/// Rust 实现的高性能网格适配器。
///
/// 负责处理坐标转换、网格准备参数计算以及高效的增量网格比对。
#[pyclass]
pub struct RustGridAdapter;

#[pymethods]
impl RustGridAdapter {
    /// 计算网格增量变化 (Diff)。
    ///
    /// 仅遍历感兴趣区域 (ROI)，直接操作 Python 列表对象，避免全量数据拷贝。
    ///
    /// Args:
    ///     old_grid: 旧网格 (Python List)。
    ///     new_grid: 新网格 (Python List)。
    ///     map_mgr: 地图管理器引用。
    ///     roi_bounds: ROI 边界 (r_start, r_end, c_start, c_end, l_start, l_end)。
    ///     is_3d: 是否为 3D 网格。
    ///
    /// Returns:
    ///     List[Tuple]: 变化点列表。
    #[staticmethod]
    #[allow(clippy::too_many_arguments)]
    pub fn calculate_incremental_changes(
        old_grid: &Bound<PyAny>,
        new_grid: &Bound<PyAny>,
        _map_mgr: &RustMapManager,
        roi_bounds: (i32, i32, i32, i32, i32, i32),
        is_3d: bool,
    ) -> PyResult<Vec<PyObject>> {
        let start_time = Instant::now();
        let (r_start, r_end, c_start, c_end, l_start, l_end) = roi_bounds;
        let mut changes = Vec::new();

        // 尝试将输入转换为 PyList
        let new_rows = new_grid.downcast::<PyList>()?;

        // old_grid 可能是 None
        let old_rows_opt = if !old_grid.is_none() {
            Some(old_grid.downcast::<PyList>()?)
        } else {
            None
        };

        let result = Python::with_gil(|py| {
            for r in r_start..r_end {
                // 边界检查与安全获取行
                if r < 0 || (r as usize) >= new_rows.len() {
                    continue;
                }

                let new_row_obj = new_rows.get_item(r as usize)?;
                let new_row_list = new_row_obj.downcast::<PyList>()?;

                // [Fix] 使用 clone() 解决 lifetime 问题
                // downcast 返回引用，item 在 scope 结束销毁，必须 clone 增加引用计数
                let old_row_list_opt = if let Some(old_rows) = old_rows_opt {
                    if r >= 0 && (r as usize) < old_rows.len() {
                        let item = old_rows.get_item(r as usize)?;
                        Some(item.downcast::<PyList>()?.clone())
                    } else {
                        None
                    }
                } else {
                    None
                };

                for c in c_start..c_end {
                    if c < 0 || (c as usize) >= new_row_list.len() {
                        continue;
                    }

                    if !is_3d {
                        // === 2D 处理 ===
                        let val_new: i32 = new_row_list.get_item(c as usize)?.extract()?;
                        let mut val_old = 0;

                        // 必须使用 as_ref() 借用，因为 old_row_list_opt 是 Bound 类型
                        if let Some(old_row) = old_row_list_opt.as_ref() {
                            if c >= 0 && (c as usize) < old_row.len() {
                                val_old = old_row.get_item(c as usize)?.extract().unwrap_or(0);
                            }
                        }

                        if val_new != val_old {
                            // PyO3 0.23: 使用 PyTuple::new 构建
                            let tuple_obj = PyTuple::new(py, &[r, c, val_new])?;
                            changes.push(tuple_obj.into_any().unbind());
                        }
                    } else {
                        // === 3D 处理 ===
                        let new_layer_obj = new_row_list.get_item(c as usize)?;
                        let new_layer_list = new_layer_obj.downcast::<PyList>()?;

                        // [Fix] 同样使用 clone()
                        let old_layer_list_opt = if let Some(old_row) = old_row_list_opt.as_ref() {
                            if c >= 0 && (c as usize) < old_row.len() {
                                let item = old_row.get_item(c as usize)?;
                                Some(item.downcast::<PyList>()?.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        };

                        for l in l_start..l_end {
                            if l < 0 || (l as usize) >= new_layer_list.len() {
                                continue;
                            }

                            let val_new: i32 = new_layer_list.get_item(l as usize)?.extract()?;
                            let mut val_old = 0;

                            if let Some(old_layers) = old_layer_list_opt.as_ref() {
                                if l >= 0 && (l as usize) < old_layers.len() {
                                    val_old =
                                        old_layers.get_item(l as usize)?.extract().unwrap_or(0);
                                }
                            }

                            if val_new != val_old {
                                let tuple_obj = PyTuple::new(py, &[r, c, l, val_new])?;
                                changes.push(tuple_obj.into_any().unbind());
                            }
                        }
                    }
                }
            }
            Ok(changes)
        });

        // 记录耗时
        let duration = start_time.elapsed();
        if let Ok(ref chgs) = result {
            // 强制打印日志，供你分析
            debug!(
                "[Rust/GridAdapter] 增量Diff计算耗时: {:.2}ms (ROI: {}x{}x{}, Changes: {})",
                duration.as_secs_f64() * 1000.0,
                r_end - r_start,
                c_end - c_start,
                l_end - l_start,
                chgs.len()
            );
        }

        result
    }

    /// 计算起终点在算法网格中的坐标。
    #[staticmethod]
    pub fn get_initial_grid_nodes(
        start: (f32, f32, f32),
        end: (f32, f32, f32),
        map_mgr: &RustMapManager,
        settings_tuple: (bool, f32, f32, f32),
    ) -> PyResult<(PyObject, PyObject, f32)> {
        let (s_x, s_y, s_z) = start;
        let (e_x, e_y, e_z) = end;
        let (is_fixed_height, cruise_z_cfg, z_margin, min_offset) = settings_tuple;

        // 使用 with_gil 返回计算结果，解决变量初始化和可变性问题
        Python::with_gil(|py| -> PyResult<(PyObject, PyObject, f32)> {
            let start_node_obj;
            let end_node_obj;
            let final_cruise_z;

            if is_fixed_height {
                final_cruise_z = cruise_z_cfg;
                // 2.5D: 忽略 Z 轴，只取前两维 (Row, Col)
                let (r_s, c_s, _) = map_mgr.world_to_grid(s_x, s_y, 0.0);
                let (r_e, c_e, _) = map_mgr.world_to_grid(e_x, e_y, 0.0);

                // 使用 PyTuple::new 构建 Python 元组
                start_node_obj = PyTuple::new(py, &[r_s, c_s])?.into_any().unbind();
                end_node_obj = PyTuple::new(py, &[r_e, c_e])?.into_any().unbind();
            } else {
                // 3D: 高度修正与映射
                final_cruise_z = 0.0;
                let min_safe = z_margin + min_offset;
                let plan_s_z = s_z.max(min_safe);
                let plan_e_z = e_z.max(min_safe);

                let (r_s, c_s, l_s) = map_mgr.world_to_grid(s_x, s_y, plan_s_z);
                let (r_e, c_e, l_e) = map_mgr.world_to_grid(e_x, e_y, plan_e_z);

                start_node_obj = PyTuple::new(py, &[r_s, c_s, l_s])?.into_any().unbind();
                end_node_obj = PyTuple::new(py, &[r_e, c_e, l_e])?.into_any().unbind();
            }

            Ok((start_node_obj, end_node_obj, final_cruise_z))
        })
    }

    /// 智能网格转物理坐标。
    #[staticmethod]
    pub fn grid_to_world_smart(
        node: &Bound<PyAny>,
        override_z: f32,
        map_mgr: &RustMapManager,
    ) -> PyResult<(f32, f32, f32)> {
        // 尝试将输入对象转换为 Tuple
        let tuple = node.downcast::<PyTuple>()?;
        let len = tuple.len();

        if len == 2 {
            // 2D 网格坐标 (Row, Col) -> 物理坐标 (X, Y, OverrideZ)
            let r: i32 = tuple.get_item(0)?.extract()?;
            let c: i32 = tuple.get_item(1)?.extract()?;
            let (wx, wy, _) = map_mgr.grid_to_world(r, c, 0);
            Ok((wx, wy, override_z))
        } else {
            // 3D 网格坐标 (Row, Col, Layer) -> 物理坐标 (X, Y, Z)
            let r: i32 = tuple.get_item(0)?.extract()?;
            let c: i32 = tuple.get_item(1)?.extract()?;
            // 兼容性处理
            let l: i32 = if len > 2 {
                tuple.get_item(2)?.extract()?
            } else {
                0
            };
            Ok(map_mgr.grid_to_world(r, c, l))
        }
    }
}
