use log::debug;
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyList, PyTuple};
use std::time::Instant;

/// 通用节点坐标 (x, y, z)
#[derive(Clone, Debug, Eq, Hash, PartialEq, Copy, PartialOrd, Ord)]
pub struct Node {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Node {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    pub fn to_tuple(&self, is_3d: bool) -> PyObject {
        Python::with_gil(|py| {
            if is_3d {
                let elements = [self.x, self.y, self.z];
                PyTuple::new(py, elements).unwrap().into_any().unbind()
            } else {
                let elements = [self.x, self.y];
                PyTuple::new(py, elements).unwrap().into_any().unbind()
            }
        })
    }
}

/// 展平的网格数据结构
#[derive(Clone)]
pub struct FlatGrid {
    pub rows: i32,
    pub cols: i32,
    pub layers: i32,
    pub data: Vec<u8>, // 0=Empty, 1=Obstacle
}

impl FlatGrid {
    #[inline(always)]
    pub fn is_valid(&self, n: &Node) -> bool {
        n.x >= 0 && n.x < self.rows && n.y >= 0 && n.y < self.cols && n.z >= 0 && n.z < self.layers
    }

    #[inline(always)]
    pub fn is_obstacle_unsafe(&self, n: &Node) -> bool {
        let idx = (n.x as usize * self.cols as usize * self.layers as usize)
            + (n.y as usize * self.layers as usize)
            + (n.z as usize);
        if idx >= self.data.len() {
            return true;
        }
        self.data[idx] == 1
    }

    #[inline(always)]
    pub fn is_safe(&self, n: &Node) -> bool {
        if !self.is_valid(n) {
            return false;
        }
        !self.is_obstacle_unsafe(n)
    }
}

/// 解析 Python 传入的网格对象。
///
/// 兼容以下格式：
/// 1. 原生 Python List: `List[List[int]]` (2D) 或 `List[List[List[int]]]` (3D)
/// 2. Rust 生成的 Bytes: `List[bytes]` (2D) 或 `List[List[bytes]]` (3D)
pub fn parse_python_grid(py_grid: &Bound<PyAny>) -> PyResult<FlatGrid> {
    let start_time = Instant::now(); // 开始计时

    let rows_list = py_grid.downcast::<PyList>()?;
    let rows = rows_list.len() as i32;
    if rows == 0 {
        return Ok(FlatGrid {
            rows: 0,
            cols: 0,
            layers: 0,
            data: vec![],
        });
    }

    // 探测第一行的数据类型
    let first_row_obj = rows_list.get_item(0)?;

    // --- Case A: 2D Grid with Bytes (List[bytes]) ---
    if let Ok(first_bytes) = first_row_obj.downcast::<PyBytes>() {
        let cols = first_bytes.len()? as i32;
        let layers = 1;
        let total_size = (rows * cols) as usize;
        let mut data = Vec::with_capacity(total_size);

        for r in 0..rows {
            let row_obj = rows_list.get_item(r as usize)?;
            let row_bytes = row_obj.downcast::<PyBytes>()?;
            data.extend_from_slice(row_bytes.as_bytes());
        }

        let duration = start_time.elapsed();
        debug!(
            "[Rust/Common] 解析Python网格(2D Bytes)耗时: {:.2}ms",
            duration.as_secs_f64() * 1000.0
        );

        return Ok(FlatGrid {
            rows,
            cols,
            layers,
            data,
        });
    }

    let first_row_list = first_row_obj.downcast::<PyList>()?;
    let cols = first_row_list.len() as i32;

    if cols == 0 {
        return Ok(FlatGrid {
            rows,
            cols,
            layers: 0,
            data: vec![],
        });
    }

    let first_col_obj = first_row_list.get_item(0)?;

    // --- Case B: 3D Grid with Bytes (List[List[bytes]]) ---
    if let Ok(layer_bytes) = first_col_obj.downcast::<PyBytes>() {
        let layers = layer_bytes.len()? as i32;
        let total_size = (rows * cols * layers) as usize;
        let mut data = Vec::with_capacity(total_size);

        for r in 0..rows {
            let row_obj = rows_list.get_item(r as usize)?;
            let row_list = row_obj.downcast::<PyList>()?;
            for c in 0..cols {
                let col_obj = row_list.get_item(c as usize)?;
                let col_bytes = col_obj.downcast::<PyBytes>()?;
                data.extend_from_slice(col_bytes.as_bytes());
            }
        }

        let duration = start_time.elapsed();
        debug!(
            "[Rust/Common] 解析Python网格(3D Bytes)耗时: {:.2}ms",
            duration.as_secs_f64() * 1000.0
        );

        return Ok(FlatGrid {
            rows,
            cols,
            layers,
            data,
        });
    }

    // --- Case C: Standard Python List ---
    let is_3d_list = first_col_obj.is_instance_of::<PyList>();
    let layers = if is_3d_list {
        first_col_obj.downcast::<PyList>()?.len() as i32
    } else {
        1
    };

    let total_size = (rows * cols * layers) as usize;
    let mut data = Vec::with_capacity(total_size);

    for r in 0..rows {
        let row_obj = rows_list.get_item(r as usize)?;
        let row_list = row_obj.downcast::<PyList>()?;
        for c in 0..cols {
            let col_item = row_list.get_item(c as usize)?;
            if is_3d_list {
                let layer_list = col_item.downcast::<PyList>()?;
                for l in 0..layers {
                    let val: u8 = layer_list.get_item(l as usize)?.extract()?;
                    data.push(val);
                }
            } else {
                let val: u8 = col_item.extract()?;
                data.push(val);
            }
        }
    }

    let duration = start_time.elapsed();
    debug!(
        "[Rust/Common] 解析Python网格(Native List)耗时: {:.2}ms (Items: {})",
        duration.as_secs_f64() * 1000.0,
        total_size
    );

    Ok(FlatGrid {
        rows,
        cols,
        layers,
        data,
    })
}
