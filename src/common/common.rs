use pyo3::prelude::*;
use pyo3::types::PyTuple;

/// 通用节点坐标 (x, y, z)
#[derive(Clone, Debug, Eq, Hash, PartialEq, Copy)]
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
                // [修复] PyO3 0.23: PyTuple::new_bound -> PyTuple::new
                let elements = [self.x, self.y, self.z];
                // PyTuple::new 现在直接返回 Bound<'py, PyTuple>
                // 如果编译器提示需要 unwrap，请根据提示添加，但在大多数 0.23 版本中它直接返回 Bound
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

pub fn parse_python_grid(py_grid: &Bound<PyAny>) -> PyResult<FlatGrid> {
    let rows_list = py_grid.downcast::<pyo3::types::PyList>()?;
    let rows = rows_list.len() as i32;
    if rows == 0 {
        return Ok(FlatGrid {
            rows: 0,
            cols: 0,
            layers: 0,
            data: vec![],
        });
    }

    let first_row = rows_list
        .get_item(0)?
        .downcast::<pyo3::types::PyList>()?
        .clone();
    let cols = first_row.len() as i32;

    let is_3d = if cols > 0 {
        first_row
            .get_item(0)?
            .is_instance_of::<pyo3::types::PyList>()
    } else {
        false
    };

    let layers = if is_3d {
        first_row
            .get_item(0)?
            .downcast::<pyo3::types::PyList>()?
            .len() as i32
    } else {
        1
    };

    let total_size = (rows * cols * layers) as usize;
    let mut data = Vec::with_capacity(total_size);

    for r in 0..rows {
        let row_obj = rows_list.get_item(r as usize)?;
        let row_list = row_obj.downcast::<pyo3::types::PyList>()?;
        for c in 0..cols {
            let col_item = row_list.get_item(c as usize)?;
            if is_3d {
                let layer_list = col_item.downcast::<pyo3::types::PyList>()?;
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

    Ok(FlatGrid {
        rows,
        cols,
        layers,
        data,
    })
}
