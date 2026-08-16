//! 系统常量定义模块
//!
//! 该模块集中管理寻路算法中使用的数学常数、移动代价权重以及浮点数比较阈值。
//! 保持与 Python 端 `smart_crane/core/constants.py` 的一致性。

/// 直线移动代价 (1.0)
pub const COST_1: f32 = 1.0;

/// 2D 对角线移动代价 sqrt(2)。
/// 使用标准库常量而非手写近似值：手写字面量与 Python 侧的 math.sqrt(2)
/// 存在末位差异，会让两个引擎在代价相等的路径上做出不同的 tie-break。
pub const COST_2: f32 = std::f32::consts::SQRT_2;

/// 3D 对角线移动代价 sqrt(3)。标准库未提供该常量，保留字面量。
pub const COST_3: f32 = 1.732_050_8;

/// 浮点数无穷大
pub const INF: f32 = f32::INFINITY;

/// 浮点数相等性比较的微小误差容忍度 (Equality Check)
pub const EPSILON: f32 = 1e-4;

/// 浮点数不等性比较的宽松容差 (Inequality Check)
pub const FLOAT_TOLERANCE: f32 = 1e-3;

/// 默认的高空 Z 值 (用于无限高模式或找不到 Z 值时)
pub const DEFAULT_Z_HIGH: f32 = 100.0;

// --- 性能优化常量 ---

/// 障碍物数量阈值
/// 当障碍物数量超过此值时，使用全局距离变换 (EDT) 算法生成网格，
/// 否则使用基于几何遍历的“绘制”算法。
/// EDT 复杂度为 O(GridSize)，绘制算法复杂度为 O(ObsCount * ObsSize)。
pub const OBSTACLE_COUNT_THRESHOLD: usize = 50;
