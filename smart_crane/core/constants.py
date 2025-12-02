from typing import Final

# =============================================================================
# 1. 数学与物理常量 (Math & Physics)
# =============================================================================
SQRT_2: Final[float] = 1.41421356
"""float: 2 的平方根，用于 2D 对角线距离计算。"""

SQRT_3: Final[float] = 1.73205081
"""float: 3 的平方根，用于 3D 对角线距离计算。"""

EPSILON: Final[float] = 1e-4
"""float: 浮点数比较的微小误差容忍值。"""

FLOAT_TOLERANCE: Final[float] = 1e-3
"""float: 用于宽松浮点数比较的容忍值。"""

INF: Final[float] = float("inf")
"""float: 正无穷大，用于初始化路径代价。"""

# =============================================================================
# 2. 默认配置值 (Default Configuration Values)
# =============================================================================
DEFAULT_MAP_WIDTH: Final[float] = 80.0
"""float: 默认地图宽度 (米)。"""

DEFAULT_MAP_LENGTH: Final[float] = 40.0
"""float: 默认地图长度 (米)。"""

DEFAULT_MAP_HEIGHT: Final[float] = 20.0
"""float: 默认地图高度 (米)。"""

DEFAULT_RESOLUTION: Final[float] = 1.0
"""float: 默认网格分辨率 (米/格)。"""

DEFAULT_CRANE_WIDTH: Final[float] = 4.0
"""float: 默认吊具宽度 (米)。"""

DEFAULT_CRANE_LENGTH: Final[float] = 4.0
"""float: 默认吊具长度 (米)。"""

DEFAULT_CRANE_HEIGHT: Final[float] = 0.0
"""float: 默认吊具高度 (米)。"""

DEFAULT_SAFE_Z_MARGIN: Final[float] = 2.0
"""float: 默认垂直安全边距 (米)。"""

DEFAULT_CRUISE_HEIGHT: Final[float] = 8.0
"""float: 默认巡航高度 (米)。"""

DEFAULT_OBS_HEIGHT: Final[float] = 1.0
"""float: 默认障碍物高度 (米)。"""

DEFAULT_HEURISTIC_WEIGHT: Final[float] = 1.0
"""float: 默认 A* 启发式权重。"""

# A* specific defaults / magic numbers centralized here
# These are used by the A* implementation (smart_crane.algorithms.pathfinding.astar)
# to avoid "magic numbers" scattered through the codebase.
A_STAR_DEFAULT_RESOLUTION: Final[float] = 0.5
"""float: A* 默认网格分辨率 (米/格)，用于 A* Planner 的默认参数。"""

A_STAR_MOVE_COST_CARDINAL: Final[float] = 1.0
"""float: A* 中沿主方向（上下左右）移动的代价值 (等价于 COST_1)。"""

A_STAR_INIT_F_SCORE: Final[float] = 0.0
"""float: A* OpenSet 初始节点的初始 f 值。"""

A_STAR_RECONSTRUCT_SAFE_MARGIN: Final[int] = 100
"""int: 路径回溯时用于防止死循环的额外步数上限（len(came_from) + margin）。"""

DEFAULT_BEZIER_SMOOTHNESS: Final[float] = 0.3
"""float: 默认贝塞尔平滑度 (0.0 - 0.5)。"""

DEFAULT_BEZIER_SEGMENTS: Final[int] = 10
"""int: 默认贝塞尔曲线插值段数。"""

# Bezier-specific runtime tuning
# 用于控制贝塞尔检查与回退的相关魔法数字
DEFAULT_BEZIER_ATTEMPTS: Final[int] = 5
"""int: 在尝试平滑化某个拐角时的最大自适应回退次数（每次将 smoothness *= 0.5）。"""

BEZIER_CHECK_STEP_MULTIPLIER: Final[int] = 5
"""int: 用于将曲线近似长度转为采样步数的乘数 (steps = int(total_len * multiplier))。"""

BEZIER_MIN_STEPS: Final[int] = 5
"""int: 在对贝塞尔曲线做碰撞检查时，最少采样步数。"""

# =============================================================================
# D* Lite 与运行时相关默认值（避免魔法数字散落在代码中）
# =============================================================================
DSLITE_DEFAULT_MAX_NODES: Final[int] = 5000
"""int: D* Lite 在小地图或默认情况下允许的最小最大扩展节点数。"""

DSLITE_MAX_NODES_MULTIPLIER: Final[int] = 5
"""int: 用于按网格总量放大最大扩展节点数的乘数（total_grid_size * multiplier）。"""

DSLITE_COST_THRESHOLD_MULTIPLIER: Final[float] = 100.0
"""float: 用于计算代价阈值的乘数（total_grid_size * multiplier）。"""

DSLITE_DEFAULT_HEURISTIC_MIN_WEIGHT: Final[float] = 1.0
"""float: D* Lite / A* 启发式最小权重（防止权重小于 1）。"""

# =============================================================================
# 日志与服务运行默认值
# =============================================================================
LOG_LEVEL_WIDTH: Final[int] = 8
"""int: 日志等级显示的固定宽度（用于格式化）。"""

LOGGER_NAME_WIDTH: Final[int] = 45
"""int: 日志记录器名称显示的固定宽度（用于格式化）。"""

DEFAULT_SERVER_HOST: Final[str] = "0.0.0.0"
"""str: 默认服务器绑定地址。"""

DEFAULT_SERVER_PORT: Final[int] = 5000
"""int: 默认服务器端口。"""

# =============================================================================
# 3. 业务逻辑常量 (Business Logic Constants)
# =============================================================================
MIN_SAFE_HEIGHT_OFFSET: Final[float] = 1.0
"""float: 最小安全高度偏移量 (防止直接贴着障碍物表面规划)。"""

DEFAULT_Z_HIGH: Final[float] = 100.0
"""float: 默认的高空 Z 值 (用于无限高模式或找不到 Z 值时)。"""

GRID_MARGIN_BUFFER: Final[int] = 1
"""int: 网格膨胀时的额外缓冲格数，用于处理边界情况。"""

GRID_CENTER_OFFSET: Final[float] = 0.5
"""float: 网格中心偏移量 (用于从网格索引转为物理中心坐标)。"""

# =============================================================================
# MapManager / 通用工具 常量
# =============================================================================
MARK_OBS_EPS: Final[float] = 0.01
"""float: 在将物理边界转换为网格索引时用于减小边界的微小偏移，避免边界恰好落在格子边缘导致占格不一致。"""

CACHE_ROUND_DIGITS: Final[int] = 3
"""int: 缓存键值中用于 round 的小数位数。"""

TIME_MS_MULTIPLIER: Final[float] = 1000.0
"""float: 将秒级时间转换为毫秒的乘数。"""

TIME_FMT_PREC: Final[int] = 2
"""int: 日志中时间展示使用的小数位精度，例如 "{:.2f}" 中的 2。"""

# 性能告警相关阈值（以毫秒为单位）
PERFORMANCE_WARNING_THRESHOLD_MS: Final[float] = 200.0
"""float: 当某个计算块耗时超过此阈值（ms）时触发性能告警。"""

GRID_OCCUPIED: Final[int] = 1
"""int: 表示网格中被障碍物占据的值。"""

GRID_FREE: Final[int] = 0
"""int: 表示网格中空闲格的值。"""

# =============================================================================
# GridFactory / Voxelization 常量
# =============================================================================
GRID_PADDING: Final[int] = 1
"""int: 在进行距离变换时的边界填充宽度（单位: 格数）。"""

GRID_DISCRETIZATION_COMPENSATION: Final[float] = 0.5
"""float: 在网格膨胀与卷积中用于补偿离散化误差的半径补偿值。"""

BOUNDARY_EPS: Final[float] = 1e-4
"""float: 用于在边界计算中微调坐标以避免浮点边界问题。"""

Z_ABOVE_MAP_DELTA: Final[float] = 1.0
"""float: 用于表示'高于地图顶端'的偏移量（无限高模式下的占据高度增量）。"""

# =============================================================================
# 服务与业务默认值
# =============================================================================
DEFAULT_MISSION_START_X: Final[float] = 5.0
DEFAULT_MISSION_START_Y: Final[float] = 5.0
DEFAULT_MISSION_START_Z: Final[float] = 5.0

DEFAULT_MISSION_END_X: Final[float] = 30.0
DEFAULT_MISSION_END_Y: Final[float] = 20.0
DEFAULT_MISSION_END_Z: Final[float] = 5.0

UUID_ID_LEN: Final[int] = 8
"""int: 随机生成的障碍物 ID 的截断长度（用于简短显示）。"""

COORD_PRINT_PREC: Final[int] = 1
"""int: 坐标在日志输出中的小数位数格式化精度（如 {:.1f}）。"""

# =============================================================================
# 4. 消息模板 (Messages)
# =============================================================================
MSG_PLAN_SUCCESS: Final[str] = "规划成功! 路径节点数: {count}"
MSG_PLAN_FAIL: Final[str] = "规划失败: {reason}"
MSG_CONFIG_UPDATE: Final[str] = "配置已更新"
MSG_CONFIG_NO_CHANGE: Final[str] = "配置未发生变化"
MSG_OBS_ADDED: Final[str] = "成功添加障碍物: {id}"
MSG_OBS_REMOVED: Final[str] = "成功移除障碍物: {id}"
MSG_OBS_NOT_FOUND: Final[str] = "未找到目标障碍物"
MSG_REBUILD_MAP: Final[str] = "地图物理尺寸变更，正在执行系统重构 (Full Rebuild)..."

# =============================================================================
# 5. 字典键名与类型标识 (Keys & Types)
# =============================================================================
KEY_STATIC_OBS: Final[str] = "static_obstacles"
KEY_DYNAMIC_OBS: Final[str] = "dynamic_obstacles"
TYPE_STATIC: Final[str] = "static"
TYPE_DYNAMIC: Final[str] = "dynamic"
SHAPE_BOX: Final[str] = "box"
SHAPE_CIRCLE: Final[str] = "circle"
ALGO_ASTAR: Final[str] = "astar"
ALGO_DSLITE: Final[str] = "dslite"

# =============================================================================
# Path / Trajectory 常量
# =============================================================================
DEFAULT_DEDUP_TOLERANCE: Final[float] = 0.01
"""float: 路径去重时判定为重复点的距离阈值（米）。"""

PATH_COORD_ROUND_DIGITS: Final[int] = 2
"""int: 最终路径点坐标舍入的小数位数。"""

PATH_MIN_HEIGHT_DIFF: Final[float] = 0.01
"""float: 判断高度差显著性的阈值（米）。"""

PATH_MIN_SEPARATION_SQ: Final[float] = 1e-4
"""float: 两点间最小分离阈值的平方（用于比较距离平方以避免 sqrt）。"""

# Path line-of-sight / sampling tuning (集中管理 LOS / path-sampling 魔法数字)
# 这些常量用于路径后处理模块(如 greedy / bezier)在做曲线/视线采样时的步数/阈值控制
PATH_MIN_NODES: Final[int] = 3
"""int: 最少路径节点数，小于该值时无法进行路径优化（例如捷径或平滑操作）。"""

PATH_LOS_MIN_DISTANCE: Final[float] = 1e-6
"""float: 判定两点距离近似为相同时使用的最小距离阈值（用于避免除以零或过小步长）。"""

PATH_LOS_SAMPLE_MULTIPLIER: Final[int] = 5
"""int: 将两点间距离转换为采样步数时使用的乘数 (steps = int(distance * multiplier))。"""

PATH_LOS_MIN_STEPS: Final[int] = 2
"""int: 对路径/视线采样时允许的最小采样步数 (即 steps >= PATH_LOS_MIN_STEPS)。"""

# =============================================================================
# Algorithms / Components 统一常量（避免模块内部散落魔法数字）
# =============================================================================
FOOTPRINT_HALF_DIVISOR: Final[float] = 2.0
"""float: 将宽度/长度转换为半径时使用的除数（即除以 2）。"""

ESCAPE_SEARCH_RADIUS_MARGIN: Final[int] = 5
"""int: 在智能脱困时，在网格膨胀半径基础上额外增加的格数。"""

ESCAPE_SEARCH_RADIUS_MAX: Final[int] = 50
"""int: 智能脱困时允许的最大搜索半径（单位: 格）。"""

GRACE_POINT_TOL_SQ: Final[float] = 0.25
"""float: 豁免点（起点/终点）判定的平方距离容差 (米^2)，用于快速判断是否在豁免区内。"""

RESOLUTION_HALF_FACTOR: Final[float] = 0.5
"""float: 将网格分辨率取半时使用的乘数（等同于 /2.0）。"""

RUST_LABEL: Final[str] = "Rust 加速"
"""str: 日志中用于标识 Rust 加速模式的标签（简体中文）。"""

PYTHON_FALLBACK_LABEL: Final[str] = "Python 回退"
"""str: 日志中用于标识 Python 回退模式的标签（简体中文）。"""
