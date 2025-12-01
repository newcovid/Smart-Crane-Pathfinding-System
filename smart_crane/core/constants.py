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

DEFAULT_BEZIER_SMOOTHNESS: Final[float] = 0.3
"""float: 默认贝塞尔平滑度 (0.0 - 0.5)。"""

DEFAULT_BEZIER_SEGMENTS: Final[int] = 10
"""int: 默认贝塞尔曲线插值段数。"""

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
