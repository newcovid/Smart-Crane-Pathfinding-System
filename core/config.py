import os
from typing import Any, Dict
from dotenv import load_dotenv

# 加载 .env 文件
load_dotenv()


class Config:
    """
    [系统配置中心]

    职责:
    1. 定义系统的物理约束 (尺寸、安全距离)。
    2. 配置算法策略 (选择 A* 还是 D* Lite)。
    3. 从环境变量加载配置，支持 Docker/云原生部署。
    """

    # =========================================================================
    # 1. 基础与网络配置
    # =========================================================================
    SECRET_KEY: str = os.environ.get("SECRET_KEY", "dev_secret_key_123")
    LOG_LEVEL: str = os.environ.get("LOG_LEVEL", "DEBUG")

    # =========================================================================
    # 2. 车间物理环境 (Physics & Map)
    # =========================================================================
    # 地图物理尺寸 (米)
    MAP_WIDTH_M: float = float(os.environ.get("MAP_WIDTH_M", 100.0))
    MAP_LENGTH_M: float = float(os.environ.get("MAP_LENGTH_M", 100.0))
    MAP_HEIGHT_M: float = float(os.environ.get("MAP_HEIGHT_M", 20.0))

    # 栅格分辨率 (米/格)
    # 0.5m 意味着 100m 的厂房会被切分为 200 个格子
    MAP_RESOLUTION_M: float = float(os.environ.get("MAP_RESOLUTION_M", 0.5))

    # 吊具足迹配置 (3D Box)
    # 形状: 'circle' (球体) 或 'box' (长方体)
    CRANE_FOOTPRINT_SHAPE: str = os.environ.get("CRANE_FOOTPRINT_SHAPE", "box")

    # 尺寸定义:
    # Width (X): 宽度 (若为球体则为直径)
    CRANE_FOOTPRINT_WIDTH: float = float(os.environ.get("CRANE_FOOTPRINT_WIDTH", 5.0))
    # Length (Y): 长度 (仅 Box 有效)
    CRANE_FOOTPRINT_LENGTH: float = float(os.environ.get("CRANE_FOOTPRINT_LENGTH", 5.0))
    # Height (Z): 高度 (若为球体则忽略，默认等于直径)
    CRANE_FOOTPRINT_HEIGHT: float = float(os.environ.get("CRANE_FOOTPRINT_HEIGHT", 2.0))

    # 定高巡航开关
    # True: 使用 "起升-平移-下落" 策略 (2.5D)
    # False: 使用全 3D A* 规划 (三轴联动)
    ENABLE_FIXED_HEIGHT_CRUISE: bool = (
        os.environ.get("ENABLE_FIXED_HEIGHT_CRUISE", "True").lower() == "true"
    )

    # 垂直安全边距 (Z-Axis Inflation)
    # 即使巡航高度高于障碍物，也必须保留此安全距离，否则视为碰撞
    CRANE_Z_SAFETY_MARGIN: float = float(os.environ.get("CRANE_Z_SAFETY_MARGIN", 0.5))

    # 兼容旧版字段
    CRANE_FOOTPRINT_M: float = CRANE_FOOTPRINT_WIDTH

    # 安全巡航高度 (米)
    # Z轴规划的目标高度
    CRANE_SAFE_TRAVEL_Z_M: float = float(os.environ.get("CRANE_SAFE_TRAVEL_Z_M", 8.0))

    # 障碍物策略
    # True: 所有障碍物视为无限高 (传统安全模式，必须绕行)
    # False: 启用 3D 高度检查，允许飞越低于巡航高度的障碍物
    OBSTACLE_INFINITE_HEIGHT: bool = (
        os.environ.get("OBSTACLE_INFINITE_HEIGHT", "True").lower() == "true"
    )

    # 默认障碍物高度 (当添加障碍未指定高度时使用)
    DEFAULT_OBSTACLE_HEIGHT_M: float = float(
        os.environ.get("DEFAULT_OBSTACLE_HEIGHT_M", 2.0)
    )

    # =========================================================================
    # 3. 算法策略配置 (Algorithm Strategy)
    # =========================================================================

    # 核心规划器选择
    # 选项: 'astar' (静态/全量), 'dslite' (动态/增量)
    PLANNER_ALGORITHM: str = os.environ.get("PLANNER_ALGORITHM", "astar").lower()

    # A* / D* 启发式选项
    # True: 使用 3D 对角线距离 (Octile)，贴合网格几何。
    # False: 使用欧几里得距离 (Euclidean)，倾向于走直线，利于平滑。
    USE_3D_OCTILE: bool = os.environ.get("USE_3D_OCTILE", "False").lower() == "true"

    # 启发式权重 (Heuristic Weight)
    # 1.0 = 标准 A* (最优路径，但慢)
    # >1.0 = 加权 A* (牺牲少量最优性，换取数十倍的速度提升)
    # 推荐值: 1.2 ~ 2.0
    HEURISTIC_WEIGHT: float = float(os.environ.get("HEURISTIC_WEIGHT", 1.5))
    # =========================================================================
    # 4. 后处理管道配置 (Post-Processing Pipeline)
    # =========================================================================

    # 捷径优化 (Greedy Shortcut)
    # 去除网格搜索产生的冗余拐点
    ENABLE_SHORTCUT_OPTIMIZATION: bool = (
        os.environ.get("ENABLE_SHORTCUT_OPTIMIZATION", "True").lower() == "true"
    )

    # 贝塞尔平滑 (Bezier Smoothing)
    # 将折线拐角替换为平滑曲线
    ENABLE_BEZIER_SMOOTHING: bool = (
        os.environ.get("ENABLE_BEZIER_SMOOTHING", "True").lower() == "true"
    )

    # 贝塞尔平滑度 (0.0 ~ 0.5)
    # 0.5 代表最大切角圆弧
    BEZIER_SMOOTHNESS: float = float(os.environ.get("BEZIER_SMOOTHNESS", 0.3))

    # [修复] 曲线细分段数 (Interpolation Segments)
    # 每个弯道生成的插值点数量。数值越高曲线越圆滑，但数据量越大。
    # 建议值: 10~20
    BEZIER_SEGMENTS: int = int(os.environ.get("BEZIER_SEGMENTS", 10))

    @classmethod
    def to_dict(cls) -> Dict[str, Any]:
        """导出所有配置为字典 (用于发送给前端)"""
        return {
            k: v
            for k, v in cls.__dict__.items()
            if not k.startswith("__") and not callable(v)
        }
