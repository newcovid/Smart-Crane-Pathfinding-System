import os
from typing import Any, Dict
from dotenv import load_dotenv

# 加载 .env 文件
load_dotenv()


class Config:
    """
    [系统配置中心]

    设计哲学:
    - 世界永远是 3D 的。
    - "2D" 仅作为特定约束下（如无限高障碍物、定高巡航）的计算加速手段。
    """

    # =========================================================================
    # 1. 基础与网络配置
    # =========================================================================
    SECRET_KEY: str = os.environ.get("SECRET_KEY", "dev_secret_key_123")
    LOG_LEVEL: str = os.environ.get("LOG_LEVEL", "INFO")

    # =========================================================================
    # 2. 车间物理环境 (Physics & Map)
    # =========================================================================
    # 地图物理尺寸 (米)
    MAP_WIDTH_M: float = float(os.environ.get("MAP_WIDTH_M", 80.0))
    MAP_LENGTH_M: float = float(os.environ.get("MAP_LENGTH_M", 40.0))
    MAP_HEIGHT_M: float = float(os.environ.get("MAP_HEIGHT_M", 20.0))

    # 栅格分辨率 (米/格)
    MAP_RESOLUTION_M: float = float(os.environ.get("MAP_RESOLUTION_M", 1.0))

    # 吊具足迹配置 (3D AABB / Cylinder)
    # 形状: 'circle' (圆柱体投影) 或 'box' (长方体)
    CRANE_FOOTPRINT_SHAPE: str = os.environ.get("CRANE_FOOTPRINT_SHAPE", "box")
    CRANE_FOOTPRINT_WIDTH: float = float(os.environ.get("CRANE_FOOTPRINT_WIDTH", 4.0))
    CRANE_FOOTPRINT_LENGTH: float = float(os.environ.get("CRANE_FOOTPRINT_LENGTH", 4.0))
    CRANE_FOOTPRINT_HEIGHT: float = float(os.environ.get("CRANE_FOOTPRINT_HEIGHT", 0.0))

    # =========================================================================
    # 3. 寻路策略配置 (Pathfinding Strategy)
    # =========================================================================

    # [关键策略] 定高巡航模式 (2.5D Strategy)
    # True: 启用“起升-平移-下落”策略。寻路算法在 2D 投影/切片面上运行以加速计算。
    # False: 启用全 3D 自由度规划。寻路算法在 3D 体素网格中运行。
    ENABLE_FIXED_HEIGHT_CRUISE: bool = (
        os.environ.get("ENABLE_FIXED_HEIGHT_CRUISE", "True").lower() == "true"
    )

    # [约束条件] 障碍物无限高假设 (Infinite Height Assumption)
    # True: 假设所有障碍物向上延伸至无穷大。此时 2D 网格是所有障碍物的 XY 投影。
    # False: 考虑障碍物的实际高度。此时 2D 网格是特定巡航高度层的切片。
    OBSTACLE_INFINITE_HEIGHT: bool = (
        os.environ.get("OBSTACLE_INFINITE_HEIGHT", "True").lower() == "true"
    )

    # 巡航高度 (米) - 仅在定高模式下生效
    CRANE_SAFE_TRAVEL_Z_M: float = float(os.environ.get("CRANE_SAFE_TRAVEL_Z_M", 8.0))

    # 垂直安全边距 (Z-Axis Safety Margin)
    # 任何模式下，吊具底部与障碍物顶部的最小净空
    CRANE_Z_SAFETY_MARGIN: float = float(os.environ.get("CRANE_Z_SAFETY_MARGIN", 2.0))

    # 默认障碍物高度
    DEFAULT_OBSTACLE_HEIGHT_M: float = float(
        os.environ.get("DEFAULT_OBSTACLE_HEIGHT_M", 1.0)
    )

    # =========================================================================
    # 4. 算法引擎配置 (Algorithm Engine)
    # =========================================================================

    # 核心规划器: 'astar' (全局最优) 或 'dslite' (动态增量)
    PLANNER_ALGORITHM: str = os.environ.get("PLANNER_ALGORITHM", "dslite").lower()

    # 性能加速开关
    # True: 优先使用 Rust 编写的高性能核心 (smart_crane_core)
    # False: 强制使用 Python 原生实现 (用于 Debug 或 性能对比)
    ENABLE_RUST_CORE: bool = (
        os.environ.get("ENABLE_RUST_CORE", "True").lower() == "true"
    )

    # 启发式距离: True=Octile(8邻域/26邻域), False=Euclidean(欧氏距离)
    USE_3D_OCTILE: bool = os.environ.get("USE_3D_OCTILE", "False").lower() == "true"

    # 启发式权重: >= 1.0. 越大越快但非最优。
    HEURISTIC_WEIGHT: float = float(os.environ.get("HEURISTIC_WEIGHT", 1.0))

    # =========================================================================
    # 5. 后处理管道 (Post-Processing)
    # =========================================================================
    ENABLE_SHORTCUT_OPTIMIZATION: bool = (
        os.environ.get("ENABLE_SHORTCUT_OPTIMIZATION", "True").lower() == "true"
    )
    ENABLE_BEZIER_SMOOTHING: bool = (
        os.environ.get("ENABLE_BEZIER_SMOOTHING", "True").lower() == "true"
    )
    BEZIER_SMOOTHNESS: float = float(os.environ.get("BEZIER_SMOOTHNESS", 0.3))
    BEZIER_SEGMENTS: int = int(os.environ.get("BEZIER_SEGMENTS", 10))

    @classmethod
    def to_dict(cls) -> Dict[str, Any]:
        return {
            k: v
            for k, v in cls.__dict__.items()
            if not k.startswith("__") and not callable(v)
        }
