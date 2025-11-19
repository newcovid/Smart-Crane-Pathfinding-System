import os
from typing import Any
from dotenv import load_dotenv

# 加载 .env 文件中的环境变量 (如果存在)
# 这允许我们在不修改代码的情况下调整配置
load_dotenv()


class Config:
    """
    应用全局配置类。

    职责：
    1. 集中管理所有硬编码参数。
    2. 从环境变量 (os.environ) 加载配置，支持 Docker/云原生部署。
    3. 提供工业标准的默认值 (Default Values)，防止配置缺失导致崩溃。

    使用建议：
    不要直接在代码中写死数字 (Magic Numbers)，应全部在此处定义。
    """

    # =========================================================================
    # === 1. 基础应用配置 (Flask & System)
    # =========================================================================

    # Flask session 加密密钥
    # 警告：在生产环境中，务必通过环境变量 SECRET_KEY 设置复杂的随机字符串
    SECRET_KEY: str = os.environ.get("SECRET_KEY", "wl123123_dev_secret")

    # 日志级别 (DEBUG, INFO, WARNING, ERROR)
    # 生产环境建议设置为 INFO 或 WARNING
    LOG_LEVEL: str = os.environ.get("LOG_LEVEL", "DEBUG")

    # =========================================================================
    # === 2. 车间地图物理参数 (Workshop Physics)
    # =========================================================================

    # [X轴] 车间宽度 (米)
    # 对应地图的列数 (Cols)
    MAP_WIDTH_M: float = float(os.environ.get("MAP_WIDTH_M", 100.0))

    # [Y轴] 车间长度 (米)
    # 对应地图的行数 (Rows)
    MAP_LENGTH_M: float = float(os.environ.get("MAP_LENGTH_M", 100.0))

    # 网格分辨率 (米/格)
    # 越小越精确，但计算量呈平方级增长。推荐 0.5 或 1.0。
    # 1.0 表示 1个格子 = 1米 x 1米
    MAP_RESOLUTION_M: float = float(os.environ.get("MAP_RESOLUTION_M", 0.5))

    # =========================================================================
    # === 3. 起重机物理参数 (Crane Specifications)
    # =========================================================================

    # 起重机足迹直径 (米)
    # 包含吊具、货物尺寸以及必要的物理缓冲距离。
    # 系统会自动将其转换为网格层面的“膨胀半径”。
    CRANE_FOOTPRINT_M: float = float(os.environ.get("CRANE_FOOTPRINT_M", 5.0))

    # 安全巡航高度 (米)
    # Z轴规划时，起重机会先提升到此高度，再进行平面移动，最后下降。
    CRANE_SAFE_TRAVEL_Z_M: float = float(os.environ.get("CRANE_SAFE_TRAVEL_Z_M", 8.0))

    # =========================================================================
    # === 4. 路径规划管线配置 (Pipeline Optimization)
    # =========================================================================

    # [阶段二] 是否开启捷径优化 (Shortcut / Line-of-Sight)
    # 作用：去除 A* 生成的锯齿状冗余点，拉直路径。
    # 建议：始终开启 (True)
    ENABLE_SHORTCUT_OPTIMIZATION: bool = (
        os.environ.get("ENABLE_SHORTCUT_OPTIMIZATION", "True").lower() == "true"
    )

    # [阶段三] 是否开启贝塞尔平滑 (Bezier Smoothing)
    # 作用：将折线拐角处理成圆滑曲线，减少机械冲击。
    # 建议：对于重型起重机，建议开启 (True)
    ENABLE_BEZIER_SMOOTHING: bool = (
        os.environ.get("ENABLE_BEZIER_SMOOTHING", "True").lower() == "true"
    )

    # 贝塞尔倒角平滑度 (0.0 ~ 0.5)
    # 0.1 = 仅在尖角处小范围倒角
    # 0.5 = 最大圆弧 (从线段中点开始倒角)
    BEZIER_SMOOTHNESS: float = float(os.environ.get("BEZIER_SMOOTHNESS", 0.3))

    # 曲线细分段数
    # 每个弯道生成的插值点数量。越高越平滑，但在前端渲染或PLC执行时数据量越大。
    BEZIER_SEGMENTS: int = int(os.environ.get("BEZIER_SEGMENTS", 10))
