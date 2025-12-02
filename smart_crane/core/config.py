import os
import logging
from typing import Dict, Any
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from smart_crane.core.constants import (
    DEFAULT_MAP_WIDTH,
    DEFAULT_MAP_LENGTH,
    DEFAULT_MAP_HEIGHT,
    DEFAULT_RESOLUTION,
    DEFAULT_CRANE_WIDTH,
    DEFAULT_CRANE_LENGTH,
    DEFAULT_CRANE_HEIGHT,
    DEFAULT_SAFE_Z_MARGIN,
    DEFAULT_CRUISE_HEIGHT,
    DEFAULT_OBS_HEIGHT,
    DEFAULT_HEURISTIC_WEIGHT,
    DEFAULT_BEZIER_SMOOTHNESS,
    DEFAULT_BEZIER_SEGMENTS,
    SHAPE_BOX,
    ALGO_DSLITE,
)


class MapSettings(BaseSettings):
    """地图物理属性配置模型。

    Attributes:
        width_m (float): 场地宽度 (X轴)。
        length_m (float): 场地长度 (Y轴)。
        height_m (float): 场地高度 (Z轴)。
        resolution_m (float): 网格分辨率。
    """

    width_m: float = Field(DEFAULT_MAP_WIDTH, alias="MAP_WIDTH_M")
    length_m: float = Field(DEFAULT_MAP_LENGTH, alias="MAP_LENGTH_M")
    height_m: float = Field(DEFAULT_MAP_HEIGHT, alias="MAP_HEIGHT_M")
    resolution_m: float = Field(DEFAULT_RESOLUTION, alias="MAP_RESOLUTION_M")


class CraneSettings(BaseSettings):
    """吊具参数与安全策略配置模型。

    Attributes:
        footprint_shape (str): 吊具形状 ('box' 或 'circle')。
        footprint_width (float): 吊具宽度。
        footprint_length (float): 吊具长度。
        footprint_height (float): 吊具高度。
        enable_fixed_height_cruise (bool): 是否启用定高巡航策略。
        obstacle_infinite_height (bool): 是否将障碍物视为无限高。
        safe_travel_z_m (float): 定高巡航的高度层。
        z_safety_margin (float): 垂直方向的安全余量。
        default_obstacle_height (float): 默认障碍物高度。
    """

    footprint_shape: str = Field(SHAPE_BOX, alias="CRANE_FOOTPRINT_SHAPE")
    footprint_width: float = Field(DEFAULT_CRANE_WIDTH, alias="CRANE_FOOTPRINT_WIDTH")
    footprint_length: float = Field(
        DEFAULT_CRANE_LENGTH, alias="CRANE_FOOTPRINT_LENGTH"
    )
    footprint_height: float = Field(
        DEFAULT_CRANE_HEIGHT, alias="CRANE_FOOTPRINT_HEIGHT"
    )

    # 策略
    enable_fixed_height_cruise: bool = Field(True, alias="ENABLE_FIXED_HEIGHT_CRUISE")
    obstacle_infinite_height: bool = Field(True, alias="OBSTACLE_INFINITE_HEIGHT")

    # 安全边距
    safe_travel_z_m: float = Field(DEFAULT_CRUISE_HEIGHT, alias="CRANE_SAFE_TRAVEL_Z_M")
    z_safety_margin: float = Field(DEFAULT_SAFE_Z_MARGIN, alias="CRANE_Z_SAFETY_MARGIN")
    default_obstacle_height: float = Field(
        DEFAULT_OBS_HEIGHT, alias="DEFAULT_OBSTACLE_HEIGHT_M"
    )


class PlannerSettings(BaseSettings):
    """路径规划算法引擎配置模型。

    Attributes:
        algorithm (str): 算法类型 ('astar', 'dslite')。
        enable_rust_core (bool): 是否启用 Rust 加速核心。
        use_3d_octile (bool): 是否使用 3D Octile 距离启发式。
        heuristic_weight (float): 启发式函数的权重。
    """

    algorithm: str = Field(ALGO_DSLITE, alias="PLANNER_ALGORITHM")
    enable_rust_core: bool = Field(True, alias="ENABLE_RUST_CORE")
    use_3d_octile: bool = Field(False, alias="USE_3D_OCTILE")
    heuristic_weight: float = Field(DEFAULT_HEURISTIC_WEIGHT, alias="HEURISTIC_WEIGHT")


class PostProcessSettings(BaseSettings):
    """后处理管道配置模型。

    Attributes:
        enable_shortcut (bool): 是否启用贪婪捷径优化。
        enable_bezier (bool): 是否启用贝塞尔平滑。
        bezier_smoothness (float): 贝塞尔曲线平滑因子。
        bezier_segments (int): 贝塞尔曲线插值段数。
    """

    enable_shortcut: bool = Field(True, alias="ENABLE_SHORTCUT_OPTIMIZATION")
    enable_bezier: bool = Field(True, alias="ENABLE_BEZIER_SMOOTHING")
    bezier_smoothness: float = Field(
        DEFAULT_BEZIER_SMOOTHNESS, alias="BEZIER_SMOOTHNESS"
    )
    bezier_segments: int = Field(DEFAULT_BEZIER_SEGMENTS, alias="BEZIER_SEGMENTS")


class AppSettings(BaseSettings):
    """应用层通用配置模型。

    Attributes:
        secret_key (str): Flask 应用密钥。
        log_level (str): 日志等级。
    """

    secret_key: str = Field("dev_secret_key_123", alias="SECRET_KEY")
    log_level: str = Field("DEBUG", alias="LOG_LEVEL")


class Settings(BaseSettings):
    """全局配置中心 (Global Settings Hub)。

    使用组合模式管理所有子模块的配置。
    支持从环境变量 (.env) 自动加载。

    Attributes:
        app (AppSettings): 应用配置。
        map (MapSettings): 地图配置。
        crane (CraneSettings): 吊具配置。
        planner (PlannerSettings): 规划器配置。
        post_process (PostProcessSettings): 后处理配置。
    """

    app: AppSettings = Field(default_factory=AppSettings)
    map: MapSettings = Field(default_factory=MapSettings)
    crane: CraneSettings = Field(default_factory=CraneSettings)
    planner: PlannerSettings = Field(default_factory=PlannerSettings)
    post_process: PostProcessSettings = Field(default_factory=PostProcessSettings)

    model_config = SettingsConfigDict(
        env_file=".env", env_file_encoding="utf-8", extra="ignore"
    )

    def to_flat_dict(self) -> Dict[str, Any]:
        """将嵌套的 Pydantic 模型展平为前端所需的 UPPERCASE 字典格式。

        Returns:
            Dict[str, Any]: 扁平化的配置字典，键名为大写的环境变量名 (Alias)。
        """
        flat = {}
        for _, section in self:
            if isinstance(section, BaseSettings):
                for field_name, field_info in section.model_fields.items():
                    key = field_info.alias or field_name.upper()
                    value = getattr(section, field_name)
                    flat[key] = value
        return flat

    def update_from_dict(self, data: Dict[str, Any]) -> None:
        """从扁平字典更新配置。

        该方法会自动识别键名所属的子配置项并进行类型转换更新。

        Args:
            data (Dict[str, Any]): 包含更新值的扁平字典。
        """
        logger = logging.getLogger("Settings")
        updated_keys = []

        # 建立 alias 到 (section, field_name) 的映射
        alias_map = {}
        for section_name, section in self:
            if isinstance(section, BaseSettings):
                for field_name, field_info in section.model_fields.items():
                    alias = field_info.alias or field_name.upper()
                    alias_map[alias] = (section, field_name)

        for k, v in data.items():
            if k in alias_map:
                section, field_name = alias_map[k]
                old_val = getattr(section, field_name)
                # 只有值发生变化时才更新
                if old_val != v:
                    setattr(section, field_name, v)
                    updated_keys.append(f"{k}: {old_val} -> {v}")

        if updated_keys:
            logger.info(
                f"配置项已更新 ({len(updated_keys)} 项): {', '.join(updated_keys)}"
            )
        else:
            logger.debug("尝试更新配置，但没有检测到实际值变更。")


# 全局单例实例
settings = Settings()
