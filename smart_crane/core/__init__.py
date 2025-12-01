"""
核心模块 (Core Module)

该包包含了系统的基础架构组件，包括：
- config: 全局配置管理 (Pydantic Settings)。
- constants: 系统常量定义。
- map_manager: 地图与障碍物状态管理。
- crane_service: 业务逻辑服务层。
- rust_bridge: Python 与 Rust 扩展的交互桥梁。
"""

from .config import Settings, settings
from .crane_service import CraneService
from .map_manager import WorkshopMapManager

__all__ = ["Settings", "settings", "CraneService", "WorkshopMapManager"]
