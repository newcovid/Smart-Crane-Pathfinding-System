"""
核心模块 (Core Module)

该包包含了系统的基础架构组件，包括：
- config: 全局配置管理 (Pydantic Settings)。
- constants: 系统常量定义。
- map_manager: 地图与障碍物状态管理。
- crane_service: 业务逻辑服务层。
- rust_bridge: Python 与 Rust 扩展的交互桥梁。

导入策略
--------
本模块使用 PEP 562 的模块级 ``__getattr__`` 做惰性导入。

原因是存在一条依赖环：算法层的 ``pathfinding.base`` 需要 ``core.constants``，
而导入任何 ``core`` 子模块都会先初始化本包；若本包在 ``__init__`` 中急切导入
``crane_service``，就会回头去加载 ``algorithms.trajectory_planner``，
后者又依赖尚未初始化完成的 ``pathfinding.base``，从而抛出
``ImportError: partially initialized module``。

惰性导入让 ``from smart_crane.core import CraneService`` 的公开用法保持不变，
同时使 ``smart_crane`` 的任意子模块都可以作为独立入口被导入，不再依赖导入顺序。
"""

from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # 仅供类型检查器与 IDE 解析，运行时不执行
    from .config import Settings, settings
    from .crane_service import CraneService
    from .map_manager import WorkshopMapManager

__all__ = ["Settings", "settings", "CraneService", "WorkshopMapManager"]

# 公开名称 -> 所在子模块
_LAZY_EXPORTS = {
    "Settings": ".config",
    "settings": ".config",
    "CraneService": ".crane_service",
    "WorkshopMapManager": ".map_manager",
}


def __getattr__(name: str) -> Any:
    """在首次访问时才真正导入对应子模块（PEP 562）。"""
    module_name = _LAZY_EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(module_name, __name__), name)
    globals()[name] = value  # 缓存，后续访问不再走 __getattr__
    return value


def __dir__() -> list[str]:
    return sorted(__all__)
