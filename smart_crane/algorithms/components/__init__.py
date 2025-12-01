"""
算法组件包 (Algorithms Components Package)

该包包含了路径规划过程中所需的辅助组件，例如：
- GridAdapter: 网格适配器，处理坐标转换和网格生成。
- PlannerFactory: 规划器工厂，负责实例化算法和后处理器。
- SafetyGuard: 安全守卫，负责碰撞检测和端点校验。
"""

from .grid_adapter import GridAdapter
from .planner_factory import PlannerFactory
from .safety_guard import SafetyGuard

__all__ = ["GridAdapter", "PlannerFactory", "SafetyGuard"]
