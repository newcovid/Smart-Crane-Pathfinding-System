"""
核心组件包 (Core Components Package)

本包包含系统核心层所需的底层计算组件，主要职责是将繁重的数学运算与业务逻辑解耦。

包含模块：
- GridFactory: 基于 NumPy/SciPy 的高性能网格生成与膨胀计算工厂。
"""

from .grid_factory import GridFactory

__all__ = ["GridFactory"]
