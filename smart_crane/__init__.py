"""
Smart Crane 智能起重机控制系统
==============================

一个基于 Python 和 Rust 混合架构的高性能路径规划与控制系统。
主要用于自动化车间内的起重机（天车）轨迹规划、避障与任务调度。

Modules:
    core: 核心架构（配置、地图、Rust 桥接、业务服务）。
    algorithms: 路径规划算法（A*、D* Lite、后处理）。
    api: Web 接口与 SocketIO 通信。

Copyright (c) 2023-2025 Smart Crane Team.
"""

import logging

# 配置默认的 NullHandler，防止在没有配置日志的情况下报错
logging.getLogger(__name__).addHandler(logging.NullHandler())

__version__ = "0.4.0"
__author__ = "Smart Crane Team"
__all__ = ["core", "algorithms"]
