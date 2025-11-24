import time
import logging
from abc import ABC, abstractmethod
from typing import List, Callable, TypeVar, Tuple, Union, Dict, Any, Optional
from contextlib import contextmanager

# =============================================================================
# 1. 类型定义 (Type Definitions)
# =============================================================================

# 定义 2D 坐标类型: (行号/y, 列号/x) 或 (x, y)
Point2D = Tuple[int, int]

# 定义 3D 坐标类型: (x, y, z)
Point3D = Tuple[int, int, int]

# 定义泛型节点类型 (Generic Node Type)
# TypeVar 是 Python 类型提示系统中的"占位符"。
# 这意味着 NodeType 可以代表 Point2D，也可以代表 Point3D，具体取决于实际使用时的上下文。
NodeType = TypeVar("NodeType", Point2D, Point3D)

# 碰撞检测回调函数类型 (Collision Checker Callback)
# 这是一个函数类型签名：
# - 输入: 一个节点坐标 (NodeType)
# - 输出: 布尔值 (bool) -> True 表示安全(无碰撞)，False 表示有障碍物
# 作用: 优化器不知道地图数据结构，它通过调用这个函数来询问"这里能不能走"。
CollisionChecker = Callable[[NodeType], bool]


# =============================================================================
# 2. 后处理器基类 (Base Class)
# =============================================================================


class PathPostProcessor(ABC):
    """
    【L1.5 层 - 路径后处理器基类】
    (Abstract Base Class for Path Post-Processing)

    设计目的:
    A* 或 D* 搜索出来的路径通常是基于网格的，会有很多锯齿（ZIG-ZAG 现象）。
    这个基类定义了一个标准的"加工流水线"，用于把原始路径变得更直、更平滑。

    核心职责 (模板方法模式):
    1. **流程控制**: 统一管理"输入 -> 计时 -> 处理 -> 统计 -> 输出"的标准流程。
    2. **容错兜底**: 如果优化算法出错了（比如数学计算溢出），基类会拦截错误并返回原始路径，防止程序崩溃。
    3. **性能监控**: 自动统计优化了多少个节点、耗时多少毫秒。

    注意:
    此类**不持有线程锁**。
    调用者（通常是 TrajectoryPlanner）有责任在调用 process() 之前，
    确保底层的地图数据已被锁定，防止在优化过程中地图突然变了。
    """

    def __init__(
        self, name: str = "BaseProcessor", logger: Optional[logging.Logger] = None
    ):
        """
        初始化后处理器。

        Args:
            name: 处理器名称 (例如 "GreedyShortcut", "BezierSmoother")，用于日志区分。
            logger: 依赖注入的日志记录器。如果不传，自动创建一个。
        """
        self.name = name
        self.logger = logger or logging.getLogger(self.__class__.__name__)

        # 性能统计指标容器 (Performance Metrics)
        # 这些数据会被前端 UI (StatsPanel) 读取并展示
        self.stats: Dict[str, Any] = {
            "processor": self.name,  # 处理器名字
            "process_time_ms": 0.0,  # 处理耗时 (毫秒)
            "input_nodes": 0,  # 优化前节点数
            "output_nodes": 0,  # 优化后节点数
            "reduction_rate": 0.0,  # 压缩率 (1 - 输出/输入)，越高说明优化效果越明显
            "timestamp": 0.0,  # 最后执行时间戳
        }

        self.logger.debug(f"[{self.name}] 处理器初始化就绪。")

    def get_name(self) -> str:
        """获取处理器名称"""
        return self.name

    def get_stats(self) -> Dict[str, Any]:
        """获取最近一次运行的性能指标"""
        return self.stats

    # =========================================================================
    # 3. 模板方法 (Template Method) - 外部调用的标准入口
    # =========================================================================

    def process(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """
        [标准入口] 执行路径处理任务。

        这是一个【模板方法】(Template Method)。它定义了算法的骨架，
        而将具体的几何运算延迟到子类 (_process_core) 中去实现。

        流程如下:
        1. 检查输入有效性。
        2. 启动计时器。
        3. 调用子类的核心逻辑 (try-except 保护)。
        4. 停止计时，计算统计数据。
        5. 返回结果。

        Args:
            path: 原始路径点列表 (由 A* 或 D* 计算出的网格坐标)。
            is_safe_fn: 一个函数，给它一个坐标，它告诉你是否撞墙。
                        注意：调用此函数时假定外部已对地图加锁。

        Returns:
            List[NodeType]: 优化后的路径点列表。
        """
        # 1. 边界检查: 如果路径为空，直接返回空
        if not path:
            return []

        input_len = len(path)

        # 使用 perf_counter 进行高精度计时 (适合测量短时间操作)
        start_time = time.perf_counter()

        self.logger.debug(f"[{self.name}] 开始处理路径，输入节点数: {input_len}")

        try:
            # 2. 调用子类核心逻辑 (多态调用)
            # 这里实际执行的是 GreedyShortcut._process_core 或 BezierSmoother._process_core
            result_path = self._process_core(path, is_safe_fn)

        except Exception as e:
            # [容错机制]
            # 如果优化过程中发生任何未预料的错误（如除以零、索引越界），
            # 记录错误日志，但**不抛出异常**。
            # 策略是：宁可不优化（返回原始路径），也不能让机器停下来。
            self.logger.error(
                f"[{self.name}] 处理过程发生严重异常，已回退到原始路径: {e}",
                exc_info=True,
            )
            return path

        # 3. 统计与记录
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000.0  # 转换为毫秒
        output_len = len(result_path)

        # 更新 Stats 字典
        self.stats["process_time_ms"] = elapsed_ms
        self.stats["input_nodes"] = input_len
        self.stats["output_nodes"] = output_len
        self.stats["timestamp"] = time.time()

        # 计算压缩率: 如果输入100个点，输出80个点，压缩率为 20% (0.2)
        if input_len > 0:
            self.stats["reduction_rate"] = 1.0 - (output_len / input_len)
        else:
            self.stats["reduction_rate"] = 0.0

        # [慢日志] 如果处理时间超过 50ms，打印警告，提示开发者关注性能
        if elapsed_ms > 50.0:
            self.logger.warning(
                f"[{self.name}] 性能警告: 耗时过长 ({elapsed_ms:.2f}ms) "
                f"节点变化: {input_len} -> {output_len}"
            )
        else:
            self.logger.debug(
                f"[{self.name}] 完成. 耗时: {elapsed_ms:.2f}ms, "
                f"节点: {input_len}->{output_len} (压缩率: {self.stats['reduction_rate']:.1%})"
            )

        return result_path

    # =========================================================================
    # 4. 抽象接口 (Abstract Interface) - 子类必须实现
    # =========================================================================

    @abstractmethod
    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """
        [核心抽象方法] 实际的几何处理逻辑。

        这是一个"填空题"，所有继承 PathPostProcessor 的子类都必须完成这道题。

        Args:
            path: 原始路径 (输入)
            is_safe_fn: 碰撞检测器 (工具)

        Returns:
            处理后的路径 (输出)

        Example:
            - 贪婪算法会尝试连接不相邻的节点，看能不能走直线。
            - 贝塞尔算法会根据三个点生成一条平滑曲线。
        """
        pass
