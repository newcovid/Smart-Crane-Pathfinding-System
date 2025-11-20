import time
import logging
from abc import ABC, abstractmethod
from typing import List, Callable, TypeVar, Tuple, Union, Dict, Any, Optional
from contextlib import contextmanager

# =============================================================================
# 1. 类型定义 (Type Definitions)
# =============================================================================

# 定义通用坐标类型 (2D 或 3D)
Point2D = Tuple[int, int]
Point3D = Tuple[int, int, int]
NodeType = TypeVar("NodeType", Point2D, Point3D)

# 碰撞检测回调函数类型: 输入一个坐标，返回 True 表示安全(无碰撞)
CollisionChecker = Callable[[NodeType], bool]


# =============================================================================
# 2. 后处理器基类 (Base Class)
# =============================================================================


class PathPostProcessor(ABC):
    """
    【L1.5 层 - 路径后处理器基类 (增强版)】
    (Abstract Base Class for Path Post-Processing - Enhanced)

    设计目的:
    作为 "管道过滤器 (Pipe & Filter)"，接收一条原始路径，输出一条优化后的路径。

    核心职责:
    1. **算法封装**: 定义标准的 _process_core 接口供子类实现。
    2. **性能监控**: 自动统计处理耗时、路径压缩率等关键指标。
    3. **可观测性**: 提供标准化的 stats 数据和日志接口。

    注意:
    此类**不持有线程锁**。
    调用者有责任在调用 process() 之前，确保 is_safe_fn 依赖的底层地图数据已被锁定，
    以防止在后处理过程中地图发生突变导致的不一致。
    """

    def __init__(
        self, name: str = "BaseProcessor", logger: Optional[logging.Logger] = None
    ):
        """
        初始化后处理器。

        Args:
            name: 处理器名称 (用于日志和统计).
            logger: 依赖注入的日志记录器.
        """
        self.name = name
        self.logger = logger or logging.getLogger(self.__class__.__name__)

        # 性能统计指标
        self.stats: Dict[str, Any] = {
            "processor": self.name,
            "process_time_ms": 0.0,  # 处理耗时
            "input_nodes": 0,  # 输入节点数
            "output_nodes": 0,  # 输出节点数
            "reduction_rate": 0.0,  # 节点压缩率 (1 - out/in)
            "timestamp": 0.0,  # 最后执行时间
        }

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
        [标准入口] 执行处理逻辑。

        此方法实现了【模板方法模式】：
        1. 自动记录输入状态。
        2. 自动计时。
        3. 调用子类的 `_process_core` 执行实际几何运算。
        4. 自动计算统计指标。

        Args:
            path: 输入的路径点列表。
            is_safe_fn: 碰撞检测回调函数。
                        注意：调用此函数时假定外部已对地图加锁。

        Returns:
            优化后的路径点列表。
        """
        # 1. 边界检查
        if not path:
            return []

        input_len = len(path)
        start_time = time.perf_counter()

        try:
            # 2. 调用子类核心逻辑
            result_path = self._process_core(path, is_safe_fn)
        except Exception as e:
            self.logger.error(f"[{self.name}] 处理过程发生异常: {e}", exc_info=True)
            # 发生错误时兜底：返回原始路径，确保起重机不会停摆
            return path

        # 3. 统计与记录
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000.0
        output_len = len(result_path)

        # 更新 Stats
        self.stats["process_time_ms"] = elapsed_ms
        self.stats["input_nodes"] = input_len
        self.stats["output_nodes"] = output_len
        self.stats["timestamp"] = time.time()

        if input_len > 0:
            self.stats["reduction_rate"] = 1.0 - (output_len / input_len)
        else:
            self.stats["reduction_rate"] = 0.0

        # 性能慢日志 (例如超过 50ms)
        if elapsed_ms > 50.0:
            self.logger.warning(
                f"[{self.name}] 耗时警告: {elapsed_ms:.2f}ms "
                f"(Nodes: {input_len}->{output_len})"
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

        由子类实现 (如 GreedyShortcut, BezierSmoother)。

        Args:
            path: 原始路径
            is_safe_fn: 碰撞检测函数

        Returns:
            处理后的路径
        """
        pass
