import time
import logging
from abc import ABC, abstractmethod
from typing import List, Callable, TypeVar, Tuple, Union, Dict, Any, Optional

# =============================================================================
# 1. 类型定义 (Type Definitions)
# =============================================================================

Point2D = Tuple[int, int]
Point3D = Tuple[int, int, int]
# 泛型节点类型，兼容 2D 和 3D 坐标
NodeType = TypeVar("NodeType", Point2D, Point3D)

# 碰撞检测回调函数签名
CollisionChecker = Callable[[Union[Point2D, Point3D]], bool]


# =============================================================================
# 2. 后处理器基类 (Base Class)
# =============================================================================


class PathPostProcessor(ABC):
    """路径后处理器抽象基类 (L1.5层)。

    定义了路径优化算法的标准接口和生命周期。
    采用模板方法模式，统一管理输入校验、性能计时、异常捕获和统计信息更新。

    Attributes:
        name (str): 处理器名称。
        logger (logging.Logger): 专用的日志记录器。
        stats (Dict[str, Any]): 运行时性能指标。
    """

    def __init__(
        self,
        name: str = "BaseProcessor",
        logger: Optional[logging.Logger] = None,
    ):
        """初始化后处理器基类。

        Args:
            name (str): 处理器名称，用于日志标识。
            logger (Optional[logging.Logger]): 父级日志记录器。
                                             如果提供，将创建子记录器 (Parent.Name)。
        """
        self.name = name

        # [日志优化] 智能嵌套
        if logger:
            self.logger = logger.getChild(name)
        else:
            self.logger = logging.getLogger(name)

        # 性能统计指标
        self.stats: Dict[str, Any] = {
            "processor": self.name,
            "process_time_ms": 0.0,
            "input_nodes": 0,
            "output_nodes": 0,
            "reduction_rate": 0.0,
            "timestamp": 0.0,
        }

        self.logger.debug("处理器初始化就绪。")

    def get_name(self) -> str:
        """获取处理器名称。"""
        return self.name

    def get_stats(self) -> Dict[str, Any]:
        """获取最近一次运行的性能指标。"""
        return self.stats

    # =========================================================================
    # 3. 模板方法 (Template Method)
    # =========================================================================

    def process(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """[标准入口] 执行路径处理任务。

        包含计时、异常保护和统计更新的模板方法。

        Args:
            path (List[NodeType]): 原始路径节点列表。
            is_safe_fn (CollisionChecker): 碰撞检测回调函数。
                                           输入坐标 (x, y) 或 (x, y, z)，返回 True 表示安全。

        Returns:
            List[NodeType]: 优化后的路径节点列表。如果发生异常，返回原始路径。
        """
        # 1. 边界检查
        if not path:
            return []

        input_len = len(path)
        start_time = time.perf_counter()

        self.logger.debug(f"开始处理路径，输入节点数: {input_len}")

        try:
            # 2. 调用子类核心逻辑
            result_path = self._process_core(path, is_safe_fn)

        except Exception as e:
            # [容错机制] 发生异常时回退到原始路径，保证系统可用性
            self.logger.error(
                f"处理过程发生未捕获异常，已回退到原始路径: {e}", exc_info=True
            )
            return path

        # 3. 统计与记录
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000.0
        output_len = len(result_path)

        self.stats["process_time_ms"] = elapsed_ms
        self.stats["input_nodes"] = input_len
        self.stats["output_nodes"] = output_len
        self.stats["timestamp"] = time.time()

        # 计算压缩率 (1 - 输出/输入)
        if input_len > 0:
            self.stats["reduction_rate"] = 1.0 - (output_len / input_len)
        else:
            self.stats["reduction_rate"] = 0.0

        # [性能日志]
        log_msg = (
            f"完成. 耗时: {elapsed_ms:.2f}ms, "
            f"节点: {input_len}->{output_len} (Rate: {self.stats['reduction_rate']:.1%})"
        )

        if elapsed_ms > 50.0:
            self.logger.warning(f"性能告警: {log_msg}")
        else:
            self.logger.debug(log_msg)

        return result_path

    # =========================================================================
    # 4. 抽象接口 (Abstract Interface)
    # =========================================================================

    @abstractmethod
    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """[核心抽象方法] 执行具体的几何处理逻辑。

        Args:
            path: 原始路径。
            is_safe_fn: 碰撞检测函数。

        Returns:
            List[NodeType]: 处理后的路径。
        """
        pass
