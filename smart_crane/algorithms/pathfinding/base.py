import time
import logging
import threading
from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any, Optional, Union, TypeVar, Generic
from contextlib import contextmanager

# 类型定义
Point2D = Tuple[int, int]
Point3D = Tuple[int, int, int]
NodeType = TypeVar("NodeType", Point2D, Point3D)
Grid = Union[List[List[int]], List[List[List[int]]], Any]


class PathPlannerBase(ABC, Generic[NodeType]):
    """路径规划器抽象基类 (L1层)。

    定义了所有路径规划算法（如 A*、D* Lite）必须实现的通用接口。
    负责统一的生命周期管理、性能监控、日志上下文建立和线程锁管理。

    Attributes:
        grid (Grid): 规划使用的网格数据（引用）。
        width_m (float): 地图物理宽度。
        length_m (float): 地图物理长度。
        height_m (float): 地图物理高度。
        resolution (float): 网格分辨率。
        logger (logging.Logger): 专用的日志记录器。
        stats (Dict[str, Any]): 运行时性能统计数据。
    """

    def __init__(
        self,
        grid: Grid,
        width_m: float,
        length_m: float,
        height_m: float = 0.0,
        resolution: float = 0.5,
        logger: Optional[logging.Logger] = None,
        grid_lock: Optional[threading.RLock] = None,
    ):
        """初始化规划器基类。

        Args:
            grid: 网格数据（2D 列表或 3D 列表）。
            width_m: 地图宽度（米）。
            length_m: 地图长度（米）。
            height_m: 地图高度（米）。
            resolution: 分辨率（米/格）。
            logger: 父级日志记录器。如果提供，将创建子记录器以保持上下文。
            grid_lock: 共享的网格线程锁。
        """
        self._grid = grid
        # 优先使用传入的锁，否则创建新锁
        self._lock = grid_lock if grid_lock is not None else threading.RLock()

        # 解析网格维度
        self.rows = 0
        self.cols = 0
        self.layers = 0

        if grid and len(grid) > 0:
            self.rows = len(grid)
            if len(grid[0]) > 0:
                self.cols = len(grid[0])
                # 简单的维度嗅探
                if isinstance(grid[0][0], list):
                    self.layers = len(grid[0][0])

        self.width_m = width_m
        self.length_m = length_m
        self.height_m = height_m
        self.resolution = resolution

        # [日志优化] 智能嵌套 Logger 名称
        # 例如: Parent="TrajectoryPlanner" -> Self="TrajectoryPlanner.AStarPlanner"
        cls_name = self.__class__.__name__
        if logger:
            self.logger = logger.getChild(cls_name)
        else:
            self.logger = logging.getLogger(cls_name)

        self.last_path: List[NodeType] = []
        self.stats: Dict[str, Any] = {
            "algorithm": self.get_name(),
            "map_dim": "3D" if self.layers > 0 else "2D",
            "grid_size": f"{self.rows}x{self.cols}x{self.layers}",
            "compute_time_ms": 0.0,
            "nodes_expanded": 0,
            "path_length_steps": 0,
            "replanning_count": 0,
            "timestamp": 0.0,
        }

        self.logger.info(
            f"初始化就绪. Dim: {self.stats['grid_size']}, "
            f"Lock: {'Shared' if grid_lock else 'Owned'}"
        )

    @property
    def grid(self) -> Grid:
        """获取当前网格引用。"""
        return self._grid

    @grid.setter
    def grid(self, value: Grid):
        """更新网格引用。子类可重写此属性以同步到底层（如 Rust）。"""
        self._grid = value

    @property
    def lock(self) -> threading.RLock:
        """获取线程锁。"""
        return self._lock

    @property
    def supports_incremental(self) -> bool:
        """是否支持增量更新。

        Returns:
            bool: True 表示算法支持动态障碍物更新（如 D* Lite），
                  False 表示每次需全量重算（如 A*）。
        """
        return False

    @abstractmethod
    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """初始化规划任务（设置起点和终点）。

        Args:
            start: 起点坐标。
            goal: 终点坐标。

        Returns:
            bool: 初始化是否成功（例如起点是否合法）。
        """
        pass

    @abstractmethod
    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """处理障碍物变更（增量更新）。

        Args:
            changes: 变更列表，每项为 (x, y, val) 或 (x, y, z, val)。
        """
        pass

    @abstractmethod
    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """[内部接口] 执行核心寻路逻辑。

        Args:
            current_pos: 当前机器人位置。

        Returns:
            Optional[List[NodeType]]: 路径节点列表，无解返回 None。
        """
        pass

    def compute_path(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """计算路径（包含加锁和性能监控）。

        Args:
            current_pos: 当前位置。

        Returns:
            Optional[List[NodeType]]: 计算出的路径。
        """
        with self._lock:
            with self.performance_monitor():
                result = self._compute_path_core(current_pos)
                if result:
                    self.stats["path_length_steps"] = len(result)
                    self.last_path = result
                else:
                    self.logger.warning(f"未能找到从 {current_pos} 到目标的路径。")
                return result

    def get_name(self) -> str:
        """获取算法名称。"""
        return self.__class__.__name__

    def get_stats(self) -> Dict[str, Any]:
        """获取性能统计信息。"""
        return self.stats

    def is_valid(self, p: Union[Point2D, Point3D]) -> bool:
        """检查坐标是否在网格范围内。"""
        if len(p) == 2:
            return 0 <= p[0] < self.rows and 0 <= p[1] < self.cols
        elif len(p) == 3:
            return (
                0 <= p[0] < self.rows
                and 0 <= p[1] < self.cols
                and 0 <= p[2] < self.layers
            )
        return False

    def is_obstacle(self, p: Union[Point2D, Point3D]) -> bool:
        """检查坐标是否为障碍物。

        Args:
            p: 坐标点。

        Returns:
            bool: True 表示是障碍物，False 表示可行。
        """
        # 注意：这里假设网格中 1 代表障碍物
        if len(p) == 2:
            return self.grid[p[0]][p[1]] == 1
        elif len(p) == 3:
            return self.grid[p[0]][p[1]][p[2]] == 1
        return True

    def is_safe(self, p: Union[Point2D, Point3D]) -> bool:
        """检查坐标是否安全（既在范围内又不是障碍物）。"""
        return self.is_valid(p) and not self.is_obstacle(p)

    @contextmanager
    def performance_monitor(self):
        """上下文管理器：用于测量代码块执行时间并更新 stats。"""
        start_time = time.perf_counter()
        try:
            yield
        finally:
            end_time = time.perf_counter()
            elapsed_ms = (end_time - start_time) * 1000.0
            self.stats["compute_time_ms"] = elapsed_ms
            self.stats["timestamp"] = time.time()

            # 仅在耗时异常时打印日志，避免刷屏
            if elapsed_ms > 200.0:
                self.logger.warning(
                    f"性能告警: 耗时 {elapsed_ms:.2f}ms "
                    f"(Explored: {self.stats.get('nodes_expanded', 'N/A')})"
                )

    @contextmanager
    def locked_context(self):
        """获取内部锁的上下文管理器。"""
        with self._lock:
            yield
