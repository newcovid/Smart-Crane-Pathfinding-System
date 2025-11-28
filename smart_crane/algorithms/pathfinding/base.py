import logging
import time
import threading
from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any, Optional, Union, TypeVar, Generic
from contextlib import contextmanager

# =============================================================================
# 1. 类型定义 (Type Definitions)
# =============================================================================

# 2D 网格坐标: (row/x, col/y)
Point2D = Tuple[int, int]

# 3D 网格坐标: (row/x, col/y, layer/z)
Point3D = Tuple[int, int, int]

# 泛型节点类型: 这是一个类型变量，允许具体的规划器在继承时指定自己处理的是 2D 还是 3D 坐标。
# 例如: class AStarPlanner(PathPlannerBase[Point2D]): ...
NodeType = TypeVar("NodeType", Point2D, Point3D)

# 网格数据结构类型:
# 它可以是 Python 原生列表 (2D List[List[int]] 或 3D List[List[List[int]]])
# 也可以兼容 numpy.ndarray (Any)，只要它支持下标索引 grid[x][y]。
Grid = Union[List[List[int]], List[List[List[int]]], Any]


# =============================================================================
# 2. 规划器基类 (Base Class)
# =============================================================================


class PathPlannerBase(ABC, Generic[NodeType]):
    """
    【L1 层 - 几何路径规划器基类 (线程安全与3D增强版)】
    (Abstract Base Class for Geometric Path Planners - Thread Safe & 3D Ready)

    这是一个基于【模板方法模式】设计的抽象基类。

    核心职责:
    1. **生命周期管理**: 定义 Init -> Update -> Compute 的标准流程。
    2. **数据结构统一**: 自动适配 2D 或 3D 地图数据，提供统一的 is_obstacle 接口。
    3. **基础设施封装**:
       - **线程安全**: 自动管理共享内存锁 (RLock)，防止并发读写导致的脏数据。
       - **性能监控**: 自动统计核心算法的耗时与效率。

    关键约定:
    1. **共享内存**: 输入的 `grid` 默认是引用传递。如果该 grid 由外部 `MapManager` 维护，
       请务必传入 `MapManager` 持有的那个 `lock` 对象。
    2. **扩展性**: 子类只需实现 `_compute_path_core` 等抽象方法，无需操心锁和监控。
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
        """
        初始化规划器基类。

        Args:
            grid: 初始的网格数据 (引用传递，共享内存)。
                  数据应当是【膨胀后】的配置空间 (C-Space)，即 1 代表不可通行的危险区。
            width_m (float): 场地物理宽度 (X轴/Cols, 米)。
            length_m (float): 场地物理长度 (Y轴/Rows, 米)。
            height_m (float): 场地物理高度 (Z轴/Layers, 米)。如果是 2D 地图，此值为 0。
            resolution (float): 网格分辨率 (米/格)。用于将网格距离转化为物理距离。
            logger (Optional[logging.Logger]): 依赖注入的日志记录器。若为 None 则自动创建。
            grid_lock (Optional[threading.RLock]): 线程锁。
                       - 如果传入 None: 基类将创建一个新的锁（假设无外部并发）。
                       - 如果传入锁对象: 基类将共享这把锁（推荐做法，确保与 MapManager 同步）。
        """
        # 1. 基础数据引用
        # 注意：Python List 是引用传递。通过 self._lock 保证多线程安全。
        self.grid = grid

        # [Thread Safety] 初始化锁
        # 使用 RLock (可重入锁) 允许同一线程多次获取锁，避免递归调用时死锁自己。
        self._lock = grid_lock if grid_lock is not None else threading.RLock()

        # 2. 维度元数据解析 (自动探测是 2D 还是 3D)
        self.rows = 0
        self.cols = 0
        self.layers = 0  # Z轴深度 (0 表示纯 2D)

        if grid and len(grid) > 0:
            self.rows = len(grid)  # Y轴
            if len(grid[0]) > 0:
                self.cols = len(grid[0])  # X轴
                # 检查 grid[0][0] 是否还是一个列表，如果是，说明是 3D 数组
                if isinstance(grid[0][0], list):
                    self.layers = len(grid[0][0])  # Z轴

        # 3. 物理属性记录
        self.width_m = width_m
        self.length_m = length_m
        self.height_m = height_m
        self.resolution = resolution

        # 4. 工具组件
        self.logger = logger or logging.getLogger(self.__class__.__name__)

        # 5. 状态监控指标 (Metrics)
        # 这些数据可以通过 get_stats() 获取，用于前端看板展示或后台分析
        self.last_path: List[NodeType] = []
        self.stats: Dict[str, Any] = {
            "algorithm": self.get_name(),
            "map_dim": "3D" if self.layers > 0 else "2D",
            "grid_size": f"{self.rows}x{self.cols}x{self.layers}",
            "compute_time_ms": 0.0,  # 最近一次计算耗时 (毫秒)
            "nodes_expanded": 0,  # 探索节点数 (衡量算法效率)
            "path_length_steps": 0,  # 路径长度 (步数)
            "replanning_count": 0,  # 重规划次数 (针对 D* Lite 等增量算法)
            "timestamp": 0.0,  # 最后一次更新的时间戳
        }

        self.logger.info(
            f"[{self.get_name()}] 初始化就绪. "
            f"Dim: {self.stats['grid_size']}, "
            f"Phys: {width_m:.1f}m x {length_m:.1f}m x {height_m:.1f}m, "
            f"Lock: {'Shared' if grid_lock else 'Owned'}"
        )

    @property
    def lock(self) -> threading.RLock:
        """获取当前持有的锁对象，供外部协同使用 (例如 MapManager 修改障碍物前需获取此锁)。"""
        return self._lock

    # =========================================================================
    # 3. 抽象接口 (Abstract Interface) - 子类必须实现
    # =========================================================================

    @abstractmethod
    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """
        初始化规划任务。

        Args:
            start: 起点网格坐标
            goal: 终点网格坐标

        Returns:
            bool: 初始化是否成功 (若起终点非法或在障碍物内则返回 False)
        """
        pass

    @abstractmethod
    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """
        处理环境变化 (增量更新)。

        Args:
            changes: 变化列表。
                - 2D格式: [(row, col, new_val), ...]
                - 3D格式: [(row, col, layer, new_val), ...]
                new_val: 0=移除障碍, 1=新增障碍

        注意:
            此方法通常由 MapManager 在持有锁的情况下调用。
            如果是 D* Lite，需要在此处更新 RHS 值。
        """
        pass

    @abstractmethod
    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """
        [核心抽象方法] 实际的算法逻辑实现。

        Args:
            current_pos: 机器人当前的网格位置。

        Returns:
            Optional[List[NodeType]]: 路径点列表，若无解返回 None。

        注意:
            实现此方法时，**无需手动加锁**，也**无需手动调用 performance_monitor**。
            这些都已由基类的 compute_path 方法统一处理。
            你只需要专注于算法本身 (如 A*, Dijkstra 等)。
        """
        pass

    # =========================================================================
    # 4. 模板方法 (Template Method) - 外部调用的标准入口
    # =========================================================================

    def compute_path(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """
        [标准入口] 计算路径。

        此方法实现了【模板方法模式】：
        1. 自动获取线程锁，防止计算过程中地图突变。
        2. 自动启动性能监控上下文，统计耗时。
        3. 调用子类的 `_compute_path_core` 执行实际计算。

        Args:
            current_pos: 机器人当前位置

        Returns:
            路径点列表 或 None
        """
        # 1. 获取大锁 (Snapshot Isolation)
        with self._lock:
            # 2. 启动性能监控 (Performance Monitoring)
            with self.performance_monitor():
                # 3. 调用子类逻辑
                result = self._compute_path_core(current_pos)

                # 4. 记录结果统计
                if result:
                    self.stats["path_length_steps"] = len(result)
                    self.last_path = result
                return result

    # =========================================================================
    # 5. 核心辅助方法 (Helper Methods)
    # =========================================================================

    def get_name(self) -> str:
        """获取算法类名称"""
        return self.__class__.__name__

    def get_stats(self) -> Dict[str, Any]:
        """获取最近一次计算的性能指标"""
        return self.stats

    def is_valid(self, p: Union[Point2D, Point3D]) -> bool:
        """
        检查坐标是否在网格索引范围内 (自适应 2D/3D)。

        Args:
            p: 坐标元组
        Returns:
            bool: True 表示坐标在数组范围内
        """
        # 2D Check
        if len(p) == 2:
            return 0 <= p[0] < self.rows and 0 <= p[1] < self.cols

        # 3D Check
        elif len(p) == 3:
            return (
                0 <= p[0] < self.rows
                and 0 <= p[1] < self.cols
                and 0 <= p[2] < self.layers
            )
        return False

    def is_obstacle(self, p: Union[Point2D, Point3D]) -> bool:
        """
        检查坐标是否是障碍物。

        Precondition:
            调用前应确保 is_valid(p) 为 True，否则可能引发 IndexError。

        Returns:
            bool: True 表示是障碍物 (值==1)，False 表示可通行。
        """
        # 假设 grid 中 1 表示障碍物
        if len(p) == 2:
            return self.grid[p[0]][p[1]] == 1
        elif len(p) == 3:
            return self.grid[p[0]][p[1]][p[2]] == 1
        return True

    def is_safe(self, p: Union[Point2D, Point3D]) -> bool:
        """
        综合安全检查 (最常用)。

        原理:
            先检查边界，再检查障碍物。利用 Python 的短路特性，
            如果 is_valid 返回 False，则不会执行 is_obstacle，避免越界。

        注意:
            此方法内部【没有锁】。因为它是设计给 `_compute_path_core` 内部
            高频循环调用的 (可能每秒调用百万次)。
            线程安全完全依赖于 `compute_path` 最外层的大锁。

        Returns:
            bool: True 表示坐标合法且无障碍，可以通行。
        """
        return self.is_valid(p) and not self.is_obstacle(p)

    @contextmanager
    def performance_monitor(self):
        """
        [Context Manager] 性能监控上下文管理器。

        作用:
            统计代码块的执行时间，并更新到 self.stats['compute_time_ms']。
            如果耗时超过阈值 (如 100ms)，会自动记录警告日志。
        """
        start_time = time.perf_counter()
        try:
            yield
        finally:
            end_time = time.perf_counter()
            elapsed_ms = (end_time - start_time) * 1000.0
            self.stats["compute_time_ms"] = elapsed_ms
            self.stats["timestamp"] = time.time()

            # 阈值告警: 如果计算时间超过 100ms，对于实时系统来说可能太慢了
            # 仅在 Debug 模式或确实太慢时打印，避免刷屏
            if elapsed_ms > 100.0:
                self.logger.warning(
                    f"[{self.get_name()}] 性能告警: 耗时 {elapsed_ms:.2f}ms "
                    f"(Explored: {self.stats.get('nodes_expanded', 'N/A')})"
                )

    @contextmanager
    def locked_context(self):
        """
        [Context Manager] 手动线程安全上下文。

        场景:
            当外部调用者 (如 MapManager) 需要执行一系列原子操作时使用。
            例如：连续添加10个障碍物，然后立即重规划，不希望中间被插队。

        用法:
            with planner.locked_context():
                planner.update_obstacles(changes)
                path = planner.compute_path(current)
        """
        with self._lock:
            yield
