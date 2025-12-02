import logging
import threading
from typing import List, Any, Optional, Union, TYPE_CHECKING

from smart_crane.core.config import Settings
from smart_crane.core.constants import ALGO_DSLITE
from smart_crane.algorithms.pathfinding.base import PathPlannerBase
from smart_crane.algorithms.pathfinding.astar import AStarPlanner
from smart_crane.algorithms.pathfinding.dslite import DLitePlanner
from smart_crane.algorithms.post_processing.base import PathPostProcessor
from smart_crane.algorithms.post_processing.greedy import GreedyShortcutProcessor
from smart_crane.algorithms.post_processing.bezier import BezierSmoothProcessor

# [类型注解优化] 仅在类型检查阶段导入 MapManager，避免运行时循环引用
if TYPE_CHECKING:
    from smart_crane.core.map_manager import WorkshopMapManager

# 类型别名定义
Grid2D = List[List[int]]
Grid3D = List[List[List[int]]]
Grid = Union[Grid2D, Grid3D]


class PlannerFactory:
    """规划器工厂 (Planner Factory)。

    负责根据全局配置（Settings）实例化具体的路径规划算法（Core Planner）
    和后处理管道（Post Processors）。
    作为对象创建的集中管理点，解耦了算法的具体实现与调用逻辑。
    """

    @staticmethod
    def create_core_planner(
        settings: Settings,
        grid: Grid,
        map_mgr: "WorkshopMapManager",
        grid_height_m: float,
        logger: Optional[logging.Logger],
        grid_lock: threading.RLock,
    ) -> PathPlannerBase:
        """创建核心路径规划器实例。

        Args:
            settings (Settings): 全局配置对象。
            grid (Grid): 用于规划的网格数据（2D 或 3D）。
            map_mgr (WorkshopMapManager): 地图管理器实例引用。
            grid_height_m (float): 网格物理高度（仅 3D 模式使用）。
            logger (Optional[logging.Logger]): 父级日志记录器。
            grid_lock (threading.RLock): 共享的网格线程锁。

        Returns:
            PathPlannerBase: 初始化完毕的规划器实例。
        """
        algo_type = settings.planner.algorithm
        use_octile = settings.planner.use_3d_octile
        h_weight = settings.planner.heuristic_weight
        enable_rust = settings.planner.enable_rust_core

        # [日志优化] 建立子 Logger: {Parent}.PlannerFactory
        if logger:
            factory_logger = logger.getChild("PlannerFactory")
        else:
            factory_logger = logging.getLogger("PlannerFactory")

        factory_logger.info(f"正在创建规划器: Type={algo_type}, Rust={enable_rust}")

        common_args = {
            "grid": grid,
            "width_m": map_mgr.width_m,
            "length_m": map_mgr.length_m,
            "height_m": grid_height_m,
            "resolution": map_mgr.resolution_m,
            "logger": logger,  # 将父级 logger 传给具体算法，算法内部会进行 getChild
            "grid_lock": grid_lock,
            "use_octile_3d": use_octile,
            "heuristic_weight": h_weight,
        }

        if algo_type == ALGO_DSLITE:
            return DLitePlanner(**common_args, enable_rust=enable_rust)
        else:
            # 默认为 A*
            return AStarPlanner(**common_args, enable_rust=enable_rust)

    @staticmethod
    def create_post_processors(
        settings: Settings,
        map_mgr: "WorkshopMapManager",
        logger: Optional[logging.Logger] = None,
    ) -> List[PathPostProcessor]:
        """创建后处理管道列表。

        根据配置决定启用哪些路径平滑或优化器。
        在此注入 MapManager 和 Settings，以便后处理器能调用 Rust 核心。

        Args:
            settings (Settings): 全局配置对象。
            map_mgr (WorkshopMapManager): 地图管理器（必须提供）。
            logger (Optional[logging.Logger]): 父级日志记录器。

        Returns:
            List[PathPostProcessor]: 按执行顺序排列的后处理器实例列表。
        """
        processors = []

        # [日志优化] 获取工厂日志用于记录构建过程
        if logger:
            factory_logger = logger.getChild("PlannerFactory")
        else:
            factory_logger = logging.getLogger("PlannerFactory")

        # 1. 贪婪捷径优化 (通常放在第一步，大幅减少节点数)
        if settings.post_process.enable_shortcut:
            factory_logger.debug("启用后处理: GreedyShortcutProcessor")
            processors.append(
                GreedyShortcutProcessor(
                    map_mgr=map_mgr, settings=settings, logger=logger
                )
            )

        # 2. 贝塞尔平滑 (通常放在最后，美化路径)
        if settings.post_process.enable_bezier:
            factory_logger.debug("启用后处理: BezierSmoothProcessor")
            processors.append(
                BezierSmoothProcessor(
                    map_mgr=map_mgr,
                    settings=settings,
                    smoothness=settings.post_process.bezier_smoothness,
                    segments=settings.post_process.bezier_segments,
                    logger=logger,
                )
            )

        factory_logger.info(f"后处理管道构建完成: {len(processors)} 个处理器。")
        return processors
