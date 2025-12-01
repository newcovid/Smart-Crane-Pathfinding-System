import math
import logging
from typing import Tuple, List, Any, Optional, Union, TYPE_CHECKING

from smart_crane.core.config import Settings
from smart_crane.core.constants import (
    SHAPE_CIRCLE,
    MIN_SAFE_HEIGHT_OFFSET,
    GRID_MARGIN_BUFFER,
)

# 使用 TYPE_CHECKING 避免运行时循环导入
if TYPE_CHECKING:
    from smart_crane.core.map_manager import WorkshopMapManager

# 类型别名定义
GridNode = Union[Tuple[int, int], Tuple[int, int, int]]
Point3D = Tuple[float, float, float]
Grid2D = List[List[int]]
Grid3D = List[List[List[int]]]


class GridAdapter:
    """负责物理坐标系与算法网格坐标系之间的转换适配器。

    该组件支持智能日志嵌套：如果由上层组件（如 TrajectoryPlanner）初始化并传入 Logger，
    它会自动创建一个子 Logger（如 TrajectoryPlanner.Grid），从而在日志中保留调用链上下文。

    Attributes:
        map_mgr (WorkshopMapManager): 地图管理器实例引用。
        logger (logging.Logger): 组件专属的日志记录器。
    """

    def __init__(
        self,
        map_mgr: "WorkshopMapManager",
        logger: Optional[logging.Logger] = None,
    ):
        """初始化网格适配器。

        Args:
            map_mgr (WorkshopMapManager): 地图管理器引用。
            logger (Optional[logging.Logger]): 父级日志记录器。
                                             如果传入，将创建名为 {Parent}.GridAdapter 的子记录器。
                                             如果不传，将创建独立的 "GridAdapter" 记录器。
        """
        self.map_mgr = map_mgr

        # [日志优化] 实现智能嵌套
        if logger:
            # 如果传入了父级 Logger (例如 "TrajectoryPlanner")
            # 使用 getChild 创建子 Logger，名称变为 "TrajectoryPlanner.GridAdapter"
            self.logger = logger.getChild(self.__class__.__name__)
        else:
            # 否则使用默认的独立名称
            self.logger = logging.getLogger(self.__class__.__name__)

        self.logger.debug("适配器初始化完成。")

    def prepare_grids(
        self, settings: Settings
    ) -> Tuple[Union[Grid2D, Grid3D], Grid2D, float]:
        """根据配置生成用于规划和可视化的网格数据。

        Args:
            settings (Settings): 全局配置对象，包含吊具尺寸和安全策略。

        Returns:
            Tuple[Union[Grid2D, Grid3D], Grid2D, float]:
                - planning_grid: 用于核心算法规划的网格（2D 或 3D）。
                - visualization_grid: 用于前端展示的 2D 投影网格。
                - grid_height_m: 网格的物理高度（仅在 3D 模式下有效，2D 模式为 0.0）。
        """
        self.logger.info("开始构建规划网格...")

        # 1. 计算水平膨胀半径
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length

        if shape == SHAPE_CIRCLE:
            radius_m = w / 2.0
        else:
            # 矩形取对角线的一半作为旋转包络半径
            radius_m = math.hypot(w, l) / 2.0

        xy_margin = radius_m / self.map_mgr.resolution_m

        self.logger.debug(
            f"几何参数: Shape={shape}, R={radius_m:.2f}m, Margin={xy_margin:.2f}px"
        )

        # 2. 计算垂直安全边距
        user_z_margin = settings.crane.z_safety_margin
        crane_h = settings.crane.footprint_height
        z_margin_obs = user_z_margin + (crane_h / 2.0)

        # 3. 策略分支
        is_fixed_height = settings.crane.enable_fixed_height_cruise
        is_infinite_obs = settings.crane.obstacle_infinite_height

        if is_fixed_height:
            # === 2.5D 模式 ===
            cruise_z = settings.crane.safe_travel_z_m
            check_z = None if is_infinite_obs else cruise_z

            self.logger.info(f"构建 2.5D 巡航网格 (巡航高度: {cruise_z}m)")

            grid_2d = self.map_mgr.get_2d_projection_grid(
                xy_margin=xy_margin, check_z=check_z, z_margin=z_margin_obs
            )
            return grid_2d, grid_2d, 0.0
        else:
            # === 3D 模式 ===
            z_margin_ceil = crane_h / 2.0

            self.logger.info(
                f"构建 3D 体素网格 (Z-Margin: Obs={z_margin_obs:.1f}m, Ceil={z_margin_ceil:.1f}m)"
            )

            grid_3d = self.map_mgr.get_3d_voxel_grid(
                xy_margin=xy_margin,
                z_margin_obs=z_margin_obs,
                z_margin_ceil=z_margin_ceil,
                is_infinite=is_infinite_obs,
            )

            # 可视化网格始终是 2D 投影，方便前端渲染
            grid_vis = self.map_mgr.get_2d_projection_grid(
                xy_margin=xy_margin, check_z=None, z_margin=z_margin_obs
            )
            return grid_3d, grid_vis, self.map_mgr.height_m

    def calculate_incremental_changes(
        self,
        settings: Settings,
        old_grid: Any,
        new_grid: Any,
        obstacle_bbox: Tuple[float, float, float, float, float],
    ) -> List[Tuple[int, ...]]:
        """计算网格增量变化 (Diff)，用于动态规划算法 (如 D* Lite)。

        Args:
            settings (Settings): 全局配置。
            old_grid (Any): 更新前的网格数据。
            new_grid (Any): 更新后的网格数据。
            obstacle_bbox (Tuple[float, float, float, float, float]):
                障碍物的包围盒 (x, y, w, h, z)。

        Returns:
            List[Tuple[int, ...]]: 变更后的节点列表。
                - 2D: [(r, c, val), ...]
                - 3D: [(r, c, l, val), ...]
        """
        changes = []
        x, y, w, h, z = obstacle_bbox

        # 计算受影响的网格区域 (ROI)
        shape = settings.crane.footprint_shape
        c_w = settings.crane.footprint_width
        c_l = settings.crane.footprint_length

        if shape == SHAPE_CIRCLE:
            radius_m = c_w / 2.0
        else:
            radius_m = math.hypot(c_w, c_l) / 2.0

        # 加上膨胀缓冲区的受影响范围
        margin_grid = (
            int(math.ceil(radius_m / self.map_mgr.resolution_m)) + GRID_MARGIN_BUFFER
        )

        r_s, c_s, _ = self.map_mgr.world_to_grid(x, y, 0.0)
        r_e, c_e, _ = self.map_mgr.world_to_grid(x + w, y + h, 0.0)

        r_start = max(0, r_s - margin_grid)
        r_end = min(self.map_mgr.rows, r_e + 1 + margin_grid)
        c_start = max(0, c_s - margin_grid)
        c_end = min(self.map_mgr.cols, c_e + 1 + margin_grid)

        self.logger.debug(
            f"计算区域 ROI: Rows[{r_start}:{r_end}], Cols[{c_start}:{c_end}]"
        )

        is_fixed_height = settings.crane.enable_fixed_height_cruise
        is_infinite_obs = settings.crane.obstacle_infinite_height

        count = 0
        if is_fixed_height:
            # === 2D Diff ===
            should_update = True
            if not is_infinite_obs:
                cruise_z = settings.crane.safe_travel_z_m
                crane_h = settings.crane.footprint_height
                z_margin_obs = settings.crane.z_safety_margin + (crane_h / 2.0)
                z_threshold = cruise_z - z_margin_obs

                if z <= z_threshold:
                    should_update = False
                    self.logger.info("障碍物低于巡航层，忽略增量更新。")

            if should_update:
                for r in range(r_start, r_end):
                    for c in range(c_start, c_end):
                        val_new = new_grid[r][c]
                        val_old = 0
                        if old_grid is not None:
                            if 0 <= r < len(old_grid) and 0 <= c < len(old_grid[0]):
                                val_old = old_grid[r][c]

                        if val_new != val_old:
                            changes.append((r, c, val_new))
                            count += 1
        else:
            # === 3D Diff ===
            l_s = 0
            l_e = self.map_mgr.layers

            if not is_infinite_obs:
                _, _, l_start_idx = self.map_mgr.world_to_grid(0.0, 0.0, 0.0)
                _, _, l_end_idx = self.map_mgr.world_to_grid(0.0, 0.0, z)
                l_s = max(0, l_start_idx - margin_grid)
                l_e = min(self.map_mgr.layers, l_end_idx + 1 + margin_grid)

            for r in range(r_start, r_end):
                for c in range(c_start, c_end):
                    for l in range(l_s, l_e):
                        val_new = new_grid[r][c][l]
                        val_old = 0
                        if old_grid is not None:
                            if (
                                0 <= r < len(old_grid)
                                and 0 <= c < len(old_grid[0])
                                and 0 <= l < len(old_grid[0][0])
                            ):
                                val_old = old_grid[r][c][l]

                        if val_new != val_old:
                            changes.append((r, c, l, val_new))
                            count += 1

        self.logger.info(f"检测到 {count} 个网格状态变化。")
        return changes

    def get_initial_grid_nodes(
        self, start: Point3D, end: Point3D, settings: Settings
    ) -> Tuple[GridNode, GridNode, float]:
        """计算起点和终点在算法网格中的坐标，以及巡航高度。

        Args:
            start (Point3D): 起点物理坐标 (x, y, z)。
            end (Point3D): 终点物理坐标 (x, y, z)。
            settings (Settings): 配置对象。

        Returns:
            Tuple[GridNode, GridNode, float]: (起点网格坐标, 终点网格坐标, 巡航物理高度)。
        """
        s_x, s_y, s_z = start
        e_x, e_y, e_z = end

        cruise_z = 0.0
        is_fixed_height = settings.crane.enable_fixed_height_cruise

        if is_fixed_height:
            cruise_z = settings.crane.safe_travel_z_m
            # 2.5D 模式下，z 坐标在网格中被忽略，仅取前两维
            start_grid = self.map_mgr.world_to_grid(s_x, s_y, 0.0)[:2]
            goal_grid = self.map_mgr.world_to_grid(e_x, e_y, 0.0)[:2]
            self.logger.debug(f"2.5D 映射: {start} -> {start_grid}")
        else:
            # 3D 模式：确保起终点不低于最小安全高度
            min_safe = settings.crane.z_safety_margin + MIN_SAFE_HEIGHT_OFFSET
            plan_s_z = max(s_z, min_safe)
            plan_e_z = max(e_z, min_safe)

            if plan_s_z > s_z or plan_e_z > e_z:
                self.logger.warning(
                    f"高度自动修正 (低于最小安全高度 {min_safe}m): "
                    f"Start: {s_z:.1f}->{plan_s_z:.1f}, End: {e_z:.1f}->{plan_e_z:.1f}"
                )

            start_grid = self.map_mgr.world_to_grid(s_x, s_y, plan_s_z)
            goal_grid = self.map_mgr.world_to_grid(e_x, e_y, plan_e_z)
            self.logger.debug(f"3D 映射: {start} -> {start_grid}")

        return start_grid, goal_grid, cruise_z

    def grid_to_world_smart(self, node: GridNode, override_z: float) -> Point3D:
        """智能网格转物理坐标。

        Args:
            node (GridNode): 网格坐标 (2D 或 3D)。
            override_z (float): 如果是 2D 网格，强制使用的 Z 轴高度。

        Returns:
            Point3D: 转换后的物理坐标 (x, y, z)。
        """
        if len(node) == 2:
            wx, wy, _ = self.map_mgr.grid_to_world(node[0], node[1], 0)
            return (wx, wy, override_z)
        else:
            wx, wy, wz = self.map_mgr.grid_to_world(node[0], node[1], node[2])
            return (wx, wy, wz)
