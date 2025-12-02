import math
import logging
import time
from typing import Tuple, List, Any, Optional, Union, TYPE_CHECKING

from smart_crane.core.config import Settings
from smart_crane.core.rust_bridge import RustBackend
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

    支持 Python 原生实现与 Rust 高性能实现。当 Rust 核心可用时，
    会将繁重的网格计算和坐标转换逻辑下沉到 Rust 层。
    """

    def __init__(
        self,
        map_mgr: "WorkshopMapManager",
        logger: Optional[logging.Logger] = None,
    ):
        """初始化网格适配器。"""
        self.map_mgr = map_mgr

        if logger:
            self.logger = logger.getChild(self.__class__.__name__)
        else:
            self.logger = logging.getLogger(self.__class__.__name__)

        self.RustAdapter = RustBackend.get_grid_adapter_class()
        mode = "Rust" if (self.RustAdapter and self.map_mgr.rust_map) else "Python"
        self.logger.debug(f"适配器初始化完成 (Mode: {mode})。")

    def prepare_grids(
        self, settings: Settings
    ) -> Tuple[Union[Grid2D, Grid3D], Grid2D, float]:
        """根据配置生成用于规划和可视化的网格数据。"""
        # 1. 计算水平膨胀半径
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length

        if shape == SHAPE_CIRCLE:
            radius_m = w / 2.0
        else:
            radius_m = math.hypot(w, l) / 2.0

        xy_margin = radius_m / self.map_mgr.resolution_m

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
        """计算网格增量变化 (Diff)。

        **Rust 加速**:
        如果可用，将调用 `RustGridAdapter.calculate_incremental_changes`。
        """
        x, y, w, h, z = obstacle_bbox

        # 1. 计算 ROI 边界 (Common Logic)
        shape = settings.crane.footprint_shape
        c_w = settings.crane.footprint_width
        c_l = settings.crane.footprint_length
        if shape == SHAPE_CIRCLE:
            radius_m = c_w / 2.0
        else:
            radius_m = math.hypot(c_w, c_l) / 2.0

        margin_grid = (
            int(math.ceil(radius_m / self.map_mgr.resolution_m)) + GRID_MARGIN_BUFFER
        )

        r_s, c_s, _ = self.map_mgr.world_to_grid(x, y, 0.0)
        r_e, c_e, _ = self.map_mgr.world_to_grid(x + w, y + h, 0.0)

        r_start = max(0, r_s - margin_grid)
        r_end = min(self.map_mgr.rows, r_e + 1 + margin_grid)
        c_start = max(0, c_s - margin_grid)
        c_end = min(self.map_mgr.cols, c_e + 1 + margin_grid)

        is_fixed_height = settings.crane.enable_fixed_height_cruise
        is_infinite_obs = settings.crane.obstacle_infinite_height

        l_s = 0
        l_e = self.map_mgr.layers
        if not is_fixed_height and not is_infinite_obs:
            _, _, l_start_idx = self.map_mgr.world_to_grid(0.0, 0.0, 0.0)
            _, _, l_end_idx = self.map_mgr.world_to_grid(0.0, 0.0, z)
            l_s = max(0, l_start_idx - margin_grid)
            l_e = min(self.map_mgr.layers, l_end_idx + 1 + margin_grid)

        # 2. 尝试使用 Rust 加速
        if self.RustAdapter and self.map_mgr.rust_map:
            try:
                roi_bounds = (r_start, r_end, c_start, c_end, l_s, l_e)
                is_3d = not is_fixed_height

                should_call_rust = True
                if is_fixed_height and not is_infinite_obs:
                    cruise_z = settings.crane.safe_travel_z_m
                    crane_h = settings.crane.footprint_height
                    z_margin_obs = settings.crane.z_safety_margin + (crane_h / 2.0)
                    if z <= (cruise_z - z_margin_obs):
                        should_call_rust = False

                if should_call_rust:
                    t0 = time.perf_counter()
                    changes = self.RustAdapter.calculate_incremental_changes(
                        old_grid,
                        new_grid,
                        self.map_mgr.rust_map,
                        roi_bounds,
                        is_3d,
                    )
                    t1 = time.perf_counter()
                    self.logger.debug(
                        f"[GridAdapter] Rust增量Diff计算完毕: {(t1-t0)*1000:.2f}ms"
                    )
                    return changes
                else:
                    return []
            except Exception as e:
                self.logger.error(f"Rust 增量计算失败，回退 Python: {e}")

        # 3. Python 回退实现
        t0 = time.perf_counter()
        changes = []
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

        t1 = time.perf_counter()
        self.logger.info(
            f"[GridAdapter] Python增量Diff计算完毕: {(t1-t0)*1000:.2f}ms (Changes: {count})"
        )
        return changes

    def get_initial_grid_nodes(
        self, start: Point3D, end: Point3D, settings: Settings
    ) -> Tuple[GridNode, GridNode, float]:
        """计算起点和终点在算法网格中的坐标。"""
        # 尝试 Rust
        if self.RustAdapter and self.map_mgr.rust_map:
            try:
                settings_tuple = (
                    settings.crane.enable_fixed_height_cruise,
                    settings.crane.safe_travel_z_m,
                    settings.crane.z_safety_margin,
                    MIN_SAFE_HEIGHT_OFFSET,
                )
                return self.RustAdapter.get_initial_grid_nodes(
                    start, end, self.map_mgr.rust_map, settings_tuple
                )
            except Exception as e:
                self.logger.error(f"Rust 坐标计算失败，回退 Python: {e}")

        # Python Fallback
        s_x, s_y, s_z = start
        e_x, e_y, e_z = end

        cruise_z = 0.0
        is_fixed_height = settings.crane.enable_fixed_height_cruise

        if is_fixed_height:
            cruise_z = settings.crane.safe_travel_z_m
            start_grid = self.map_mgr.world_to_grid(s_x, s_y, 0.0)[:2]
            goal_grid = self.map_mgr.world_to_grid(e_x, e_y, 0.0)[:2]
        else:
            min_safe = settings.crane.z_safety_margin + MIN_SAFE_HEIGHT_OFFSET
            plan_s_z = max(s_z, min_safe)
            plan_e_z = max(e_z, min_safe)
            start_grid = self.map_mgr.world_to_grid(s_x, s_y, plan_s_z)
            goal_grid = self.map_mgr.world_to_grid(e_x, e_y, plan_e_z)

        return start_grid, goal_grid, cruise_z

    def grid_to_world_smart(self, node: GridNode, override_z: float) -> Point3D:
        """智能网格转物理坐标。"""
        if self.RustAdapter and self.map_mgr.rust_map:
            try:
                return self.RustAdapter.grid_to_world_smart(
                    node, override_z, self.map_mgr.rust_map
                )
            except Exception:
                pass

        if len(node) == 2:
            wx, wy, _ = self.map_mgr.grid_to_world(node[0], node[1], 0)
            return (wx, wy, override_z)
        else:
            wx, wy, wz = self.map_mgr.grid_to_world(node[0], node[1], node[2])
            return (wx, wy, wz)
