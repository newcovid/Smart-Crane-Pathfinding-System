import math
import threading
import copy
import logging
from typing import List, Tuple, Optional, Dict, Union, Any

try:
    import numpy as np
    from scipy.ndimage import distance_transform_edt

    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

Grid2D = List[List[int]]
Grid3D = List[List[List[int]]]


class WorkshopMapManager:
    """
    车间地图管理器 (3D Voxel Engine) - V3.6 Ceiling Check Fix

    更新内容:
    1. [修复] 3D 物理边界检查逻辑：
       - 区分了【障碍物/地板安全余量】和【天花板防穿模余量】。
       - 天花板检查不再包含巨大的 Safety Margin，仅检查 (Z + 半高 > MapHeight)，
         允许起重机升至最高点附近作业。
    """

    def __init__(
        self,
        width_m: float,
        length_m: float,
        resolution_m: float,
        height_m: float = 20.0,
        logger: Optional[Any] = None,
    ):
        self.width_m = width_m
        self.length_m = length_m
        self.height_m = height_m
        self.resolution_m = resolution_m

        if hasattr(logger, "info"):
            self.log = logger.info
        elif callable(logger):
            self.log = logger
        else:
            self.log = print

        self._lock = threading.RLock()

        self.cols = int(math.ceil(width_m / resolution_m))
        self.rows = int(math.ceil(length_m / resolution_m))
        self.layers = int(math.ceil(height_m / resolution_m))

        self.static_obstacles: Dict[str, dict] = {}
        self.dynamic_obstacles: Dict[str, dict] = {}

        self._inflated_grid_caches: Dict[Tuple, Grid2D] = {}
        self._3d_grid_caches: Dict[Tuple, Grid3D] = {}

        self.log(
            f"[MapMgr] Init: {self.width_m}x{self.length_m}x{self.height_m}m "
            f"Grid: {self.rows}x{self.cols}x{self.layers}"
        )

    def get_full_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "width_m": self.width_m,
                "length_m": self.length_m,
                "height_m": self.height_m,
                "resolution_m": self.resolution_m,
                "static_obstacles": copy.deepcopy(self.static_obstacles),
                "dynamic_obstacles": copy.deepcopy(self.dynamic_obstacles),
            }

    def _invalidate_cache(self) -> None:
        self._inflated_grid_caches.clear()
        self._3d_grid_caches.clear()

    def world_to_grid(
        self, x_m: float, y_m: float, z_m: float = 0.0
    ) -> Tuple[int, int, int]:
        col = int(x_m / self.resolution_m)
        row = int(y_m / self.resolution_m)
        layer = int(z_m / self.resolution_m)
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        layer = max(0, min(layer, self.layers - 1))
        return (row, col, layer)

    def grid_to_world(
        self, row: int, col: int, layer: int = 0
    ) -> Tuple[float, float, float]:
        x_m = (col + 0.5) * self.resolution_m
        y_m = (row + 0.5) * self.resolution_m
        z_m = (layer + 0.5) * self.resolution_m
        return (x_m, y_m, z_m)

    # --- Obstacle Management ---

    def add_static_obstacle(
        self, obs_id: str, x: float, y: float, w: float, h: float, z: float = 100.0
    ):
        with self._lock:
            self.static_obstacles[obs_id] = {
                "x_m": x,
                "y_m": y,
                "w_m": w,
                "h_m": h,
                "z_m": z,
            }
            self._invalidate_cache()
            self.log(f"[MapMgr] StaticObs Added: {obs_id} at ({x},{y}) {w}x{h}")

    def remove_static_obstacle(self, obs_id: str):
        with self._lock:
            if obs_id in self.static_obstacles:
                del self.static_obstacles[obs_id]
                self._invalidate_cache()

    def update_dynamic_obstacle(
        self, obs_id: str, x: float, y: float, w: float, h: float, z: float = 100.0
    ):
        with self._lock:
            self.dynamic_obstacles[obs_id] = {
                "x_m": x,
                "y_m": y,
                "w_m": w,
                "h_m": h,
                "z_m": z,
            }
            self._invalidate_cache()

    def remove_dynamic_obstacle(self, obs_id: str):
        with self._lock:
            if obs_id in self.dynamic_obstacles:
                del self.dynamic_obstacles[obs_id]
                self._invalidate_cache()

    def find_obstacle_near(self, x_m: float, y_m: float) -> Optional[Tuple[str, str]]:
        with self._lock:
            for oid, o in self.dynamic_obstacles.items():
                if (
                    o["x_m"] <= x_m <= o["x_m"] + o["w_m"]
                    and o["y_m"] <= y_m <= o["y_m"] + o["h_m"]
                ):
                    return (oid, "dynamic")
            for oid, o in self.static_obstacles.items():
                if (
                    o["x_m"] <= x_m <= o["x_m"] + o["w_m"]
                    and o["y_m"] <= y_m <= o["y_m"] + o["h_m"]
                ):
                    return (oid, "static")
            return None

    # --- Grid Generation ---

    def _mark_obstacle_area(self, grid: Grid2D, x: float, y: float, w: float, h: float):
        r_s, c_s, _ = self.world_to_grid(x, y)
        r_e, c_e, _ = self.world_to_grid(x + w - 0.01, y + h - 0.01)
        for r in range(r_s, r_e + 1):
            for c in range(c_s, c_e + 1):
                if 0 <= r < self.rows and 0 <= c < self.cols:
                    grid[r][c] = 1

    def _get_base_grid_2d(
        self, check_z_height: Optional[float], z_safety_margin: float
    ) -> Grid2D:
        grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        all_obs = list(self.static_obstacles.values()) + list(
            self.dynamic_obstacles.values()
        )

        z_threshold = None
        if check_z_height is not None:
            z_threshold = check_z_height - z_safety_margin

        for o in all_obs:
            obs_h = o.get("z_m", 100.0)
            if z_threshold is None or obs_h > z_threshold:
                self._mark_obstacle_area(grid, o["x_m"], o["y_m"], o["w_m"], o["h_m"])
        return grid

    def get_inflated_grid(
        self, xy_margin: float, check_z: Optional[float] = None, z_margin: float = 0.0
    ) -> Grid2D:
        with self._lock:
            key = (round(xy_margin, 3), check_z, round(z_margin, 3))
            if key in self._inflated_grid_caches:
                return self._inflated_grid_caches[key]

            base = self._get_base_grid_2d(check_z, z_margin)
            inflated = _create_inflated_grid_2d(base, xy_margin)

            self._inflated_grid_caches[key] = inflated
            return inflated

    def get_3d_inflated_grid(
        self, xy_margin: float, z_margin_obs: float, z_margin_ceil: float
    ) -> Grid3D:
        """
        生成 3D 膨胀体素网格 (分离了障碍物与天花板的膨胀逻辑)。

        Args:
            xy_margin: 水平膨胀半径
            z_margin_obs: 对地/障碍物的 Z 轴安全膨胀 (包含 SafetyMargin + HalfHeight)
            z_margin_ceil: 对天花板的 Z 轴防穿模膨胀 (通常仅 HalfHeight，无 SafetyMargin)
        """
        with self._lock:
            key = (round(xy_margin, 3), round(z_margin_obs, 3), round(z_margin_ceil, 3))
            if key in self._3d_grid_caches:
                return self._3d_grid_caches[key]

            self.log(
                f"[MapMgr] 3D Grid: xy={xy_margin:.1f}, z_obs={z_margin_obs:.1f}, z_ceil={z_margin_ceil:.1f}"
            )

            # 1. 基础高度图
            height_map = np.zeros((self.rows, self.cols), dtype=np.float32)
            all_obs = list(self.static_obstacles.values()) + list(
                self.dynamic_obstacles.values()
            )
            for o in all_obs:
                r_s, c_s, _ = self.world_to_grid(o["x_m"], o["y_m"])
                r_e, c_e, _ = self.world_to_grid(
                    o["x_m"] + o["w_m"], o["y_m"] + o["h_m"]
                )

                # 障碍物占据高度 = 本体 + 障碍物余量
                obs_occupy_z = o.get("z_m", 100.0) + z_margin_obs

                r_s, r_e = max(0, r_s), min(self.rows, r_e + 1)
                c_s, c_e = max(0, c_s), min(self.cols, c_e + 1)
                if r_s < r_e and c_s < c_e:
                    height_map[r_s:r_e, c_s:c_e] = np.maximum(
                        height_map[r_s:r_e, c_s:c_e], obs_occupy_z
                    )

            # 2. 水平膨胀
            if xy_margin > 0 and HAS_SCIPY:
                from scipy.ndimage import maximum_filter

                k_size = int(math.ceil(xy_margin)) * 2 + 1
                height_map = maximum_filter(height_map, size=k_size)

            # 3. 3D 体素化
            z_coords = (np.arange(self.layers) + 0.5) * self.resolution_m

            # A. 障碍物遮挡 (Low Obstacles)
            is_obstacle = z_coords.reshape(1, 1, -1) < height_map.reshape(
                self.rows, self.cols, 1
            )

            # B. 天花板碰撞 (Ceiling Collision)
            # 逻辑: 当前高度 + 天花板余量 > 地图高度 -> 碰撞
            is_ceiling_hit = (z_coords + z_margin_ceil) > self.height_m

            # C. 地板碰撞 (Floor Collision)
            # 逻辑: 当前高度 - 障碍物余量 (此处复用 obs 余量作为对地余量) < 0 -> 碰撞
            is_floor_hit = (z_coords - z_margin_obs) < 0

            # 4. 合并
            final_grid_mask = (
                is_obstacle
                | is_ceiling_hit.reshape(1, 1, -1)
                | is_floor_hit.reshape(1, 1, -1)
            )

            voxel_grid_np = final_grid_mask.astype(np.int8)
            grid_3d = voxel_grid_np.tolist()
            self._3d_grid_caches[key] = grid_3d
            return grid_3d


def _create_inflated_grid_2d(grid: Grid2D, safety_margin: float) -> Grid2D:
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    if rows == 0:
        return []
    if HAS_SCIPY:
        np_grid = np.array(grid, dtype=np.int8)
        feature_mask = (np_grid == 0).astype(int)
        dist_map = distance_transform_edt(feature_mask)
        inflated_np = (dist_map <= safety_margin).astype(int)
        inflated_np = np.maximum(inflated_np, np_grid)
        return inflated_np.tolist()
    return grid
