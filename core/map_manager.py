import math
import threading
import copy
import logging
from typing import List, Tuple, Optional, Dict, Union, Any

try:
    import numpy as np
    from scipy.ndimage import distance_transform_edt, maximum_filter

    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# 类型别名
Grid2D = List[List[int]]
Grid3D = List[List[List[int]]]


class WorkshopMapManager:
    """
    【车间地图管理器 (3D Core) - V5 Wall Inflation Fix】

    Fixes:
    1. [Boundary Safety] 增加了车间边界的膨胀处理。
       现在四周墙壁被视为无限高的硬障碍物，算法会自动保持安全距离。
    2. 几何碰撞检测增加了边界检查。
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

        self.log(f"[MapMgr] Init: {self.width_m}x{self.length_m}x{self.height_m}m")

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

    # --- 坐标转换 ---

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

    # --- 障碍物管理 ---

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
            self.log(f"[MapMgr] Added Static: {obs_id}")

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

    # --- 几何碰撞检测 (真理层) ---

    def check_collision_raw(
        self,
        x: float,
        y: float,
        z: float,
        xy_margin: float,
        z_margin: float,
        ignore_z: bool = False,
    ) -> bool:
        """
        [真理层] 基于连续 3D 几何的精确碰撞检测。
        包含：1. 障碍物检测 2. 车间边界检测
        """
        # 1. 边界检测 (Wall Collision)
        # 如果 (x, y) 距离任何一面墙小于 xy_margin，视为碰撞
        if x - xy_margin < 0 or x + xy_margin > self.width_m:
            return True
        if y - xy_margin < 0 or y + xy_margin > self.length_m:
            return True

        # 天花板和地板检测 (可选，视业务需求)
        if z + z_margin > self.height_m or z - z_margin < 0:
            pass  # 暂时允许贴地和贴顶，通常由硬件限位控制

        with self._lock:
            all_obs = list(self.static_obstacles.values()) + list(
                self.dynamic_obstacles.values()
            )

            for o in all_obs:
                if not (
                    x + xy_margin > o["x_m"] and x - xy_margin < o["x_m"] + o["w_m"]
                ):
                    continue
                if not (
                    y + xy_margin > o["y_m"] and y - xy_margin < o["y_m"] + o["h_m"]
                ):
                    continue

                if ignore_z:
                    return True

                obs_z = o.get("z_m", 100.0)
                if z - z_margin <= obs_z:
                    return True

            return False

    # --- 网格生成 ---

    def _mark_obstacle_area(self, grid: Grid2D, x: float, y: float, w: float, h: float):
        r_s, c_s, _ = self.world_to_grid(x, y)
        r_e, c_e, _ = self.world_to_grid(x + w - 0.01, y + h - 0.01)
        r_s, r_e = max(0, r_s), min(self.rows - 1, r_e)
        c_s, c_e = max(0, c_s), min(self.cols - 1, c_e)
        for r in range(r_s, r_e + 1):
            for c in range(c_s, c_e + 1):
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

    def get_2d_projection_grid(
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

    def get_3d_voxel_grid(
        self,
        xy_margin: float,
        z_margin_obs: float,
        z_margin_ceil: float,
        is_infinite: bool = False,
    ) -> Grid3D:
        with self._lock:
            key = (
                round(xy_margin, 3),
                round(z_margin_obs, 3),
                round(z_margin_ceil, 3),
                is_infinite,
            )
            if key in self._3d_grid_caches:
                return self._3d_grid_caches[key]

            height_map = np.zeros((self.rows, self.cols), dtype=np.float32)
            all_obs = list(self.static_obstacles.values()) + list(
                self.dynamic_obstacles.values()
            )

            for o in all_obs:
                r_s, c_s, _ = self.world_to_grid(o["x_m"], o["y_m"])
                r_e, c_e, _ = self.world_to_grid(
                    o["x_m"] + o["w_m"], o["y_m"] + o["h_m"]
                )

                if is_infinite:
                    obs_occupy_z = self.height_m + 1.0
                else:
                    obs_occupy_z = o.get("z_m", 100.0) + z_margin_obs

                r_s, r_e = max(0, r_s), min(self.rows, r_e + 1)
                c_s, c_e = max(0, c_s), min(self.cols, c_e + 1)
                if r_s < r_e and c_s < c_e:
                    height_map[r_s:r_e, c_s:c_e] = np.maximum(
                        height_map[r_s:r_e, c_s:c_e], obs_occupy_z
                    )

            if xy_margin > 0 and HAS_SCIPY:
                radius = int(math.ceil(xy_margin))
                y, x = np.ogrid[-radius : radius + 1, -radius : radius + 1]
                mask = x**2 + y**2 <= xy_margin**2

                # [Fix] 墙壁膨胀: 使用 mode='constant' 和 cval=INF
                # 这样 maximum_filter 会认为图像边界外全是无限高的墙，从而将边界内的像素值拉高
                height_map = maximum_filter(
                    height_map,
                    footprint=mask,
                    mode="constant",
                    cval=self.height_m + 1.0,
                )

            z_coords = (np.arange(self.layers) + 0.5) * self.resolution_m
            is_obstacle = z_coords.reshape(1, 1, -1) < height_map.reshape(
                self.rows, self.cols, 1
            )
            is_ceiling_hit = (z_coords + z_margin_ceil) > self.height_m
            is_floor_hit = (z_coords - z_margin_obs) < 0
            final_grid_mask = (
                is_obstacle
                | is_ceiling_hit.reshape(1, 1, -1)
                | is_floor_hit.reshape(1, 1, -1)
            )

            grid_3d = final_grid_mask.astype(np.int8).tolist()
            self._3d_grid_caches[key] = grid_3d
            return grid_3d


def _create_inflated_grid_2d(grid: Grid2D, safety_margin: float) -> Grid2D:
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    if rows == 0:
        return []

    if HAS_SCIPY:
        np_grid = np.array(grid, dtype=np.int8)
        if np.sum(np_grid) == 0:
            # [Optimization] 如果是空地图，也需要考虑墙壁膨胀
            # 但为了简单，直接 padding 算一次 EDT
            pass

        # [Fix] 墙壁膨胀: 手动 Padding 一圈 1 (障碍物)
        # np.pad(array, pad_width, mode='constant', constant_values=1)
        np_grid_padded = np.pad(
            np_grid, pad_width=1, mode="constant", constant_values=1
        )

        feature_mask = (np_grid_padded == 0).astype(int)
        dist_map = distance_transform_edt(feature_mask)

        # Crop back to original size
        dist_map = dist_map[1:-1, 1:-1]

        inflated_np = (dist_map <= safety_margin).astype(int)
        inflated_np = np.maximum(inflated_np, np_grid)
        return inflated_np.tolist()

    return grid
