import math
import threading
import copy
import logging
from typing import List, Tuple, Optional, Dict, Any

from smart_crane.core.components.grid_factory import GridFactory
from smart_crane.core.rust_bridge import RustBackend
from smart_crane.core.constants import DEFAULT_Z_HIGH, GRID_CENTER_OFFSET

# 类型别名
Grid2D = List[List[int]]
Grid3D = List[List[List[int]]]


class WorkshopMapManager:
    """车间地图管理器 (Workshop Map Manager)。

    负责维护车间的物理属性、障碍物数据（静态/动态）以及坐标转换逻辑。
    它充当 GridFactory 的数据源，管理着物理世界(World)到网格世界(Grid)的映射。

    **Rust 集成说明**:
    如果 Rust 核心可用，本类将作为一个代理 (Proxy)，将繁重的计算任务（碰撞检测、网格生成）
    转发给底层的 `RustMapManager` 实例，同时在 Python 层保留一份数据副本用于前端同步。

    Attributes:
        width_m, length_m, height_m: 车间物理尺寸。
        resolution_m: 网格分辨率。
        static_obstacles: 静态障碍物数据库。
        dynamic_obstacles: 动态障碍物数据库。
        logger: 日志记录器。
    """

    def __init__(
        self,
        width_m: float,
        length_m: float,
        resolution_m: float,
        height_m: float = 20.0,
        logger: Optional[logging.Logger] = None,
    ):
        """初始化地图管理器。

        Args:
            width_m: 宽度（米）。
            length_m: 长度（米）。
            resolution_m: 分辨率（米）。
            height_m: 高度（米）。
            logger: 父级 Logger。
        """
        self.width_m = width_m
        self.length_m = length_m
        self.height_m = height_m
        self.resolution_m = resolution_m
        self._lock = threading.RLock()

        # [日志优化]
        if logger:
            self.logger = logger.getChild(self.__class__.__name__)
        else:
            self.logger = logging.getLogger(self.__class__.__name__)

        self.cols = int(math.ceil(width_m / resolution_m))
        self.rows = int(math.ceil(length_m / resolution_m))
        self.layers = int(math.ceil(height_m / resolution_m))

        self.static_obstacles: Dict[str, dict] = {}
        self.dynamic_obstacles: Dict[str, dict] = {}

        # 缓存系统：避免频繁重新计算网格
        self._inflated_grid_caches: Dict[Tuple, Grid2D] = {}
        self._3d_grid_caches: Dict[Tuple, Grid3D] = {}

        # Rust 核心集成
        self.rust_map = None
        if RustBackend.is_available():
            RustClass = RustBackend.get_map_manager_class()
            if RustClass:
                try:
                    self.rust_map = RustClass(width_m, length_m, height_m, resolution_m)
                    self.logger.info("Rust 高性能地图内核加载成功。")
                except Exception as e:
                    self.logger.error(f"Rust 地图内核实例化失败: {e}", exc_info=True)
                    self.rust_map = None

        self.logger.info(
            f"初始化完成. 尺寸: {self.cols}x{self.rows}x{self.layers}, "
            f"分辨率: {self.resolution_m}m, 模式: {'Rust加速' if self.rust_map else 'Python原生'}"
        )

    @property
    def lock(self) -> threading.RLock:
        """获取线程锁。"""
        return self._lock

    def get_full_state(self) -> Dict[str, Any]:
        """获取地图全量状态（用于前端同步）。"""
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
        """清除网格缓存（当障碍物变动时调用）。"""
        self._inflated_grid_caches.clear()
        self._3d_grid_caches.clear()
        # self.logger.debug("缓存已清除。") # 减少日志噪音

    def world_to_grid(
        self, x_m: float, y_m: float, z_m: float = 0.0
    ) -> Tuple[int, int, int]:
        """物理坐标转网格索引。"""
        # 简单计算，直接在 Python 侧完成即可，无需调用 Rust
        col = int(x_m / self.resolution_m)
        row = int(y_m / self.resolution_m)
        layer = int(z_m / self.resolution_m)
        return (
            max(0, min(row, self.rows - 1)),
            max(0, min(col, self.cols - 1)),
            max(0, min(layer, self.layers - 1)),
        )

    def grid_to_world(
        self, row: int, col: int, layer: int = 0
    ) -> Tuple[float, float, float]:
        """网格索引转物理坐标（返回网格中心点）。"""
        return (
            (col + GRID_CENTER_OFFSET) * self.resolution_m,
            (row + GRID_CENTER_OFFSET) * self.resolution_m,
            (layer + GRID_CENTER_OFFSET) * self.resolution_m,
        )

    def add_static_obstacle(
        self,
        obs_id: str,
        x: float,
        y: float,
        w: float,
        h: float,
        z: float = DEFAULT_Z_HIGH,
    ):
        """添加静态障碍物。"""
        with self._lock:
            # 1. 更新 Python 状态
            self.static_obstacles[obs_id] = {
                "x_m": x,
                "y_m": y,
                "w_m": w,
                "h_m": h,
                "z_m": z,
            }
            # 2. 同步到 Rust
            if self.rust_map:
                self.rust_map.add_static_obstacle(obs_id, x, y, w, h, z)

            self._invalidate_cache()
            self.logger.info(f"新增静态障碍: {obs_id}")

    def remove_static_obstacle(self, obs_id: str):
        """移除静态障碍物。"""
        with self._lock:
            # 1. 更新 Python 状态
            if obs_id in self.static_obstacles:
                del self.static_obstacles[obs_id]
                # 2. 同步到 Rust
                if self.rust_map:
                    self.rust_map.remove_static_obstacle(obs_id)

                self._invalidate_cache()
                self.logger.info(f"移除静态障碍: {obs_id}")

    def update_dynamic_obstacle(
        self,
        obs_id: str,
        x: float,
        y: float,
        w: float,
        h: float,
        z: float = DEFAULT_Z_HIGH,
    ):
        """更新/添加动态障碍物。"""
        with self._lock:
            # 1. 更新 Python 状态
            self.dynamic_obstacles[obs_id] = {
                "x_m": x,
                "y_m": y,
                "w_m": w,
                "h_m": h,
                "z_m": z,
            }
            # 2. 同步到 Rust
            if self.rust_map:
                self.rust_map.update_dynamic_obstacle(obs_id, x, y, w, h, z)

            self._invalidate_cache()

    def remove_dynamic_obstacle(self, obs_id: str):
        """移除动态障碍物。"""
        with self._lock:
            # 1. 更新 Python 状态
            if obs_id in self.dynamic_obstacles:
                del self.dynamic_obstacles[obs_id]
                # 2. 同步到 Rust
                if self.rust_map:
                    self.rust_map.remove_dynamic_obstacle(obs_id)

                self._invalidate_cache()

    def find_obstacle_near(self, x_m: float, y_m: float) -> Optional[Tuple[str, str]]:
        """查找指定坐标附近的障碍物。

        此操作频率低且仅涉及简单查询，保留在 Python 侧执行。
        """
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

    def check_collision_raw(
        self,
        x: float,
        y: float,
        z: float,
        xy_margin: float,
        z_margin: float,
        ignore_z: bool = False,
    ) -> bool:
        """物理坐标下的精确碰撞检测 (Hard Collision Check)。

        直接判断点是否在障碍物的物理包围盒内，支持安全边距。

        Args:
            x, y, z: 检查点坐标。
            xy_margin: 水平安全边距。
            z_margin: 垂直安全边距。
            ignore_z: 是否忽略高度检测（视为无限高）。

        Returns:
            bool: True 表示发生碰撞。
        """
        # 优先使用 Rust 进行高频碰撞检测
        if self.rust_map:
            return self.rust_map.check_collision_raw(
                x, y, z, xy_margin, z_margin, ignore_z
            )

        # Fallback to Python implementation
        # 边界检查
        if x - xy_margin < 0 or x + xy_margin > self.width_m:
            return True
        if y - xy_margin < 0 or y + xy_margin > self.length_m:
            return True

        xy_margin_sq = xy_margin * xy_margin

        with self._lock:
            all_obs = list(self.static_obstacles.values()) + list(
                self.dynamic_obstacles.values()
            )

            for o in all_obs:
                # 计算点到矩形的最近距离（Clamp方法）
                closest_x = max(o["x_m"], min(x, o["x_m"] + o["w_m"]))
                closest_y = max(o["y_m"], min(y, o["y_m"] + o["h_m"]))

                dist_sq = (x - closest_x) ** 2 + (y - closest_y) ** 2

                if dist_sq < xy_margin_sq:
                    if ignore_z:
                        return True
                    obs_z = o.get("z_m", DEFAULT_Z_HIGH)
                    # 检查 Z 轴
                    if z - z_margin <= obs_z:
                        return True
            return False

    def _mark_obstacle_area(self, grid: Grid2D, x: float, y: float, w: float, h: float):
        """[内部] 在 2D 网格上标记障碍物区域（仅 Python 模式使用）。"""
        r_s, c_s, _ = self.world_to_grid(x, y)
        r_e, c_e, _ = self.world_to_grid(x + w - 0.01, y + h - 0.01)
        r_s, r_e = max(0, r_s), min(self.rows - 1, r_e)
        c_s, c_e = max(0, c_s), min(self.cols - 1, c_e)
        for r in range(r_s, r_e + 1):
            for c in range(c_s, c_e + 1):
                grid[r][c] = 1

    def get_2d_projection_grid(
        self, xy_margin: float, check_z: Optional[float] = None, z_margin: float = 0.0
    ) -> Grid2D:
        """获取带膨胀的 2D 投影网格。

        Args:
            xy_margin: 水平膨胀半径 (网格单位)。
            check_z: 检查高度（米）。如果提供，忽略该高度以下的障碍物。
            z_margin: 垂直安全边距（米）。

        Returns:
            Grid2D: 0/1 矩阵。
        """
        with self._lock:
            # 缓存键值需要包含所有参数
            key = (round(xy_margin, 3), check_z, round(z_margin, 3))
            if key in self._inflated_grid_caches:
                return self._inflated_grid_caches[key]

            # 1. Rust 加速路径
            if self.rust_map:
                # [关键修复] Rust 返回的是 List[bytes]，前端 JSON 序列化需要 List[List[int]]
                # 必须进行类型转换，否则前端 Canvas2d 无法识别 bytes 数据
                inflated_raw = self.rust_map.get_2d_projection_grid(
                    xy_margin, z_margin, check_z
                )
                inflated = [list(row) for row in inflated_raw]

                self._inflated_grid_caches[key] = inflated
                return inflated

            # 2. Python 原生路径
            base_grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
            all_obs = list(self.static_obstacles.values()) + list(
                self.dynamic_obstacles.values()
            )
            z_threshold = None if check_z is None else (check_z - z_margin)

            for o in all_obs:
                obs_h = o.get("z_m", DEFAULT_Z_HIGH)
                if z_threshold is None or obs_h > z_threshold:
                    self._mark_obstacle_area(
                        base_grid, o["x_m"], o["y_m"], o["w_m"], o["h_m"]
                    )

            # 调用 GridFactory 进行膨胀计算
            inflated = GridFactory.create_inflated_grid_2d(
                base_grid, xy_margin, self.logger
            )

            self._inflated_grid_caches[key] = inflated
            return inflated

    def get_3d_voxel_grid(
        self,
        xy_margin: float,
        z_margin_obs: float,
        z_margin_ceil: float,
        is_infinite: bool = False,
    ) -> Grid3D:
        """获取 3D 体素网格。

        Args:
            xy_margin: 水平膨胀。
            z_margin_obs: 障碍物垂直膨胀。
            z_margin_ceil: 天花板垂直避让。
            is_infinite: 是否开启无限高模式。

        Returns:
            Grid3D: 3D 0/1 矩阵。
        """
        with self._lock:
            key = (
                round(xy_margin, 3),
                round(z_margin_obs, 3),
                round(z_margin_ceil, 3),
                is_infinite,
            )
            if key in self._3d_grid_caches:
                return self._3d_grid_caches[key]

            # 1. Rust 加速路径
            if self.rust_map:
                # [关键修复] Rust 返回 List[List[bytes]]，需转换为 List[List[List[int]]]
                grid_3d_raw = self.rust_map.get_3d_voxel_grid(
                    xy_margin, z_margin_obs, z_margin_ceil, is_infinite
                )
                grid_3d = [[list(col) for col in row] for row in grid_3d_raw]

                self._3d_grid_caches[key] = grid_3d
                return grid_3d

            # 2. Python 原生路径
            all_obs = list(self.static_obstacles.values()) + list(
                self.dynamic_obstacles.values()
            )

            # 调用 GridFactory 进行复杂的 3D 建模
            grid_3d = GridFactory.create_3d_voxel_grid(
                map_dims=(self.rows, self.cols, self.layers),
                resolution=self.resolution_m,
                obstacles=all_obs,
                margins=(xy_margin, z_margin_obs, z_margin_ceil),
                is_infinite=is_infinite,
                logger=self.logger,  # 传递 Logger
                coord_converter=self,  # 传递自身用于坐标转换
            )

            self._3d_grid_caches[key] = grid_3d
            return grid_3d
