import math
import threading
import copy
import logging
import time  # 引入 time 模块进行性能分析
from typing import List, Tuple, Optional, Dict, Any

from smart_crane.core.components.grid_factory import GridFactory
from smart_crane.core.rust_bridge import RustBackend
from smart_crane.core.constants import (
    DEFAULT_Z_HIGH,
    GRID_CENTER_OFFSET,
    MARK_OBS_EPS,
    CACHE_ROUND_DIGITS,
    TIME_MS_MULTIPLIER,
    TIME_FMT_PREC,
    GRID_OCCUPIED,
    GRID_FREE,
)

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
        height_m: float = DEFAULT_Z_HIGH,
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

        # 静态层膨胀缓存。与上面的成品缓存分开失效：
        # 动态障碍物变更只清成品缓存，静态层可继续复用。
        # 依据是膨胀对集合可分配——dist(A∪B) = min(dist(A), dist(B))，
        # 故 inflate(A∪B) == inflate(A) OR inflate(B)。
        self._static_grid_caches: Dict[Tuple, Grid2D] = {}

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
            f"初始化完成。尺寸: {self.cols}x{self.rows}x{self.layers}，分辨率: {self.resolution_m}m，模式: {'Rust加速' if self.rust_map else 'Python原生'}"
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

    def adopt_lock(self, lock: threading.RLock) -> None:
        """改用外部传入的锁。

        仅供地图重建时使用：重建会产生新的 MapManager 实例，若同时换掉锁对象，
        正持有旧锁的线程与拿到新锁的线程之间就不再互斥。沿用同一把锁可避免
        这个窗口。

        必须在新实例被其他线程访问之前调用。
        """
        self._lock = lock

    def invalidate_cache(self) -> None:
        """公开的缓存失效入口，供上层在持有锁时调用。"""
        self._invalidate_cache()

    def _invalidate_cache(self, static_changed: bool = True) -> None:
        """清空网格缓存。

        Args:
            static_changed: 静态障碍物是否发生变化。为 False 时保留静态层
                膨胀缓存——动态障碍物变更不影响静态部分，而静态部分在障碍物
                较多时正是膨胀开销的主要来源。
        """
        self._inflated_grid_caches.clear()
        self._3d_grid_caches.clear()
        if static_changed:
            self._static_grid_caches.clear()

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

            self._invalidate_cache(static_changed=False)

    def remove_dynamic_obstacle(self, obs_id: str):
        """移除动态障碍物。"""
        with self._lock:
            # 1. 更新 Python 状态
            if obs_id in self.dynamic_obstacles:
                del self.dynamic_obstacles[obs_id]
                # 2. 同步到 Rust
                if self.rust_map:
                    self.rust_map.remove_dynamic_obstacle(obs_id)

                self._invalidate_cache(static_changed=False)

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
        r_e, c_e, _ = self.world_to_grid(x + w - MARK_OBS_EPS, y + h - MARK_OBS_EPS)
        r_s, r_e = max(0, r_s), min(self.rows - 1, r_e)
        c_s, c_e = max(0, c_s), min(self.cols - 1, c_e)
        for r in range(r_s, r_e + 1):
            for c in range(c_s, c_e + 1):
                grid[r][c] = GRID_OCCUPIED

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
            key = (round(xy_margin, CACHE_ROUND_DIGITS), check_z, round(z_margin, CACHE_ROUND_DIGITS))
            if key in self._inflated_grid_caches:
                return self._inflated_grid_caches[key]

            t_start = time.perf_counter()

            # 1. Rust 加速路径
            if self.rust_map:
                # [关键修复] Rust 返回的是 List[bytes]，前端 JSON 序列化需要 List[List[int]]
                # 必须进行类型转换，否则前端 Canvas2d 无法识别 bytes 数据
                inflated_raw = self.rust_map.get_2d_projection_grid(
                    xy_margin, z_margin, check_z
                )

                t_rust_calc = time.perf_counter()

                # 数据转换: List[bytes] -> List[List[int]]
                # 警告：对于大地图，这一步可能比 Rust 计算本身还要慢
                inflated = [list(row) for row in inflated_raw]

                t_total = time.perf_counter()

                self.logger.info(
                    f"[MapManager] Rust 2D 网格生成完毕。"
                    f" Rust计算: {(t_rust_calc - t_start)*TIME_MS_MULTIPLIER:.{TIME_FMT_PREC}f}ms，"
                    f" 数据转换: {(t_total - t_rust_calc)*TIME_MS_MULTIPLIER:.{TIME_FMT_PREC}f}ms"
                )

                self._inflated_grid_caches[key] = inflated
                return inflated

            # 2. Python 原生路径：静态层缓存 + 动态层叠加
            z_threshold = None if check_z is None else (check_z - z_margin)

            static_grid = self._static_grid_caches.get(key)
            if static_grid is None:
                static_grid = self._inflate_subset(
                    self.static_obstacles.values(), xy_margin, z_threshold
                )
                self._static_grid_caches[key] = static_grid
                self.logger.debug(
                    f"[MapManager] 静态层膨胀已重算并缓存"
                    f"（{len(self.static_obstacles)} 个静态障碍）"
                )

            if not self.dynamic_obstacles:
                inflated = static_grid
            else:
                # 动态层不再走第二遍全图 EDT：那对个位数的障碍物是纯粹的浪费
                # （实测 300x300 下一次 EDT 约 5.8ms，与合并计算的总成本相当，
                #  拆分反而变慢）。改为在静态层副本上按几何直接绘制，
                # 与 Rust 侧障碍物稀疏时选用的绘制分支同构。
                inflated = [row[:] for row in static_grid]
                for o in self.dynamic_obstacles.values():
                    obs_h = o.get("z_m", DEFAULT_Z_HIGH)
                    if z_threshold is None or obs_h > z_threshold:
                        self._paint_inflated_obstacle(
                            inflated, o["x_m"], o["y_m"], o["w_m"], o["h_m"], xy_margin
                        )

            t_total = time.perf_counter()
            self.logger.info(
                f"[MapManager] Python 2D 网格生成完毕。 总耗时: {(t_total - t_start)*TIME_MS_MULTIPLIER:.{TIME_FMT_PREC}f}ms"
            )

            self._inflated_grid_caches[key] = inflated
            return inflated

    def _paint_inflated_obstacle(
        self,
        grid: Grid2D,
        x_m: float,
        y_m: float,
        w_m: float,
        h_m: float,
        xy_margin: float,
    ) -> None:
        """就地绘制单个障碍物的膨胀区域，结果与 EDT 分支逐格一致。

        关键在于距离的度量对象必须和 EDT 相同。EDT 作用于
        ``_mark_obstacle_area`` 栅格化之后的种子格，量的是
        **格心到最近种子格心** 的距离；而"格心到连续矩形"的距离总是更小，
        因为栅格化会把部分覆盖的格子整格算作占据，相当于种子向外取整。
        用后者会得到系统性偏小的膨胀层——即放宽安全边界。

        对矩形种子盒 ``[r_s..r_e] x [c_s..c_e]``，该距离有闭式解::

            dr = max(0, r_s - r, r - r_e)
            dc = max(0, c_s - c, c - c_e)
            占据  <=>  dr^2 + dc^2 <= (xy_margin + 0.5)^2

        因此无需跑全图 EDT。扫描范围只是种子盒外扩 margin，
        成本与障碍物尺寸相关、与地图面积无关——这正是它适合稀疏动态层的原因。
        """
        r_s, c_s, _ = self.world_to_grid(x_m, y_m)
        r_e, c_e, _ = self.world_to_grid(x_m + w_m - MARK_OBS_EPS, y_m + h_m - MARK_OBS_EPS)
        r_s, r_e = max(0, r_s), min(self.rows - 1, r_e)
        c_s, c_e = max(0, c_s), min(self.cols - 1, c_e)
        if r_s > r_e or c_s > c_e:
            return

        thresh = xy_margin + GRID_CENTER_OFFSET
        thresh_sq = thresh * thresh
        pad = int(math.ceil(thresh))

        for r in range(max(0, r_s - pad), min(self.rows - 1, r_e + pad) + 1):
            dr = max(0, r_s - r, r - r_e)
            dr_sq = dr * dr
            if dr_sq > thresh_sq:
                continue
            row = grid[r]
            for c in range(max(0, c_s - pad), min(self.cols - 1, c_e + pad) + 1):
                if row[c] == GRID_OCCUPIED:
                    continue
                dc = max(0, c_s - c, c - c_e)
                if dr_sq + dc * dc <= thresh_sq:
                    row[c] = GRID_OCCUPIED

    def _inflate_subset(
        self, obstacles, xy_margin: float, z_threshold: Optional[float]
    ) -> Grid2D:
        """对给定的障碍物子集做一次 C-Space 膨胀。

        拆成子集分别膨胀再求并集是合法的：欧氏距离变换满足
        ``dist(A∪B) = min(dist(A), dist(B))``，因此
        ``dist(A∪B) ≤ m`` 等价于 ``dist(A) ≤ m 或 dist(B) ≤ m``。
        """
        base_grid = [[GRID_FREE for _ in range(self.cols)] for _ in range(self.rows)]
        for o in obstacles:
            obs_h = o.get("z_m", DEFAULT_Z_HIGH)
            if z_threshold is None or obs_h > z_threshold:
                self._mark_obstacle_area(
                    base_grid, o["x_m"], o["y_m"], o["w_m"], o["h_m"]
                )
        return GridFactory.create_inflated_grid_2d(base_grid, xy_margin, self.logger)

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
                round(xy_margin, CACHE_ROUND_DIGITS),
                round(z_margin_obs, CACHE_ROUND_DIGITS),
                round(z_margin_ceil, CACHE_ROUND_DIGITS),
                is_infinite,
            )
            if key in self._3d_grid_caches:
                return self._3d_grid_caches[key]

            t_start = time.perf_counter()

            # 1. Rust 加速路径
            if self.rust_map:
                # [关键修复] Rust 返回 List[List[bytes]]，需转换为 List[List[List[int]]]
                grid_3d_raw = self.rust_map.get_3d_voxel_grid(
                    xy_margin, z_margin_obs, z_margin_ceil, is_infinite
                )

                t_rust_calc = time.perf_counter()

                # 数据转换: List[List[bytes]] -> List[List[List[int]]]
                grid_3d = [[list(col) for col in row] for row in grid_3d_raw]

                t_total = time.perf_counter()
                self.logger.info(
                    f"[MapManager] Rust 3D 网格生成完毕。"
                    f" Rust计算: {(t_rust_calc - t_start)*TIME_MS_MULTIPLIER:.{TIME_FMT_PREC}f}ms，"
                    f" 数据转换: {(t_total - t_rust_calc)*TIME_MS_MULTIPLIER:.{TIME_FMT_PREC}f}ms"
                )

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

            t_total = time.perf_counter()
            self.logger.info(
                f"[MapManager] Python 3D 网格生成完毕。 总耗时: {(t_total - t_start)*TIME_MS_MULTIPLIER:.{TIME_FMT_PREC}f}ms"
            )

            self._3d_grid_caches[key] = grid_3d
            return grid_3d
