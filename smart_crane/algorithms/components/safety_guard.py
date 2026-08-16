import math
import logging
from typing import Tuple, Optional, Callable, Union, List, TYPE_CHECKING

from smart_crane.core.config import Settings
from smart_crane.core.constants import (
    SHAPE_CIRCLE,
    EPSILON,
    FOOTPRINT_HALF_DIVISOR,
    ESCAPE_SEARCH_RADIUS_MARGIN,
    ESCAPE_SEARCH_RADIUS_MAX,
    GRACE_POINT_TOL_SQ,
    RESOLUTION_HALF_FACTOR,
    RUST_LABEL,
    PYTHON_FALLBACK_LABEL,
)
from smart_crane.core.rust_bridge import RustBackend

if TYPE_CHECKING:
    from smart_crane.core.map_manager import WorkshopMapManager
    from smart_crane.algorithms.pathfinding.base import PathPlannerBase

# 类型别名
GridNode = Union[Tuple[int, int], Tuple[int, int, int]]
Point3D = Tuple[float, float, float]


class SafetyGuard:
    """安全守卫 (Safety Guard)。

    负责规划前后的安全校验、端点合法性检查以及紧急脱困策略。
    通过物理碰撞检测和启发式搜索，确保机器人不会在危险区域启动或停止。
    """

    def __init__(
        self,
        map_mgr: "WorkshopMapManager",
        logger: Optional[logging.Logger] = None,
    ):
        """初始化安全守卫。"""
        self.map_mgr = map_mgr
        if logger:
            self.logger = logger.getChild(self.__class__.__name__)
        else:
            self.logger = logging.getLogger(self.__class__.__name__)

        # 尝试获取 Rust 实现
        self.RustGuard = RustBackend.get_safety_guard_class()
        mode_cn = (
            RUST_LABEL
            if (self.RustGuard and self.map_mgr.rust_map)
            else PYTHON_FALLBACK_LABEL
        )
        self.logger.debug(f"SafetyGuard 已就绪（{mode_cn}）。")

    def validate_endpoints(
        self, start_pt: Point3D, end_pt: Point3D, settings: Settings
    ) -> Tuple[bool, str, bool]:
        """校验任务的起点和终点是否合法。"""
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length

        xy_margin = (
            (w / FOOTPRINT_HALF_DIVISOR)
            if shape == SHAPE_CIRCLE
            else (math.hypot(w, l) / FOOTPRINT_HALF_DIVISOR)
        )
        z_safety = settings.crane.z_safety_margin
        crane_h = settings.crane.footprint_height
        z_margin = z_safety + (crane_h / FOOTPRINT_HALF_DIVISOR)
        hard_eps = EPSILON

        # 1. 物理硬碰撞检测 (Hard Collision)
        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], hard_eps, hard_eps, ignore_z=False
        ):
            msg = "起点位于障碍物内部 (硬碰撞)，任务拒绝。"
            self.logger.error(msg)
            return False, msg, False

        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], hard_eps, hard_eps, ignore_z=False
        ):
            msg = "终点位于障碍物内部 (硬碰撞)，任务拒绝。"
            self.logger.error(msg)
            return False, msg, False

        # 2. 膨胀层软碰撞检测 (Soft Collision)
        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            msg = "终点位于安全缓冲区（膨胀层）内，禁止停靠。"
            self.logger.warning(msg)
            return False, msg, False

        start_needs_escape = False
        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            self.logger.info("起点位于安全缓冲区内，标记为需要脱困。")
            start_needs_escape = True

        return True, "Valid", start_needs_escape

    def smart_escape(
        self,
        node: GridNode,
        ref_goal: GridNode,
        planner: "PathPlannerBase",
        settings: Settings,
    ) -> Optional[GridNode]:
        """智能脱困搜索 (Smart Escape)。

        **Rust 加速**:
        如果 Rust 可用，调用 `RustSafetyGuard.smart_escape` 进行高性能搜索。
        """
        # 1. 参数准备
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length
        radius_m = (
            (w / FOOTPRINT_HALF_DIVISOR)
            if shape == SHAPE_CIRCLE
            else (math.hypot(w, l) / FOOTPRINT_HALF_DIVISOR)
        )
        resolution = self.map_mgr.resolution_m
        grid_inflation_radius = int(math.ceil(radius_m / resolution))
        max_search_r = min(
            grid_inflation_radius + ESCAPE_SEARCH_RADIUS_MARGIN,
            ESCAPE_SEARCH_RADIUS_MAX,
        )

        # 垂直参数
        z_margin = (
            settings.crane.z_safety_margin
            + settings.crane.footprint_height / FOOTPRINT_HALF_DIVISOR
        )
        # 加上分辨率的一半作为网格容错
        check_radius = radius_m + (resolution * RESOLUTION_HALF_FACTOR)
        is_infinite = settings.crane.obstacle_infinite_height

        # =====================================================================
        # 1. 尝试使用 Rust 核心 [FIXED]
        # =====================================================================
        if self.RustGuard and self.map_mgr.rust_map:
            try:
                # 转换节点为 tuple (x, y, z)
                s_node_3d = (node[0], node[1], node[2] if len(node) > 2 else 0)
                g_node_3d = (
                    ref_goal[0],
                    ref_goal[1],
                    ref_goal[2] if len(ref_goal) > 2 else 0,
                )

                result = self.RustGuard.smart_escape(
                    s_node_3d,
                    g_node_3d,
                    self.map_mgr.rust_map,
                    check_radius,
                    z_margin,
                    max_search_r,
                    is_infinite,
                    len(node) > 2,  # search_3d：与 Python 回退实现的 dims 判据一致
                )

                if result:
                    # 如果是 2D 模式，转换回 2D tuple
                    if len(node) == 2:
                        return (result[0], result[1])
                    return result
                else:
                    self.logger.error("Rust 脱困搜索失败：搜索范围耗尽。")
                    return None

            except Exception as e:
                self.logger.error(f"Rust 脱困搜索异常，降级至 Python：{e}")
                # Fallback to Python...

        # =====================================================================
        # 2. Python 原生实现 (Fallback)
        # =====================================================================
        self.logger.warning(
            f"{PYTHON_FALLBACK_LABEL}：尝试脱困，起点={node}，最大半径={max_search_r} 格"
        )

        if planner.is_safe(node):
            return node

        dims = len(node)
        for r in range(1, max_search_r + 1):
            candidates: List[GridNode] = []

            if dims == 2:
                cx, cy = node  # type: ignore
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        if max(abs(dx), abs(dy)) == r:
                            n = (cx + dx, cy + dy)
                            if planner.is_safe(n):
                                candidates.append(n)
            else:
                cx, cy, cz = node  # type: ignore
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        for dz in range(-r, r + 1):
                            if max(abs(dx), abs(dy), abs(dz)) == r:
                                n = (cx + dx, cy + dy, cz + dz)
                                if planner.is_safe(n):
                                    candidates.append(n)

            if candidates:
                best_node = min(
                    candidates,
                    key=lambda n: sum((n[i] - ref_goal[i]) ** 2 for i in range(dims)),
                )
                self.logger.info(f"脱困成功：半径 {r} 格， 新起点: {best_node}")
                return best_node

        self.logger.error(f"脱困失败：在半径 {max_search_r} 格范围内未找到安全节点。")
        return None

    def create_collision_checker(
        self, grace_start: Point3D, grace_end: Point3D, settings: Settings
    ) -> Callable[[Point3D], bool]:
        """创建一个带豁免区的碰撞检测闭包函数。"""
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length

        radius = (
            (w / FOOTPRINT_HALF_DIVISOR)
            if shape == SHAPE_CIRCLE
            else (math.hypot(w, l) / FOOTPRINT_HALF_DIVISOR)
        )
        radius += self.map_mgr.resolution_m * RESOLUTION_HALF_FACTOR

        z_margin = (
            settings.crane.z_safety_margin
            + settings.crane.footprint_height / FOOTPRINT_HALF_DIVISOR
        )
        is_infinite = settings.crane.obstacle_infinite_height

        def check(pt: Tuple[float, ...]) -> bool:
            x, y, z = pt[0], pt[1], pt[2]

            if (x - grace_start[0]) ** 2 + (y - grace_start[1]) ** 2 + (
                z - grace_start[2]
            ) ** 2 < GRACE_POINT_TOL_SQ:
                return True
            if (x - grace_end[0]) ** 2 + (y - grace_end[1]) ** 2 + (
                z - grace_end[2]
            ) ** 2 < GRACE_POINT_TOL_SQ:
                return True

            if self.map_mgr.check_collision_raw(
                x, y, z, radius, z_margin, ignore_z=is_infinite
            ):
                return False
            return True

        return check
