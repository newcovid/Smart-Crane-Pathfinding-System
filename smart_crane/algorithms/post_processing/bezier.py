import logging
import math
from typing import List, Optional, TYPE_CHECKING

from .base import PathPostProcessor, CollisionChecker, NodeType
from smart_crane.core.rust_bridge import RustBackend
from smart_crane.core.constants import (
    SHAPE_CIRCLE,
    DEFAULT_BEZIER_SMOOTHNESS,
    DEFAULT_BEZIER_SEGMENTS,
    DEFAULT_BEZIER_ATTEMPTS,
    BEZIER_CHECK_STEP_MULTIPLIER,
    BEZIER_MIN_STEPS,
    PATH_MIN_NODES,
    RESOLUTION_HALF_FACTOR,
    FOOTPRINT_HALF_DIVISOR,
)

if TYPE_CHECKING:
    from smart_crane.core.map_manager import WorkshopMapManager
    from smart_crane.core.config import Settings


class BezierSmoothProcessor(PathPostProcessor):
    """贝塞尔平滑处理器 (Bezier Curve Smoother)。

    利用二次贝塞尔曲线 (Quadratic Bezier Curve) 将路径中的折线拐角打磨成平滑的圆弧。
    包含自适应回退机制：如果生成的曲线发生碰撞，自动减小曲率半径重试。

    **Rust 加速支持**:
    如果 Rust 核心可用，将调用 `RustPostProcessor.bezier_smooth` 进行高性能计算。
    """

    def __init__(
        self,
        map_mgr: Optional["WorkshopMapManager"] = None,
        settings: Optional["Settings"] = None,
        smoothness: float = DEFAULT_BEZIER_SMOOTHNESS,
        segments: int = DEFAULT_BEZIER_SEGMENTS,
        logger: Optional[logging.Logger] = None,
    ):
        """初始化平滑器。

        Args:
            map_mgr: 地图管理器。
            settings: 全局配置。
            smoothness: 平滑度因子 (0.0 ~ 0.5)。
            segments: 曲线插值段数。
            logger: 父级日志记录器。
        """
        super().__init__(
            name="BezierSmoother", map_mgr=map_mgr, settings=settings, logger=logger
        )

        self.default_smoothness = max(0.0, min(0.5, smoothness))
        self.num_segments = segments

        self.logger.info(
            f"初始化完成. 平滑度={self.default_smoothness}, 插值段数={self.num_segments}"
        )

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """执行贝塞尔平滑逻辑。"""
        if not path or len(path) < PATH_MIN_NODES:
            return path

        # =====================================================================
        # 1. 尝试使用 Rust 核心
        # =====================================================================
        if (
            self.map_mgr
            and self.map_mgr.rust_map
            and self.settings
            and RustBackend.is_available()
        ):
            RustProcessor = RustBackend.get_post_processor_class()
            if RustProcessor:
                try:
                    self.stats["mode"] = "rust"
                    # 计算参数 (复用 Greedy 的计算逻辑)
                    shape = self.settings.crane.footprint_shape
                    w = self.settings.crane.footprint_width
                    l = self.settings.crane.footprint_length
                    radius_m = (
                        (w / FOOTPRINT_HALF_DIVISOR)
                        if shape == SHAPE_CIRCLE
                        else (math.hypot(w, l) / FOOTPRINT_HALF_DIVISOR)
                    )
                    radius_m += self.map_mgr.resolution_m * RESOLUTION_HALF_FACTOR
                    z_margin = (
                        self.settings.crane.z_safety_margin
                        + self.settings.crane.footprint_height / FOOTPRINT_HALF_DIVISOR
                    )
                    is_infinite = self.settings.crane.obstacle_infinite_height

                    # 调用 Rust
                    smoothed = RustProcessor.bezier_smooth(
                        path,  # type: ignore
                        self.map_mgr.rust_map,
                        self.default_smoothness,
                        self.num_segments,
                        radius_m,
                        z_margin,
                        is_infinite,
                        path[0],  # grace_start
                        path[-1],  # grace_end
                    )
                    return smoothed

                except Exception as e:
                    self.logger.error(f"Rust 贝塞尔平滑失败，降级至 Python: {e}")
                    # Fallback...

        # =====================================================================
        # 2. Python 原生实现 (Fallback)
        # =====================================================================
        self.stats["mode"] = "python"
        smoothed_path = []
        smoothed_path.append(path[0])

        input_nodes_count = len(path)
        curved_corners = 0

        for i in range(1, len(path) - 1):
            p0 = path[i - 1]
            p1 = path[i]
            p2 = path[i + 1]

            current_smoothness = self.default_smoothness
            best_segment = None

            for attempt in range(DEFAULT_BEZIER_ATTEMPTS):
                if self._check_curve_safety(p0, p1, p2, current_smoothness, is_safe_fn):
                    best_segment = self._generate_curve(p0, p1, p2, current_smoothness)
                    if attempt > 0:
                        self.logger.debug(
                            f"拐角 {i} 触发自适应回退: 尝试 {attempt} 次后成功 "
                            f"(Smoothness: {self.default_smoothness} -> {current_smoothness:.4f})"
                        )
                    break
                else:
                    current_smoothness *= 0.5

            if best_segment:
                smoothed_path.extend(best_segment)
                curved_corners += 1
            else:
                self.logger.debug(f"拐角 {i} 空间狭窄无法平滑，保留硬连接。")
                smoothed_path.append(p1)

        smoothed_path.append(path[-1])

        self.logger.debug(
            f"平滑统计: 处理 {curved_corners}/{input_nodes_count-2} 个拐角"
        )
        return smoothed_path

    def _check_curve_safety(
        self,
        p0: NodeType,
        p1: NodeType,
        p2: NodeType,
        s: float,
        is_safe_fn: CollisionChecker,
    ) -> bool:
        """检查生成的贝塞尔曲线是否安全。"""
        dims = len(p0)
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        d1 = math.sqrt(sum((p1[d] - p0[d]) ** 2 for d in range(dims)))
        d2 = math.sqrt(sum((p2[d] - p1[d]) ** 2 for d in range(dims)))
        total_len = (d1 + d2) * s

        steps = int(total_len * BEZIER_CHECK_STEP_MULTIPLIER)
        steps = max(steps, BEZIER_MIN_STEPS)

        for k in range(steps + 1):
            t = k / steps
            check_point_list = []
            for d in range(dims):
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]
                check_point_list.append(val)
            check_point = tuple(check_point_list)
            if not is_safe_fn(check_point):  # type: ignore
                return False
        return True

    def _generate_curve(
        self, p0: NodeType, p1: NodeType, p2: NodeType, s: float
    ) -> List[tuple]:
        """生成贝塞尔曲线点集。"""
        dims = len(p0)
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        points = []
        # 必须从 i=0（即 t=0 的 q0）开始输出。
        # _check_curve_safety 是从 t=0 起做碰撞检查的，若这里跳过 q0，
        # 实际路径就变成"上一点 → curve(1/n)"这条弦——它比校验过的
        # "q0 → curve(1/n)"更贴近拐角内侧，且从未被任何检查覆盖。
        # q0 位于 p0→p1 线段上，与上一段的终点不重合，不会产生重复点。
        for i in range(0, self.num_segments + 1):
            t = i / self.num_segments
            pt_list = []
            for d in range(dims):
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]
                pt_list.append(val)
            points.append(tuple(pt_list))
        return points
