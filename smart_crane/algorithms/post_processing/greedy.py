import logging
import math
from typing import List, Optional, TYPE_CHECKING

from .base import PathPostProcessor, CollisionChecker, NodeType
from smart_crane.core.rust_bridge import RustBackend
from smart_crane.core.constants import SHAPE_CIRCLE

if TYPE_CHECKING:
    from smart_crane.core.map_manager import WorkshopMapManager
    from smart_crane.core.config import Settings


class GreedyShortcutProcessor(PathPostProcessor):
    """贪婪捷径优化器 (Greedy Shortcut Optimizer)。

    通过视线检查 (Line-of-Sight) 去除路径中的冗余节点。
    如果节点 A 和节点 C 之间可以直接连线且无碰撞，则跳过中间的节点 B。

    **Rust 加速支持**:
    如果 Rust 核心可用，将调用 `RustPostProcessor.greedy_shortcut` 进行高性能计算。
    """

    def __init__(
        self,
        map_mgr: Optional["WorkshopMapManager"] = None,
        settings: Optional["Settings"] = None,
        logger: Optional[logging.Logger] = None,
    ):
        """初始化贪婪优化器。

        Args:
            map_mgr: 地图管理器 (用于 Rust 模式)。
            settings: 全局配置 (用于计算物理参数)。
            logger: 父级日志记录器。
        """
        super().__init__(
            name="GreedyShortcut", map_mgr=map_mgr, settings=settings, logger=logger
        )
        self.logger.info("贪婪捷径策略初始化完成。")

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """执行贪婪路径简化。

        Args:
            path: 原始路径。
            is_safe_fn: 碰撞检测函数 (Python 模式用)。

        Returns:
            List[NodeType]: 简化后的路径。
        """
        # 路径点少于 3 个无法优化
        if not path or len(path) < 3:
            self.logger.debug(f"路径太短 ({len(path)} 节点)，无需优化。")
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
                    # 计算参数
                    shape = self.settings.crane.footprint_shape
                    w = self.settings.crane.footprint_width
                    l = self.settings.crane.footprint_length
                    radius_m = (
                        (w / 2.0) if shape == SHAPE_CIRCLE else (math.hypot(w, l) / 2.0)
                    )
                    # 增加半个分辨率缓冲 (同 Python logic)
                    radius_m += self.map_mgr.resolution_m / 2.0

                    z_margin = (
                        self.settings.crane.z_safety_margin
                        + self.settings.crane.footprint_height / 2.0
                    )
                    is_infinite = self.settings.crane.obstacle_infinite_height

                    # 豁免区：路径的起点和终点
                    # 这里的 path 已经是物理坐标 (x, y, z)
                    grace_start = path[0]
                    grace_end = path[-1]

                    # 调用 Rust
                    # Rust 签名: (path, map_manager, check_radius, check_z_margin, ignore_z, grace_start, grace_end)
                    optimized = RustProcessor.greedy_shortcut(
                        path,  # type: ignore
                        self.map_mgr.rust_map,
                        radius_m,
                        z_margin,
                        is_infinite,
                        grace_start,  # type: ignore
                        grace_end,  # type: ignore
                    )

                    return optimized

                except Exception as e:
                    self.logger.error(f"Rust 贪婪优化失败，降级至 Python: {e}")
                    # Fallback to Python...

        # =====================================================================
        # 2. Python 原生实现 (Fallback)
        # =====================================================================
        self.stats["mode"] = "python"
        optimized_path = [path[0]]
        current_idx = 0
        total_nodes = len(path)
        shortcut_count = 0

        while current_idx < total_nodes - 1:
            check_idx = total_nodes - 1
            found_shortcut = False

            while check_idx > current_idx + 1:
                start_node = path[current_idx]
                target_node = path[check_idx]

                if self._has_line_of_sight(start_node, target_node, is_safe_fn):
                    optimized_path.append(target_node)
                    self.logger.debug(
                        f"发现捷径: Node[{current_idx}] -> Node[{check_idx}] "
                        f"(Skip {check_idx - current_idx - 1})"
                    )
                    current_idx = check_idx
                    found_shortcut = True
                    shortcut_count += 1
                    break
                check_idx -= 1

            if not found_shortcut:
                current_idx += 1
                if current_idx < total_nodes:
                    optimized_path.append(path[current_idx])

        self.logger.debug(
            f"优化统计: {shortcut_count} 次捷径操作，"
            f"压缩比: {1.0 - len(optimized_path)/len(path):.1%}"
        )

        return optimized_path

    def _has_line_of_sight(
        self, start: NodeType, end: NodeType, is_safe_fn: CollisionChecker
    ) -> bool:
        """检查两点之间是否存在无障碍视线 (Line of Sight)。"""
        dims = len(start)
        dist_sq = sum((start[i] - end[i]) ** 2 for i in range(dims))
        distance = math.sqrt(dist_sq)

        if distance < 1e-6:
            return True

        steps = int(distance * 5)
        steps = max(steps, 2)
        deltas = [(end[i] - start[i]) for i in range(dims)]

        for i in range(1, steps):
            t = i / steps
            current_point = []
            for d in range(dims):
                val = start[d] + deltas[d] * t
                current_point.append(val)
            pt = tuple(current_point)
            if not is_safe_fn(pt):  # type: ignore
                return False
        return True
