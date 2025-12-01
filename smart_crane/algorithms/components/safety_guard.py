import math
import logging
from typing import Tuple, Optional, Callable, Union, TYPE_CHECKING

from smart_crane.core.config import Settings
from smart_crane.core.constants import SHAPE_CIRCLE

if TYPE_CHECKING:
    from smart_crane.core.map_manager import WorkshopMapManager
    from smart_crane.algorithms.pathfinding.base import PathPlannerBase

# 类型别名
GridNode = Union[Tuple[int, int], Tuple[int, int, int]]
Point3D = Tuple[float, float, float]


class SafetyGuard:
    """
    【安全守卫 (Safety Guard)】
    负责规划前后的安全校验、端点合法性检查以及紧急脱困策略。
    """

    def __init__(
        self,
        map_mgr: "WorkshopMapManager",
        logger: Optional[logging.Logger] = None,
    ):
        """初始化安全守卫。

        Args:
            map_mgr (WorkshopMapManager): 地图管理器。
            logger (Optional[logging.Logger]): 父级日志记录器。
        """
        self.map_mgr = map_mgr

        # [日志优化] 实现智能嵌套
        if logger:
            self.logger = logger.getChild(self.__class__.__name__)
        else:
            self.logger = logging.getLogger(self.__class__.__name__)

        self.logger.debug("SafetyGuard 安全守卫已就绪。")

    def validate_endpoints(
        self, start_pt: Point3D, end_pt: Point3D, settings: Settings
    ) -> Tuple[bool, str, bool]:
        """校验任务的起点和终点是否合法。

        Args:
            start_pt (Point3D): 起点物理坐标 (x, y, z)。
            end_pt (Point3D): 终点物理坐标 (x, y, z)。
            settings (Settings): 配置对象。

        Returns:
            Tuple[bool, str, bool]:
                - is_valid: 端点是否有效。
                - message: 错误信息或状态描述。
                - start_needs_escape: 起点是否位于膨胀层内（需要脱困）。
        """
        self.logger.debug(f"正在校验端点: Start={start_pt}, End={end_pt}")

        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length

        # 计算碰撞检测半径
        xy_margin = (w / 2.0) if shape == SHAPE_CIRCLE else (math.hypot(w, l) / 2.0)

        z_safety = settings.crane.z_safety_margin
        crane_h = settings.crane.footprint_height
        z_margin = z_safety + (crane_h / 2.0)

        # 1. 物理硬碰撞检测 (Core Collision)
        # 检查是否直接位于障碍物物理体积内部（忽略安全边距）
        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], 0, 0, ignore_z=False
        ):
            msg = "起点位于障碍物实体内部 (硬碰撞)，任务拒绝。"
            self.logger.error(msg)
            return False, msg, False

        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], 0, 0, ignore_z=False
        ):
            msg = "终点位于障碍物实体内部 (硬碰撞)，任务拒绝。"
            self.logger.error(msg)
            return False, msg, False

        # 2. 膨胀层软碰撞检测 (Soft Collision / Inflation Zone)
        # 检查是否位于安全缓冲区内
        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            msg = "终点位于安全缓冲区(膨胀层)内，禁止停靠。"
            self.logger.warning(msg)
            return False, msg, False

        # 起点位于膨胀层是允许的，但需要触发脱困程序
        start_needs_escape = False
        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            self.logger.info("起点位于安全缓冲区内，标记为[需要脱困]。")
            start_needs_escape = True

        return True, "Valid", start_needs_escape

    def smart_escape(
        self, node: GridNode, ref_goal: GridNode, planner: "PathPlannerBase"
    ) -> Optional[GridNode]:
        """智能脱困搜索。

        当起点位于障碍物膨胀层（不可行区域）时，在起点附近搜索最近的可行网格点作为实际规划起点。

        Args:
            node (GridNode): 原始起点网格坐标。
            ref_goal (GridNode): 目标点网格坐标（用于启发式选择最近的脱困点）。
            planner (PathPlannerBase): 规划器实例（用于检查节点安全性）。

        Returns:
            Optional[GridNode]: 找到的脱困点坐标，若未找到则返回 None。
        """
        # 如果当前点本身是安全的，无需脱困
        if planner.is_safe(node):
            return node

        self.logger.warning(f"正在尝试从节点 {node} 脱困...")

        dims = len(node)
        # 搜索半径逐步扩大，最大 5 格
        for r in range(1, 6):
            candidates = []
            if dims == 2:
                cx, cy = node  # type: ignore
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        # 仅检查当前半径的外壳（On Shell），避免重复检查
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
                # 贪婪策略：选择离目标点最近的候选点
                best_node = min(
                    candidates,
                    key=lambda n: sum((n[i] - ref_goal[i]) ** 2 for i in range(dims)),
                )
                self.logger.info(f"脱困成功! 半径: {r}, 新起点: {best_node}")
                return best_node

        self.logger.error("脱困失败: 5格半径内无安全节点。")
        return None

    def create_collision_checker(
        self, grace_start: Point3D, grace_end: Point3D, settings: Settings
    ) -> Callable[[Point3D], bool]:
        """创建一个针对当前任务的碰撞检测闭包函数。

        用于后处理阶段（如平滑算法）的快速连续碰撞检测。
        会自动豁免起点和终点附近的微小区域，防止因浮点误差导致的误报。

        Args:
            grace_start (Point3D): 豁免起点。
            grace_end (Point3D): 豁免终点。
            settings (Settings): 全局配置。

        Returns:
            Callable[[Point3D], bool]: 函数，输入 (x,y,z)，返回 True(安全) / False(碰撞)。
        """
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length

        # 计算包络半径
        radius = (w / 2.0) if shape == SHAPE_CIRCLE else (math.hypot(w, l) / 2.0)
        # 额外增加半个分辨率的缓冲，确保不会在网格边缘产生穿模
        radius += self.map_mgr.resolution_m / 2.0

        z_margin = (
            settings.crane.z_safety_margin + settings.crane.footprint_height / 2.0
        )
        is_infinite = settings.crane.obstacle_infinite_height

        def check(pt: Tuple[float, ...]) -> bool:
            x, y, z = pt[0], pt[1], pt[2]

            # 豁免区检测 (Grace Zone) - 半径 0.5m
            # 如果点在起点或终点极近范围内，直接放行
            if (x - grace_start[0]) ** 2 + (y - grace_start[1]) ** 2 + (
                z - grace_start[2]
            ) ** 2 < 0.25:
                return True
            if (x - grace_end[0]) ** 2 + (y - grace_end[1]) ** 2 + (
                z - grace_end[2]
            ) ** 2 < 0.25:
                return True

            # 调用 MapManager 进行精确检测
            if self.map_mgr.check_collision_raw(
                x, y, z, radius, z_margin, ignore_z=is_infinite
            ):
                return False  # 碰撞
            return True  # 安全

        return check
