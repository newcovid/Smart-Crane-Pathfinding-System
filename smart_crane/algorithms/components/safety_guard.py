import math
import logging
from typing import Tuple, Optional, Callable, Union, List, TYPE_CHECKING

from smart_crane.core.config import Settings
from smart_crane.core.constants import SHAPE_CIRCLE

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

    Attributes:
        map_mgr (WorkshopMapManager): 地图管理器引用。
        logger (logging.Logger): 日志记录器。
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

        # [日志优化] 智能嵌套: Parent.SafetyGuard
        if logger:
            self.logger = logger.getChild("SafetyGuard")
        else:
            self.logger = logging.getLogger("SafetyGuard")

        self.logger.debug("SafetyGuard 安全守卫已就绪。")

    def validate_endpoints(
        self, start_pt: Point3D, end_pt: Point3D, settings: Settings
    ) -> Tuple[bool, str, bool]:
        """校验任务的起点和终点是否合法。

        同时检查硬碰撞（物理体积）和软碰撞（安全膨胀层）。

        Args:
            start_pt (Point3D): 起点物理坐标 (x, y, z)。
            end_pt (Point3D): 终点物理坐标 (x, y, z)。
            settings (Settings): 全局配置。

        Returns:
            Tuple[bool, str, bool]:
                - is_valid: 端点是否有效（终点必须安全，起点允许在膨胀层）。
                - message: 状态描述或错误信息。
                - start_needs_escape: 起点是否位于安全缓冲区内（需要触发脱困）。
        """
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length

        # 计算碰撞检测半径
        xy_margin = (w / 2.0) if shape == SHAPE_CIRCLE else (math.hypot(w, l) / 2.0)

        z_safety = settings.crane.z_safety_margin
        crane_h = settings.crane.footprint_height
        z_margin = z_safety + (crane_h / 2.0)

        # 1. 物理硬碰撞检测 (Hard Collision)
        # 起点和终点都不能位于障碍物实体内部
        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], 0, 0, ignore_z=False
        ):
            msg = "起点位于障碍物内部 (硬碰撞)，任务拒绝。"
            self.logger.error(msg)
            return False, msg, False

        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], 0, 0, ignore_z=False
        ):
            msg = "终点位于障碍物内部 (硬碰撞)，任务拒绝。"
            self.logger.error(msg)
            return False, msg, False

        # 2. 膨胀层软碰撞检测 (Soft Collision)
        # 终点必须完全安全，不能在膨胀层内
        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            msg = "终点位于安全缓冲区(膨胀层)内，禁止停靠。"
            self.logger.warning(msg)
            return False, msg, False

        # 起点允许在膨胀层内（例如机器人刚放下货物，周围都是箱子），但需要标记脱困
        start_needs_escape = False
        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            self.logger.info("起点位于安全缓冲区内，标记为[需要脱困]。")
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

        当起点位于不可行区域（如膨胀层）时，以同心壳层向外搜索最近的安全节点。

        [修复魔术数字]: 搜索半径不再固定为 5，而是根据吊具尺寸和分辨率动态计算，
        确保能够跳出膨胀层。

        Args:
            node (GridNode): 原始起点网格坐标。
            ref_goal (GridNode): 目标点网格坐标（用于启发式选择最优脱困点）。
            planner (PathPlannerBase): 规划器实例（用于检查节点安全性）。
            settings (Settings): 全局配置（用于计算动态搜索半径）。

        Returns:
            Optional[GridNode]: 找到的脱困点坐标，若未找到则返回 None。
        """
        # 如果当前点本身是安全的，直接返回
        if planner.is_safe(node):
            return node

        # 1. 动态计算搜索半径上限
        # 计算膨胀半径在网格上的跨度
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length
        radius_m = (w / 2.0) if shape == SHAPE_CIRCLE else (math.hypot(w, l) / 2.0)

        resolution = self.map_mgr.resolution_m
        grid_inflation_radius = int(math.ceil(radius_m / resolution))

        # 设定最大搜索范围：膨胀半径 + 5格缓冲
        # 这确保了搜索范围一定能覆盖到膨胀层的边缘外侧
        max_search_r = grid_inflation_radius + 5

        # 限制一个硬上限，防止在极端分辨率下搜索过久
        max_search_r = min(max_search_r, 50)

        self.logger.warning(
            f"尝试脱困: Node={node}, MaxRadius={max_search_r} (Res={resolution}m)"
        )

        dims = len(node)

        # 2. 同心壳层搜索 (Concentric Shell Search)
        for r in range(1, max_search_r + 1):
            candidates: List[GridNode] = []

            if dims == 2:
                cx, cy = node  # type: ignore
                # 优化循环：只遍历外壳 (Shell)，不遍历内部
                # 外壳由四条边组成
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        # 切比雪夫距离：max(|dx|, |dy|) == r 即为当前层的外壳
                        if max(abs(dx), abs(dy)) == r:
                            n = (cx + dx, cy + dy)
                            if planner.is_safe(n):
                                candidates.append(n)
            else:
                cx, cy, cz = node  # type: ignore
                # 3D 外壳搜索
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        for dz in range(-r, r + 1):
                            if max(abs(dx), abs(dy), abs(dz)) == r:
                                n = (cx + dx, cy + dy, cz + dz)
                                if planner.is_safe(n):
                                    candidates.append(n)

            if candidates:
                # 贪婪策略：在当前壳层中，选择欧氏距离离目标最近的点
                # 这样可以确保脱困的方向大致是朝向目标的
                best_node = min(
                    candidates,
                    key=lambda n: sum((n[i] - ref_goal[i]) ** 2 for i in range(dims)),
                )
                self.logger.info(f"脱困成功! 半径: {r}, 新起点: {best_node}")
                return best_node

        self.logger.error(f"脱困失败: 在半径 {max_search_r} 格范围内未找到安全节点。")
        return None

    def create_collision_checker(
        self, grace_start: Point3D, grace_end: Point3D, settings: Settings
    ) -> Callable[[Point3D], bool]:
        """创建一个带豁免区的碰撞检测闭包函数。

        用于后处理阶段（如平滑算法）的密集检测。
        由于后处理曲线可能微小地偏离网格中心，为了防止起终点因浮点误差被误判为碰撞，
        设置了半径 0.5m 的豁免区 (Grace Zone)。

        Args:
            grace_start (Point3D): 豁免起点。
            grace_end (Point3D): 豁免终点。
            settings (Settings): 全局配置。

        Returns:
            Callable[[Point3D], bool]: 检测函数，返回 True(安全) 或 False(碰撞)。
        """
        shape = settings.crane.footprint_shape
        w = settings.crane.footprint_width
        l = settings.crane.footprint_length

        # 计算包络半径
        # 额外增加半个分辨率的缓冲，防止在网格边缘产生穿模
        radius = (w / 2.0) if shape == SHAPE_CIRCLE else (math.hypot(w, l) / 2.0)
        radius += self.map_mgr.resolution_m / 2.0

        z_margin = (
            settings.crane.z_safety_margin + settings.crane.footprint_height / 2.0
        )
        is_infinite = settings.crane.obstacle_infinite_height

        def check(pt: Tuple[float, ...]) -> bool:
            x, y, z = pt[0], pt[1], pt[2]

            # 豁免区检测 (Grace Zone) - 半径平方 0.25 (即 0.5m)
            # 允许路径端点稍微“蹭”到一点障碍物边缘（通常是自身占据的位置）
            if (x - grace_start[0]) ** 2 + (y - grace_start[1]) ** 2 + (
                z - grace_start[2]
            ) ** 2 < 0.25:
                return True
            if (x - grace_end[0]) ** 2 + (y - grace_end[1]) ** 2 + (
                z - grace_end[2]
            ) ** 2 < 0.25:
                return True

            # 调用 MapManager 进行精确检测
            # 返回 True 表示碰撞，取反返回 Is Safe
            if self.map_mgr.check_collision_raw(
                x, y, z, radius, z_margin, ignore_z=is_infinite
            ):
                return False
            return True

        return check
