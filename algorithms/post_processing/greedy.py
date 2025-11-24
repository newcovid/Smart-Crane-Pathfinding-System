import logging
import math
from typing import List, Tuple, Any

# 导入基类和类型定义
# PathPostProcessor: 后处理器基类
# CollisionChecker: 一个函数类型，用来检查某个坐标是否会撞墙
# NodeType: 泛型坐标类型，可能是 (x, y) 也可能是 (x, y, z)
from .base import PathPostProcessor, CollisionChecker, NodeType


class GreedyShortcutProcessor(PathPostProcessor):
    """
    【L1.5 层 - 贪婪捷径优化器 (Greedy Shortcut Optimizer)】
    (Line-of-Sight Path Smoother)

    功能简介:
    A*或D*规划出来的路线是沿着网格一格一格走的（像走楼梯一样，Zig-Zag）。
    这个优化器的作用是检查能不能直接从 A 点走到 C 点，而不经过 B 点？
    如果 A 和 C 之间没有障碍物（视线通畅），那就可以直接走直线过去。

    核心逻辑:
    1. 站在路径的起点（锚点）。
    2. 往后看路径上最远的那个点（终点）。
    3. 检查能否直接连线过去而不撞墙。
    4. 如果能，就直接连过去，中间的点全扔掉。
    5. 如果不能，就看倒数第二个点，以此类推。
    """

    def __init__(self):
        """
        初始化贪婪优化器。
        """
        # 调用父类的初始化，设置名字为 "GreedyShortcut"
        super().__init__(name="GreedyShortcut")
        self.logger.info(f"[{self.name}] 贪婪捷径策略初始化完成。")

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """
        执行贪婪路径简化。

        Args:
            path: 原始路径（通常包含很多冗余的网格点）。
            is_safe_fn: 碰撞检测函数（用来判断直线连线是否安全）。

        Returns:
            List[NodeType]: 简化后的路径（点变少了，线变直了）。
        """
        # 1. 基础检查：如果路径太短（少于3个点），就没有中间点可以跳过，直接返回
        if not path or len(path) < 3:
            self.logger.debug(f"[{self.name}] 路径太短 ({len(path)} 节点)，无需优化。")
            return path

        # 初始化优化后的路径，先把起点放进去
        optimized_path = [path[0]]

        # current_idx 指向原始路径中“当前已确认安全的最远点”
        # 我们从这个点开始，尝试寻找下一个跳跃点
        current_idx = 0
        total_nodes = len(path)

        # 统计我们成功“抄近道”的次数，用于日志展示
        shortcut_count = 0

        # 2. 主循环：只要还没处理到终点，就继续找
        while current_idx < total_nodes - 1:
            # check_idx 是我们尝试连接的目标点索引
            # 贪婪策略：我们总是先尝试最远的点（也就是终点），然后慢慢往回缩
            check_idx = total_nodes - 1

            found_shortcut = False  # 标记是否找到了捷径

            # 内循环：从路径末尾开始反向遍历，寻找能直接连通的最远邻居
            # 限制条件: check_idx > current_idx + 1
            # 意思是我们至少要跳过一个中间点，如果只是连相邻的点，那就不是捷径了
            while check_idx > current_idx + 1:
                target_node = path[check_idx]
                start_node = path[current_idx]

                # 执行视线检查 (Ray Casting / Line of Sight)
                # 问：从 start_node 直线走到 target_node 安全吗？
                if self._has_line_of_sight(start_node, target_node, is_safe_fn):
                    # 发现捷径！可以直接从 current_idx 跳到 check_idx
                    optimized_path.append(target_node)

                    # 记录一下日志 (DEBUG级别)
                    self.logger.debug(
                        f"[{self.name}] 发现捷径: 节点 {current_idx} -> {check_idx} "
                        f"(跳过了 {check_idx - current_idx - 1} 个中间点)"
                    )

                    # 更新当前锚点为刚才找到的目标点
                    current_idx = check_idx
                    found_shortcut = True
                    shortcut_count += 1
                    break  # 既然找到了最远的捷径，内循环就可以结束了，继续往下找

                # 如果不能直连，就尝试近一点的那个点
                check_idx -= 1

            # 3. 兜底逻辑
            # 如果内循环跑完了，发现一个捷径也没找到（说明前面全是障碍物，必须沿着原路走）
            if not found_shortcut:
                # 那就只能老老实实走一步：把原路径的下一个点加进来
                current_idx += 1
                if current_idx < total_nodes:
                    optimized_path.append(path[current_idx])

        # 优化结束
        self.logger.debug(
            f"[{self.name}] 优化完成. 成功通过 {shortcut_count} 次捷径操作，"
            f"将 {total_nodes} 个节点压缩为 {len(optimized_path)} 个。"
        )

        return optimized_path

    def _has_line_of_sight(
        self, start: NodeType, end: NodeType, is_safe_fn: CollisionChecker
    ) -> bool:
        """
        [辅助方法] 检查两点之间是否存在视线 (Line of Sight)。

        原理:
        在两点之间画一条虚构的直线，然后在直线上每隔很短的距离（采样步长）取一个点，
        检查这个点是否撞墙。只要有一个点撞墙，就说明视线被阻挡。

        Args:
            start: 起点坐标 (2D 或 3D)。
            end: 终点坐标。
            is_safe_fn: 外部传入的碰撞检测函数。

        Returns:
            bool: True 表示视线通畅（无障碍），False 表示有阻挡。
        """
        # 1. 维度自适应 (自动识别是 2D 还是 3D)
        dims = len(start)

        # 2. 计算两点之间的直线距离 (欧几里得距离)
        # 公式: sqrt((x1-x2)^2 + (y1-y2)^2 + ...)
        dist_sq = sum((start[i] - end[i]) ** 2 for i in range(dims))
        distance = math.sqrt(dist_sq)

        # 特殊情况处理：如果两点重合或极近，认为无障碍
        if distance < 1e-6:
            return True

        # 3. 确定采样步数 (Sampling Steps)
        # 策略：每 1.0 距离单位（米）至少采样 5 次 (即 0.2m 精度)
        # 为什么要采样这么密？因为如果采样太稀疏，可能会漏掉那种很薄的墙壁。
        steps = int(distance * 5)

        # 至少检查 2 个点，防止除以零错误
        steps = max(steps, 2)

        # 4. 预计算各轴的增量 (Delta)
        # 也就是每一步在 x, y, z 轴上分别要走多远
        deltas = [(end[i] - start[i]) for i in range(dims)]

        # 5. 采样循环
        # 注意：我们通常不需要检查起点 (i=0) 和终点 (i=steps)，
        # 因为在之前的逻辑中，起点和终点通常已经被验证为安全的网格点了。
        # 我们主要关心的是“中间”有没有障碍物。
        for i in range(1, steps):
            # t 是进度比例，从 0.0 到 1.0
            t = i / steps

            # 线性插值公式: P(t) = Start + Delta * t
            current_point = []
            for d in range(dims):
                val = start[d] + deltas[d] * t

                # 这里保留浮点数 (float)，不要用 int() 取整
                # 因为 is_safe_fn (也就是 TrajectoryPlanner 里的 check_collision_raw)
                # 是支持浮点坐标判断的。如果这里取整，会导致射线“瞬移”到网格中心，
                # 从而漏掉位于网格边缘的障碍物。
                current_point.append(val)

            # 转换成元组，方便传参
            pt = tuple(current_point)

            # 只要有一个采样点撞墙，就立刻返回 False
            if not is_safe_fn(pt):
                return False

        # 跑完全程都没撞墙，说明视线通畅
        return True
