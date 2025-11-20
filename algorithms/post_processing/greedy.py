import logging
import math
from typing import List, Tuple
from .base import PathPostProcessor, CollisionChecker, NodeType


class GreedyShortcutProcessor(PathPostProcessor):
    """
    【L1.5 层 - 贪婪捷径优化器】
    (Greedy Shortcut / Line-of-Sight Optimizer)

    核心算法:
    该处理器实现了基于 "视线检查 (Line-of-Sight)" 的路径简化算法。
    它模仿人类的直觉：如果从点 A 到点 C 之间没有障碍物阻挡，
    那么就没有必要经过中间点 B，而是直接走直线 A->C。

    特性:
    1. **维度自适应**: 自动支持 2D (x,y) 或 3D (x,y,z) 路径。
    2. **高密度采样**: 在视线检查时使用亚像素级采样，防止穿墙。
    3. **贪婪策略**: 总是尝试连接当前点与路径上最远的可见点。
    """

    def __init__(self):
        """初始化贪婪优化器。"""
        super().__init__(name="GreedyShortcut")

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """
        [核心实现] 执行贪婪路径简化。

        注意: 此方法由基类 process() 调用，无需自行处理异常或统计。

        Args:
            path: 原始路径点列表。
            is_safe_fn: 碰撞检测回调。

        Returns:
            简化后的稀疏路径。
        """
        # 1. 基础检查：路径过短无需优化
        if not path or len(path) < 3:
            return path

        # 初始化优化后的路径，放入起点
        optimized_path = [path[0]]

        # current_idx 指向原始路径中“当前已确认的最远安全点”
        current_idx = 0
        total_nodes = len(path)

        # 2. 主循环：直到处理完倒数第二个节点
        # 我们总是试图寻找从 path[current_idx] 出发，能直达的最远节点
        while current_idx < total_nodes - 1:
            check_idx = total_nodes - 1
            found_shortcut = False

            # 内循环：从终点开始反向遍历，寻找最远的可见邻居
            # 这是一个贪婪策略 (Greedy Strategy)
            while check_idx > current_idx + 1:
                target_node = path[check_idx]

                # 执行视线检查 (Ray Casting)
                if self._has_line_of_sight(path[current_idx], target_node, is_safe_fn):
                    # 发现捷径！A -> ... -> Z 之间畅通无阻
                    # 直接将 Z 加入路径，丢弃中间的所有节点
                    optimized_path.append(target_node)

                    # 更新索引，跳跃到 Z 继续往后找
                    current_idx = check_idx
                    found_shortcut = True
                    break

                check_idx -= 1

            # 3. 兜底逻辑
            # 如果从当前点出发，连一个隔代的节点都看不到（到处都是障碍物），
            # 只能老老实实走原始路径的下一步。
            if not found_shortcut:
                current_idx += 1
                if current_idx < total_nodes:
                    optimized_path.append(path[current_idx])

        return optimized_path

    def _has_line_of_sight(
        self, start: NodeType, end: NodeType, is_safe_fn: CollisionChecker
    ) -> bool:
        """
        [辅助方法] 检查两点之间是否存在障碍物 (支持 N 维)。

        实现原理:
        使用高密度线性插值进行采样。不使用 Bresenham 算法是因为我们需要
        亚网格级别的检测精度，以防止在障碍物夹角处发生“穿模”。
        """
        # 1. 维度自适应 (2D 或 3D)
        dims = len(start)

        # 2. 计算欧几里得距离
        dist_sq = sum((start[i] - end[i]) ** 2 for i in range(dims))
        distance = math.sqrt(dist_sq)

        # 如果两点重合或极近，认为无障碍
        if distance < 1e-6:
            return True

        # 3. 确定采样步数
        # 策略：每 1.0 距离单位（一个网格边长）至少采样 5 次
        # 这是一个激进的安全策略，确保不会穿过薄壁障碍物
        steps = int(distance * 5)
        steps = max(steps, 2)  # 至少检查两点

        # 4. 预计算各轴的增量 (Delta)
        deltas = [(end[i] - start[i]) for i in range(dims)]

        # 5. 采样循环
        # 注意：不需要检查起点 (i=0) 和终点 (i=steps)，因为它们通常在路径上已经是合法的。
        # 我们主要检查中间连线是否碰到障碍物。
        for i in range(1, steps):
            t = i / steps

            # 线性插值公式: P(t) = Start + Delta * t
            current_point = []
            for d in range(dims):
                val = start[d] + deltas[d] * t
                # 四舍五入取整到最近的网格中心进行检查
                current_point.append(int(round(val)))

            # 转换为 Tuple 以适配 NodeType 泛型 (Tuple[int, ...])
            pt = tuple(current_point)

            # 调用外部传入的检测函数 (通常是 Planner.is_safe)
            # 如果任意一个采样点不安全，则视线受阻
            if not is_safe_fn(pt):
                return False

        return True
