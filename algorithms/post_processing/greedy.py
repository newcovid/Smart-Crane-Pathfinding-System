import logging
import math
from typing import List, Tuple
from .base import PathPostProcessor, CollisionChecker, NodeType


class GreedyShortcutProcessor(PathPostProcessor):
    """
    【L1.5 层 - 贪婪捷径优化器 (Float Fix)】
    (Greedy Shortcut / Line-of-Sight Optimizer)

    修复记录:
    - Fix: 移除了坐标采样的 int(round(...)) 强制取整，支持浮点世界坐标。
      解决了在世界坐标系下进行视线检查时的精度丢失问题。
    """

    def __init__(self):
        """初始化贪婪优化器。"""
        super().__init__(name="GreedyShortcut")

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """
        [核心实现] 执行贪婪路径简化。
        """
        # 1. 基础检查：路径过短无需优化
        if not path or len(path) < 3:
            return path

        # 初始化优化后的路径，放入起点
        optimized_path = [path[0]]

        # current_idx 指向原始路径中“当前已确认的最远安全点”
        current_idx = 0
        total_nodes = len(path)

        # 2. 主循环
        while current_idx < total_nodes - 1:
            check_idx = total_nodes - 1
            found_shortcut = False

            # 内循环：从终点开始反向遍历，寻找最远的可见邻居
            while check_idx > current_idx + 1:
                target_node = path[check_idx]

                # 执行视线检查 (Ray Casting)
                if self._has_line_of_sight(path[current_idx], target_node, is_safe_fn):
                    # 发现捷径！
                    optimized_path.append(target_node)
                    current_idx = check_idx
                    found_shortcut = True
                    break

                check_idx -= 1

            # 3. 兜底逻辑
            if not found_shortcut:
                current_idx += 1
                if current_idx < total_nodes:
                    optimized_path.append(path[current_idx])

        return optimized_path

    def _has_line_of_sight(
        self, start: NodeType, end: NodeType, is_safe_fn: CollisionChecker
    ) -> bool:
        """
        [辅助方法] 检查两点之间是否存在障碍物 (支持 N 维浮点坐标)。
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
        # 策略：每 1.0 距离单位（米）至少采样 5 次 (即 0.2m 精度)
        # 这足以覆盖大部分网格边缘情况
        steps = int(distance * 5)
        steps = max(steps, 2)  # 至少检查两点

        # 4. 预计算各轴的增量 (Delta)
        deltas = [(end[i] - start[i]) for i in range(dims)]

        # 5. 采样循环
        # 注意：不需要检查起点 (i=0) 和终点 (i=steps)
        for i in range(1, steps):
            t = i / steps

            # 线性插值公式: P(t) = Start + Delta * t
            current_point = []
            for d in range(dims):
                val = start[d] + deltas[d] * t
                # [Fix] 移除 int(round(val))，保留浮点精度
                # is_safe_fn (即 world_is_safe) 会负责将浮点坐标正确映射回网格
                current_point.append(val)

            pt = tuple(current_point)

            if not is_safe_fn(pt):
                return False

        return True
