import logging
import math
from typing import List, Optional

from .base import PathPostProcessor, CollisionChecker, NodeType


class GreedyShortcutProcessor(PathPostProcessor):
    """贪婪捷径优化器 (Greedy Shortcut Optimizer)。

    通过视线检查 (Line-of-Sight) 去除路径中的冗余节点。
    如果节点 A 和节点 C 之间可以直接连线且无碰撞，则跳过中间的节点 B。
    """

    def __init__(self, logger: Optional[logging.Logger] = None):
        """初始化贪婪优化器。

        Args:
            logger (Optional[logging.Logger]): 父级日志记录器。
        """
        super().__init__(name="GreedyShortcut", logger=logger)
        self.logger.info("贪婪捷径策略初始化完成。")

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """执行贪婪路径简化。

        Args:
            path: 原始路径。
            is_safe_fn: 碰撞检测函数。

        Returns:
            List[NodeType]: 简化后的路径。
        """
        # 路径点少于 3 个无法优化
        if not path or len(path) < 3:
            self.logger.debug(f"路径太短 ({len(path)} 节点)，无需优化。")
            return path

        optimized_path = [path[0]]

        # current_idx 指向 optimized_path 中最后一个确定的锚点
        # 在原始路径 path 中的索引
        current_idx = 0
        total_nodes = len(path)
        shortcut_count = 0

        # 主循环：从当前锚点向后寻找最远的可达点
        while current_idx < total_nodes - 1:
            # 贪婪策略：从终点开始反向遍历
            check_idx = total_nodes - 1
            found_shortcut = False

            # 内循环：尝试连接 current_idx 和 check_idx
            # 至少要跳过一个中间点 (check_idx > current_idx + 1)
            while check_idx > current_idx + 1:
                start_node = path[current_idx]
                target_node = path[check_idx]

                # 视线检查
                if self._has_line_of_sight(start_node, target_node, is_safe_fn):
                    # 发现捷径，直接连接到 target_node
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

            # 兜底：如果无法跳过任何点，则前进一步
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
        """检查两点之间是否存在无障碍视线 (Line of Sight)。

        Args:
            start: 起点。
            end: 终点。
            is_safe_fn: 碰撞检测函数。

        Returns:
            bool: True 表示视线通畅。
        """
        dims = len(start)

        # 欧几里得距离平方
        dist_sq = sum((start[i] - end[i]) ** 2 for i in range(dims))
        distance = math.sqrt(dist_sq)

        # 极短距离直接通过
        if distance < 1e-6:
            return True

        # 采样密度：每米至少 5 个点 (0.2m 精度)
        steps = int(distance * 5)
        steps = max(steps, 2)

        deltas = [(end[i] - start[i]) for i in range(dims)]

        # 采样检测 (跳过起点和终点)
        for i in range(1, steps):
            t = i / steps
            current_point = []

            for d in range(dims):
                # 线性插值: p = start + delta * t
                val = start[d] + deltas[d] * t
                current_point.append(val)

            # 必须保留浮点数进行精确碰撞检测
            pt = tuple(current_point)
            if not is_safe_fn(pt):  # type: ignore
                return False

        return True
