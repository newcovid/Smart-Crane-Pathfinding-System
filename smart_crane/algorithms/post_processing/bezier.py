import logging
import math
from typing import List, Optional

from .base import PathPostProcessor, CollisionChecker, NodeType


class BezierSmoothProcessor(PathPostProcessor):
    """贝塞尔平滑处理器 (Bezier Curve Smoother)。

    利用二次贝塞尔曲线 (Quadratic Bezier Curve) 将路径中的折线拐角打磨成平滑的圆弧。
    包含自适应回退机制：如果生成的曲线发生碰撞，自动减小曲率半径重试。
    """

    def __init__(
        self,
        smoothness: float = 0.3,
        segments: int = 10,
        logger: Optional[logging.Logger] = None,
    ):
        """初始化平滑器。

        Args:
            smoothness (float): 平滑度因子 (0.0 ~ 0.5)。
                                0.5 表示从线段中点开始切角（最大弧度）。
            segments (int): 曲线插值段数，数值越大曲线越圆滑，计算量越大。
            logger (Optional[logging.Logger]): 父级日志记录器。
        """
        super().__init__(name="BezierSmoother", logger=logger)

        self.default_smoothness = max(0.0, min(0.5, smoothness))
        self.num_segments = segments

        self.logger.info(
            f"初始化完成. 平滑度={self.default_smoothness}, 插值段数={self.num_segments}"
        )

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """执行贝塞尔平滑逻辑。

        Args:
            path: 原始路径。
            is_safe_fn: 碰撞检测函数。

        Returns:
            List[NodeType]: 平滑后的路径。
        """
        if not path or len(path) < 3:
            return path

        smoothed_path = []
        smoothed_path.append(path[0])

        input_nodes_count = len(path)
        curved_corners = 0

        # 遍历每一个拐角点 P1 (从第2个点到倒数第2个点)
        for i in range(1, len(path) - 1):
            p0 = path[i - 1]  # 前驱点
            p1 = path[i]  # 拐角点 (控制点)
            p2 = path[i + 1]  # 后继点

            current_smoothness = self.default_smoothness
            best_segment = None

            # 自适应回退机制 (Adaptive Fallback)
            # 尝试生成曲线，如果碰撞则减半平滑度重试，最多 5 次
            for attempt in range(5):
                if self._check_curve_safety(p0, p1, p2, current_smoothness, is_safe_fn):
                    best_segment = self._generate_curve(p0, p1, p2, current_smoothness)

                    if attempt > 0:
                        self.logger.debug(
                            f"拐角 {i} 触发自适应回退: 尝试 {attempt} 次后成功 "
                            f"(Smoothness: {self.default_smoothness} -> {current_smoothness:.4f})"
                        )
                    break
                else:
                    # 碰撞，减小半径
                    current_smoothness *= 0.5

            if best_segment:
                # 成功生成曲线，加入曲线点集（替代原始拐角点 p1）
                smoothed_path.extend(best_segment)
                curved_corners += 1
            else:
                # 无法平滑，保留原始硬拐角
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
        """检查生成的贝塞尔曲线是否安全。

        通过在假设的曲线上进行高密度采样来检测碰撞。

        Args:
            p0, p1, p2: 路径上的连续三点。
            s: 平滑度因子。
            is_safe_fn: 碰撞检测回调。

        Returns:
            bool: True 表示安全。
        """
        dims = len(p0)

        # 计算切点 (Anchor Points)
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        # 估算曲线弧长
        d1 = math.sqrt(sum((p1[d] - p0[d]) ** 2 for d in range(dims)))
        d2 = math.sqrt(sum((p2[d] - p1[d]) ** 2 for d in range(dims)))
        total_len = (d1 + d2) * s

        # 采样密度: 每 0.2m 检测一次
        steps = int(total_len * 5)
        steps = max(steps, 5)

        for k in range(steps + 1):
            t = k / steps
            check_point_list = []
            for d in range(dims):
                # 二阶贝塞尔公式: B(t) = (1-t)^2 * q0 + 2t(1-t) * p1 + t^2 * q2
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]
                check_point_list.append(val)

            check_point = tuple(check_point_list)
            if not is_safe_fn(check_point):  # type: ignore
                return False

        return True

    def _generate_curve(
        self, p0: NodeType, p1: NodeType, p2: NodeType, s: float
    ) -> List[tuple]:
        """生成贝塞尔曲线点集。

        Args:
            p0, p1, p2: 路径三点。
            s: 平滑度。

        Returns:
            List[tuple]: 曲线上的点列表（不包含起点，包含终点）。
        """
        dims = len(p0)

        # 确定起止切点
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        points = []
        # 插值生成中间点
        for i in range(1, self.num_segments + 1):
            t = i / self.num_segments
            pt_list = []

            for d in range(dims):
                # 二阶贝塞尔公式
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]
                pt_list.append(val)

            points.append(tuple(pt_list))

        return points
