import logging
import math
from typing import List, Tuple, Union
from .base import PathPostProcessor, CollisionChecker, NodeType


class BezierSmoothProcessor(PathPostProcessor):
    """
    【L1.5 层 - 贝塞尔平滑处理器 (Float Precision Fix)】
    (Bezier Curve Smoother & Trajectory Refiner)

    修复记录:
    - Fix: _check_curve_safety 中移除了坐标取整 (int(round(...)))。
      现在使用浮点坐标进行碰撞检测，防止曲线在障碍物边缘因取整误差而导致“穿模”。
    """

    def __init__(self, smoothness: float = 0.3, segments: int = 10):
        super().__init__(name="BezierSmoother")
        self.default_smoothness = max(0.0, min(0.5, smoothness))
        self.num_segments = segments

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """
        [核心实现] 执行贝塞尔平滑。
        """
        if not path or len(path) < 3:
            return path

        smoothed_path = []
        smoothed_path.append(path[0])

        for i in range(1, len(path) - 1):
            p0 = path[i - 1]
            p1 = path[i]
            p2 = path[i + 1]

            current_smoothness = self.default_smoothness
            best_segment = None

            # 自适应回退循环
            for attempt in range(5):
                # 碰撞模拟: 必须使用 float 进行精确检查
                if self._check_curve_safety(p0, p1, p2, current_smoothness, is_safe_fn):
                    best_segment = self._generate_curve(p0, p1, p2, current_smoothness)
                    if attempt > 0:
                        self.logger.debug(
                            f"拐点 {i} 触发自适应回退: 尝试 {attempt} 次后成功 (s={current_smoothness:.4f})"
                        )
                    break
                else:
                    current_smoothness *= 0.5

            if best_segment:
                smoothed_path.extend(best_segment)
            else:
                smoothed_path.append(p1)

        smoothed_path.append(path[-1])
        return smoothed_path

    def _check_curve_safety(
        self,
        p0: NodeType,
        p1: NodeType,
        p2: NodeType,
        s: float,
        is_safe_fn: CollisionChecker,
    ) -> bool:
        """
        [内部方法] 使用高密度采样检查曲线安全性。
        Fix: 保留浮点精度，直接传给 is_safe_fn。
        """
        dims = len(p0)

        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        d1 = math.sqrt(sum((p1[d] - p0[d]) ** 2 for d in range(dims)))
        d2 = math.sqrt(sum((p2[d] - p1[d]) ** 2 for d in range(dims)))
        total_len = (d1 + d2) * s

        # 采样策略：每 0.2 距离单位检查一个点
        steps = int(total_len * 5)
        steps = max(steps, 5)

        for k in range(steps + 1):
            t = k / steps

            check_point = []
            for d in range(dims):
                # 二阶贝塞尔公式
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]

                # [Fixed] 不要取整！保留 float 传给碰撞检测器
                # CollisionChecker (在 TrajectoryPlanner 中定义) 负责将 float 转为 grid index
                check_point.append(val)

            # 传入 Tuple[float, ...]
            if not is_safe_fn(tuple(check_point)):
                return False  # 撞墙

        return True

    def _generate_curve(
        self, p0: NodeType, p1: NodeType, p2: NodeType, s: float
    ) -> List[tuple]:
        dims = len(p0)
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        points = []
        for i in range(1, self.num_segments + 1):
            t = i / self.num_segments
            pt_list = []
            for d in range(dims):
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]
                pt_list.append(val)
            points.append(tuple(pt_list))

        return points

    def _cast_to_input_type(
        self, float_path: List[tuple], sample: NodeType
    ) -> List[NodeType]:
        return float_path
