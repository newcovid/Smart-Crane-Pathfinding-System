import logging
import math
from typing import List, Tuple, Union
from .base import PathPostProcessor, CollisionChecker, NodeType


class BezierSmoothProcessor(PathPostProcessor):
    """
    【L1.5 层 - 贝塞尔平滑处理器】
    (Bezier Curve Smoother & Trajectory Refiner)

    核心算法:
    使用二阶贝塞尔曲线 (Quadratic Bezier Curve) 替换路径中的尖锐拐角。

    特性:
    1. **自适应回退 (Adaptive Fallback)**:
       如果标准的大圆弧（高平滑度）会导致碰撞，算法会自动尝试减小圆弧半径。
       如果最小圆弧依然碰撞，则保留原始尖角，确保安全第一。
    2. **类型兼容**:
       虽然输入通常是整数网格坐标，但平滑后的路径会保留浮点精度，
       以便下游的 L2 轨迹生成层能生成更细腻的控制指令。
    3. **维度自适应**: 支持 2D/3D 平滑。
    """

    def __init__(self, smoothness: float = 0.3, segments: int = 10):
        """
        初始化平滑器。

        Args:
            smoothness (float): 初始平滑因子 (0.0 ~ 0.5)。
                                0.5 表示从线段中点开始切角 (最大圆弧)。
                                0.1 表示仅在拐点附近切角。
            segments (int): 每个弯道生成的细分点数量。数量越高曲线越圆滑，但数据量越大。
        """
        super().__init__(name="BezierSmoother")
        # 限制平滑度在合理物理范围内
        self.default_smoothness = max(0.0, min(0.5, smoothness))
        self.num_segments = segments

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """
        [核心实现] 执行贝塞尔平滑。
        """
        # 1. 基础检查
        if not path or len(path) < 3:
            return path

        # 初始化结果路径，首先放入起点
        # 注意：这里我们可能会混入 float 类型的坐标，这在 Python 中是允许的
        smoothed_path = []
        smoothed_path.append(path[0])

        # 2. 遍历每一个“拐角” (P1)
        # 结构: P0(前一点) ----- P1(拐点) ----- P2(后一点)
        for i in range(1, len(path) - 1):
            p0 = path[i - 1]
            p1 = path[i]
            p2 = path[i + 1]

            current_smoothness = self.default_smoothness
            best_segment = None

            # 3. 自适应尝试循环 (Adaptive Fallback Loop)
            # 尝试 5 次，每次失败后将平滑度减半 (0.3 -> 0.15 -> 0.075 ...)
            for attempt in range(5):
                # A. 碰撞模拟 (Simulate)
                # 使用高密度点阵检查假想的曲线是否安全
                if self._check_curve_safety(p0, p1, p2, current_smoothness, is_safe_fn):
                    # B. 生成曲线 (Generate)
                    # 如果安全，生成用于输出的稀疏点
                    best_segment = self._generate_curve(p0, p1, p2, current_smoothness)

                    if attempt > 0:
                        self.logger.debug(
                            f"拐点 {i} 触发自适应回退: 尝试 {attempt} 次后成功 (s={current_smoothness:.4f})"
                        )
                    break
                else:
                    # 碰撞！减小半径重试
                    current_smoothness *= 0.5

            # 4. 结果合并
            if best_segment:
                smoothed_path.extend(best_segment)
            else:
                # 如果缩到很小都不行（比如在死胡同里掉头），说明空间太狭窄
                # 只能走尖角，确保不撞墙
                # self.logger.debug(f"拐点 {i} 平滑失败: 空间不足，保持尖角")
                smoothed_path.append(p1)

        # 加入终点
        smoothed_path.append(path[-1])

        # 5. 类型标准化
        # 尝试将结果转换回与输入一致的类型 (例如，如果输入全是整数，这里也尝试四舍五入回整数)
        # 但通常建议保留浮点数以获得更好控制效果，这里提供一个可选的转换逻辑
        return self._cast_to_input_type(smoothed_path, path[0])

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
        这完全是为了碰撞检测，采样密度远高于最终输出的 segments。
        """
        dims = len(p0)

        # 计算贝塞尔控制点 Q0, Q2
        # Q0 = P1 + (P0 - P1) * s
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        # 估算曲线物理长度，以决定采样数
        # 简单估算上限：两腰长度之和 * s
        d1 = math.sqrt(sum((p1[d] - p0[d]) ** 2 for d in range(dims)))
        d2 = math.sqrt(sum((p2[d] - p1[d]) ** 2 for d in range(dims)))
        total_len = (d1 + d2) * s

        # 采样策略：每 0.2 距离单位检查一个点 (比 Greedy 更密，因为曲线容易蹭墙角)
        steps = int(total_len * 5)
        steps = max(steps, 5)  # 兜底至少检查5个点

        for k in range(steps + 1):
            t = k / steps

            # 二阶贝塞尔公式: B(t) = (1-t)^2 * Q0 + 2t(1-t) * P1 + t^2 * Q2
            check_point = []
            for d in range(dims):
                # 计算浮点坐标
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]
                # 取整去查网格 (Grid 是离散的)
                check_point.append(int(round(val)))

            if not is_safe_fn(tuple(check_point)):
                return False  # 撞墙

        return True

    def _generate_curve(
        self, p0: NodeType, p1: NodeType, p2: NodeType, s: float
    ) -> List[tuple]:
        """
        [内部方法] 生成最终输出的曲线点。
        这些点不进行取整，保留浮点精度，供 L2 轨迹生成层使用。
        """
        dims = len(p0)

        # 计算控制点
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        points = []
        # 注意范围：range(1, segments + 1)
        # 我们不生成 t=0 (那是上一段的终点)
        # 也不生成 t=1 (留给下一段，或者本段的最后一个点作为终点)
        # 这样拼接起来才是连续的
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
        """
        [辅助方法] 类型转换。
        如果输入是整数类型的坐标 (int)，且用户希望保持输出一致性，
        此方法会将浮点路径四舍五入回整数。

        注意：在实际工控中，通常建议保留浮点数。
        """
        # 简单判断：如果输入的样本坐标里全是 int
        if isinstance(sample[0], int):
            # 这里演示保留浮点数，因为对于贝塞尔平滑来说，强制转回 int 会丢失所有平滑效果，
            # 导致曲线变成锯齿，失去了平滑的意义。
            # 因此，我们**有意**返回浮点数 Tuple，即使输入是整数。
            # 这符合 Python 的 Duck Typing 哲学。
            return float_path

            # 如果非要转回整数，取消下面注释：
            # return [tuple(int(round(x)) for x in p) for p in float_path]

        return float_path
