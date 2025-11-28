import logging
import math
from typing import List, Tuple, Union, Any

# 导入基类和类型定义
from .base import PathPostProcessor, CollisionChecker, NodeType


class BezierSmoothProcessor(PathPostProcessor):
    """
    【L1.5 层 - 贝塞尔平滑处理器 (Bezier Curve Smoother)】

    功能简介:
    这个处理器的作用是将路径中的"尖角"打磨成"圆角"。
    原始路径通常由一系列直线段组成 (A -> B -> C)，在 B 点会有一个生硬的转向。
    本算法利用二次贝塞尔曲线 (Quadratic Bezier Curve) 在 B 点附近生成一条平滑的弧线，
    让机器人的运动更加流畅，减少急停急转。

    核心逻辑:
    1. 遍历路径中的每三个连续点 (P0, P1, P2)，其中 P1 是拐角点。
    2. 在 P0-P1 和 P1-P2 线段上分别选取起贝塞尔起点 q0 和终点 q2。
    3. 以 P1 为控制点，生成连接 q0 和 q2 的抛物线。
    4. **安全性检查**: 如果生成的曲线会碰到障碍物，则自动缩小曲线半径重试 (自适应回退)。
    """

    def __init__(self, smoothness: float = 0.3, segments: int = 10):
        """
        初始化平滑器。

        Args:
            smoothness (float): 平滑度因子 (0.0 ~ 0.5)。
                                - 0.1: 拐弯半径很小，贴着角转。
                                - 0.5: 拐弯半径最大，从线段中点就开始切角。
                                - 默认 0.3 是一个比较平衡的值。
            segments (int): 插值密度。
                            也就是把那个弯角切成多少段小直线。数值越大曲线越圆滑，但计算量越大。
        """
        super().__init__(name="BezierSmoother")

        # 限制平滑度在合理范围内 (0.0 到 0.5)
        # 为什么最大是 0.5？因为如果超过 0.5，两端的曲线可能会重叠打架。
        self.default_smoothness = max(0.0, min(0.5, smoothness))

        self.num_segments = segments

        self.logger.info(
            f"[{self.name}] 初始化完成. 平滑度: {self.default_smoothness}, "
            f"曲线插值段数: {self.num_segments}"
        )

    def _process_core(
        self, path: List[NodeType], is_safe_fn: CollisionChecker
    ) -> List[NodeType]:
        """
        [核心实现] 执行贝塞尔平滑逻辑。

        Args:
            path: 原始路径点列表。
            is_safe_fn: 碰撞检测函数 (输入坐标，返回是否安全)。

        Returns:
            List[NodeType]: 平滑后的路径点列表。
        """
        # 1. 基础检查：如果路径点少于 3 个，构不成拐角，无法平滑，直接返回
        if not path or len(path) < 3:
            return path

        # 初始化结果列表，先把起点放进去
        smoothed_path = []
        smoothed_path.append(path[0])

        input_nodes_count = len(path)
        curved_corners = 0  # 统计成功平滑了多少个拐角

        # 2. 遍历路径中间的每一个点 (作为拐角 P1)
        # 也就是从第 2 个点遍历到倒数第 2 个点
        for i in range(1, len(path) - 1):
            p0 = path[i - 1]  # 前一个点 (Pre)
            p1 = path[i]  # 当前拐角点 (Cur / Control Point)
            p2 = path[i + 1]  # 后一个点 (Next)

            current_smoothness = self.default_smoothness
            best_segment = None

            # 3. 自适应回退机制 (Adaptive Fallback)
            # 就像开车过弯，如果发现弯太急会撞墙，就试着转小一点的弯，直到能过去为止。
            # 尝试 5 次，每次把平滑度减半。
            for attempt in range(5):
                # 碰撞模拟: 检查按照当前平滑度生成的曲线是否安全
                # 必须使用 float 进行精确检查，防止因取整误差导致的"穿模"
                if self._check_curve_safety(p0, p1, p2, current_smoothness, is_safe_fn):
                    # 安全！生成曲线点集
                    best_segment = self._generate_curve(p0, p1, p2, current_smoothness)

                    if attempt > 0:
                        self.logger.debug(
                            f"[{self.name}] 拐角 {i} 触发自适应回退: 尝试 {attempt} 次后成功 "
                            f"(平滑度由 {self.default_smoothness} 降级为 {current_smoothness:.4f})"
                        )
                    break  # 找到可行解，跳出尝试循环
                else:
                    # 撞墙了，把平滑度减半，下次循环再试
                    current_smoothness *= 0.5

            if best_segment:
                # 如果成功生成了曲线，把曲线上的点加入路径
                # 注意：这里我们加入的是曲线点，跳过了原始的尖角点 p1
                smoothed_path.extend(best_segment)
                curved_corners += 1
            else:
                # 实在没法平滑 (怎么转都会撞)，只能保留原始的尖角 p1
                self.logger.debug(
                    f"[{self.name}] 拐角 {i} 无法平滑 (空间太狭窄)，保留硬拐角。"
                )
                smoothed_path.append(p1)

        # 4. 把终点放进去
        smoothed_path.append(path[-1])

        self.logger.debug(
            f"[{self.name}] 平滑处理完毕. 处理拐角: {curved_corners}/{input_nodes_count-2}"
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
        """
        [内部方法] 检查假设生成的曲线是否安全。

        原理:
        我们在曲线上进行高密度的采样 (Sampling)，确保曲线上的每一个点都在安全区域内。
        """
        dims = len(p0)  # 自适应 2D 或 3D

        # 计算曲线的起点 q0 和终点 q2
        # q0 是 p1 指向 p0 方向上距离 p1 一定比例的点
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        # q2 是 p1 指向 p2 方向上距离 p1 一定比例的点
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        # 估算曲线的大致长度 (两条切线长度之和 * s)
        d1 = math.sqrt(sum((p1[d] - p0[d]) ** 2 for d in range(dims)))
        d2 = math.sqrt(sum((p2[d] - p1[d]) ** 2 for d in range(dims)))
        total_len = (d1 + d2) * s

        # 采样策略：每 0.2 米检查一个点，确保不会漏掉障碍物
        # 比如曲线长 5 米，我们就检查 25 个点
        steps = int(total_len * 5)
        steps = max(steps, 5)  # 至少检查 5 个点

        for k in range(steps + 1):
            t = k / steps  # t 从 0.0 变到 1.0

            # 贝塞尔公式计算采样点坐标
            check_point_list = []
            for d in range(dims):
                # 二阶贝塞尔公式: B(t) = (1-t)^2 * q0 + 2t(1-t) * p1 + t^2 * q2
                # 这里的起点是 q0, 控制点是 p1, 终点是 q2
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]

                # 不要取整, 保留 float 传给碰撞检测器。
                # 如果这里强制 int()，会导致 1.9m 变成 1m，导致碰撞误判或漏判。
                check_point_list.append(val)

            # 调用外部传入的 "安检员" 进行检查
            check_point = tuple(check_point_list)
            if not is_safe_fn(check_point):
                return False  # 只要有一个点不安全，整个曲线就废弃

        return True

    def _generate_curve(
        self, p0: NodeType, p1: NodeType, p2: NodeType, s: float
    ) -> List[tuple]:
        """
        [内部方法] 实际生成曲线点集。

        这一步只负责数学计算，不再进行安全检查 (因为前面已经检查过了)。
        """
        dims = len(p0)

        # 1. 确定曲线的起止点 (Anchor Points)
        # 这是一个线性插值公式: P_new = P_start + (P_end - P_start) * ratio
        q0 = tuple(p1[d] + (p0[d] - p1[d]) * s for d in range(dims))
        q2 = tuple(p1[d] + (p2[d] - p1[d]) * s for d in range(dims))

        points = []
        # 2. 生成中间的插值点
        # 注意: i 从 1 开始，到 num_segments 结束。
        # 我们通常希望包含 q2 (终点)，但不包含 q0 (起点)，因为 q0 会由上一段路径的终点（或直线）衔接。
        for i in range(1, self.num_segments + 1):
            t = i / self.num_segments

            pt_list = []
            for d in range(dims):
                # 二阶贝塞尔核心公式
                val = (1 - t) ** 2 * q0[d] + 2 * t * (1 - t) * p1[d] + t**2 * q2[d]
                pt_list.append(val)

            points.append(tuple(pt_list))

        return points

    def _cast_to_input_type(
        self, float_path: List[tuple], sample: NodeType
    ) -> List[NodeType]:
        """
        保留类型转换接口 (目前暂未深度使用，直接返回 float 坐标即可)。
        后续如果需要强制转回 int 网格坐标可在此实现。
        """
        return float_path
