import math
import logging
from typing import List, Tuple, Callable, Optional

# 获取模块级日志记录器
logger = logging.getLogger(__name__)


class GreedyShortcutOptimizer:
    """
    【阶段二：路径简化器】
    贪婪捷径算法 (Greedy Shortcut / Line-of-Sight)

    === 算法原理 ===
    该算法通过“视线检查” (Line-of-Sight) 来消除路径中不必要的中间节点。
    它模仿人类走路时的直觉：如果你能直接看到远处的路标（且中间没有障碍物），
    你就不会沿着弯弯曲曲的网格线走，而是直接走直线过去。

    === 逻辑流程 ===
    1. 从路径起点 A 开始。
    2. 尽可能向后看路径上的节点（从终点 Z 开始往前倒推：Z, Y, X...）。
    3. 检查 A 到目标节点（如 X）之间是否有障碍物。
    4. 如果没有障碍物（视线通透），则直接连接 A->X，抛弃中间的 B, C...
    5. 将当前位置更新为 X，重复上述过程，直到到达终点。
    """

    def optimize(
        self, path: List[Tuple[int, int]], grid: List[List[int]]
    ) -> List[Tuple[int, int]]:
        """
        执行贪婪路径优化。

        参数:
            path: 原始路径列表，格式为 [(row, col), ...]
            grid: 2D 网格地图，1 表示障碍物，0 表示通行区域

        返回:
            优化后的稀疏路径列表
        """
        # 基础检查：路径过短无需优化
        if not path or len(path) < 3:
            return path

        logger.info(f"启动路径简化: 原始路径节点数 {len(path)}")

        optimized_path = [path[0]]
        current_idx = 0
        total_nodes_original = len(path)

        # 主循环：直到处理完倒数第二个节点
        while current_idx < len(path) - 1:
            check_idx = len(path) - 1
            found_shortcut = False

            # 内循环：贪婪地从最远端开始尝试连接
            while check_idx > current_idx + 1:
                target_node = path[check_idx]

                # 执行视线检查 (Ray Casting)
                if self._has_line_of_sight(path[current_idx], target_node, grid):
                    # 发现捷径！直接连接 Current -> Target
                    optimized_path.append(target_node)

                    # 记录日志（仅在跳跃步数较大时）
                    skipped_nodes = check_idx - current_idx - 1
                    if skipped_nodes > 5:
                        logger.debug(
                            f"发现捷径: 节点 {current_idx} -> {check_idx}, 跳过 {skipped_nodes} 个中间点"
                        )

                    current_idx = check_idx
                    found_shortcut = True
                    break

                check_idx -= 1

            # 如果没有找到任何捷径，只能老老实实走一步到下一个节点
            if not found_shortcut:
                current_idx += 1
                if current_idx < len(path):
                    optimized_path.append(path[current_idx])

        logger.info(
            f"路径简化完成: {total_nodes_original} -> {len(optimized_path)} 节点"
        )
        return optimized_path

    def _has_line_of_sight(
        self, start: Tuple[int, int], end: Tuple[int, int], grid: List[List[int]]
    ) -> bool:
        """
        检查两点之间是否存在障碍物。

        === 实现细节：高密度采样 (High-Density Sampling) ===
        为了防止“穿模”（即直线穿过障碍物的边角但未检测到），
        这里不使用标准的 Bresenham 算法，而是使用浮点数高密度线性插值。
        采样步长设为 0.1 格子，确保即使是障碍物的边缘也能被检测到。
        """
        # 取起点重点坐标
        r0, c0 = start
        r1, c1 = end
        rows = len(grid)
        cols = len(grid[0])

        dx = c1 - c0
        dy = r1 - r0
        # 求起点终点距离, x^2+y^2开平方
        distance = math.hypot(dx, dy)

        if distance == 0:
            return True

        # 采样密度：每 1.0 距离单位（一个格子）采样 10 次
        # 这是一个非常激进的安全策略，确保绝对不会穿越障碍物
        steps = int(distance * 10)
        if steps == 0:
            steps = 1

        for i in range(1, steps):
            t = i / steps
            # 线性插值公式: P(t) = Start + (End - Start) * t
            curr_r = r0 + dy * t
            curr_c = c0 + dx * t

            # 四舍五入取整，确定当前采样点落在哪个格子里
            ir = int(round(curr_r))
            ic = int(round(curr_c))

            # 越界检查
            if not (0 <= ir < rows and 0 <= ic < cols):
                return False

            # 碰撞检查 (1 表示障碍物)
            if grid[ir][ic] == 1:
                return False

        return True


class BezierCurveSmoother:
    """
    【阶段三：路径平滑器】
    高精度自适应贝塞尔平滑 (High-Precision Adaptive Bezier)

    === 算法原理：二阶贝塞尔曲线 (Quadratic Bezier Curve) ===
    对于路径中的折线段 P0 -> P1 -> P2，其中 P1 是拐角点。
    我们使用二阶贝塞尔公式生成一条平滑曲线来切过这个角：
    B(t) = (1-t)^2 * Q0 + 2t(1-t) * P1 + t^2 * Q2
    其中 Q0 和 Q2 是 P1 附近的控制点。

    === 核心特性 ===
    1. **分离检测与输出**：
       - 碰撞检测使用【高密度采样】，确保曲线不会稍微蹭到障碍物。
       - 最终输出使用【固定段数】，保持路径点数量可控，方便后端传输给前端或 PLC。
    2. **智能回退 (Adaptive Fallback)**：
       - 如果默认的大半径曲线会碰到障碍物，算法会自动缩小半径（衰减 smoothness）。
       - 它会尝试更紧贴 P1 点的小转弯。
       - 如果缩到最小还不行，则放弃平滑，保持尖角（确保安全第一）。
    """

    def __init__(self, smoothness: float = 0.3, num_segments: int = 10):
        """
        初始化平滑器。

        参数:
            smoothness: 平滑因子 (0.0 ~ 0.5)。
                        0.5 表示从线段中点开始起圆弧（圆弧最大）。
                        0.1 表示只在拐角处很小一段距离做圆弧。
            num_segments: 每一段贝塞尔曲线生成的插值点数量。
        """
        # 限制平滑度在合理范围内
        self.default_smoothness = max(0.0, min(0.5, smoothness))
        self.num_segments = num_segments
        logger.info(
            f"贝塞尔平滑器就绪: Smoothness={self.default_smoothness}, Segments={self.num_segments}"
        )

    def smooth(
        self,
        path: List[Tuple[float, float]],
        collision_check_fn: Optional[Callable[[float, float], bool]] = None,
    ) -> List[Tuple[float, float]]:
        """
        执行路径平滑处理。

        参数:
            path: 2D 物理坐标路径 [(x, y), ...]
            collision_check_fn: 碰撞检测回调函数 func(x, y) -> is_safe_bool

        返回:
            平滑后的路径列表
        """
        if not path or len(path) < 3:
            return path

        logger.info("开始贝塞尔平滑处理...")
        smoothed_path = []
        smoothed_path.append(path[0])  # 保留起点

        # 遍历路径中的每一个拐角 (P1)
        # P0 ----- P1
        #          |
        #          |
        #          P2
        for i in range(1, len(path) - 1):
            p0 = path[i - 1]  # 前一点
            p1 = path[i]  # 拐点 (控制点)
            p2 = path[i + 1]  # 后一点

            # === 自适应逻辑 ===
            current_smoothness = self.default_smoothness
            best_segment = None

            # 尝试循环：从最大圆弧开始尝试，如果碰撞则缩小半径
            # 衰减序列示例: 0.3 -> 0.15 -> 0.075 -> ...
            for attempt in range(5):
                # 1. 高密度安全检查 (Simulate)
                # 这里并不生成最终点，而是生成大量临时点去探测碰撞
                if self._check_curve_safety_high_res(
                    p0, p1, p2, current_smoothness, collision_check_fn
                ):
                    # 2. 如果安全，生成最终的稀疏输出点
                    best_segment = self._generate_curve_segment(
                        p0, p1, p2, current_smoothness, self.num_segments
                    )
                    if attempt > 0:
                        logger.debug(
                            f"拐点 {i} 自适应回退: 尝试 {attempt} 次后成功, 平滑度降为 {current_smoothness:.4f}"
                        )
                    break
                else:
                    # 不安全，缩小圆弧半径（更贴近拐点 P1）重试
                    current_smoothness *= 0.5

            if best_segment:
                smoothed_path.extend(best_segment)
            else:
                # 如果缩到很小都不行（比如死胡同里的直角转弯），说明物理空间不够
                # 只能走尖角，确保不撞墙
                logger.debug(f"拐点 {i} 平滑失败: 障碍物过于贴近，保持尖角")
                smoothed_path.append(p1)

        smoothed_path.append(path[-1])  # 保留终点
        return smoothed_path

    def _check_curve_safety_high_res(
        self,
        p0: Tuple[float, float],
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        s: float,
        check_fn: Optional[Callable[[float, float], bool]],
    ) -> bool:
        """
        使用高密度采样检查假想的曲线是否安全。
        这完全是为了碰撞检测，不影响最终输出的点的密度。
        """
        if not check_fn:
            return True

        # 1. 估算曲线的大致物理长度
        # 两个向量的长度
        d1 = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        d2 = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        # 根据贝塞尔曲线的几何性质（凸包性质），
        # 二阶贝塞尔曲线一定被包裹在它的控制多边形（三角形）内部。
        # 两点之间线段最短。曲线的长度 一定小于 两条控制线段之和。
        # 贝塞尔曲线的实际长度肯定小于两腰之和 (d1 + d2) * s * 2 是一个非常宽裕的上限
        total_len = (d1 + d2) * s * 2

        # 2. 决定采样密度
        # 策略：每 0.1 米至少检查一个点
        # 这样可以保证哪怕是网格角落的一个微小障碍物也能被探测到
        check_steps = int(total_len * 10)
        check_steps = max(check_steps, 10)  # 兜底，至少检查10个点

        # 3. 计算控制点 (Control Points)
        # Q0: 在 P0 -> P1 线段上，距离 P1 比例为 s 的点
        # Q2: 在 P1 -> P2 线段上，距离 P1 比例为 s 的点
        v1_x, v1_y = p1[0] - p0[0], p1[1] - p0[1]
        v2_x, v2_y = p2[0] - p1[0], p2[1] - p1[1]
        q0 = (p1[0] - v1_x * s, p1[1] - v1_y * s)
        q2 = (p1[0] + v2_x * s, p1[1] + v2_y * s)

        # 4. 采样循环
        for k in range(check_steps + 1):
            t = k / check_steps

            # 贝塞尔公式展开 (优化版)
            # B(t) = (1-t)^2 * Q0 + 2t(1-t) * P1 + t^2 * Q2
            term1 = (1 - t) ** 2
            term2 = 2 * (1 - t) * t
            term3 = t**2

            bx = term1 * q0[0] + term2 * p1[0] + term3 * q2[0]
            by = term1 * q0[1] + term2 * p1[1] + term3 * q2[1]

            # 调用外部传入的物理碰撞检测函数
            if not check_fn(bx, by):
                return False  # 检测到碰撞，该平滑度不可用

        return True

    def _generate_curve_segment(
        self,
        p0: Tuple[float, float],
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        s: float,
        segments: int,
    ) -> List[Tuple[float, float]]:
        """
        生成最终用于输出的曲线点。
        这里使用较低的密度 (segments)，以减少数据量。
        """
        # 计算向量
        v1_x, v1_y = p1[0] - p0[0], p1[1] - p0[1]
        v2_x, v2_y = p2[0] - p1[0], p2[1] - p1[1]

        # 计算起止插值点
        q0 = (p1[0] - v1_x * s, p1[1] - v1_y * s)
        q2 = (p1[0] + v2_x * s, p1[1] + v2_y * s)

        points = []
        for j in range(segments + 1):
            t = j / segments

            # 贝塞尔公式
            term1 = (1 - t) ** 2
            term2 = 2 * (1 - t) * t
            term3 = t**2

            bx = term1 * q0[0] + term2 * p1[0] + term3 * q2[0]
            by = term1 * q0[1] + term2 * p1[1] + term3 * q2[1]
            points.append((bx, by))

        return points
