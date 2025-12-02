import heapq
import math
import logging
from typing import List, Tuple, Dict, Optional, Set, Any

from .base import PathPlannerBase, NodeType
from smart_crane.core.rust_bridge import RustBackend
from smart_crane.core.constants import (
    SQRT_2,
    SQRT_3,
    EPSILON,
    INF,
    A_STAR_DEFAULT_RESOLUTION,
    A_STAR_MOVE_COST_CARDINAL,
    A_STAR_INIT_F_SCORE,
    A_STAR_RECONSTRUCT_SAFE_MARGIN,
    DSLITE_DEFAULT_MAX_NODES,
    DSLITE_MAX_NODES_MULTIPLIER,
    DSLITE_DEFAULT_HEURISTIC_MIN_WEIGHT,
)


class AStarPlanner(PathPlannerBase[NodeType]):
    """A* (A-Star) 静态路径规划器。

    支持 2D 和 3D 网格。
    核心逻辑支持 Python 原生实现和 Rust 加速实现双引擎切换。
    """

    def __init__(
        self,
        grid: Any,
        width_m: float,
        length_m: float,
        height_m: float = 0.0,
        resolution: float = A_STAR_DEFAULT_RESOLUTION,
        logger: Optional[logging.Logger] = None,
        grid_lock=None,
        use_octile_3d: bool = False,
        heuristic_weight: float = 1.0,
        enable_rust: bool = True,
    ):
        """初始化 A* 规划器。

        Args:
            grid: 网格数据。
            width_m, length_m, height_m: 地图物理尺寸。
            resolution: 网格分辨率。
            logger: 父级日志记录器。
            grid_lock: 线程锁。
            use_octile_3d: 是否在 3D 环境下使用 Octile 距离（否则使用欧氏距离）。
            heuristic_weight: 启发式函数的权重 (W >= 1.0)。W > 1 为 WA*。
            enable_rust: 是否尝试启用 Rust 核心加速。
        """
        # 调用基类初始化，基类会自动处理 logger 的嵌套命名 (如 Parent.AStarPlanner)
        super().__init__(
            grid, width_m, length_m, height_m, resolution, logger, grid_lock
        )

        self.rust_planner = None
        self.enable_rust = enable_rust
        self.use_octile_3d = use_octile_3d
        # 最小启发式权重由常量控制，防止权重小于 1
        self.heuristic_weight = max(
            DSLITE_DEFAULT_HEURISTIC_MIN_WEIGHT, heuristic_weight
        )

        self.start_node: Optional[NodeType] = None
        self.goal_node: Optional[NodeType] = None

        # 移动代价常量
        self.COST_1 = A_STAR_MOVE_COST_CARDINAL
        self.COST_2 = SQRT_2
        self.COST_3 = SQRT_3

        # 计算最大扩展节点数，作为熔断机制防止死循环
        layers_count = self.layers if self.layers > 0 else 1
        total_grid_size = self.rows * self.cols * layers_count
        # 使用集中常量来计算最大扩展节点数（作为熔断保护）
        self.max_nodes_expanded = max(
            DSLITE_DEFAULT_MAX_NODES, total_grid_size * DSLITE_MAX_NODES_MULTIPLIER
        )

        # 尝试加载 Rust 核心
        if RustBackend.is_available() and self.enable_rust:
            RustClass = RustBackend.get_astar_class()
            if RustClass:
                try:
                    self.rust_planner = RustClass(
                        self.grid,
                        self.width_m,
                        self.length_m,
                        self.height_m,
                        self.resolution,
                        self.use_octile_3d,
                        self.heuristic_weight,
                    )
                    self.logger.info("Rust 高性能核心加载成功!")
                except Exception as e:
                    self.logger.error(f"Rust 实例化失败: {e}", exc_info=True)
                    self.rust_planner = None
        else:
            mode_msg = "用户禁用 Rust" if not self.enable_rust else "未检测到 Rust 扩展"
            self.logger.info(f"{mode_msg}，使用 Python 原生模式。")

    @property
    def grid(self):
        return self._grid

    @grid.setter
    def grid(self, value):
        """设置网格数据，并同步更新 Rust 核心（如果存在）。"""
        self._grid = value
        if self.rust_planner:
            try:
                self.rust_planner.update_grid(value)
            except Exception as e:
                self.logger.error(f"网格同步至 Rust 失败: {e}")

    @property
    def supports_incremental(self) -> bool:
        """A* 是静态算法，不支持增量更新。"""
        return False

    def initialize(self, start: NodeType, goal: NodeType) -> bool:
        """初始化起终点。

        Args:
            start: 起点坐标。
            goal: 终点坐标。

        Returns:
            bool: 是否初始化成功。
        """
        if start is None or goal is None:
            self.logger.error("初始化失败: 起点或终点为空。")
            return False

        if not self.is_valid(start):
            self.logger.error(f"起点 {start} 超出网格边界。")
            return False
        if not self.is_valid(goal):
            self.logger.error(f"终点 {goal} 超出网格边界。")
            return False

        # 检查障碍物情况
        if self.is_obstacle(start):
            self.logger.warning(f"起点 {start} 位于障碍物内 (可能需要脱困)。")
        if self.is_obstacle(goal):
            self.logger.error(f"终点 {goal} 位于障碍物内，不可达。")
            return False

        self.start_node = start
        self.goal_node = goal

        # 如果使用 Rust，也需要同步初始化 Rust 状态
        if self.rust_planner:
            s_tuple = (
                int(start[0]),
                int(start[1]),
                int(start[2]) if len(start) > 2 else 0,
            )
            e_tuple = (int(goal[0]), int(goal[1]), int(goal[2]) if len(goal) > 2 else 0)
            if not self.rust_planner.initialize(s_tuple, e_tuple):
                self.logger.error("Rust 核心初始化拒绝 (端点无效)。")
                return False

        return True

    def update_obstacles(self, changes: List[Tuple[int, ...]]):
        """A* 不支持增量更新，此方法仅记录日志。"""
        if changes:
            self.logger.debug(
                f"收到 {len(changes)} 处网格更新，但 A* 不支持增量计算，将在下次 Plan 时全量重算。"
            )

    def _compute_path_core(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """执行路径搜索的主入口。"""
        goal_node = self.goal_node
        if goal_node is None:
            self.logger.warning("未设置终点，无法规划。")
            return None

        # 1. 优先尝试 Rust 模式
        if self.enable_rust and self.rust_planner:
            s_tuple = (
                int(current_pos[0]),
                int(current_pos[1]),
                int(current_pos[2]) if len(current_pos) > 2 else 0,
            )
            e_tuple = (
                int(goal_node[0]),
                int(goal_node[1]),
                int(goal_node[2]) if len(goal_node) > 2 else 0,
            )

            try:
                # Rust 返回 (path, nodes_expanded)
                result_tuple = self.rust_planner.compute_path(s_tuple, e_tuple)
                rust_path, nodes_expanded = result_tuple
                self.stats["nodes_expanded"] = nodes_expanded

                if rust_path:
                    # 转换回元组列表 (Rust 返回的是对象列表)
                    # 注意：rust_path 里的元素已经是 tuple
                    return rust_path
                else:
                    self.logger.info("Rust 核心未能找到路径。")
                    return None
            except Exception as e:
                self.logger.error(
                    f"Rust 核心崩溃，回退到 Python 模式: {e}", exc_info=True
                )
                # Fallback to Python

        # 2. 回退到 Python 模式
        return self._compute_path_python(current_pos)

    def _compute_path_python(self, current_pos: NodeType) -> Optional[List[NodeType]]:
        """Python 原生 A* 实现。"""
        start_node = current_pos
        goal_node = self.goal_node

        # OpenSet: 优先队列 (f_score, node)
        open_set: List[Tuple[float, NodeType]] = []
        heapq.heappush(open_set, (A_STAR_INIT_F_SCORE, start_node))

        # 记录路径来源
        came_from: Dict[NodeType, NodeType] = {}

        # g_score: 从起点到当前的代价
        g_score: Dict[NodeType, float] = {start_node: 0.0}

        # 辅助集合用于快速查找 OpenSet 成员
        open_set_hash: Set[NodeType] = {start_node}

        MAX_NODES = self.max_nodes_expanded
        nodes_expanded = 0

        while open_set:
            # 取出 f 值最小的节点
            current_f, current = heapq.heappop(open_set)

            # Lazy Removal 处理
            if current in open_set_hash:
                open_set_hash.remove(current)
            else:
                continue

            nodes_expanded += 1
            if current == goal_node:
                self.stats["nodes_expanded"] = nodes_expanded
                return self._reconstruct_path(came_from, current)

            # 熔断机制
            if nodes_expanded > MAX_NODES:
                self.logger.warning(
                    f"搜索超时: 已扩展 {nodes_expanded} 节点，强制停止。"
                )
                return None

            # 遍历邻居
            for neighbor, move_cost in self._get_neighbors(current):
                tentative_g = g_score[current] + move_cost

                # 如果找到了更优路径
                if tentative_g < g_score.get(neighbor, INF) - EPSILON:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_node)

                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f, neighbor))
                        open_set_hash.add(neighbor)

        self.stats["nodes_expanded"] = nodes_expanded
        self.logger.info("OpenSet 耗尽，无路径。")
        return None

    def _heuristic(self, a: NodeType, b: NodeType) -> float:
        """计算启发式代价 H(n)。"""
        dims = len(a)
        h = 0.0
        if dims == 2:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            # 对角线距离 (Octile)
            h = (dx + dy) + (self.COST_2 - 2) * min(dx, dy)
        else:
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            dz = abs(a[2] - b[2])
            if self.use_octile_3d:
                # 3D Octile
                delta = sorted([dx, dy, dz])
                h = (
                    delta[0] * self.COST_3
                    + (delta[1] - delta[0]) * self.COST_2
                    + (delta[2] - delta[1]) * self.COST_1
                )
            else:
                # 欧几里得距离
                h = math.sqrt(dx * dx + dy * dy + dz * dz)

        return h * self.heuristic_weight

    def _get_neighbors(self, node: NodeType) -> List[Tuple[NodeType, float]]:
        """获取当前节点的合法邻居及其移动代价。"""
        neighbors = []
        dims = len(node)

        if dims == 2:
            r, c = int(node[0]), int(node[1])
            # 8-连通
            moves = [
                (0, 1, self.COST_1),
                (0, -1, self.COST_1),
                (1, 0, self.COST_1),
                (-1, 0, self.COST_1),
                (1, 1, self.COST_2),
                (1, -1, self.COST_2),
                (-1, 1, self.COST_2),
                (-1, -1, self.COST_2),
            ]
            for dr, dc, cost in moves:
                nr, nc = r + dr, c + dc
                if not self.is_safe((nr, nc)):
                    continue
                # 对角线移动时的穿墙检查 (防止穿过两个角对角的障碍物)
                if dr != 0 and dc != 0:
                    if self.is_obstacle((r + dr, c)) or self.is_obstacle((r, c + dc)):
                        continue
                neighbors.append(((nr, nc), cost))
        else:
            x, y, z = int(node[0]), int(node[1]), int(node[2])
            # 26-连通
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        nx, ny, nz = x + dx, y + dy, z + dz
                        if not self.is_safe((nx, ny, nz)):
                            continue
                        # 简单的对角线检查 (3D 完整检查比较复杂，这里做简化检查)
                        if dx != 0 and dy != 0:
                            if self.is_obstacle((x + dx, y, z)) or self.is_obstacle(
                                (x, y + dy, z)
                            ):
                                continue

                        dist_sq = dx * dx + dy * dy + dz * dz
                        cost = (
                            self.COST_1
                            if dist_sq == 1
                            else (self.COST_2 if dist_sq == 2 else self.COST_3)
                        )
                        neighbors.append(((nx, ny, nz), cost))
        return neighbors

    def _reconstruct_path(
        self, came_from: Dict[NodeType, NodeType], current: NodeType
    ) -> List[NodeType]:
        """从最终节点回溯重建路径。"""
        path = [current]
        # 防止死循环的安全上限
        max_steps = len(came_from) + A_STAR_RECONSTRUCT_SAFE_MARGIN
        steps = 0
        while current in came_from:
            current = came_from[current]
            path.append(current)
            steps += 1
            if steps > max_steps:
                self.logger.error("路径回溯出现死循环，强制中断。")
                break
        return path[::-1]
