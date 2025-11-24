import logging
import math
import time
import threading
from typing import List, Tuple, Dict, Any, Optional, Callable, Union

# 引入核心组件
from core.map_manager import WorkshopMapManager
from algorithms.base import PathPlannerBase
from algorithms.astar import AStarPlanner
from algorithms.dslite import DLitePlanner
from algorithms.post_processing.base import PathPostProcessor
from algorithms.post_processing.greedy import GreedyShortcutProcessor
from algorithms.post_processing.bezier import BezierSmoothProcessor

# 定义类型别名，方便阅读
# 3D 坐标: (x, y, z) 浮点数
Point3D = Tuple[float, float, float]
# 网格节点: 可能是 (row, col) 或 (row, col, layer)
GridNode = Union[Tuple[int, int], Tuple[int, int, int]]


class TrajectoryPlanner:
    """
    【核心轨迹规划器 (Trajectory Planner)】

    角色定位:
    它是整个路径规划系统的"总指挥" (Director)。
    它负责协调 MapManager(地图)、PathPlanner(算法) 和 PostProcessors(优化器) 协同工作。

    主要职责:
    1. **环境抽象**: 根据吊具尺寸和安全策略，将物理地图转化为"膨胀"后的配置空间网格 (C-Space)。
    2. **策略路由**: 决定是跑 2D 算法(定高巡航)还是 3D 算法。
    3. **安全卫士**: 检查起终点合法性，执行"智能脱困" (Smart Escape)。
    4. **流程编排**:
       [起点校验] -> [坐标转换] -> [核心寻路] -> [路径拼接] -> [截弯取直] -> [平滑处理] -> [数据统计]
    """

    def __init__(
        self,
        map_mgr: WorkshopMapManager,
        config: Dict[str, Any],
        logger: Optional[logging.Logger] = None,
        grid_lock: Optional[threading.RLock] = None,
    ):
        """
        初始化规划指挥官。

        Args:
            map_mgr: 地图管理器，提供原始障碍物数据。
            config: 系统配置字典 (包含吊具尺寸、算法选择等)。
            logger: 日志记录器。
            grid_lock: 共享的线程锁，确保规划时地图不会被修改。
        """
        self.map_mgr = map_mgr
        self.config = config.copy()  # 复制一份配置，防止外部意外修改
        self.logger = logger or logging.getLogger("TrajectoryPlanner")
        self.grid_lock = grid_lock or threading.RLock()

        # 核心规划器实例 (A* 或 D* Lite)
        self.core_planner: Optional[PathPlannerBase] = None

        # 后处理流水线 (贪婪优化、贝塞尔平滑)
        self.post_processors: List[PathPostProcessor] = []

        # 当前用于规划的网格数据 (可能包含膨胀层)
        self.active_planning_grid = None

        # 用于前端展示的网格 (通常是 2D 投影)
        self.visualization_grid = None

        # 初始化内部组件
        self._initialize_planner()
        self.logger.info("[TrajPlanner] 轨迹规划器启动就绪。")

    def _initialize_planner(self, force_rebuild: bool = False):
        """
        [内部方法] 初始化或重建核心规划器。
        当配置发生变更(如算法切换、安全距离修改)时，需要调用此方法。
        """
        with self.grid_lock:
            # 1. 读取关键配置
            algo_type = self.config.get("PLANNER_ALGORITHM", "astar")  # 算法类型
            use_octile = self.config.get("USE_3D_OCTILE", False)  # 距离计算方式
            h_weight = self.config.get("HEURISTIC_WEIGHT", 1.5)  # 启发式权重

            # 2. 准备网格数据 (这是最耗时的一步，涉及到地图膨胀)
            # plan_grid: 给算法跑路用的 (如果是 2D 模式，这就是个 2D 数组)
            # vis_grid: 给前端画图用的
            # grid_height_m: 此时规划空间的逻辑高度
            plan_grid, vis_grid, grid_height_m = self._prepare_grids()

            self.active_planning_grid = plan_grid
            self.visualization_grid = vis_grid

            # 3. 组装参数
            common_args = {
                "grid": plan_grid,
                "width_m": self.map_mgr.width_m,
                "length_m": self.map_mgr.length_m,
                "height_m": grid_height_m,
                "resolution": self.map_mgr.resolution_m,
                "logger": self.logger,
                "grid_lock": self.grid_lock,
                "use_octile_3d": use_octile,
                "heuristic_weight": h_weight,
            }

            # 4. 实例化核心算法
            if algo_type == "dslite":
                self.core_planner = DLitePlanner(**common_args)
            else:
                self.core_planner = AStarPlanner(**common_args)

            # 5. 组装后处理流水线
            self.post_processors = []

            # 5.1 捷径优化 (Greedy Shortcut) - 把折线拉直
            if self.config.get("ENABLE_SHORTCUT_OPTIMIZATION", True):
                self.post_processors.append(GreedyShortcutProcessor())

            # 5.2 贝塞尔平滑 (Bezier Smooth) - 把直角磨圆
            if self.config.get("ENABLE_BEZIER_SMOOTHING", True):
                self.post_processors.append(
                    BezierSmoothProcessor(
                        smoothness=float(self.config.get("BEZIER_SMOOTHNESS", 0.3)),
                        segments=int(self.config.get("BEZIER_SEGMENTS", 10)),
                    )
                )

            self.logger.info(
                f"[TrajPlanner] 核心重构完成: 算法={algo_type.upper()}, "
                f"无限高模式={self.config.get('OBSTACLE_INFINITE_HEIGHT')}, "
                f"优化器数量={len(self.post_processors)}"
            )

    def _prepare_grids(self) -> Tuple[Any, Any, float]:
        """
        [关键逻辑] 准备网格数据。

        这里体现了"业务逻辑"到"算法逻辑"的转换。
        起重机不是一个点，它有体积（吊具尺寸）。
        不能直接在原始地图上跑 A*，因为那样算出来的路径会让吊具边缘撞墙。

        解决方案：配置空间膨胀 (C-Space Inflation)。
        把障碍物向外胖一圈（胖的程度 = 吊具半径 + 安全距离）。
        这样我们可以把起重机简化为一个"质点"来跑算法。
        """
        cfg = self.config

        # 1. 计算膨胀半径 (xy_margin)
        shape = cfg.get("CRANE_FOOTPRINT_SHAPE", "box")
        w, l = cfg.get("CRANE_FOOTPRINT_WIDTH", 5.0), cfg.get(
            "CRANE_FOOTPRINT_LENGTH", 5.0
        )
        # 如果是圆形，半径就是宽度的一半；如果是矩形，取外接圆半径确保旋转安全
        radius_m = (w / 2.0) if shape == "circle" else (math.hypot(w, l) / 2.0)
        # 转换为网格格数
        xy_margin = radius_m / self.map_mgr.resolution_m

        # 2. 计算垂直安全余量 (z_margin)
        user_z_margin = cfg.get("CRANE_Z_SAFETY_MARGIN", 0.5)
        crane_h = cfg.get("CRANE_FOOTPRINT_HEIGHT", 2.0)
        # 障碍物膨胀高度 = 用户设定的安全距离 + 吊具本身的一半高度(中心点规划)
        z_margin_obs = user_z_margin + (crane_h / 2.0)

        is_fixed_height = cfg.get("ENABLE_FIXED_HEIGHT_CRUISE", True)
        is_infinite_obs = cfg.get("OBSTACLE_INFINITE_HEIGHT", True)

        # 3. 分情况生成网格
        if is_fixed_height:
            # --- 方案 A: 2.5D 定高巡航 ---
            # 只关心在"巡航高度"这一层，哪些地方有障碍物。
            cruise_z = cfg.get("CRANE_SAFE_TRAVEL_Z_M", 10.0)

            # 如果是无限高模式，无需检查高度，直接投影所有障碍物
            check_z = None if is_infinite_obs else cruise_z

            # 获取 2D 投影网格
            grid_2d = self.map_mgr.get_2d_projection_grid(
                xy_margin=xy_margin, check_z=check_z, z_margin=z_margin_obs
            )
            # 对于 2D 算法，grid_height_m 传 0.0 即可
            return grid_2d, grid_2d, 0.0
        else:
            # --- 方案 B: 全 3D 自由规划 ---
            # 生成一个体素网格 (Voxel Grid)
            z_margin_ceil = crane_h / 2.0
            grid_3d = self.map_mgr.get_3d_voxel_grid(
                xy_margin=xy_margin,
                z_margin_obs=z_margin_obs,
                z_margin_ceil=z_margin_ceil,
                is_infinite=is_infinite_obs,
            )
            # 同时也生成一个 2D 投影用于前端显示 (active_inflated_grid)
            grid_vis = self.map_mgr.get_2d_projection_grid(
                xy_margin=xy_margin, check_z=None, z_margin=z_margin_obs
            )
            return grid_3d, grid_vis, self.map_mgr.height_m

    def update_configuration(self, new_config: Dict[str, Any]) -> bool:
        """
        动态更新配置。

        Returns:
            bool: 如果配置变化导致需要重建规划器，返回 True。
        """
        need_rebuild = False

        # 影响网格生成的参数 (需要重算膨胀)
        keys_affecting_grid = [
            "CRANE_FOOTPRINT",
            "CRANE_Z_SAFETY",
            "ENABLE_FIXED_HEIGHT",
            "CRANE_SAFE_TRAVEL",
            "OBSTACLE_INFINITE",
            "MAP_RESOLUTION",
        ]
        # 影响算法行为的参数 (需要重置算法实例)
        keys_affecting_algo = [
            "PLANNER_ALGORITHM",
            "USE_3D_OCTILE",
            "HEURISTIC_WEIGHT",
            "ENABLE_SHORTCUT",
            "ENABLE_BEZIER",
            "BEZIER_SMOOTHNESS",
            "BEZIER_SEGMENTS",
        ]

        for k, v in new_config.items():
            if k in self.config and self.config[k] != v:
                self.config[k] = v
                if any(key in k for key in keys_affecting_grid):
                    # 如果网格生成参数变了，必须清空 MapManager 的缓存
                    self.map_mgr._invalidate_cache()
                    need_rebuild = True
                elif any(key in k for key in keys_affecting_algo):
                    need_rebuild = True

        if need_rebuild:
            self.logger.info("[TrajPlanner] 配置变更触发重建...")
            self._initialize_planner(force_rebuild=True)
            return True
        return False

    def handle_obstacle_update(self, x, y, w, h, z, is_add: bool):
        """
        处理障碍物动态增删 (为 D* Lite 设计)。

        Args:
            x, y, w, h, z: 障碍物的物理包围盒。
            is_add: True 为新增，False 为移除。
        """
        with self.grid_lock:
            # 记录旧网格 (用于对比差异)
            old_grid = self.active_planning_grid
            # 1. 无论什么算法，先强制全量刷新 Grid 数据，确保底层数据是最新的
            plan_grid, vis_grid, _ = self._prepare_grids()
            self.active_planning_grid = plan_grid
            self.visualization_grid = vis_grid

            # 将新的网格引用赋予核心规划器
            self.core_planner.grid = plan_grid

            # 2. 如果是 D* Lite，它支持增量更新 (Incremental Update)
            # 我们不需要重置它，而是告诉它"哪些格子变了"，让它自己修补路径
            if isinstance(self.core_planner, DLitePlanner):
                if self.core_planner.start_node and self.core_planner.goal_node:
                    # 计算受影响的网格区域 (Bounding Box)
                    changes = []
                    # 获取配置状态
                    # 定高巡航模式
                    is_fixed_height = self.config.get(
                        "ENABLE_FIXED_HEIGHT_CRUISE", True
                    )
                    # 障碍物无限高模式
                    is_infinite_obs = self.config.get("OBSTACLE_INFINITE_HEIGHT", True)

                    # --- 态计算膨胀影响范围 ---
                    # 1. 获取吊具尺寸配置 (保持与 _prepare_grids 逻辑一致)
                    cfg = self.config
                    shape = cfg.get("CRANE_FOOTPRINT_SHAPE", "box")
                    c_w = cfg.get("CRANE_FOOTPRINT_WIDTH", 5.0)
                    c_l = cfg.get("CRANE_FOOTPRINT_LENGTH", 5.0)

                    # 2. 计算物理膨胀半径 (Radius)
                    # 如果是圆形，半径就是宽度的一半；如果是矩形，取外接圆半径确保旋转安全
                    radius_m = (
                        (c_w / 2.0)
                        if shape == "circle"
                        else (math.hypot(c_w, c_l) / 2.0)
                    )

                    # 3. 转换为网格数 (Margin Grid)
                    # 向上取整并额外+1作为安全余量，防止浮点误差导致边界格子未更新
                    margin_grid = (
                        int(math.ceil(radius_m / self.map_mgr.resolution_m)) + 1
                    )

                    self.logger.debug(
                        f"[Traj] 增量Diff计算: 物理膨胀半径={radius_m:.2f}m, 扫描外扩={margin_grid}格"
                    )

                    # 将物理坐标转为网格索引范围
                    r_s, c_s, _ = self.map_mgr.world_to_grid(x, y, 0)
                    r_e, c_e, _ = self.map_mgr.world_to_grid(x + w, y + h, 0)

                    r_start = max(0, r_s - margin_grid)
                    r_end = min(self.map_mgr.rows, r_e + 1 + margin_grid)
                    c_start = max(0, c_s - margin_grid)
                    c_end = min(self.map_mgr.cols, c_e + 1 + margin_grid)

                    # --- 分支 A: 2D 定高巡航 ---
                    if is_fixed_height:
                        should_update = True

                        # 如果不是无限高模式，必须检查障碍物是否够得着巡航高度
                        if not is_infinite_obs:
                            cruise_z = self.config.get("CRANE_SAFE_TRAVEL_Z_M", 10.0)
                            # 计算垂直安全余量 (需与 prepare_grids 保持一致)
                            crane_h = self.config.get("CRANE_FOOTPRINT_HEIGHT", 2.0)
                            z_margin_obs = self.config.get(
                                "CRANE_Z_SAFETY_MARGIN", 0.5
                            ) + (crane_h / 2.0)

                            # 判定阈值：如果障碍物高度 z 还没碰到 (cruise_z - 安全余量)，则忽略
                            z_threshold = cruise_z - z_margin_obs
                            # 注意：传入的 z 参数是障碍物的高度 (Height) 或 顶部坐标
                            if z <= z_threshold:
                                should_update = False
                                self.logger.debug(
                                    f"[Traj] 障碍物过矮 (H={z} <= Thr={z_threshold:.2f})，不影响 2D 巡航层，跳过更新。"
                                )

                        if should_update:
                            for r in range(r_start, r_end):
                                for c in range(c_start, c_end):
                                    # 仅当网格状态真正改变时才通知算法
                                    val_new = plan_grid[r][c]
                                    val_old = (
                                        0  # 默认旧状态为0 (针对首次运行或越界保护)
                                    )

                                    # 尝试获取旧值
                                    if old_grid is not None:
                                        # 边界检查，防止地图尺寸突变导致越界
                                        if 0 <= r < len(old_grid) and 0 <= c < len(
                                            old_grid[0]
                                        ):
                                            val_old = old_grid[r][c]

                                    # 只有新旧不一致时，才视为有效 Change
                                    if val_new != val_old:
                                        changes.append((r, c, val_new))

                    # --- 分支 B: 3D 自由规划 ---
                    else:
                        l_s = 0
                        l_e = self.map_mgr.layers

                        # 如果不是无限高模式，计算 Z 轴的影响范围
                        if not is_infinite_obs:
                            _, _, l_start_idx = self.map_mgr.world_to_grid(
                                0, 0, 0
                            )  # 障碍物底面通常是0
                            _, _, l_end_idx = self.map_mgr.world_to_grid(
                                0, 0, z
                            )  # 障碍物顶面是z
                            l_s = max(0, l_start_idx - margin_grid)
                            l_e = min(self.map_mgr.layers, l_end_idx + 1 + margin_grid)

                        for r in range(r_start, r_end):
                            for c in range(c_start, c_end):
                                for l in range(l_s, l_e):
                                    # 3D 模式的对比逻辑
                                    val_new = plan_grid[r][c][l]
                                    val_old = 0

                                    if old_grid is not None:
                                        if (
                                            0 <= r < len(old_grid)
                                            and 0 <= c < len(old_grid[0])
                                            and 0 <= l < len(old_grid[0][0])
                                        ):
                                            val_old = old_grid[r][c][l]

                                    if val_new != val_old:
                                        changes.append((r, c, l, val_new))

                    if changes:
                        self.logger.debug(
                            f"[Traj] D* Lite 增量更新触发: 影响 {len(changes)} 个体素/格点。"
                        )
                        # 调用 D* Lite 的专用接口，这会触发 rhs 更新和路径修复
                        self.core_planner.update_obstacles(changes)
                else:
                    self.logger.debug(
                        "[Traj] D* Lite 虽然激活但当前无任务，跳过增量更新。"
                    )

    # --- 安全校验 ---
    def _validate_endpoints(
        self, start_pt: Point3D, end_pt: Point3D
    ) -> Tuple[bool, str, bool]:
        """
        校验起点和终点的合法性。

        Returns:
            (是否合法, 错误信息, 起点是否需要脱困)
        """
        # 计算当前的安全边距
        shape = self.config.get("CRANE_FOOTPRINT_SHAPE", "box")
        w, l = self.config.get("CRANE_FOOTPRINT_WIDTH", 5.0), self.config.get(
            "CRANE_FOOTPRINT_LENGTH", 5.0
        )
        # xy_margin: 水平方向膨胀距离
        xy_margin = (w / 2.0) if shape == "circle" else (math.hypot(w, l) / 2.0)

        z_safety = self.config.get("CRANE_Z_SAFETY_MARGIN", 0.5)
        crane_h = self.config.get("CRANE_FOOTPRINT_HEIGHT", 2.0)
        # z_margin: 垂直方向膨胀距离
        z_margin = z_safety + (crane_h / 2.0)

        # 1. 检查物理碰撞 (Hard Collision) - 绝对不能撞
        # margin=0 表示检查物体本身的体积是否重叠
        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], 0, 0, ignore_z=False
        ):
            return False, "起点位于障碍物内部 (发生物理碰撞)，无法规划。", False

        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], 0, 0, ignore_z=False
        ):
            return False, "终点位于障碍物内部 (发生物理碰撞)，无法规划。", False

        # 2. 检查软限制 (Soft Collision) - 安全距离
        # 终点必须严格满足安全距离，否则无法停靠
        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            return False, "终点位于安全缓冲区(膨胀层)内，禁止停靠。", False

        # 3. 起点特殊处理 - 允许"带病上岗" (智能脱困)
        # 如果起点在膨胀层内（但没发生物理碰撞），可能是之前放货后周围加了东西。
        # 这种情况下，允许规划，但标记需要"脱困"。
        start_needs_escape = False
        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            start_needs_escape = True

        return True, "Valid", start_needs_escape

    def plan(
        self, start: Dict[str, float], end: Dict[str, float]
    ) -> Tuple[Optional[List[Point3D]], Dict[str, Any], str]:
        """
        [主入口] 执行轨迹规划任务。

        Args:
            start: 起点字典 {'x': 1, 'y': 2, 'z': 3}
            end: 终点字典

        Returns:
            (路径点列表, 统计数据字典, 状态消息)
        """
        # 初始化统计容器，用于记录各阶段耗时和元数据
        stats = {
            "timings": {},
            "grid_meta": {},
            "processors_stats": [],
            "path_meta": {},
        }

        t_total_start = time.perf_counter()
        msg_list = []  # 用于收集过程中的警告或提示

        try:
            # 加上大锁，确保规划期间地图不发生突变
            with self.grid_lock:
                is_fixed_height = self.config.get("ENABLE_FIXED_HEIGHT_CRUISE", True)

                # 解析坐标
                exact_start = (
                    float(start["x"]),
                    float(start["y"]),
                    float(start.get("z", 0.0)),
                )
                exact_end = (float(end["x"]), float(end["y"]), float(end.get("z", 0.0)))

                # --- 步骤 1: 网格准备 (Grid Prep) ---
                t_grid_start = time.perf_counter()
                # 显式刷新一次网格，确保膨胀层是最新的
                self.active_planning_grid, _, _ = self._prepare_grids()
                stats["timings"]["grid_prep_ms"] = (
                    time.perf_counter() - t_grid_start
                ) * 1000

                # 记录网格规模信息
                if is_fixed_height:
                    stats["grid_meta"] = {
                        "type": "2D Projection (Fixed Height)",
                        "dims": [self.map_mgr.rows, self.map_mgr.cols, 1],
                        "total_voxels": self.map_mgr.rows * self.map_mgr.cols,
                    }
                else:
                    stats["grid_meta"] = {
                        "type": "3D Voxel",
                        "dims": [
                            self.map_mgr.rows,
                            self.map_mgr.cols,
                            self.map_mgr.layers,
                        ],
                        "total_voxels": self.map_mgr.rows
                        * self.map_mgr.cols
                        * self.map_mgr.layers,
                    }

                # --- 步骤 2: 安全校验 (Validation) ---
                is_valid, err_msg, start_needs_escape = self._validate_endpoints(
                    exact_start, exact_end
                )
                if not is_valid:
                    self.logger.warning(f"[Plan] 校验失败: {err_msg}")
                    return None, stats, err_msg
                if start_needs_escape:
                    msg_list.append("起点自动脱困触发")
                    self.logger.info("[Plan] 起点位于膨胀层内，尝试脱困...")

                # --- 步骤 3: 坐标转换 (To Grid) ---
                # 将物理坐标转换为网格索引
                # cruise_z_level: 如果是 2D 模式，这个值记录了巡航高度的物理 Z 值
                planner_start_node, planner_end_node, cruise_z_level = (
                    self._get_initial_grid_nodes(
                        exact_start, exact_end, is_fixed_height
                    )
                )

                # --- 步骤 4: 智能脱困 (Smart Escape) ---
                start_node_final = planner_start_node

                # 如果标记了需要脱困，或者起点在当前网格（算法视角）中被视为障碍
                if start_needs_escape or not self.core_planner.is_safe(
                    planner_start_node
                ):
                    # 在起点附近搜寻一个最近的安全网格
                    escape_node = self._smart_escape(
                        planner_start_node, planner_end_node
                    )
                    if escape_node:
                        start_node_final = escape_node
                        self.logger.info(f"[Plan] 找到安全脱困点: {escape_node}")
                    else:
                        return None, stats, "起点被完全封死，无法脱困"

                # 再次确认终点是否安全
                if not self.core_planner.is_safe(planner_end_node):
                    return None, stats, "终点网格不可达 (在障碍物或膨胀层内)"
                end_node_final = planner_end_node

                # --- 步骤 5: 核心寻路 (Core Pathfinding) ---
                t_algo_start = time.perf_counter()

                # 初始化算法 (D* Lite 这里会复用之前的树)
                if not self.core_planner.initialize(start_node_final, end_node_final):
                    return None, stats, "规划器初始化失败 (可能是起终点逻辑错误)"

                # 执行搜索
                raw_grid_path = self.core_planner.compute_path(start_node_final)
                stats["timings"]["pathfinding_ms"] = (
                    time.perf_counter() - t_algo_start
                ) * 1000

                # 从算法实例中提取统计信息 (如扩展节点数)
                stats.update(self.core_planner.get_stats())

                if not raw_grid_path:
                    self.logger.warning("[Plan] 核心算法返回空路径 (无解)")
                    return None, stats, "未找到路径"

                # --- 步骤 6: 路径拼接与还原 (Splicing & Reconstruction) ---
                t_splice_start = time.perf_counter()

                # 将网格路径转回物理坐标路径
                cruise_segment_world = []
                for node in raw_grid_path:
                    cruise_segment_world.append(
                        self._grid_to_world_smart(node, cruise_z_level)
                    )

                path_to_optimize = []
                if is_fixed_height:
                    # 2D 模式下，路径实际上是:
                    # 起点(地面) -> 起点上方(巡航高度) -> ...巡航路径... -> 终点上方(巡航高度) -> 终点(地面)
                    # 我们这里只构造中间的 "巡航段" 加上两头的连接点，垂直升降由设备控制逻辑决定
                    # 但为了可视化连贯，我们在路径中显式加上起点和终点的巡航高度映射

                    # 桥接点: 起点垂直投影到巡航高度的点
                    bridge_start = (exact_start[0], exact_start[1], cruise_z_level)
                    bridge_end = (exact_end[0], exact_end[1], cruise_z_level)

                    path_to_optimize = (
                        [bridge_start] + cruise_segment_world + [bridge_end]
                    )
                else:
                    # 3D 模式下，直接连接物理起点和终点
                    path_to_optimize = (
                        [exact_start] + cruise_segment_world + [exact_end]
                    )

                # 简单的去重
                path_to_optimize = self._deduplicate_path(path_to_optimize)
                stats["timings"]["splicing_ms"] = (
                    time.perf_counter() - t_splice_start
                ) * 1000

                # --- 步骤 7: 后处理优化 (Post-Processing) ---
                # 创建一个基于连续浮点坐标的碰撞检测器，供优化器使用
                is_safe_fn = self._create_3d_collision_checker(
                    grace_start=path_to_optimize[0], grace_end=path_to_optimize[-1]
                )

                optimized_path = path_to_optimize
                # 依次经过所有配置的处理器 (如: 捷径 -> 平滑)
                for processor in self.post_processors:
                    optimized_path = processor.process(optimized_path, is_safe_fn)
                    # 收集该处理器的性能数据
                    stats["processors_stats"].append(processor.get_stats())

                # --- 步骤 8: 最终组装 ---
                final_path = []
                if is_fixed_height:
                    # 在 2D 模式下，如果实际起点高度和巡航高度不一致，需要补上垂直段的显示
                    # 比如从 Z=0 提升到 Z=10
                    if abs(exact_start[2] - optimized_path[0][2]) > 0.01:
                        final_path.append(exact_start)
                    final_path.extend(optimized_path)
                    if abs(exact_end[2] - optimized_path[-1][2]) > 0.01:
                        final_path.append(exact_end)
                else:
                    final_path = optimized_path

                # 最后一次去重和坐标保留小数位
                final_path = self._deduplicate_path(final_path)
                final_path = [tuple(round(v, 2) for v in pt) for pt in final_path]

                # 总耗时统计
                stats["timings"]["total_ms"] = (
                    time.perf_counter() - t_total_start
                ) * 1000
                stats["path_meta"]["final_nodes"] = len(final_path)

                final_msg = "Success"
                if msg_list:
                    final_msg = f"Success ({'; '.join(msg_list)})"

                self.logger.info(
                    f"[Plan] 规划完成. 总耗时: {stats['timings']['total_ms']:.1f}ms"
                )

                return final_path, stats, final_msg

        except Exception as e:
            self.logger.exception("Planning Error")
            return None, stats, str(e)

    def _deduplicate_path(
        self, path: List[Point3D], tolerance: float = 0.01
    ) -> List[Point3D]:
        """移除路径中连续重复的点"""
        if not path:
            return []
        new_path = [path[0]]
        tol_sq = tolerance * tolerance
        for i in range(1, len(path)):
            prev = new_path[-1]
            curr = path[i]
            dist_sq = (
                (prev[0] - curr[0]) ** 2
                + (prev[1] - curr[1]) ** 2
                + (prev[2] - curr[2]) ** 2
            )
            if dist_sq > tol_sq:
                new_path.append(curr)
        # 确保终点不丢失
        if len(path) > 1 and new_path[-1] != path[-1]:
            if self._dist_sq(new_path[-1], path[-1]) <= tol_sq:
                new_path[-1] = path[-1]
            else:
                new_path.append(path[-1])
        return new_path

    def _dist_sq(self, p1, p2):
        return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2

    def _get_initial_grid_nodes(
        self, start: Point3D, end: Point3D, is_fixed_height: bool
    ):
        """
        根据模式计算规划用的起始网格节点。
        """
        s_x, s_y, s_z = start
        e_x, e_y, e_z = end
        cruise_z = 0.0

        if is_fixed_height:
            # 2D 模式: 忽略 Z，只转换 X, Y
            cruise_z = self.config.get("CRANE_SAFE_TRAVEL_Z_M", 10.0)
            start_grid = self.map_mgr.world_to_grid(s_x, s_y)[:2]
            goal_grid = self.map_mgr.world_to_grid(e_x, e_y)[:2]
        else:
            # 3D 模式: 需要确保起终点至少高于最小安全高度
            min_safe = self.config.get("CRANE_Z_SAFETY_MARGIN", 0.5) + 1.0
            plan_s_z = max(s_z, min_safe)
            plan_e_z = max(e_z, min_safe)
            start_grid = self.map_mgr.world_to_grid(s_x, s_y, plan_s_z)
            goal_grid = self.map_mgr.world_to_grid(e_x, e_y, plan_e_z)

        return start_grid, goal_grid, cruise_z

    def _smart_escape(self, node: GridNode, ref_goal: GridNode) -> Optional[GridNode]:
        """
        [智能脱困]
        当起点不安全时，在起点周围搜索一个最近的安全网格作为临时起点。

        策略:
        以 node 为中心，向外辐射搜索 (BFS 思想)，直到找到一个 is_safe=True 的点。
        """
        if self.core_planner.is_safe(node):
            return node

        dims = len(node)
        # 搜索半径: 1格 到 5格
        for r in range(1, 6):
            candidates = []

            # 生成该半径下的所有候选点
            if dims == 2:
                cx, cy = node
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        # 只检查"外壳"上的点，内部的已经检查过了
                        if max(abs(dx), abs(dy)) == r:
                            n = (cx + dx, cy + dy)
                            if self.core_planner.is_safe(n):
                                candidates.append(n)
            else:
                # 3D 搜索
                cx, cy, cz = node
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        for dz in range(-r, r + 1):
                            if max(abs(dx), abs(dy), abs(dz)) == r:
                                n = (cx + dx, cy + dy, cz + dz)
                                if self.core_planner.is_safe(n):
                                    candidates.append(n)

            if candidates:
                # 如果有多个安全点，选择离目标(ref_goal)最近的那一个
                # 这样可以顺便让机器人往目标方向挪一点
                return min(
                    candidates,
                    key=lambda n: sum((n[i] - ref_goal[i]) ** 2 for i in range(dims)),
                )
        return None

    def _grid_to_world_smart(self, node: GridNode, override_z: float) -> Point3D:
        """智能坐标还原"""
        if len(node) == 2:
            # 2D 网格 -> 3D 物理 (使用 override_z 作为高度)
            wx, wy, _ = self.map_mgr.grid_to_world(node[0], node[1], 0)
            return (wx, wy, override_z)
        else:
            # 3D 网格 -> 3D 物理
            wx, wy, wz = self.map_mgr.grid_to_world(node[0], node[1], node[2])
            return (wx, wy, wz)

    def _create_3d_collision_checker(
        self, grace_start: Point3D, grace_end: Point3D
    ) -> Callable[[Point3D], bool]:
        """
        [闭包] 创建一个 3D 连续空间碰撞检测器。

        Args:
            grace_start, grace_end: 豁免点。
            在路径优化时，起点和终点本身可能在膨胀层边缘，如果不豁免，射线检测第一步就会失败。
        """
        radius = self.config.get("CRANE_FOOTPRINT_WIDTH", 5.0) / 2.0
        z_margin = (
            self.config.get("CRANE_Z_SAFETY_MARGIN", 0.5)
            + self.config.get("CRANE_FOOTPRINT_HEIGHT", 2.0) / 2.0
        )
        is_infinite = self.config.get("OBSTACLE_INFINITE_HEIGHT", True)

        def check(pt: Tuple[float, ...]) -> bool:
            # 1. 豁免检查 (如果在起点/终点附近 0.5m 内，直接放行)
            x, y, z = pt[0], pt[1], pt[2]
            if (x - grace_start[0]) ** 2 + (y - grace_start[1]) ** 2 + (
                z - grace_start[2]
            ) ** 2 < 0.5:
                return True
            if (x - grace_end[0]) ** 2 + (y - grace_end[1]) ** 2 + (
                z - grace_end[2]
            ) ** 2 < 0.5:
                return True

            # 2. 调用 MapManager 的真理层检测
            # 注意: 这里使用的是 "Raw" 检测，需要传入具体的 margin
            if self.map_mgr.check_collision_raw(
                x, y, z, radius, z_margin, ignore_z=is_infinite
            ):
                return False  # 撞了
            return True  # 安全

        return check
