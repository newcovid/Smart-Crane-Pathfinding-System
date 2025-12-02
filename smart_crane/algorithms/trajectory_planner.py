import logging
import time
import threading
from typing import List, Tuple, Dict, Any, Optional

from smart_crane.core.map_manager import WorkshopMapManager
from smart_crane.algorithms.pathfinding.base import PathPlannerBase
from smart_crane.algorithms.post_processing.base import PathPostProcessor
from smart_crane.core.config import Settings

from smart_crane.algorithms.components.grid_adapter import GridAdapter
from smart_crane.algorithms.components.planner_factory import PlannerFactory
from smart_crane.algorithms.components.safety_guard import SafetyGuard

from smart_crane.core import constants

# 类型别名
Point3D = Tuple[float, float, float]


class TrajectoryPlanner:
    """轨迹规划器 (Trajectory Planner)。

    这是路径规划模块的核心协调者（Facade 模式），负责串联整个规划流程：
    1.  调用 GridAdapter 准备网格数据。
    2.  调用 SafetyGuard 进行安全校验和脱困。
    3.  通过 PlannerFactory 创建核心算法（A*/D*）并计算路径。
    4.  调用 PostProcessors 进行路径平滑。
    5.  组装最终结果并返回。

    Attributes:
        map_mgr (WorkshopMapManager): 地图管理器引用。
        settings (Settings): 全局配置引用。
        logger (logging.Logger): 专用的日志记录器。
        grid_lock (threading.RLock): 共享的网格线程锁。
        grid_adapter (GridAdapter): 网格适配器组件。
        safety_guard (SafetyGuard): 安全守卫组件。
        core_planner (Optional[PathPlannerBase]): 当前激活的核心规划器实例。
        post_processors (List[PathPostProcessor]): 后处理管道列表。
    """

    def __init__(
        self,
        map_mgr: WorkshopMapManager,
        settings: Settings,
        logger: Optional[logging.Logger] = None,
        grid_lock: Optional[threading.RLock] = None,
    ):
        """初始化轨迹规划器。

        Args:
            map_mgr: 地图管理器。
            settings: 全局配置。
            logger: 父级日志记录器。
            grid_lock: 线程锁。如果不传，将创建一个新的。
        """
        self.map_mgr = map_mgr
        self.settings = settings
        self.grid_lock = grid_lock or threading.RLock()

        # [日志优化] 智能嵌套: Parent.TrajectoryPlanner
        if logger:
            self.logger = logger.getChild(self.__class__.__name__)
        else:
            self.logger = logging.getLogger(self.__class__.__name__)

        # 初始化子组件 (传递 logger 以保持上下文链)
        self.grid_adapter = GridAdapter(map_mgr, self.logger)
        self.safety_guard = SafetyGuard(map_mgr, self.logger)

        self.core_planner: Optional[PathPlannerBase] = None
        self.post_processors: List[PathPostProcessor] = []

        # 网格缓存
        self.active_planning_grid = None
        self.visualization_grid = None
        self._pending_grid_prep_ms = 0.0

        # 初次构建
        self.initialize_planner()
        self.logger.info("轨迹规划器启动就绪。")

    def initialize_planner(self, force_rebuild: bool = False):
        """初始化或重建规划器实例。

        通常在系统启动或配置发生重大变更（如切换算法、改变地图尺寸）时调用。

        Args:
            force_rebuild: (保留参数) 强制重建标志。
        """
        t_start = time.perf_counter()

        with self.grid_lock:
            # 1. 准备网格 (通过适配器)
            # grid_h 用于告诉规划器当前的物理层高，以便计算 3D 启发式
            plan_grid, vis_grid, grid_h = self.grid_adapter.prepare_grids(self.settings)
            self.active_planning_grid = plan_grid
            self.visualization_grid = vis_grid

            # 2. 创建核心算法实例 (通过工厂)
            self.core_planner = PlannerFactory.create_core_planner(
                self.settings,
                plan_grid,
                self.map_mgr,
                grid_h,
                self.logger,  # 传递 logger
                self.grid_lock,
            )

            # 3. 创建后处理管道
            # [Fix] 显式传递 map_mgr 给工厂，以便注入到 PostProcessors 中
            self.post_processors = PlannerFactory.create_post_processors(
                self.settings, self.map_mgr, self.logger
            )

            # 统计耗时
            elapsed = (time.perf_counter() - t_start) * constants.TIME_MS_MULTIPLIER
            self._pending_grid_prep_ms += elapsed

            algo_type = self.settings.planner.algorithm
            self.logger.info(f"核心重构完成: Algorithm={algo_type.upper()}")

    def update_configuration(self):
        """处理配置变更。

        当收到前端发来的配置更新请求时调用。
        目前采取全量刷新策略：清除缓存并重建规划器。
        """
        self.logger.info("正在应用新配置并重构规划器...")
        self.map_mgr._invalidate_cache()
        self.initialize_planner(force_rebuild=True)

    def handle_obstacle_update(
        self, x: float, y: float, w: float, h: float, z: float, is_add: bool
    ):
        """处理动态障碍物更新事件。

        如果当前算法支持增量更新（如 D* Lite），则计算差异并推送给算法；
        否则仅更新网格数据，留待下次规划时全量计算。

        Args:
            x, y, w, h, z: 障碍物的包围盒参数。
            is_add: True 表示添加，False 表示移除。
        """
        t_start = time.perf_counter()

        with self.grid_lock:
            old_grid = self.active_planning_grid

            # [性能埋点] 1. 网格重生成 (Grid Rebuild)
            t1 = time.perf_counter()
            plan_grid, vis_grid, _ = self.grid_adapter.prepare_grids(self.settings)
            self.active_planning_grid = plan_grid
            self.visualization_grid = vis_grid

            # 更新核心规划器的网格引用 (这里可能会触发 Rust 侧的网格全量拷贝)
            self.core_planner.grid = plan_grid
            t2 = time.perf_counter()

            # 检查是否可以执行增量更新
            supports_incremental = getattr(
                self.core_planner, "supports_incremental", False
            )

            # 只有当已有任务（起点终点已设定）时，增量更新才有意义
            has_mission = (
                getattr(self.core_planner, "start_node", None) is not None
                and getattr(self.core_planner, "goal_node", None) is not None
            )

            t_diff = 0.0
            t_algo = 0.0

            if supports_incremental and has_mission:
                # [性能埋点] 2. 差异计算 (Diff Calculation)
                t3 = time.perf_counter()
                changes = self.grid_adapter.calculate_incremental_changes(
                    self.settings, old_grid, plan_grid, (x, y, w, h, z)
                )
                t4 = time.perf_counter()
                t_diff = (t4 - t3) * constants.TIME_MS_MULTIPLIER

                if changes:
                    self.logger.debug(f"触发增量更新: {len(changes)} 处网格变化。")
                    # [性能埋点] 3. 算法增量更新 (Algo Update / Replanning)
                    t5 = time.perf_counter()
                    self.core_planner.update_obstacles(changes)
                    t6 = time.perf_counter()
                    t_algo = (t6 - t5) * constants.TIME_MS_MULTIPLIER
            else:
                self.logger.debug("跳过增量更新 (算法不支持或无活跃任务)。")

            # 打印详细耗时分析
            t_grid_rebuild = (t2 - t1) * constants.TIME_MS_MULTIPLIER
            self.logger.debug(
                f"[Perf] HandleUpdate: GridRebuild={t_grid_rebuild:.2f}ms, "
                f"Diff={t_diff:.2f}ms, AlgoUpdate={t_algo:.2f}ms"
            )

        elapsed = (time.perf_counter() - t_start) * constants.TIME_MS_MULTIPLIER
        self._pending_grid_prep_ms += elapsed

    def plan(
        self, start: Dict[str, float], end: Dict[str, float]
    ) -> Tuple[Optional[List[Point3D]], Dict[str, Any], str]:
        """执行一次完整的路径规划任务。

        Args:
            start: 起点坐标字典 {'x': 1.0, 'y': 2.0, 'z': 0.0}。
            end: 终点坐标字典。

        Returns:
            Tuple[Optional[List[Point3D]], Dict[str, Any], str]:
                - path: 路径点列表 [(x,y,z), ...]，失败返回 None。
                - stats: 性能统计字典。
                - message: 结果描述信息。
        """
        stats = {
            "timings": {"total_ms": 0},
            "grid_meta": {},
            "processors_stats": [],
            "path_meta": {},
        }
        msg_list = []
        t_total_start = time.perf_counter()

        # 继承之前操作（如初始化、障碍物更新）积累的耗时
        pending_cost = self._pending_grid_prep_ms
        self._pending_grid_prep_ms = 0.0

        try:
            with self.grid_lock:
                # 0. 参数解析
                exact_start = (
                    float(start["x"]),
                    float(start["y"]),
                    float(start.get("z", 0.0)),
                )
                exact_end = (
                    float(end["x"]),
                    float(end["y"]),
                    float(end.get("z", 0.0)),
                )

                # Phase 1: Grid Prep (网格准备)
                t_grid_start = time.perf_counter()

                # 确保规划器持有最新的网格
                if self.core_planner.grid is not self.active_planning_grid:
                    self.core_planner.grid = self.active_planning_grid

                stats["timings"]["grid_prep_ms"] = (
                    time.perf_counter() - t_grid_start
                ) * constants.TIME_MS_MULTIPLIER + pending_cost

                stats["grid_meta"] = {
                    "dims": [
                        self.map_mgr.rows,
                        self.map_mgr.cols,
                        self.map_mgr.layers,
                    ],
                    "total_voxels": self.map_mgr.rows
                    * self.map_mgr.cols
                    * max(1, self.map_mgr.layers),
                }

                # Phase 2: Validation (安全校验)
                is_valid, err, needs_escape = self.safety_guard.validate_endpoints(
                    exact_start, exact_end, self.settings
                )
                if not is_valid:
                    self.logger.warning(f"规划请求被拒绝: {err}")
                    return None, stats, err

                if needs_escape:
                    msg_list.append("起点自动脱困")
                    self.logger.info("触发起点自动脱困程序。")

                # Phase 3: Coord Conversion (坐标转换)
                # 获取网格坐标 (Grid Node) 和巡航高度 (Cruise Z)
                s_node, e_node, cruise_z = self.grid_adapter.get_initial_grid_nodes(
                    exact_start, exact_end, self.settings
                )

                # Phase 4: Smart Escape (智能脱困)
                final_s_node = s_node
                has_escaped = False

                # 如果起点在膨胀层内，或者直接不可行，尝试搜索附近的脱困点
                if needs_escape or not self.core_planner.is_safe(s_node):
                    esc_node = self.safety_guard.smart_escape(
                        s_node, e_node, self.core_planner, self.settings
                    )
                    if esc_node:
                        final_s_node = esc_node
                        has_escaped = True
                        if needs_escape:
                            self.logger.info(f"脱困点确认: {esc_node}")
                    else:
                        return None, stats, "起点被封死，无法脱困"

                if not self.core_planner.is_safe(e_node):
                    return None, stats, "终点不可达 (位于障碍物内)"

                # Phase 5: Core Pathfinding (核心寻路)
                t_algo_start = time.perf_counter()

                # 初始化算法状态
                if not self.core_planner.initialize(final_s_node, e_node):
                    return None, stats, "规划器初始化失败"

                # 计算路径
                raw_path = self.core_planner.compute_path(final_s_node)

                stats["timings"]["pathfinding_ms"] = (
                    time.perf_counter() - t_algo_start
                ) * constants.TIME_MS_MULTIPLIER
                stats.update(self.core_planner.get_stats())

                if not raw_path:
                    self.logger.info("核心算法未找到可行路径。")
                    return None, stats, "无解路径"

                # Phase 6: Splicing (路径还原与吸附优化)
                t_splice_start = time.perf_counter()

                # 将网格路径还原为物理坐标（全部位于巡航层）
                path_cruise = [
                    self.grid_adapter.grid_to_world_smart(n, cruise_z) for n in raw_path
                ]

                # [核心修复] 吸附优化 (Snapping)
                # 只有将路径的第一个点和最后一个点替换为用户输入的精确坐标，
                # 贪婪算法（Greedy）才能有机会将整条路径拉直为 "Start -> End"。
                if path_cruise:
                    # 1. 处理起点吸附
                    if has_escaped:
                        # 脱困情况下，起点是算法算出的安全点，不应替换为可能不安全的 exact_start
                        pass
                    else:
                        if self.settings.crane.enable_fixed_height_cruise:
                            # 2.5D: 仅吸附 XY，Z 保持巡航高度
                            path_cruise[0] = (
                                exact_start[0],
                                exact_start[1],
                                path_cruise[0][2],
                            )
                        else:
                            # [修复] 3D: 完全吸附 XYZ，消除因网格中心化导致的高度差
                            # 这样 Greedy 算法看到的就是精确的 Start 点，而非网格中心的 5.5m
                            path_cruise[0] = exact_start

                    # 2. 处理终点吸附 (终点总是安全的)
                    if self.settings.crane.enable_fixed_height_cruise:
                        path_cruise[-1] = (
                            exact_end[0],
                            exact_end[1],
                            path_cruise[-1][2],
                        )
                    else:
                        # [修复] 3D: 完全吸附 XYZ
                        path_cruise[-1] = exact_end

                # Phase 7: Post Processing (后处理 - 仅针对巡航段)
                # 警告：不要将垂直起降段放入后处理器，否则贝塞尔平滑可能会切角导致碰撞
                opt_cruise_path = path_cruise

                # 创建一个针对巡航层的碰撞检测器 (用于 Python 回退模式)
                if path_cruise:
                    cruise_checker = self.safety_guard.create_collision_checker(
                        path_cruise[0], path_cruise[-1], self.settings
                    )
                    for proc in self.post_processors:
                        opt_cruise_path = proc.process(opt_cruise_path, cruise_checker)
                        stats["processors_stats"].append(proc.get_stats())

                # Phase 8: Final Assembly (最终组装 - 逻辑修复)
                # 逻辑：Entry Maneuver -> Cruise Path -> Exit Maneuver
                final_path = []
                is_fixed_h = self.settings.crane.enable_fixed_height_cruise

                if is_fixed_h:
                    # === 1. Entry Phase (起步阶段) ===
                    if has_escaped:
                        # [修复] 脱困逻辑：先平移出危险区，再提升
                        # Path: Start(Z_low) -> Escape(Z_low) -> Escape(Z_high) -> Cruise...

                        # 1.1 加入原始起点
                        final_path.append(exact_start)

                        # 1.2 加入脱困点 (低空)
                        # 脱困点是 opt_cruise_path[0] (即网格计算出的安全点)
                        esc_pt_cruise = opt_cruise_path[0]
                        esc_pt_low = (
                            esc_pt_cruise[0],
                            esc_pt_cruise[1],
                            exact_start[2],
                        )

                        # 只有当脱困点距离起点有一定距离时才添加，避免重合
                        if (
                            self._dist_sq(exact_start, esc_pt_low)
                            > constants.PATH_MIN_SEPARATION_SQ
                        ):
                            final_path.append(esc_pt_low)

                        # 1.3 加入脱困点 (高空/巡航层)
                        # 这是一个关键的拐角点，必须保留
                        final_path.append(esc_pt_cruise)

                    else:
                        # [正常] 标准逻辑：原地提升
                        # Path: Start(Z_low) -> Start(Z_high) -> Cruise...

                        # 2.1 加入原始起点
                        final_path.append(exact_start)

                        # 2.2 加入起飞点 (Start High)
                        start_pt_cruise = (
                            exact_start[0],
                            exact_start[1],
                            cruise_z,
                        )

                        # 只有当高度差显著时才添加
                        if (
                            abs(exact_start[2] - cruise_z)
                            > constants.PATH_MIN_HEIGHT_DIFF
                        ):
                            final_path.append(start_pt_cruise)

                    # === 2. Cruise Phase (巡航阶段) ===
                    # 连接到 opt_cruise_path。
                    # 注意：opt_cruise_path[0] 此时与 final_path[-1] 可能重合
                    if len(opt_cruise_path) > 1:
                        final_path.extend(opt_cruise_path[1:])
                    elif len(opt_cruise_path) == 1:
                        if (
                            not final_path
                            or self._dist_sq(final_path[-1], opt_cruise_path[0])
                            > constants.PATH_MIN_SEPARATION_SQ
                        ):
                            final_path.append(opt_cruise_path[0])

                    # === 3. Exit Phase (降落阶段) ===
                    # 校验：确保最后一点确实在终点上方
                    end_pt_cruise = (exact_end[0], exact_end[1], cruise_z)

                    if (
                        not final_path
                        or self._dist_sq(final_path[-1], end_pt_cruise)
                        > constants.PATH_MIN_SEPARATION_SQ
                    ):
                        final_path.append(end_pt_cruise)

                    # 3.2 降落到终点
                    if abs(exact_end[2] - cruise_z) > constants.PATH_MIN_HEIGHT_DIFF:
                        final_path.append(exact_end)

                else:
                    # === 3D 模式 ===
                    # 由于我们在 Phase 6 已经将 opt_cruise_path 的首尾点吸附为 exact_start 和 exact_end，
                    # 且 Greedy 优化器已经拉直了路径，所以这里只需直接使用 opt_cruise_path。
                    # 为保险起见，进行一次边界检查。

                    # 1. 补起点 (防止 path 被过度优化或脱困导致缺失)
                    if (
                        opt_cruise_path
                        and self._dist_sq(exact_start, opt_cruise_path[0])
                        > constants.PATH_MIN_SEPARATION_SQ
                    ):
                        final_path.append(exact_start)

                    # 2. 中间路径
                    final_path.extend(opt_cruise_path)

                    # 3. 补终点
                    if (
                        opt_cruise_path
                        and self._dist_sq(exact_end, opt_cruise_path[-1])
                        > constants.PATH_MIN_SEPARATION_SQ
                    ):
                        final_path.append(exact_end)

                # 最终清理：移除极其微小的抖动点，并保留指定小数位
                final_path = self._deduplicate_path(
                    final_path, tolerance=constants.DEFAULT_DEDUP_TOLERANCE
                )
                final_path = [
                    tuple(round(v, constants.PATH_COORD_ROUND_DIGITS) for v in pt)
                    for pt in final_path
                ]

                stats["timings"]["splicing_ms"] = (
                    time.perf_counter() - t_splice_start
                ) * constants.TIME_MS_MULTIPLIER
                stats["timings"]["total_ms"] = (
                    time.perf_counter() - t_total_start
                ) * constants.TIME_MS_MULTIPLIER + pending_cost
                stats["path_meta"]["final_nodes"] = len(final_path)

                msg = f"成功 ({'; '.join(msg_list)})" if msg_list else "成功"
                self.logger.info(
                    f"规划完成。节点数: {len(final_path)}, 总耗时: {stats['timings']['total_ms']:.{constants.TIME_FMT_PREC}f}ms"
                )

                return final_path, stats, msg

        except Exception as e:
            self.logger.exception("规划过程发生未捕获异常")
            return None, stats, str(e)

    def _dist_sq(self, p1: Point3D, p2: Point3D) -> float:
        """计算两点距离平方。"""
        return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2

    def _deduplicate_path(
        self, path: List[Point3D], tolerance: float = constants.DEFAULT_DEDUP_TOLERANCE
    ) -> List[Point3D]:
        """移除路径中重复或极近的连续点。

        Args:
            path: 原始路径。
            tolerance: 判定为重复点的距离阈值（米）。

        Returns:
            List[Point3D]: 去重后的路径。
        """
        if not path:
            return []

        new_path = [path[0]]
        tol_sq = tolerance * tolerance

        for i in range(1, len(path)):
            prev = new_path[-1]
            curr = path[i]
            dist_sq = self._dist_sq(prev, curr)
            # 只有距离大于阈值才加入
            if dist_sq > tol_sq:
                new_path.append(curr)

        # 确保终点不被意外移除（如果终点和倒数第二点很近，可能会被跳过，需强制补回）
        if len(path) > 1 and new_path[-1] != path[-1]:
            # 再次检查距离，避免重合
            if self._dist_sq(new_path[-1], path[-1]) > tol_sq:
                new_path.append(path[-1])
            else:
                # 如果重合但不是同一个对象，更新为精确终点
                new_path[-1] = path[-1]

        return new_path
