import logging
import math
import time
import threading
from typing import List, Tuple, Dict, Any, Optional, Callable, Union

from core.map_manager import WorkshopMapManager
from algorithms.base import PathPlannerBase
from algorithms.astar import AStarPlanner
from algorithms.dslite import DLitePlanner
from algorithms.post_processing.base import PathPostProcessor
from algorithms.post_processing.greedy import GreedyShortcutProcessor
from algorithms.post_processing.bezier import BezierSmoothProcessor

Point3D = Tuple[float, float, float]


class TrajectoryPlanner:
    """
    【核心轨迹规划器 - V8 Stats Enhanced】

    Fixes:
    1. [Stats] 全面增强统计数据收集，支持前端新版看板。
       - grid_meta: 网格维度、总格点数。
       - timings: 细分为 grid_prep, pathfinding, splicing, optimization。
       - pipeline: 详细记录每个后处理器的输入/输出节点数。
    """

    def __init__(
        self,
        map_mgr: WorkshopMapManager,
        config: Dict[str, Any],
        logger: Optional[logging.Logger] = None,
        grid_lock: Optional[threading.RLock] = None,
    ):
        self.map_mgr = map_mgr
        self.config = config.copy()
        self.logger = logger or logging.getLogger("TrajectoryPlanner")
        self.grid_lock = grid_lock or threading.RLock()

        self.core_planner: Optional[PathPlannerBase] = None
        self.post_processors: List[PathPostProcessor] = []
        self.active_planning_grid = None
        self.visualization_grid = None

        self._initialize_planner()

    def _initialize_planner(self, force_rebuild: bool = False):
        with self.grid_lock:
            algo_type = self.config.get("PLANNER_ALGORITHM", "astar")
            use_octile = self.config.get("USE_3D_OCTILE", False)
            h_weight = self.config.get("HEURISTIC_WEIGHT", 1.5)

            # 注意：这里只是初始化，真正的网格生成耗时会在 plan() 中通过 _prepare_grids 再次发生或被缓存
            plan_grid, vis_grid, grid_height_m = self._prepare_grids()
            self.active_planning_grid = plan_grid
            self.visualization_grid = vis_grid

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

            if algo_type == "dslite":
                self.core_planner = DLitePlanner(**common_args)
            else:
                self.core_planner = AStarPlanner(**common_args)

            self.post_processors = []
            if self.config.get("ENABLE_SHORTCUT_OPTIMIZATION", True):
                self.post_processors.append(GreedyShortcutProcessor())
            if self.config.get("ENABLE_BEZIER_SMOOTHING", True):
                self.post_processors.append(
                    BezierSmoothProcessor(
                        smoothness=float(self.config.get("BEZIER_SMOOTHNESS", 0.3)),
                        segments=int(self.config.get("BEZIER_SEGMENTS", 10)),
                    )
                )

            self.logger.info(
                f"[TrajPlanner] Rebuilt: Algo={algo_type}, Infinite={self.config.get('OBSTACLE_INFINITE_HEIGHT')}"
            )

    def _prepare_grids(self) -> Tuple[Any, Any, float]:
        cfg = self.config
        shape = cfg.get("CRANE_FOOTPRINT_SHAPE", "box")
        w, l = cfg.get("CRANE_FOOTPRINT_WIDTH", 5.0), cfg.get(
            "CRANE_FOOTPRINT_LENGTH", 5.0
        )
        radius_m = (w / 2.0) if shape == "circle" else (math.hypot(w, l) / 2.0)
        xy_margin = radius_m / self.map_mgr.resolution_m

        user_z_margin = cfg.get("CRANE_Z_SAFETY_MARGIN", 0.5)
        crane_h = cfg.get("CRANE_FOOTPRINT_HEIGHT", 2.0)
        z_margin_obs = user_z_margin + (crane_h / 2.0)

        is_fixed_height = cfg.get("ENABLE_FIXED_HEIGHT_CRUISE", True)
        is_infinite_obs = cfg.get("OBSTACLE_INFINITE_HEIGHT", True)

        if is_fixed_height:
            cruise_z = cfg.get("CRANE_SAFE_TRAVEL_Z_M", 10.0)
            check_z = None if is_infinite_obs else cruise_z
            grid_2d = self.map_mgr.get_2d_projection_grid(
                xy_margin=xy_margin, check_z=check_z, z_margin=z_margin_obs
            )
            return grid_2d, grid_2d, 0.0
        else:
            z_margin_ceil = crane_h / 2.0
            grid_3d = self.map_mgr.get_3d_voxel_grid(
                xy_margin=xy_margin,
                z_margin_obs=z_margin_obs,
                z_margin_ceil=z_margin_ceil,
                is_infinite=is_infinite_obs,
            )
            grid_vis = self.map_mgr.get_2d_projection_grid(
                xy_margin=xy_margin, check_z=None, z_margin=z_margin_obs
            )
            return grid_3d, grid_vis, self.map_mgr.height_m

    def update_configuration(self, new_config: Dict[str, Any]) -> bool:
        need_rebuild = False
        keys_affecting_grid = [
            "CRANE_FOOTPRINT",
            "CRANE_Z_SAFETY",
            "ENABLE_FIXED_HEIGHT",
            "CRANE_SAFE_TRAVEL",
            "OBSTACLE_INFINITE",
            "MAP_RESOLUTION",
        ]
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
                    self.map_mgr._invalidate_cache()
                    need_rebuild = True
                elif any(key in k for key in keys_affecting_algo):
                    need_rebuild = True

        if need_rebuild:
            self._initialize_planner(force_rebuild=True)
            return True
        return False

    def handle_obstacle_update(self, x, y, w, h, z, is_add: bool):
        with self.grid_lock:
            plan_grid, vis_grid, _ = self._prepare_grids()
            self.active_planning_grid = plan_grid
            self.visualization_grid = vis_grid
            self.core_planner.grid = plan_grid
            if isinstance(self.core_planner, DLitePlanner):
                self.core_planner.initialize(
                    self.core_planner.start_node, self.core_planner.goal_node
                )

    # --- 安全校验 ---
    def _validate_endpoints(
        self, start_pt: Point3D, end_pt: Point3D
    ) -> Tuple[bool, str, bool]:
        shape = self.config.get("CRANE_FOOTPRINT_SHAPE", "box")
        w, l = self.config.get("CRANE_FOOTPRINT_WIDTH", 5.0), self.config.get(
            "CRANE_FOOTPRINT_LENGTH", 5.0
        )
        xy_margin = (w / 2.0) if shape == "circle" else (math.hypot(w, l) / 2.0)

        z_safety = self.config.get("CRANE_Z_SAFETY_MARGIN", 0.5)
        crane_h = self.config.get("CRANE_FOOTPRINT_HEIGHT", 2.0)
        z_margin = z_safety + (crane_h / 2.0)

        if self.map_mgr.check_collision_raw(
            start_pt[0], start_pt[1], start_pt[2], 0, 0, ignore_z=False
        ):
            return False, "起点位于障碍物内部 (物理碰撞)，无法规划。", False

        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], 0, 0, ignore_z=False
        ):
            return False, "终点位于障碍物内部 (物理碰撞)，无法规划。", False

        if self.map_mgr.check_collision_raw(
            end_pt[0], end_pt[1], end_pt[2], xy_margin, z_margin, ignore_z=False
        ):
            return False, "终点位于安全缓冲区(膨胀层)内，禁止停靠。", False

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
        执行轨迹规划并收集详细统计数据。
        """
        # 初始化统计容器
        stats = {
            "timings": {},
            "grid_meta": {},
            "processors_stats": [],
            "path_meta": {},
        }

        t_total_start = time.perf_counter()
        msg_list = []

        try:
            with self.grid_lock:
                is_fixed_height = self.config.get("ENABLE_FIXED_HEIGHT_CRUISE", True)

                exact_start = (
                    float(start["x"]),
                    float(start["y"]),
                    float(start.get("z", 0.0)),
                )
                exact_end = (float(end["x"]), float(end["y"]), float(end.get("z", 0.0)))

                # [Stat] 1. 网格准备耗时 (Grid Prep)
                t_grid_start = time.perf_counter()
                # 这里调用 _prepare_grids 可能会触发 MapManager 的计算或读取缓存
                # 注意：为了更准确统计，我们在这里显式调用一次，确保 active_planning_grid 是最新的
                self.active_planning_grid, _, _ = self._prepare_grids()
                stats["timings"]["grid_prep_ms"] = (
                    time.perf_counter() - t_grid_start
                ) * 1000

                # [Stat] 收集网格元数据
                if is_fixed_height:
                    stats["grid_meta"] = {
                        "type": "2D Projection",
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

                # --- 安全校验 ---
                is_valid, err_msg, start_needs_escape = self._validate_endpoints(
                    exact_start, exact_end
                )
                if not is_valid:
                    return None, stats, err_msg
                if start_needs_escape:
                    msg_list.append("起点自动脱困")

                # --- 坐标转换 ---
                planner_start_node, planner_end_node, cruise_z_level = (
                    self._get_initial_grid_nodes(
                        exact_start, exact_end, is_fixed_height
                    )
                )

                # --- 智能脱困 ---
                start_node_final = planner_start_node
                if start_needs_escape or not self.core_planner.is_safe(
                    planner_start_node
                ):
                    escape_node = self._smart_escape(
                        planner_start_node, planner_end_node
                    )
                    if escape_node:
                        start_node_final = escape_node
                    else:
                        return None, stats, "起点脱困失败"

                if not self.core_planner.is_safe(planner_end_node):
                    return None, stats, "终点网格不可达"
                end_node_final = planner_end_node

                # [Stat] 2. 核心算法耗时 (Core Algo)
                t_algo_start = time.perf_counter()
                if not self.core_planner.initialize(start_node_final, end_node_final):
                    return None, stats, "规划器初始化失败"

                raw_grid_path = self.core_planner.compute_path(start_node_final)
                stats["timings"]["pathfinding_ms"] = (
                    time.perf_counter() - t_algo_start
                ) * 1000

                # 合并核心算法统计 (nodes_expanded 等)
                stats.update(self.core_planner.get_stats())

                if not raw_grid_path:
                    return None, stats, "未找到路径"

                # [Stat] 3. 拼接转换耗时 (Splicing)
                t_splice_start = time.perf_counter()

                cruise_segment_world = []
                for node in raw_grid_path:
                    cruise_segment_world.append(
                        self._grid_to_world_smart(node, cruise_z_level)
                    )

                path_to_optimize = []
                if is_fixed_height:
                    bridge_start = (exact_start[0], exact_start[1], cruise_z_level)
                    bridge_end = (exact_end[0], exact_end[1], cruise_z_level)
                    path_to_optimize = (
                        [bridge_start] + cruise_segment_world + [bridge_end]
                    )
                else:
                    path_to_optimize = (
                        [exact_start] + cruise_segment_world + [exact_end]
                    )

                path_to_optimize = self._deduplicate_path(path_to_optimize)
                stats["timings"]["splicing_ms"] = (
                    time.perf_counter() - t_splice_start
                ) * 1000

                # [Stat] 4. 后处理耗时 (Optimization)
                is_safe_fn = self._create_3d_collision_checker(
                    grace_start=path_to_optimize[0], grace_end=path_to_optimize[-1]
                )

                optimized_path = path_to_optimize
                # 收集每个处理器的详细数据
                for processor in self.post_processors:
                    optimized_path = processor.process(optimized_path, is_safe_fn)
                    stats["processors_stats"].append(processor.get_stats())

                # 最终组装
                final_path = []
                if is_fixed_height:
                    if abs(exact_start[2] - optimized_path[0][2]) > 0.01:
                        final_path.append(exact_start)
                    final_path.extend(optimized_path)
                    if abs(exact_end[2] - optimized_path[-1][2]) > 0.01:
                        final_path.append(exact_end)
                else:
                    final_path = optimized_path

                final_path = self._deduplicate_path(final_path)
                final_path = [tuple(round(v, 2) for v in pt) for pt in final_path]

                stats["timings"]["total_ms"] = (
                    time.perf_counter() - t_total_start
                ) * 1000
                stats["path_meta"]["final_nodes"] = len(final_path)

                final_msg = "Success"
                if msg_list:
                    final_msg = f"Success ({'; '.join(msg_list)})"

                return final_path, stats, final_msg

        except Exception as e:
            self.logger.exception("Planning Error")
            return None, stats, str(e)

    def _deduplicate_path(
        self, path: List[Point3D], tolerance: float = 0.01
    ) -> List[Point3D]:
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
        s_x, s_y, s_z = start
        e_x, e_y, e_z = end
        cruise_z = 0.0
        if is_fixed_height:
            cruise_z = self.config.get("CRANE_SAFE_TRAVEL_Z_M", 10.0)
            start_grid = self.map_mgr.world_to_grid(s_x, s_y)[:2]
            goal_grid = self.map_mgr.world_to_grid(e_x, e_y)[:2]
        else:
            min_safe = self.config.get("CRANE_Z_SAFETY_MARGIN", 0.5) + 1.0
            plan_s_z = max(s_z, min_safe)
            plan_e_z = max(e_z, min_safe)
            start_grid = self.map_mgr.world_to_grid(s_x, s_y, plan_s_z)
            goal_grid = self.map_mgr.world_to_grid(e_x, e_y, plan_e_z)
        return start_grid, goal_grid, cruise_z

    def _smart_escape(self, node, ref_goal) -> Optional[Tuple]:
        if self.core_planner.is_safe(node):
            return node
        dims = len(node)
        for r in range(1, 6):
            candidates = []
            if dims == 2:
                cx, cy = node
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        if max(abs(dx), abs(dy)) == r:
                            n = (cx + dx, cy + dy)
                            if self.core_planner.is_safe(n):
                                candidates.append(n)
            else:
                cx, cy, cz = node
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        for dz in range(-r, r + 1):
                            if max(abs(dx), abs(dy), abs(dz)) == r:
                                n = (cx + dx, cy + dy, cz + dz)
                                if self.core_planner.is_safe(n):
                                    candidates.append(n)
            if candidates:
                return min(
                    candidates,
                    key=lambda n: sum((n[i] - ref_goal[i]) ** 2 for i in range(dims)),
                )
        return None

    def _grid_to_world_smart(self, node, override_z: float) -> Point3D:
        if len(node) == 2:
            wx, wy, _ = self.map_mgr.grid_to_world(node[0], node[1], 0)
            return (wx, wy, override_z)
        else:
            wx, wy, wz = self.map_mgr.grid_to_world(node[0], node[1], node[2])
            return (wx, wy, wz)

    def _create_3d_collision_checker(
        self, grace_start: Point3D, grace_end: Point3D
    ) -> Callable[[Point3D], bool]:
        radius = self.config.get("CRANE_FOOTPRINT_WIDTH", 5.0) / 2.0
        z_margin = (
            self.config.get("CRANE_Z_SAFETY_MARGIN", 0.5)
            + self.config.get("CRANE_FOOTPRINT_HEIGHT", 2.0) / 2.0
        )
        is_infinite = self.config.get("OBSTACLE_INFINITE_HEIGHT", True)

        def check(pt: Tuple[float, ...]) -> bool:
            x, y, z = pt[0], pt[1], pt[2]
            if (x - grace_start[0]) ** 2 + (y - grace_start[1]) ** 2 + (
                z - grace_start[2]
            ) ** 2 < 0.5:
                return True
            if (x - grace_end[0]) ** 2 + (y - grace_end[1]) ** 2 + (
                z - grace_end[2]
            ) ** 2 < 0.5:
                return True

            if self.map_mgr.check_collision_raw(
                x, y, z, radius, z_margin, ignore_z=is_infinite
            ):
                return False
            return True

        return check
