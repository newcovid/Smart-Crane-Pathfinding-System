import uuid
import threading
import logging
import time
import math
import numpy as np
from typing import Dict, Any, Optional, List, Tuple, Union

from .map_manager import WorkshopMapManager
from algorithms.base import PathPlannerBase
from algorithms.astar import AStarPlanner
from algorithms.dslite import DLitePlanner
from algorithms.post_processing.greedy import GreedyShortcutProcessor
from algorithms.post_processing.bezier import BezierSmoothProcessor


class CraneService:
    """
    【业务服务层 (Service Layer)】 - V3.6 Ceiling Fix
    """

    def __init__(self, config: Any, logger: logging.Logger):
        self.logger = logger
        self.config_obj = config
        self.current_config = self._load_config_dict(config)

        self.mission_state = {
            "start": {"x": 5.0, "y": 5.0},
            "end": {"x": 30.0, "y": 20.0},
        }
        self.last_calculated_path: List[Tuple[float, float, float]] = []
        self.last_stats: Dict[str, Any] = {}
        self.grid_lock = threading.RLock()
        self.map_mgr: Optional[WorkshopMapManager] = None
        self.planner: Optional[PathPlannerBase] = None
        self.post_processors = []

        try:
            self._init_components(force_rebuild=True)
            self.logger.info(">>> CraneService (V3.6 Ceiling Fix) 初始化完成")
        except Exception as e:
            self.logger.critical(f"CraneService 初始化失败: {e}", exc_info=True)

    def _load_config_dict(self, config_cls) -> Dict[str, Any]:
        return {
            "MAP_WIDTH_M": getattr(config_cls, "MAP_WIDTH_M", 100.0),
            "MAP_LENGTH_M": getattr(config_cls, "MAP_LENGTH_M", 100.0),
            "MAP_HEIGHT_M": getattr(config_cls, "MAP_HEIGHT_M", 20.0),
            "MAP_RESOLUTION_M": getattr(config_cls, "MAP_RESOLUTION_M", 0.5),
            "CRANE_FOOTPRINT_SHAPE": getattr(
                config_cls, "CRANE_FOOTPRINT_SHAPE", "box"
            ),
            "CRANE_FOOTPRINT_WIDTH": getattr(config_cls, "CRANE_FOOTPRINT_WIDTH", 5.0),
            "CRANE_FOOTPRINT_LENGTH": getattr(
                config_cls, "CRANE_FOOTPRINT_LENGTH", 5.0
            ),
            "CRANE_FOOTPRINT_HEIGHT": getattr(
                config_cls, "CRANE_FOOTPRINT_HEIGHT", 2.0
            ),
            "CRANE_Z_SAFETY_MARGIN": getattr(config_cls, "CRANE_Z_SAFETY_MARGIN", 0.5),
            "ENABLE_FIXED_HEIGHT_CRUISE": getattr(
                config_cls, "ENABLE_FIXED_HEIGHT_CRUISE", True
            ),
            "CRANE_SAFE_TRAVEL_Z_M": getattr(config_cls, "CRANE_SAFE_TRAVEL_Z_M", 11.0),
            "OBSTACLE_INFINITE_HEIGHT": getattr(
                config_cls, "OBSTACLE_INFINITE_HEIGHT", True
            ),
            "DEFAULT_OBSTACLE_HEIGHT_M": getattr(
                config_cls, "DEFAULT_OBSTACLE_HEIGHT_M", 2.0
            ),
            "PLANNER_ALGORITHM": getattr(config_cls, "PLANNER_ALGORITHM", "astar"),
            "USE_3D_OCTILE": getattr(config_cls, "USE_3D_OCTILE", False),
            "HEURISTIC_WEIGHT": getattr(config_cls, "HEURISTIC_WEIGHT", 1.5),
            "ENABLE_SHORTCUT_OPTIMIZATION": getattr(
                config_cls, "ENABLE_SHORTCUT_OPTIMIZATION", True
            ),
            "ENABLE_BEZIER_SMOOTHING": getattr(
                config_cls, "ENABLE_BEZIER_SMOOTHING", True
            ),
            "BEZIER_SMOOTHNESS": getattr(config_cls, "BEZIER_SMOOTHNESS", 0.3),
            "BEZIER_SEGMENTS": getattr(config_cls, "BEZIER_SEGMENTS", 10),
        }

    def _get_working_grid_params(self) -> Tuple[float, Optional[float], float, float]:
        """
        计算膨胀参数。
        Returns: (xy_margin, cruise_z, z_margin_obs, z_margin_ceil)
        """
        shape = self.current_config.get("CRANE_FOOTPRINT_SHAPE", "box")
        width = float(self.current_config.get("CRANE_FOOTPRINT_WIDTH", 5.0))
        length = float(self.current_config.get("CRANE_FOOTPRINT_LENGTH", 5.0))
        height = float(self.current_config.get("CRANE_FOOTPRINT_HEIGHT", 2.0))

        if shape == "circle":
            radius_m = width / 2.0
        else:
            radius_m = math.hypot(width, length) / 2.0
        xy_margin = (radius_m / self.map_mgr.resolution_m) + 0.1

        user_z_margin = float(self.current_config["CRANE_Z_SAFETY_MARGIN"])

        # 1. 对障碍物/地板: 包含 SafetyMargin
        z_margin_obs = user_z_margin + (height / 2.0)

        # 2. 对天花板: 仅包含 HalfHeight (无 SafetyMargin)
        z_margin_ceil = height / 2.0

        cruise_z = None
        if (
            self.current_config["ENABLE_FIXED_HEIGHT_CRUISE"]
            and not self.current_config["OBSTACLE_INFINITE_HEIGHT"]
        ):
            cruise_z = float(self.current_config["CRANE_SAFE_TRAVEL_Z_M"])

        return xy_margin, cruise_z, z_margin_obs, z_margin_ceil

    def _init_components(self, force_rebuild: bool = False):
        cfg = self.current_config
        with self.grid_lock:
            if force_rebuild or self.map_mgr is None:
                self.logger.info(
                    f"[Init] 重建地图: {cfg['MAP_WIDTH_M']}x{cfg['MAP_LENGTH_M']}x{cfg['MAP_HEIGHT_M']}"
                )
                self.map_mgr = WorkshopMapManager(
                    width_m=float(cfg["MAP_WIDTH_M"]),
                    length_m=float(cfg["MAP_LENGTH_M"]),
                    height_m=float(cfg["MAP_HEIGHT_M"]),
                    resolution_m=float(cfg["MAP_RESOLUTION_M"]),
                    logger=self.logger.debug,
                )
                if force_rebuild:
                    self.map_mgr.add_static_obstacle(
                        "default_wall", 20, 15, 2, 10, z=100.0
                    )

            xy_margin, cruise_z, z_margin_obs, z_margin_ceil = (
                self._get_working_grid_params()
            )

            if cfg["ENABLE_FIXED_HEIGHT_CRUISE"]:
                grid = self.map_mgr.get_inflated_grid(xy_margin, cruise_z, z_margin_obs)
                planner_height_m = 0.0
            else:
                # 传递拆分后的 Z 余量
                grid = self.map_mgr.get_3d_inflated_grid(
                    xy_margin, z_margin_obs, z_margin_ceil
                )
                planner_height_m = self.map_mgr.height_m

            planner_type = str(cfg.get("PLANNER_ALGORITHM", "astar")).lower()
            common_args = {
                "grid": grid,
                "width_m": self.map_mgr.width_m,
                "length_m": self.map_mgr.length_m,
                "height_m": planner_height_m,
                "resolution": self.map_mgr.resolution_m,
                "logger": self.logger,
                "grid_lock": self.grid_lock,
                "use_octile_3d": cfg["USE_3D_OCTILE"],
            }

            if planner_type == "dslite":
                self.logger.info(f"[Init] 切换规划器 -> D* Lite")
                self.planner = DLitePlanner(**common_args)
            else:
                self.logger.info(
                    f"[Init] 切换规划器 -> A* (W={cfg.get('HEURISTIC_WEIGHT', 1.5)})"
                )
                self.planner = AStarPlanner(
                    **common_args,
                    heuristic_weight=float(cfg.get("HEURISTIC_WEIGHT", 1.5)),
                )

            self.post_processors = []
            if cfg["ENABLE_SHORTCUT_OPTIMIZATION"]:
                self.post_processors.append(GreedyShortcutProcessor())
            if cfg["ENABLE_BEZIER_SMOOTHING"]:
                self.post_processors.append(
                    BezierSmoothProcessor(
                        smoothness=float(cfg["BEZIER_SMOOTHNESS"]),
                        segments=int(cfg["BEZIER_SEGMENTS"]),
                    )
                )

    def update_configuration(self, new_settings: Dict[str, Any]) -> Tuple[bool, str]:
        try:
            rebuild_keys = [
                "MAP_WIDTH_M",
                "MAP_LENGTH_M",
                "MAP_HEIGHT_M",
                "MAP_RESOLUTION_M",
            ]
            needs_rebuild = False
            for k in rebuild_keys:
                if k in new_settings:
                    old_val = float(self.current_config.get(k, 0))
                    new_val = float(new_settings[k])
                    if abs(old_val - new_val) > 1e-6:
                        needs_rebuild = True
                        break

            old_static, old_dynamic = {}, {}
            if needs_rebuild and self.map_mgr:
                state = self.map_mgr.get_full_state()
                old_static = state.get("static_obstacles", {})
                old_dynamic = state.get("dynamic_obstacles", {})

            updated_count = 0
            for k, v in new_settings.items():
                if k in self.current_config:
                    try:
                        if isinstance(self.current_config[k], bool):
                            new_val = str(v).lower() == "true" or v is True
                        elif isinstance(self.current_config[k], int):
                            new_val = int(float(v))
                        elif isinstance(self.current_config[k], float):
                            new_val = float(v)
                        else:
                            new_val = v
                        if self.current_config[k] != new_val:
                            self.current_config[k] = new_val
                            updated_count += 1
                    except ValueError:
                        pass

            if updated_count == 0 and not needs_rebuild:
                return True, "配置未发生变化"
            self.logger.info(
                f"[Config] 更新 {updated_count} 项. Rebuild={needs_rebuild}"
            )
            self._init_components(force_rebuild=needs_rebuild)

            if needs_rebuild:
                new_w = self.current_config["MAP_WIDTH_M"]
                new_l = self.current_config["MAP_LENGTH_M"]
                for obs in old_static.values():
                    if obs["x_m"] < new_w and obs["y_m"] < new_l:
                        self.map_mgr.add_static_obstacle(
                            uuid.uuid4().hex,
                            obs["x_m"],
                            obs["y_m"],
                            obs["w_m"],
                            obs["h_m"],
                            obs.get("z_m", 100.0),
                        )
                for obs in old_dynamic.values():
                    if obs["x_m"] < new_w and obs["y_m"] < new_l:
                        self.map_mgr.update_dynamic_obstacle(
                            uuid.uuid4().hex,
                            obs["x_m"],
                            obs["y_m"],
                            obs["w_m"],
                            obs["h_m"],
                            obs.get("z_m", 100.0),
                        )
            return True, "配置更新成功"
        except Exception as e:
            self.logger.exception("配置更新失败")
            return False, str(e)

    # ... (update_mission_state, get_full_state, add/remove_obstacle 保持不变) ...
    def update_mission_state(self, data: Dict[str, Any]):
        if "start" in data:
            self.mission_state["start"] = data["start"]
        if "end" in data:
            self.mission_state["end"] = data["end"]

    def get_full_state(self) -> Dict[str, Any]:
        try:
            with self.grid_lock:
                state = (
                    self.map_mgr.get_full_state()
                    if self.map_mgr
                    else {
                        "width_m": 100,
                        "length_m": 100,
                        "height_m": 20,
                        "static_obstacles": {},
                        "dynamic_obstacles": {},
                    }
                )
                state["system_config"] = self.current_config
                state["mission_state"] = self.mission_state
                return state
        except Exception:
            return {}

    def add_obstacle(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        with self.grid_lock:
            try:
                if not self.map_mgr:
                    return False, "地图组件未就绪"
                oid = uuid.uuid4().hex[:8]
                default_h = float(
                    self.current_config.get("DEFAULT_OBSTACLE_HEIGHT_M", 2.0)
                )
                z_height = float(data.get("z", default_h))
                if data.get("type") == "static":
                    self.map_mgr.add_static_obstacle(
                        oid, data["x"], data["y"], data["w"], data["h"], z_height
                    )
                else:
                    self.map_mgr.update_dynamic_obstacle(
                        oid, data["x"], data["y"], data["w"], data["h"], z_height
                    )
                self._init_components(force_rebuild=False)
                return True, "Added"
            except Exception as e:
                return False, str(e)

    def remove_obstacle_near(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        with self.grid_lock:
            if not self.map_mgr:
                return False, "地图组件未就绪"
            res = self.map_mgr.find_obstacle_near(data["x"], data["y"])
            if res:
                oid, otype = res
                if otype == "static":
                    self.map_mgr.remove_static_obstacle(oid)
                else:
                    self.map_mgr.remove_dynamic_obstacle(oid)
                self._init_components(force_rebuild=False)
                return True, f"Removed {oid}"
            return False, "Not found"

    def plan_path(self) -> Tuple[Optional[List], Dict[str, Any], str]:
        with self.grid_lock:
            try:
                if not self.map_mgr or not self.planner:
                    return None, {}, "系统未完全初始化"
                t_start = time.perf_counter()
                stats_report = {
                    "pipeline_total_ms": 0.0,
                    "l1_planner": {},
                    "l1_5_processors": [],
                }
                s_cfg = self.mission_state["start"]
                e_cfg = self.mission_state["end"]

                map_w = self.map_mgr.width_m
                map_l = self.map_mgr.length_m
                if not (0 <= s_cfg["x"] <= map_w and 0 <= s_cfg["y"] <= map_l):
                    return None, {}, "起点超出边界"
                if not (0 <= e_cfg["x"] <= map_w and 0 <= e_cfg["y"] <= map_l):
                    return None, {}, "终点超出边界"

                is_fixed_height = self.current_config["ENABLE_FIXED_HEIGHT_CRUISE"]
                res = self.map_mgr.resolution_m

                def world_to_cont(wx, wy, wz=0):
                    row_idx = wy / res - 0.5
                    col_idx = wx / res - 0.5
                    if is_fixed_height:
                        return (row_idx, col_idx)
                    return (row_idx, col_idx, wz / res - 0.5)

                def cont_to_world(n):
                    row_val, col_val = n[0], n[1]
                    wx = (col_val + 0.5) * res
                    wy = (row_val + 0.5) * res
                    if len(n) == 3:
                        return (wx, wy, (n[2] + 0.5) * res)
                    return (wx, wy)

                prefix_path_world = []
                suffix_path_world = []
                _, _, safe_z_margin, _ = (
                    self._get_working_grid_params()
                )  # 这里的 safe_z_margin 是 z_margin_obs
                safe_z_threshold = safe_z_margin + 0.2

                if is_fixed_height:
                    start_node = self.map_mgr.world_to_grid(s_cfg["x"], s_cfg["y"])[:2]
                    goal_node = self.map_mgr.world_to_grid(e_cfg["x"], e_cfg["y"])[:2]
                else:
                    # Auto-Lift Start
                    start_z_raw = s_cfg.get("z", 1.0)
                    if start_z_raw < safe_z_threshold:
                        lifted_z = safe_z_threshold + res
                        check_node = self.map_mgr.world_to_grid(
                            s_cfg["x"], s_cfg["y"], lifted_z
                        )
                        if self.planner.is_obstacle(check_node):
                            return (
                                None,
                                {},
                                f"起点上方有障碍物，无法起升至 {lifted_z:.1f}m",
                            )
                        prefix_path_world.append((s_cfg["x"], s_cfg["y"], start_z_raw))
                        prefix_path_world.append((s_cfg["x"], s_cfg["y"], lifted_z))
                        start_node = check_node
                    else:
                        start_node = self.map_mgr.world_to_grid(
                            s_cfg["x"], s_cfg["y"], start_z_raw
                        )

                    # Auto-Drop End
                    end_z_raw = e_cfg.get("z", 1.0)
                    if end_z_raw < safe_z_threshold:
                        lifted_z = safe_z_threshold + res
                        check_node = self.map_mgr.world_to_grid(
                            e_cfg["x"], e_cfg["y"], lifted_z
                        )
                        if self.planner.is_obstacle(check_node):
                            return None, {}, f"终点上方有障碍物"
                        goal_node = check_node
                        suffix_path_world.append((e_cfg["x"], e_cfg["y"], lifted_z))
                        suffix_path_world.append((e_cfg["x"], e_cfg["y"], end_z_raw))
                    else:
                        goal_node = self.map_mgr.world_to_grid(
                            e_cfg["x"], e_cfg["y"], end_z_raw
                        )

                if not self.planner.initialize(start_node, goal_node):
                    return None, {}, "初始化失败：起点/终点被阻挡"
                grid_path = self.planner.compute_path(start_node)
                stats_report["l1_planner"] = self.planner.get_stats()
                if not grid_path:
                    return None, stats_report, "无法找到路径"

                path_for_smoothing = []
                for node in grid_path:
                    if is_fixed_height:
                        path_for_smoothing.append(
                            world_to_cont(
                                *(self.map_mgr.grid_to_world(node[0], node[1]))
                            )
                        )
                    else:
                        path_for_smoothing.append(
                            world_to_cont(
                                *(self.map_mgr.grid_to_world(node[0], node[1], node[2]))
                            )
                        )

                def continuous_is_safe(node):
                    return self.planner.is_safe(tuple(int(round(x)) for x in node))

                optimized_path_cont = path_for_smoothing
                for processor in self.post_processors:
                    optimized_path_cont = processor.process(
                        optimized_path_cont, continuous_is_safe
                    )
                    stats_report["l1_5_processors"].append(processor.get_stats())

                final_path_3d = []
                final_path_3d.extend(prefix_path_world)
                if is_fixed_height:
                    safe_z = float(self.current_config["CRANE_SAFE_TRAVEL_Z_M"])
                    final_path_3d = []
                    final_path_3d.append((s_cfg["x"], s_cfg["y"], 0.5))
                    final_path_3d.append((s_cfg["x"], s_cfg["y"], safe_z))
                    for node in optimized_path_cont:
                        pt = cont_to_world(node)
                        final_path_3d.append((pt[0], pt[1], safe_z))
                    final_path_3d.append((e_cfg["x"], e_cfg["y"], safe_z))
                    final_path_3d.append((e_cfg["x"], e_cfg["y"], 1.0))
                else:
                    for node in optimized_path_cont:
                        final_path_3d.append(cont_to_world(node))
                final_path_3d.extend(suffix_path_world)

                t_end = time.perf_counter()
                stats_report["pipeline_total_ms"] = (t_end - t_start) * 1000.0
                self.last_calculated_path = final_path_3d
                self.last_stats = stats_report
                self.logger.info(
                    f"[Plan] 成功. Nodes: {len(grid_path)} -> {len(optimized_path_cont)}"
                )
                return final_path_3d, stats_report, "Success"
            except Exception as e:
                self.logger.exception("规划异常")
                return None, {}, f"Internal Error: {str(e)}"
