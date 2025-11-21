import logging
import threading
import uuid
import json
import math
from typing import Dict, Any, Optional, List, Tuple

# 尝试导入 numpy 用于类型检查
try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

from .map_manager import WorkshopMapManager
from algorithms.trajectory_planner import TrajectoryPlanner


class CraneService:
    """
    【业务服务层 (Service Layer) - V3 Stable】

    Fixes:
    1. [Critical] 增加数据清洗 (Sanitization)，防止 numpy 类型或 NaN 导致 SocketIO 序列化崩溃。
    2. 修复了修改配置后障碍物丢失的问题。
    3. 确保 visualization_grid 为 2D 投影，保护前端性能。
    """

    def __init__(self, config: Any, logger: logging.Logger):
        self.logger = logger
        self.config_class = config
        self.current_config = self._load_initial_config(config)

        self.mission_state = {
            "start": {"x": 5.0, "y": 5.0, "z": 5.0},
            "end": {"x": 30.0, "y": 20.0, "z": 5.0},
        }

        self.last_calculated_path = []
        self.last_stats = {}

        # 初始化地图与规划器
        self.map_mgr = WorkshopMapManager(
            width_m=float(self.current_config["MAP_WIDTH_M"]),
            length_m=float(self.current_config["MAP_LENGTH_M"]),
            height_m=float(self.current_config["MAP_HEIGHT_M"]),
            resolution_m=float(self.current_config["MAP_RESOLUTION_M"]),
            logger=self.logger.debug,
        )

        self.planner = TrajectoryPlanner(
            map_mgr=self.map_mgr,
            config=self.current_config,
            logger=self.logger,
            grid_lock=self.map_mgr._lock,
        )

        self.logger.info(">>> CraneService (V3 Stable) Ready.")

    def _load_initial_config(self, config_cls) -> Dict[str, Any]:
        cfg = {}
        for key in dir(config_cls):
            if key.isupper():
                cfg[key] = getattr(config_cls, key)
        return cfg

    # --- [核心修复] 数据清洗 ---

    def _sanitize_payload(self, data: Any) -> Any:
        """
        递归清洗数据，确保可以被 JSON 序列化。
        处理: numpy类型 -> 原生类型, NaN/Inf -> None
        """
        if data is None:
            return None

        if HAS_NUMPY:
            if isinstance(data, (np.integer, np.int64, np.int32)):
                return int(data)
            if isinstance(data, (np.floating, np.float64, np.float32)):
                val = float(data)
                if math.isnan(val) or math.isinf(val):
                    return None
                return val
            if isinstance(data, np.ndarray):
                return self._sanitize_payload(data.tolist())

        if isinstance(data, dict):
            return {k: self._sanitize_payload(v) for k, v in data.items()}
        if isinstance(data, list):
            return [self._sanitize_payload(item) for item in data]
        if isinstance(data, tuple):
            return [self._sanitize_payload(item) for item in data]
        if isinstance(data, float):
            if math.isnan(data) or math.isinf(data):
                return None
            return data

        return data

    # --- 配置与状态 ---

    def get_full_state(self) -> Dict[str, Any]:
        base = self.map_mgr.get_full_state()
        # 确保 active_inflated_grid 是 2D 投影，避免 3D 数据传输卡死
        base["active_inflated_grid"] = self.planner.visualization_grid
        base["system_config"] = self.current_config
        base["mission_state"] = self.mission_state
        # 清洗整个状态包
        return self._sanitize_payload(base)

    def update_configuration(self, new_settings: Dict[str, Any]) -> Tuple[bool, str]:
        try:
            changes = {}
            for k, v in new_settings.items():
                if k in self.current_config:
                    # 基础类型转换
                    target_type = type(self.current_config[k])
                    if target_type == bool:
                        val = str(v).lower() == "true" or v is True
                    elif target_type == int:
                        val = int(float(v))
                    elif target_type == float:
                        val = float(v)
                    else:
                        val = v

                    if self.current_config[k] != val:
                        self.current_config[k] = val
                        changes[k] = val

            if not changes:
                return True, "No changes"

            # 检查地图尺寸变化 (需要重建)
            dim_keys = [
                "MAP_WIDTH_M",
                "MAP_LENGTH_M",
                "MAP_HEIGHT_M",
                "MAP_RESOLUTION_M",
            ]
            if any(k in changes for k in dim_keys):
                self.logger.warning("Map dimensions changed, rebuilding map system...")

                # 1. 备份障碍物
                with self.map_mgr._lock:
                    old_static = self.map_mgr.static_obstacles.copy()
                    old_dynamic = self.map_mgr.dynamic_obstacles.copy()

                # 2. 重建 MapManager
                self.map_mgr = WorkshopMapManager(
                    width_m=float(self.current_config["MAP_WIDTH_M"]),
                    length_m=float(self.current_config["MAP_LENGTH_M"]),
                    height_m=float(self.current_config["MAP_HEIGHT_M"]),
                    resolution_m=float(self.current_config["MAP_RESOLUTION_M"]),
                    logger=self.logger.debug,
                )

                # 3. 恢复障碍物
                for oid, o in old_static.items():
                    self.map_mgr.add_static_obstacle(
                        oid, o["x_m"], o["y_m"], o["w_m"], o["h_m"], o["z_m"]
                    )
                for oid, o in old_dynamic.items():
                    self.map_mgr.update_dynamic_obstacle(
                        oid, o["x_m"], o["y_m"], o["w_m"], o["h_m"], o["z_m"]
                    )

                # 4. 重连 Planner 并强制重建
                self.planner.map_mgr = self.map_mgr
                self.planner.grid_lock = self.map_mgr._lock
                self.planner._initialize_planner(force_rebuild=True)
            else:
                # 仅更新 Planner 配置
                self.planner.update_configuration(self.current_config)

            return True, "Configuration updated"
        except Exception as e:
            self.logger.exception("Config update failed")
            return False, str(e)

    def update_mission_state(self, data: Dict[str, Any]):
        if "start" in data:
            self.mission_state["start"].update(data["start"])
        if "end" in data:
            self.mission_state["end"].update(data["end"])

    # --- 障碍物操作 ---

    def add_obstacle(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        try:
            oid = uuid.uuid4().hex[:8]
            x, y, w, h = (
                float(data["x"]),
                float(data["y"]),
                float(data["w"]),
                float(data["h"]),
            )
            z = float(data.get("z", self.current_config["DEFAULT_OBSTACLE_HEIGHT_M"]))

            if data.get("type") == "static":
                self.map_mgr.add_static_obstacle(oid, x, y, w, h, z)
            else:
                self.map_mgr.update_dynamic_obstacle(oid, x, y, w, h, z)

            self.planner.handle_obstacle_update(x, y, w, h, z, is_add=True)
            return True, "Obstacle added"
        except Exception as e:
            return False, str(e)

    def remove_obstacle_near(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        res = self.map_mgr.find_obstacle_near(data["x"], data["y"])
        if not res:
            return False, "Not found"

        oid, otype = res
        obs = self.map_mgr.static_obstacles.get(
            oid
        ) or self.map_mgr.dynamic_obstacles.get(oid)
        if not obs:
            return False, "Error"

        if otype == "static":
            self.map_mgr.remove_static_obstacle(oid)
        else:
            self.map_mgr.remove_dynamic_obstacle(oid)

        self.planner.handle_obstacle_update(
            obs["x_m"],
            obs["y_m"],
            obs["w_m"],
            obs["h_m"],
            obs.get("z_m", 0),
            is_add=False,
        )
        return True, f"Removed {oid}"

    # --- 规划 ---

    def plan_path(self) -> Tuple[Optional[List], Dict[str, Any], str]:
        path, stats, msg = self.planner.plan(
            self.mission_state["start"], self.mission_state["end"]
        )
        self.last_calculated_path = path
        self.last_stats = stats

        # [CRITICAL] 清洗数据，防止 Numpy 类型导致 Socket 发送失败
        safe_path = self._sanitize_payload(path)
        safe_stats = self._sanitize_payload(stats)

        # 增加调试日志，确认数据量级
        path_len = len(safe_path) if safe_path else 0
        self.logger.info(
            f"[Service] Plan finished. Path nodes: {path_len}. Safe payload generated."
        )

        return safe_path, safe_stats, msg
