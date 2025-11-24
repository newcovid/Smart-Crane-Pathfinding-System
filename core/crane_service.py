import logging
import threading
import uuid
import json
import math
from typing import Dict, Any, Optional, List, Tuple, Union

# 尝试导入 numpy，用于后续的数据清洗检查
# 作用: 算法层经常会输出 numpy 的数据类型 (int64, float32 等)，
# 这些类型直接传给 Flask 的 json.dumps 会报错，所以需要特殊清洗。
try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

# 导入核心组件
from .map_manager import WorkshopMapManager
from algorithms.trajectory_planner import TrajectoryPlanner


class CraneService:
    """
    【业务服务层 (Crane Service) - V3 稳定版】

    角色定位:
    这是整个后端系统的"大管家" (Facade)。
    它不直接负责算路，也不直接管地图数据，
    而是负责协调各方资源，处理前端请求，并保证数据传输的安全性和稳定性。

    主要改进:
    - [Refactor] 优化了 Logger 的传递方式。现在 MapManager 和 Planner 使用各自独立的 logger 命名空间，
      方便在控制台区分日志来源 (例如 [MapMgr] vs [TrajPlanner])。
    - [Data Safety] 强化了数据清洗逻辑，防止 NumPy 类型导致 SocketIO 断连。
    """

    def __init__(self, config: Any, logger: logging.Logger):
        """
        初始化服务。

        Args:
            config: 配置对象，包含系统参数。
            logger: 主程序的日志记录器。
        """
        self.logger = logger
        self.config_class = config

        # 1. 加载初始配置 (转为字典方便操作)
        self.current_config = self._load_initial_config(config)

        # 2. 初始化任务状态 (防止前端刚连上时是空的)
        self.mission_state = {
            "start": {"x": 5.0, "y": 5.0, "z": 5.0},
            "end": {"x": 30.0, "y": 20.0, "z": 5.0},
        }

        # 缓存: 保存最后一次计算结果，用于新客户端连接时的状态同步
        self.last_calculated_path: List[Tuple[float, float, float]] = []
        self.last_stats: Dict[str, Any] = {}

        self.logger.info(">>> [Service] 正在初始化核心组件...")

        # 3. 初始化地图管理器 (Map Manager)
        # [关键修改] 传入 logger=None。
        # 这样 MapManager 会自动创建名为 "MapManager" 的 logger。
        # 日志效果: [MapMgr] 地图初始化完成...
        self.map_mgr = WorkshopMapManager(
            width_m=float(self.current_config["MAP_WIDTH_M"]),
            length_m=float(self.current_config["MAP_LENGTH_M"]),
            height_m=float(self.current_config["MAP_HEIGHT_M"]),
            resolution_m=float(self.current_config["MAP_RESOLUTION_M"]),
            logger=None,
        )

        # 4. 初始化轨迹规划器 (Planner)
        # [关键修改] 传入 logger=None。
        # 这样 Planner 会自动创建名为 "TrajectoryPlanner" 的 logger。
        # 此外，传入 map_mgr._lock，确保规划器和地图管理器共用同一把锁。
        self.planner = TrajectoryPlanner(
            map_mgr=self.map_mgr,
            config=self.current_config,
            logger=None,
            grid_lock=self.map_mgr._lock,
        )

        self.logger.info(">>> [Service] CraneService 启动完成 (V3 Stable).")

    def _load_initial_config(self, config_cls) -> Dict[str, Any]:
        """从配置类中提取所有大写变量作为配置字典。"""
        cfg = {}
        for key in dir(config_cls):
            if key.isupper():  # 约定：只有大写变量才是配置项
                cfg[key] = getattr(config_cls, key)
        return cfg

    # =========================================================================
    # 数据清洗 (Data Sanitization)
    # =========================================================================

    def _sanitize_payload(self, data: Any) -> Any:
        """
        [关键方法] 递归清洗数据，确保可以被 JSON 序列化。

        背景:
        后端的算法库 (如 numpy, scipy) 经常产生特殊数据类型 (int64, float32)
        或者是无效数值 (NaN, Infinity)。
        如果把这些数据直接发给前端，Websocket 会直接断开连接，而且报错很难查。

        处理逻辑:
        1. NumPy 类型 -> 转换为 Python 原生 int/float。
        2. NaN/Inf -> 转换为 None (让前端去处理 None)。
        3. 字典/列表 -> 递归处理内部元素。
        """
        if data is None:
            return None

        # 1. 处理 NumPy 类型
        if HAS_NUMPY:
            if isinstance(data, (np.integer, np.int64, np.int32)):
                return int(data)
            if isinstance(data, (np.floating, np.float64, np.float32)):
                val = float(data)
                # 检查是否是 NaN (非数字) 或 Inf (无穷大)
                if math.isnan(val) or math.isinf(val):
                    return None
                return val
            if isinstance(data, np.ndarray):
                return self._sanitize_payload(data.tolist())

        # 2. 递归处理容器
        if isinstance(data, dict):
            return {k: self._sanitize_payload(v) for k, v in data.items()}
        if isinstance(data, list):
            return [self._sanitize_payload(item) for item in data]
        if isinstance(data, tuple):
            return [self._sanitize_payload(item) for item in data]

        # 3. 处理原生浮点数的特殊值
        if isinstance(data, float):
            if math.isnan(data) or math.isinf(data):
                return None
            return data

        return data

    # =========================================================================
    # 配置与状态管理
    # =========================================================================

    def get_full_state(self) -> Dict[str, Any]:
        """
        获取系统的全量状态快照。
        通常用于客户端刚连接时，一次性同步所有数据。
        """
        # 拿到底层地图数据
        base = self.map_mgr.get_full_state()

        # 拿到前端可视化用的网格 (通常是 2D 投影)
        # 注意：这里不传 3D 体素网格，因为数据量太大，会卡死浏览器
        base["active_inflated_grid"] = self.planner.visualization_grid

        base["system_config"] = self.current_config
        base["mission_state"] = self.mission_state

        # 必须清洗！确保万无一失
        return self._sanitize_payload(base)

    def update_configuration(self, new_settings: Dict[str, Any]) -> Tuple[bool, str]:
        """
        [热更新] 动态更新系统配置。

        逻辑:
        1. 对比新旧配置，找出变化项。
        2. 如果涉及"伤筋动骨"的地图尺寸变更 (Width/Length/Resolution)，则必须销毁并重建系统。
        3. 如果只是改改算法参数，则通知 Planner 内部刷新即可。
        """
        try:
            changes = {}
            for k, v in new_settings.items():
                if k in self.current_config:
                    # 强类型转换：前端传来的可能是字符串 "10.0"，需要转回 float
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
                return True, "配置未发生变化"

            self.logger.info(f"[Config] 检测到 {len(changes)} 项配置变更")

            # 检查是否涉及地图尺寸修改 (需要全量重建)
            dim_keys = [
                "MAP_WIDTH_M",
                "MAP_LENGTH_M",
                "MAP_HEIGHT_M",
                "MAP_RESOLUTION_M",
            ]
            if any(k in changes for k in dim_keys):
                self.logger.warning(
                    "[Config] 地图物理尺寸变更，正在执行系统重构 (Full Rebuild)..."
                )

                # 1. 备份当前的障碍物数据 (否则重建后障碍物会丢)
                with self.map_mgr._lock:
                    old_static = self.map_mgr.static_obstacles.copy()
                    old_dynamic = self.map_mgr.dynamic_obstacles.copy()

                # 2. 销毁并重建 MapManager
                # [Change] 同样传入 logger=None，保持一致性
                self.map_mgr = WorkshopMapManager(
                    width_m=float(self.current_config["MAP_WIDTH_M"]),
                    length_m=float(self.current_config["MAP_LENGTH_M"]),
                    height_m=float(self.current_config["MAP_HEIGHT_M"]),
                    resolution_m=float(self.current_config["MAP_RESOLUTION_M"]),
                    logger=None,
                )

                # 3. 恢复障碍物
                self.logger.info("[Config] 正在恢复障碍物数据...")
                for oid, o in old_static.items():
                    self.map_mgr.add_static_obstacle(
                        oid, o["x_m"], o["y_m"], o["w_m"], o["h_m"], o["z_m"]
                    )
                for oid, o in old_dynamic.items():
                    self.map_mgr.update_dynamic_obstacle(
                        oid, o["x_m"], o["y_m"], o["w_m"], o["h_m"], o["z_m"]
                    )

                # 4. 重建 Planner 并挂载新地图
                self.planner.map_mgr = self.map_mgr
                self.planner.grid_lock = self.map_mgr._lock
                self.planner._initialize_planner(force_rebuild=True)

                self.logger.info("[Config] 系统重构完成。")
            else:
                # 如果只是改参数，通知 Planner 更新即可
                self.planner.update_configuration(self.current_config)

            return True, "配置更新成功"
        except Exception as e:
            self.logger.exception("[Config] 更新过程中发生严重错误")
            return False, str(e)

    def update_mission_state(self, data: Dict[str, Any]):
        """更新任务状态 (起点/终点坐标)"""
        if "start" in data:
            self.mission_state["start"].update(data["start"])
        if "end" in data:
            self.mission_state["end"].update(data["end"])

    # =========================================================================
    # 障碍物操作
    # =========================================================================

    def add_obstacle(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """
        添加障碍物。

        Args:
            data: 包含 x, y, w, h, type 等信息的字典。
        """
        try:
            oid = uuid.uuid4().hex[:8]
            x, y, w, h = (
                float(data["x"]),
                float(data["y"]),
                float(data["w"]),
                float(data["h"]),
            )
            z = float(data.get("z", self.current_config["DEFAULT_OBSTACLE_HEIGHT_M"]))

            self.logger.info(
                f"[Obstacle] 添加请求: {data.get('type')} at ({x},{y}), Size: {w}x{h}"
            )

            if data.get("type") == "static":
                self.map_mgr.add_static_obstacle(oid, x, y, w, h, z)
            else:
                self.map_mgr.update_dynamic_obstacle(oid, x, y, w, h, z)

            # 通知 Planner 地图变了 (可能会触发 D* Lite 的增量更新)
            self.planner.handle_obstacle_update(x, y, w, h, z, is_add=True)
            return True, f"成功添加 {oid}"
        except Exception as e:
            self.logger.error(f"[Obstacle] 添加失败: {e}")
            return False, str(e)

    def remove_obstacle_near(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """移除指定坐标附近的障碍物。"""
        # 1. 先查表，看看这里有没有障碍物
        res = self.map_mgr.find_obstacle_near(data["x"], data["y"])
        if not res:
            self.logger.warning(
                f"[Obstacle] 移除失败: ({data['x']},{data['y']}) 附近无障碍物。"
            )
            return False, "未找到目标"

        oid, otype = res
        obs = self.map_mgr.static_obstacles.get(
            oid
        ) or self.map_mgr.dynamic_obstacles.get(oid)

        if not obs:
            return False, "数据一致性错误 (ID存在但实体丢失)"

        self.logger.info(f"[Obstacle] 正在移除 {otype} 障碍物: {oid}")

        if otype == "static":
            self.map_mgr.remove_static_obstacle(oid)
        else:
            self.map_mgr.remove_dynamic_obstacle(oid)

        # 通知 Planner 处理移除后的逻辑
        self.planner.handle_obstacle_update(
            obs["x_m"],
            obs["y_m"],
            obs["w_m"],
            obs["h_m"],
            obs.get("z_m", 0),
            is_add=False,
        )
        return True, f"成功移除 {oid}"

    # =========================================================================
    # 核心规划接口
    # =========================================================================

    def plan_path(self) -> Tuple[Optional[List], Dict[str, Any], str]:
        """
        [核心] 执行路径规划。

        Returns:
            (路径点列表, 统计数据, 状态消息)
        """
        # 调用核心规划器
        path, stats, msg = self.planner.plan(
            self.mission_state["start"], self.mission_state["end"]
        )

        # 缓存结果
        self.last_calculated_path = path
        self.last_stats = stats

        # [CRITICAL] 数据清洗 (Sanitization)
        # 必须把数据洗干净再发给前端，否则 SocketIO 会崩
        safe_path = self._sanitize_payload(path)
        safe_stats = self._sanitize_payload(stats)

        # 打印结果日志
        path_len = len(safe_path) if safe_path else 0
        status_tag = "成功" if safe_path else "失败"
        log_level = logging.INFO if safe_path else logging.WARNING

        self.logger.log(
            log_level, f"[Plan] 规划{status_tag}. 路径节点数: {path_len}. 消息: {msg}"
        )

        return safe_path, safe_stats, msg
