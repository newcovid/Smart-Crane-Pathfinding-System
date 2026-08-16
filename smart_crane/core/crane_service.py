import logging
import uuid
import math
from typing import Dict, Any, Optional, List, Tuple

try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

from smart_crane.core.config import Settings
from smart_crane.core.map_manager import WorkshopMapManager
from smart_crane.core.rust_bridge import RustBackend
from smart_crane.algorithms.trajectory_planner import TrajectoryPlanner
from smart_crane.core.constants import (
    MSG_CONFIG_UPDATE,
    MSG_CONFIG_NO_CHANGE,
    MSG_REBUILD_MAP,
    MSG_OBS_ADDED,
    MSG_OBS_REMOVED,
    MSG_OBS_NOT_FOUND,
    TYPE_STATIC,
)
from smart_crane.core.constants import (
    DEFAULT_MISSION_START_X,
    DEFAULT_MISSION_START_Y,
    DEFAULT_MISSION_START_Z,
    DEFAULT_MISSION_END_X,
    DEFAULT_MISSION_END_Y,
    DEFAULT_MISSION_END_Z,
    UUID_ID_LEN,
    COORD_PRINT_PREC,
)


class CraneService:
    """业务服务层 (Crane Service)。

    作为系统的核心控制器，负责协调配置管理、地图状态维护、任务调度以及路径规划请求。
    它充当 Web 接口 (Flask/SocketIO) 与底层算法 (Planner/MapManager) 之间的桥梁。

    Attributes:
        settings (Settings): 全局配置对象。
        logger (logging.Logger): 专用的日志记录器。
        map_mgr (WorkshopMapManager): 地图管理器实例。
        planner (TrajectoryPlanner): 轨迹规划器实例。
        mission_state (Dict[str, Any]): 当前任务状态（起点、终点）。
    """

    def __init__(self, settings: Settings, logger: Optional[logging.Logger] = None):
        """初始化起重机服务。

        Args:
            settings (Settings): 全局配置对象。
            logger (Optional[logging.Logger]): 父级日志记录器。
        """
        # [日志优化] 建立日志层级
        if logger:
            self.logger = logger.getChild(self.__class__.__name__)
        else:
            self.logger = logging.getLogger(self.__class__.__name__)

        self.settings = settings

        # 把配置里的 ENABLE_RUST_CORE 落到全局后端开关上。
        # 必须在构造 MapManager / Planner 之前执行——它们在初始化时就会
        # 询问 RustBackend.is_available()。此前这个开关只传给了规划器，
        # 地图生成、C-Space 膨胀、脱困、后处理都绕过了它。
        RustBackend.set_enabled(settings.planner.enable_rust_core)

        # 初始化任务状态（使用常量避免魔法数字）
        self.mission_state = {
            "start": {
                "x": DEFAULT_MISSION_START_X,
                "y": DEFAULT_MISSION_START_Y,
                "z": DEFAULT_MISSION_START_Z,
            },
            "end": {
                "x": DEFAULT_MISSION_END_X,
                "y": DEFAULT_MISSION_END_Y,
                "z": DEFAULT_MISSION_END_Z,
            },
        }

        self.last_calculated_path: List[Tuple[float, float, float]] = []
        self.last_stats: Dict[str, Any] = {}

        self.logger.info("正在初始化核心组件...")

        # [依赖注入] 将 Logger 向下传递，建立完整的日志链
        # 结果: Parent.CraneService.MapManager
        self.map_mgr = WorkshopMapManager(
            width_m=self.settings.map.width_m,
            length_m=self.settings.map.length_m,
            height_m=self.settings.map.height_m,
            resolution_m=self.settings.map.resolution_m,
            logger=self.logger,
        )

        # 结果: Parent.CraneService.TrajectoryPlanner
        self.planner = TrajectoryPlanner(
            map_mgr=self.map_mgr,
            settings=self.settings,
            logger=self.logger,
            grid_lock=self.map_mgr.lock,  # 共享地图锁
        )

        self.logger.info("业务服务层 启动完成。")

    def _sanitize_payload(self, data: Any) -> Any:
        """数据清洗，处理 NumPy 类型和非数值 (NaN/Inf)。

        确保数据可以被 JSON 序列化传输给前端。

        Args:
            data (Any): 原始数据。

        Returns:
            Any: 清洗后的数据。
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

    def get_full_state(self) -> Dict[str, Any]:
        """获取系统全量状态快照。

        Returns:
            Dict[str, Any]: 包含地图信息、障碍物、配置、任务状态和可视化的字典。
        """
        base = self.map_mgr.get_full_state()
        base["active_inflated_grid"] = self.planner.visualization_grid
        base["system_config"] = self.settings.to_flat_dict()
        base["mission_state"] = self.mission_state
        return self._sanitize_payload(base)

    def update_configuration(
        self, new_settings_dict: Dict[str, Any]
    ) -> Tuple[bool, str]:
        """[热更新] 动态更新系统配置。

        如果地图物理尺寸发生变化，将触发系统全量重构 (Full Rebuild)。

        Args:
            new_settings_dict (Dict[str, Any]): 新的配置字典。

        Returns:
            Tuple[bool, str]: (是否成功, 消息)。
        """
        try:
            # 1. 记录旧的地图关键参数
            old_map_dims = (
                self.settings.map.width_m,
                self.settings.map.length_m,
                self.settings.map.height_m,
                self.settings.map.resolution_m,
            )

            # 2. 更新 Settings 对象。
            #    分节已开启 validate_assignment，非法值会抛 ValidationError，
            #    由本方法末尾的 except 兜住并作为失败结果返回给客户端。
            old_dump = self.settings.model_dump()
            unknown_keys = self.settings.update_from_dict(new_settings_dict)
            new_dump = self.settings.model_dump()

            if unknown_keys:
                # 不静默丢弃：前端拼错键名必须能看到，否则会以为设置生效了。
                return False, f"存在未知配置项: {', '.join(sorted(unknown_keys))}"

            if old_dump == new_dump:
                return True, MSG_CONFIG_NO_CHANGE

            self.logger.info("配置已更新")

            # 配置可能改动了 ENABLE_RUST_CORE，同步到全局后端开关
            RustBackend.set_enabled(self.settings.planner.enable_rust_core)

            # 3. 检查是否需要重建地图
            new_map_dims = (
                self.settings.map.width_m,
                self.settings.map.length_m,
                self.settings.map.height_m,
                self.settings.map.resolution_m,
            )

            if old_map_dims != new_map_dims:
                self.logger.warning(MSG_REBUILD_MAP)

                # 备份障碍物数据
                with self.map_mgr.lock:
                    old_static = self.map_mgr.static_obstacles.copy()
                    old_dynamic = self.map_mgr.dynamic_obstacles.copy()

                # 重建 MapManager（沿用同一把锁，见下方 adopt_lock）
                previous_lock = self.map_mgr.lock
                self.map_mgr = WorkshopMapManager(
                    width_m=self.settings.map.width_m,
                    length_m=self.settings.map.length_m,
                    height_m=self.settings.map.height_m,
                    resolution_m=self.settings.map.resolution_m,
                    logger=self.logger,  # 传递 logger
                )

                # 恢复数据
                self.logger.info("正在恢复障碍物数据...")
                for oid, o in old_static.items():
                    self.map_mgr.add_static_obstacle(
                        oid, o["x_m"], o["y_m"], o["w_m"], o["h_m"], o["z_m"]
                    )
                for oid, o in old_dynamic.items():
                    self.map_mgr.update_dynamic_obstacle(
                        oid, o["x_m"], o["y_m"], o["w_m"], o["h_m"], o["z_m"]
                    )

                # 重建 Planner 并重新绑定
                self.planner.map_mgr = self.map_mgr
                # 注意：不能把 planner 的锁替换成新 MapManager 的锁。
                # 若此刻另一个事件正持有旧锁在 plan() 中执行，替换之后
                # 新请求拿到的是另一把锁，互斥即告失效。
                # 这里改为把旧锁沿用到新的 MapManager 上，全程只有一把锁。
                self.map_mgr.adopt_lock(previous_lock)
                self.planner.grid_adapter.map_mgr = self.map_mgr  # 关键：更新适配器引用
                self.planner.safety_guard.map_mgr = self.map_mgr  # 关键：更新守卫引用
                self.planner.initialize_planner()

                self.logger.info("系统重构完成。")
            else:
                # 仅更新 Planner 配置 (如算法参数)
                self.planner.update_configuration()

            return True, MSG_CONFIG_UPDATE
        except Exception as e:
            self.logger.exception("更新过程中发生严重错误")
            return False, str(e)

    def update_mission_state(self, data: Dict[str, Any]):
        """更新任务坐标状态。"""
        if "start" in data:
            self.mission_state["start"].update(data["start"])
        if "end" in data:
            self.mission_state["end"].update(data["end"])

    def add_obstacle(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """添加障碍物。

        Args:
            data (Dict[str, Any]): 障碍物参数 {type, x, y, w, h, z}。

        Returns:
            Tuple[bool, str]: (是否成功, 消息)。
        """
        try:
            oid = uuid.uuid4().hex[:UUID_ID_LEN]
            x, y, w, h = (
                float(data["x"]),
                float(data["y"]),
                float(data["w"]),
                float(data["h"]),
            )
            # 使用配置中的默认高度
            z = float(data.get("z", self.settings.crane.default_obstacle_height))

            self.logger.info(
                f"添加障碍物请求: {data.get('type')} 位于 ({x:.{COORD_PRINT_PREC}f},{y:.{COORD_PRINT_PREC}f})，尺寸: {w:.{COORD_PRINT_PREC}f}x{h:.{COORD_PRINT_PREC}f}"
            )

            if data.get("type") == TYPE_STATIC:
                self.map_mgr.add_static_obstacle(oid, x, y, w, h, z)
            else:
                self.map_mgr.update_dynamic_obstacle(oid, x, y, w, h, z)

            # 通知规划器处理增量更新
            self.planner.handle_obstacle_update(x, y, w, h, z, is_add=True)
            return True, MSG_OBS_ADDED.format(id=oid)
        except Exception as e:
            self.logger.error(f"障碍物添加失败: {e}")
            return False, str(e)

    def remove_obstacle_near(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """移除指定坐标附近的障碍物。

        Args:
            data (Dict[str, Any]): 包含点击位置 {x, y}。

        Returns:
            Tuple[bool, str]: (是否成功, 消息)。
        """
        if "x" not in data or "y" not in data:
            # 这里原本直接 data["x"]，且整个方法没有 try/except。
            # 缺字段会抛 KeyError 被 Flask-SocketIO 吞掉，客户端既收不到
            # operation_success 也收不到 operation_failed，界面就那么卡住。
            return False, "请求缺少必需字段 x / y"

        # 查找与读取必须在同一把锁内完成，否则是典型的 TOCTOU：
        # find_obstacle_near 自己加锁返回 id 后释放锁，此刻另一个事件
        # 可能已经把该障碍物删掉了。
        with self.map_mgr.lock:
            res = self.map_mgr.find_obstacle_near(data["x"], data["y"])
            if not res:
                return False, MSG_OBS_NOT_FOUND

            oid, otype = res
            obs = self.map_mgr.static_obstacles.get(
                oid
            ) or self.map_mgr.dynamic_obstacles.get(oid)

            if not obs:
                return False, "数据一致性错误"
            obs = dict(obs)  # 出锁前拷贝一份，后续读取不再依赖共享状态

        self.logger.info(f"正在移除 {otype} 障碍物: {oid}")

        if otype == TYPE_STATIC:
            self.map_mgr.remove_static_obstacle(oid)
        else:
            self.map_mgr.remove_dynamic_obstacle(oid)

        # 通知规划器处理增量更新
        self.planner.handle_obstacle_update(
            obs["x_m"],
            obs["y_m"],
            obs["w_m"],
            obs["h_m"],
            obs.get("z_m", 0),
            is_add=False,
        )
        return True, MSG_OBS_REMOVED.format(id=oid)

    def plan_path(self) -> Tuple[Optional[List], Dict[str, Any], str]:
        """触发路径规划。

        Returns:
            Tuple: (路径列表, 统计信息, 消息)。
        """
        path, stats, msg = self.planner.plan(
            self.mission_state["start"], self.mission_state["end"]
        )
        self.last_calculated_path = path
        self.last_stats = stats
        return self._sanitize_payload(path), self._sanitize_payload(stats), msg
