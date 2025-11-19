import uuid
import copy
from typing import Dict, Any, Optional, List, Tuple
from logging import Logger

# 导入底层算法模块
# 确保 smart_crane_pathfinding.py 在同一目录下或 PYTHONPATH 中
from smart_crane_pathfinding import WorkshopMapManager, IntelligentCranePlanner


class CraneService:
    """
    起重机业务服务层 (Crane Service Layer)

    === 核心职责 ===
    1. **状态管理 (State Management)**: 维护全局唯一的车间地图状态 (WorkshopMapManager) 和规划器实例。
    2. **配置热重载 (Hot Reload)**: 支持在不重启服务的情况下动态调整地图尺寸、分辨率及算法参数。
    3. **数据持久化 (Data Persistence)**:
       - 智能识别配置变更类型。
       - 在调整算法参数时，复用现有地图实例，零成本保留数据。
       - 在调整地图尺寸时，自动迁移合法范围内的障碍物。
    4. **异常熔断 (Error Handling)**: 捕获底层算法异常，将其转换为对前端友好的错误消息。
    5. **任务状态同步 (Mission Sync)**: [新增] 统一管理起点/终点坐标，确保多端协同(Multi-Client)时状态一致。

    === 线程安全说明 ===
    本类持有的 map_mgr (WorkshopMapManager) 内部实现了 threading.RLock，
    因此在 Flask 多线程环境下调用 add_obstacle/plan_path 等方法是安全的。
    """

    def __init__(self, config: Any, logger: Logger):
        """
        初始化起重机服务。

        Args:
            config: 配置对象，通常是 Flask 的 app.config 或 Config 类实例。
                    必须包含 MAP_WIDTH_M, MAP_LENGTH_M 等键值。
            logger: 应用级日志记录器，用于追踪系统运行状态。
        """
        self.logger = logger
        self.logger.info(">>> [系统启动] 正在初始化 CraneService 业务服务层...")

        # [配置快照] 保存当前的配置状态字典。
        # 作用：不仅用于初始化，更用于后续 update_configuration 时进行"新旧值比对" (Diff)，
        # 从而决定是执行昂贵的"重建"操作，还是轻量的"参数更新"操作。
        self.current_config = {
            "MAP_WIDTH_M": getattr(config, "MAP_WIDTH_M", 100.0),
            "MAP_LENGTH_M": getattr(config, "MAP_LENGTH_M", 100.0),
            "MAP_RESOLUTION_M": getattr(config, "MAP_RESOLUTION_M", 0.5),
            "CRANE_FOOTPRINT_M": getattr(config, "CRANE_FOOTPRINT_M", 5.0),
            "CRANE_SAFE_TRAVEL_Z_M": getattr(config, "CRANE_SAFE_TRAVEL_Z_M", 8.0),
            "ENABLE_SHORTCUT_OPTIMIZATION": getattr(
                config, "ENABLE_SHORTCUT_OPTIMIZATION", True
            ),
            "ENABLE_BEZIER_SMOOTHING": getattr(config, "ENABLE_BEZIER_SMOOTHING", True),
            "BEZIER_SMOOTHNESS": getattr(config, "BEZIER_SMOOTHNESS", 0.3),
            "BEZIER_SEGMENTS": getattr(config, "BEZIER_SEGMENTS", 10),
        }

        # 显式声明类型，便于 IDE 智能提示
        self.map_mgr: Optional[WorkshopMapManager] = None
        self.planner: Optional[IntelligentCranePlanner] = None

        # [新增] 任务协同状态 (Mission State)
        # 用于在后端存储当前的起点和终点，解决多浏览器打开时坐标不同步的 Bug。
        # 默认给一个初始位置，避免前端 undefined
        self.mission_state = {
            "start": {"x": 10.0, "y": 10.0},
            "end": {"x": 40.0, "y": 30.0},
        }

        # 初始化核心组件 (强制首次构建)
        self._init_components(rebuild_map=True)

        # 初始化默认场景
        self._init_default_scene()

        self.logger.info(">>> [系统启动] CraneService 初始化完成，核心组件就绪。")

    def _init_components(self, rebuild_map: bool = True) -> None:
        """
        [内部方法] 初始化或更新核心组件 (MapManager 和 Planner)。

        Args:
            rebuild_map (bool):
                - True: 强制销毁旧的 MapManager 并创建新实例。这通常会导致数据丢失，
                  因此调用前必须手动备份障碍物数据（参见 update_configuration）。
                  适用于：地图物理尺寸、分辨率发生变化的场景。
                - False: 复用现有的 MapManager 实例，仅重新初始化 Planner。
                  适用于：仅修改起重机尺寸、算法开关、贝塞尔参数的场景。
        """
        cfg = self.current_config

        # 1. 初始化地图管理器 (WorkshopMapManager)
        # 它是数字孪生环境的核心，负责维护网格和障碍物
        if rebuild_map or self.map_mgr is None:
            self.logger.debug(
                f"[组件构建] 正在重建 MapManager (W={cfg['MAP_WIDTH_M']}, L={cfg['MAP_LENGTH_M']})..."
            )
            self.map_mgr = WorkshopMapManager(
                width_m=float(cfg["MAP_WIDTH_M"]),
                length_m=float(cfg["MAP_LENGTH_M"]),
                resolution_m=float(cfg["MAP_RESOLUTION_M"]),
                logger=self.logger.debug,  # 委托日志输出
            )
        else:
            self.logger.debug("[组件构建] 复用现有 MapManager (地图结构未变更)")

        # 2. 初始化智能规划器 (IntelligentCranePlanner)
        # 规划器是轻量级的，且直接依赖最新的配置参数 (如 ENABLE_BEZIER)，
        # 因此我们总是重新创建一个新的 Planner 实例，并注入 (新或旧的) map_mgr。
        self.planner = IntelligentCranePlanner(
            map_manager=self.map_mgr,
            crane_footprint_m=float(cfg["CRANE_FOOTPRINT_M"]),
            safe_travel_z_m=float(cfg["CRANE_SAFE_TRAVEL_Z_M"]),
            enable_shortcut=bool(cfg["ENABLE_SHORTCUT_OPTIMIZATION"]),
            enable_bezier=bool(cfg["ENABLE_BEZIER_SMOOTHING"]),
            bezier_smoothness=float(cfg["BEZIER_SMOOTHNESS"]),
            bezier_segments=int(cfg["BEZIER_SEGMENTS"]),
            logger=self.logger.debug,
        )

    def _init_default_scene(self) -> None:
        """[内部方法] 加载默认演示场景数据。"""
        # 仅在第一次启动时添加，避免每次重启都添加重复数据
        # 如果需要持久化，建议此处改为从数据库读取
        self.map_mgr.add_static_obstacle("cnc_machine_default", 20.0, 18.0, 3.0, 6.0)
        self.logger.info("[场景加载] 已注入默认演示障碍物。")

    def _migrate_obstacles(self, old_static: Dict, old_dynamic: Dict) -> int:
        """
        [内部方法] 障碍物数据迁移。
        当地图尺寸发生剧烈变化导致必须重建 MapManager 时，
        此方法尝试将旧地图中的障碍物"搬运"到新地图中。

        Args:
            old_static: 旧的静态障碍物字典
            old_dynamic: 旧的动态障碍物字典

        Returns:
            int: 成功保留的障碍物数量
        """
        count = 0

        # 定义一个内部辅助函数来处理迁移逻辑
        def _try_migrate(obs_dict, add_func):
            nonlocal count
            for oid, obs in obs_dict.items():
                # 边界检查：如果新地图变小了，障碍物可能现在位于地图外
                # 我们只保留还在新地图范围内的障碍物
                if (obs["x_m"] + obs["w_m"] <= self.map_mgr.width_m) and (
                    obs["y_m"] + obs["h_m"] <= self.map_mgr.length_m
                ):
                    add_func(oid, obs["x_m"], obs["y_m"], obs["w_m"], obs["h_m"])
                    count += 1
                else:
                    self.logger.warning(
                        f"[数据迁移] 障碍物 {oid} 超出新地图边界，已被自动丢弃。"
                    )

        _try_migrate(old_static, self.map_mgr.add_static_obstacle)
        _try_migrate(old_dynamic, self.map_mgr.update_dynamic_obstacle)

        return count

    def update_configuration(self, new_settings: Dict[str, Any]) -> Tuple[bool, str]:
        """
        [核心方法] 动态更新系统配置（支持热重载）。

        逻辑流程：
        1. 接收前端传来的新配置字典。
        2. 智能比对 (Diff)：检查是否修改了影响地图结构的参数 (长/宽/分辨率)。
        3. 决策：
           - 情况 A (结构变更): 备份数据 -> 重建地图 -> 恢复数据。
           - 情况 B (参数微调): 直接复用地图 -> 仅更新规划器参数。

        Args:
            new_settings: 包含新配置项的字典。

        Returns:
            (bool, str): (操作是否成功, 结果描述消息)
        """
        try:
            self.logger.info(f"[配置更新] 收到请求: {new_settings}")

            # 1. 识别哪些参数会触发“地图重建”
            # 只有影响 Grid 物理维度的参数才需要重建 MapManager
            rebuild_keys = {"MAP_WIDTH_M", "MAP_LENGTH_M", "MAP_RESOLUTION_M"}

            needs_rebuild = False
            for key in rebuild_keys:
                # 检查 key 是否存在 且 值是否真的发生了变化 (浮点数比较)
                if key in new_settings:
                    old_val = float(self.current_config.get(key, 0))
                    new_val = float(new_settings[key])
                    if abs(old_val - new_val) > 1e-6:  # 使用 epsilon 比较浮点数
                        needs_rebuild = True
                        self.logger.info(
                            f"[配置检测] 参数 {key} 变更: {old_val} -> {new_val}，触发地图重建。"
                        )
                        break

            # 2. 备份旧数据 (仅当需要重建时才需要备份)
            old_static_obs = {}
            old_dynamic_obs = {}
            if needs_rebuild:
                self.logger.info("[数据保护] 正在备份现有障碍物数据...")
                old_state = self.map_mgr.get_full_state()
                old_static_obs = old_state.get("static_obstacles", {})
                old_dynamic_obs = old_state.get("dynamic_obstacles", {})

            # 3. 更新内部配置状态
            for key in self.current_config.keys():
                if key in new_settings:
                    val = new_settings[key]
                    # 执行严格的类型转换，防止前端传来的字符串污染配置
                    if key in [
                        "ENABLE_SHORTCUT_OPTIMIZATION",
                        "ENABLE_BEZIER_SMOOTHING",
                    ]:
                        # 兼容 "true"/"True" 字符串或 boolean 类型
                        self.current_config[key] = (
                            str(val).lower() == "true"
                            if isinstance(val, str)
                            else bool(val)
                        )
                    elif key == "BEZIER_SEGMENTS":
                        self.current_config[key] = int(val)
                    else:
                        self.current_config[key] = float(val)

            # 4. 执行组件初始化
            # 这里传入 needs_rebuild 标志位，决定是否复用旧地图
            self._init_components(rebuild_map=needs_rebuild)

            # 5. 结果处理与数据恢复
            msg = "系统配置已更新"

            if needs_rebuild:
                # 如果重建了地图，必须手动把备份的数据灌回去
                restored_count = self._migrate_obstacles(
                    old_static_obs, old_dynamic_obs
                )
                msg += (
                    f"，地图已根据新尺寸重建，并成功保留了 {restored_count} 个障碍物。"
                )
            else:
                # 如果没重建，说明复用了旧对象，数据自然还在
                msg += "，算法参数即时生效 (数据已保留)。"

                # [特殊情况处理]
                # 如果修改了 CRANE_FOOTPRINT_M (起重机尺寸)，虽然 MapManager 没重建，
                # 但之前缓存的膨胀网格可能不再适用。
                # 不过不用担心，MapManager.get_inflated_grid 是基于 (margin) 做缓存 key 的。
                # 一旦 footprint 变了，计算出的 margin 浮点数也会变，
                # 所以系统会自动计算新的膨胀网格，旧缓存虽然还在内存里但不会被命中，属于安全行为。
                pass

            self.logger.info(f"[配置更新] 完成: {msg}")
            return True, msg

        except Exception as e:
            self.logger.exception("配置更新过程中发生严重错误")
            return False, f"配置更新失败: {str(e)}"

    def get_full_state(self) -> Dict[str, Any]:
        """
        获取当前系统的完整状态快照。
        用于前端页面初次加载时的状态同步 (Hydration)。

        Returns:
            Dict: 包含地图尺寸、所有障碍物、当前配置参数的大字典。
        """
        # 1. 获取底层地图状态 (尺寸 + 障碍物)
        state = self.map_mgr.get_full_state()

        # 2. 补充规划器特有状态 (如当前起重机占地大小)
        state["crane_footprint_m"] = self.planner.footprint

        # 3. 补充当前系统配置 (用于前端设置面板的回显)
        state["system_config"] = self.current_config

        # 4. [新增] 补充任务协同状态 (Mission State)
        # 确保新连接的客户端（浏览器B）能立刻获取到当前存储的起点和终点
        state["mission_state"] = self.mission_state

        return state

    def update_mission_state(self, start: Dict, end: Dict) -> bool:
        """
        [新增] 更新当前的任务坐标状态。
        当用户在任意一个前端拖拽起点/终点时调用。

        Args:
            start: {'x': float, 'y': float}
            end:   {'x': float, 'y': float}
        """
        try:
            # 简单的格式校验
            if "x" not in start or "y" not in start:
                raise ValueError("Start coordinate missing x or y")
            if "x" not in end or "y" not in end:
                raise ValueError("End coordinate missing x or y")

            # 更新内存状态
            self.mission_state["start"] = {
                "x": float(start["x"]),
                "y": float(start["y"]),
            }
            self.mission_state["end"] = {"x": float(end["x"]), "y": float(end["y"])}

            self.logger.debug(
                f"[协同] 坐标同步: Start={self.mission_state['start']}, End={self.mission_state['end']}"
            )
            return True
        except Exception as e:
            self.logger.error(f"[协同] 坐标同步失败: {e}")
            return False

    def add_obstacle(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """
        [业务逻辑] 添加障碍物。

        Args:
            data: 包含障碍物信息的字典 {'type', 'x', 'y', 'w', 'h'}

        Returns:
            (bool, str): (操作是否成功, 提示消息)
        """
        try:
            # 参数提取与默认值处理
            obs_type = data.get("type", "dynamic")
            x = float(data["x"])
            y = float(data["y"])
            w = float(data.get("w", 2.0))
            h = float(data.get("h", 2.0))

            # 生成简短的唯一 ID
            obs_id = f"{obs_type}_{str(uuid.uuid4())[:8]}"

            # [安全性检查] 边界检查
            # 严禁添加位于地图负坐标或超出边界的物体，这会导致底层数组索引越界崩溃
            if x < 0 or y < 0:
                return False, "非法坐标: 坐标值不能为负数"
            if x + w > self.map_mgr.width_m or y + h > self.map_mgr.length_m:
                return (
                    False,
                    f"非法坐标: 障碍物超出地图边界 (当前地图: {self.map_mgr.width_m}x{self.map_mgr.length_m})",
                )

            self.logger.info(
                f"[操作] 添加障碍物: {obs_id} @ ({x:.1f},{y:.1f}) size {w}x{h}"
            )

            if obs_type == "static":
                self.map_mgr.add_static_obstacle(obs_id, x, y, w, h)
            else:
                self.map_mgr.update_dynamic_obstacle(obs_id, x, y, w, h)

            return True, f"障碍物 {obs_id} 添加成功"

        except ValueError:
            return False, "数据格式错误：坐标必须为数值"
        except Exception as e:
            self.logger.exception("添加障碍物时发生未知异常")
            return False, f"系统错误: {str(e)}"

    def remove_obstacle_near(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """
        [业务逻辑] 移除指定坐标附近的障碍物。
        通常响应前端的鼠标点击删除操作。

        Args:
            data: {'x': float, 'y': float}
        """
        try:
            x = float(data["x"])
            y = float(data["y"])

            # 调用 MapManager 的空间查询能力，查找该点是否命中了某个物体
            result = self.map_mgr.find_obstacle_near(x, y)

            if result:
                oid, otype = result
                self.logger.info(f"[操作] 移除障碍物: {oid} ({otype})")
                if otype == "static":
                    self.map_mgr.remove_static_obstacle(oid)
                else:
                    self.map_mgr.remove_dynamic_obstacle(oid)
                return True, f"已移除 {oid}"

            return False, "该位置没有检测到障碍物"
        except Exception as e:
            self.logger.error(f"移除障碍物异常: {e}")
            return False, f"操作失败: {e}"

    def plan_path(
        self, data: Dict[str, Any]
    ) -> Tuple[Optional[List[Tuple[float, float, float]]], str]:
        """
        [业务逻辑] 执行路径规划。

        Args:
            data: 包含起终点信息 {'start': {'x',...}, 'end': {'x',...}}

        Returns:
            (PathList, Message):
            - PathList: [(x,y,z), ...] 成功时返回路径点列表，失败返回 None
            - Message: 状态描述
        """
        try:
            # [新增] 在规划路径的同时，强制更新服务端的 Mission State。
            # 这样如果用户 A 点击了“开始规划”，用户 B 看到的起终点也会瞬间跳变到 A 设定的位置。
            # 这是一个良好的 UX 实践，保证"所见即所得"。
            if "start" in data and "end" in data:
                self.update_mission_state(data["start"], data["end"])

            # 默认高度处理：
            # 起点默认 Z=0.5 (代表吊钩在地面附近，准备起吊)
            # 终点默认 Z=1.0 (代表吊钩在台面高度，准备卸货)
            s = (float(data["start"]["x"]), float(data["start"]["y"]), 0.5)
            e = (float(data["end"]["x"]), float(data["end"]["y"]), 1.0)

            self.logger.info(f"[寻路] 请求: Start={s}, End={e}")

            # 调用规划器核心
            # 注意：所有的避障、膨胀、捷径、平滑逻辑都封装在 planner.find_path_3d 中
            path = self.planner.find_path_3d(s, e)

            if path:
                return path, "规划成功"

            # 如果返回空，说明 A* 没找到路
            self.logger.warning("[寻路] 失败: 规划器返回空路径")
            return (
                None,
                "无法生成路径：可能是起点/终点位于障碍物内，或者路径被障碍物完全封死。",
            )

        except KeyError as k:
            return None, f"请求参数缺失: {k}"
        except Exception as err:
            self.logger.exception("路径规划发生未预期异常")
            return None, f"内部服务错误: {err}"
