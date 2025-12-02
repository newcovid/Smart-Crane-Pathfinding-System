# ==============================================================================
# app.py
# 智能起重机路径规划系统主入口
#
# 本模块负责：
# 1. 初始化 Flask 应用与 SocketIO
# 2. 配置高保真日志系统（含 WebSocket 实时推送）
# 3. 注册 HTTP 路由与 SocketIO 事件
# 4. 实例化核心业务服务
# ==============================================================================

import eventlet

eventlet.monkey_patch()  # 必须在导入其他库之前打补丁，以支持协程


import logging
from typing import Dict, Any, Optional


from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit


from smart_crane.core.config import settings
from smart_crane.core.crane_service import CraneService
from smart_crane.core.constants import (
    MSG_PLAN_SUCCESS,
    MSG_PLAN_FAIL,
    LOG_LEVEL_WIDTH,
    LOGGER_NAME_WIDTH,
    DEFAULT_SERVER_HOST,
    DEFAULT_SERVER_PORT,
)


# ==============================================================================
# 1. 自定义智能日志系统 (Smart Logging System)
# ==============================================================================


class SmartLogFormatter(logging.Formatter):
    """
    高保真日志格式化器。

    特性：
      1. 忠实还原：不缩写、不替换，完整保留包名与模块名。
      2. 视觉对齐：采用宽列+左对齐，分隔符 '|' 垂直对齐。

    Attributes:
        width_level (int): 日志等级字段宽度。
        width_name (int): 模块名字段宽度。
    """

    width_level: int
    width_name: int

    def __init__(
        self, fmt: Optional[str] = None, datefmt: Optional[str] = None
    ) -> None:
        super().__init__(fmt, datefmt)
        # 从常量中读取格式化宽度，避免魔法数字
        self.width_level = LOG_LEVEL_WIDTH
        self.width_name = LOGGER_NAME_WIDTH

    def format(self, record: logging.LogRecord) -> str:
        """
        格式化日志记录。

        Args:
            record (logging.LogRecord): 日志记录对象。
        Returns:
            str: 格式化后的日志字符串。
        """
        # 备份原始属性，防止污染其他 Handler
        orig_levelname = record.levelname
        orig_name = record.name

        # 格式化 Level (左对齐，固定宽度)
        record.levelname = f"{orig_levelname:<{self.width_level}}"

        # 格式化 Name (左对齐填充)
        display_name = orig_name
        if len(display_name) < self.width_name:
            record.name = f"{display_name:<{self.width_name}}"
        else:
            record.name = display_name

        # 执行父类格式化
        formatted_msg = super().format(record)

        # 还原现场
        record.levelname = orig_levelname
        record.name = orig_name

        return formatted_msg


class SocketIOLogHandler(logging.Handler):
    """
    SocketIO 日志处理器。
    实时将日志消息推送到前端 WebSocket。
    """

    def emit(self, record: logging.LogRecord) -> None:
        """
        发送日志到前端。

        Args:
            record (logging.LogRecord): 日志记录对象。
        """
        try:
            log_text: str = self.format(record)
            if "socketio" in globals() and socketio is not None:
                # 获取原始等级名称用于前端配色 (如 "INFO", "ERROR")
                level_raw: str = record.levelname.strip().upper()
                socketio.emit(
                    "server_log",
                    {
                        "level": level_raw,
                        "msg": log_text,
                        "time": record.created,
                    },
                )
        except Exception:
            self.handleError(record)


# ==============================================================================
# 2. 应用与日志初始化
# ==============================================================================

# =============================
# 应用与日志系统初始化
# =============================
app: Flask = Flask(__name__, template_folder="templates")
app.config["SECRET_KEY"] = settings.app.secret_key

socketio: SocketIO = SocketIO(app, async_mode="eventlet", cors_allowed_origins="*")

# 日志系统配置
log_level_name: str = settings.app.log_level.upper()
log_level: int = getattr(logging, log_level_name, logging.INFO)

# 清除默认 Handlers，避免重复输出
logging.getLogger().handlers = []

# 日志格式定义
LOG_FORMAT_STR: str = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"
DATE_FORMAT_STR: str = "%H:%M:%S"

smart_formatter: SmartLogFormatter = SmartLogFormatter(
    fmt=LOG_FORMAT_STR, datefmt=DATE_FORMAT_STR
)

# 控制台日志 Handler
console_handler: logging.StreamHandler = logging.StreamHandler()
console_handler.setFormatter(smart_formatter)
console_handler.setLevel(log_level)

# WebSocket 日志 Handler
web_log_handler: SocketIOLogHandler = SocketIOLogHandler()
web_log_handler.setFormatter(smart_formatter)
web_log_handler.setLevel(log_level)

# 绑定到 Root Logger
root_logger: logging.Logger = logging.getLogger()
root_logger.setLevel(log_level)
root_logger.addHandler(console_handler)
root_logger.addHandler(web_log_handler)

# 主 Logger
logger: logging.Logger = logging.getLogger("APP Server")
logger.info(f"系统日志服务已就绪 (Level: {log_level_name})")


# ==============================================================================
# 3. 实例化业务服务
# ==============================================================================

# =============================
# 业务服务实例化
# =============================
crane_service: CraneService = CraneService(settings=settings, logger=root_logger)


# ==============================================================================
# 4. HTTP 路由
# ==============================================================================


@app.route("/")
def index() -> str:
    """
    首页路由。
    Returns:
        str: 渲染后的主页面 HTML。
    """
    return render_template("main.html")


# ==============================================================================
# 5. SocketIO 事件处理
# ==============================================================================


@socketio.on("connect")
def handle_connect() -> None:
    """
    处理客户端连接事件。
    向新连接的客户端推送当前地图状态与最新路径。
    """
    logger.info(f"客户端接入 (SID: {request.sid})")
    state: Dict[str, Any] = crane_service.get_full_state()
    emit("update_map_state", state)

    if crane_service.last_calculated_path:
        emit("update_path", crane_service.last_calculated_path)
        if crane_service.last_stats:
            emit("planning_stats", crane_service.last_stats)


@socketio.on("update_settings")
def handle_update_settings(data: Dict[str, Any]) -> None:
    """
    处理配置更新事件。
    Args:
        data (Dict[str, Any]): 新配置数据。
    """
    logger.info("收到配置更新指令")
    success: bool
    msg: str
    success, msg = crane_service.update_configuration(data)

    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": msg})
    else:
        logger.error(f"配置更新异常: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("request_path")
def handle_request_path(data: Dict[str, Any]) -> None:
    """
    处理路径规划请求。
    Args:
        data (Dict[str, Any]): 包含起点/终点等任务参数，以及可选的配置更新数据(settings)。
    """
    # [优化] 1. 优先处理请求中携带的配置更新
    # 这样确保了在同一逻辑流中，配置更新绝对先于路径规划完成
    if "settings" in data:
        logger.debug("规划请求包含配置数据，正在应用配置...")
        success, msg = crane_service.update_configuration(data["settings"])
        if not success:
            logger.warning(f"规划前配置自动更新失败: {msg}")
            # 配置失败通常不应阻塞规划（除非参数完全非法），这里选择记录日志并继续尝试规划
        elif msg != "配置未发生变化":
            # 如果配置确实变了，通知所有客户端更新地图状态
            socketio.emit("update_map_state", crane_service.get_full_state())

    # [优化] 2. 更新任务起终点
    if "start" in data or "end" in data:
        crane_service.update_mission_state(data)
        socketio.emit("sync_mission_coordinates", data, include_self=False)

    logger.info("启动路径规划引擎...")
    path: list
    stats: dict
    msg: str

    # [优化] 3. 执行规划
    path, stats, msg = crane_service.plan_path()

    if path:
        emit("update_path", path)
        emit("planning_stats", stats)
        logger.info(MSG_PLAN_SUCCESS.format(count=len(path)))
        emit("operation_success", {"message": f"规划成功 ({len(path)} 节点)"})
    else:
        emit("update_path", [])
        emit("planning_stats", stats)
        logger.warning(MSG_PLAN_FAIL.format(reason=msg))
        emit("operation_failed", {"message": msg})


@socketio.on("add_obstacle")
def handle_add_obstacle(data: Dict[str, Any]) -> None:
    """
    处理新增障碍物事件。
    Args:
        data (Dict[str, Any]): 障碍物坐标。
    """
    logger.info(f"新增障碍物: ({data.get('x')}, {data.get('y')})")
    success: bool
    msg: str
    success, msg = crane_service.add_obstacle(data)

    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": msg})

        path, stats, _ = crane_service.plan_path()
        if path:
            emit("update_path", path)
            emit("planning_stats", stats)
        else:
            emit("update_path", [])
            emit("planning_stats", stats)
    else:
        logger.error(f"障碍物添加被拒绝: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("remove_obstacle_near")
def handle_remove_obstacle(data: Dict[str, Any]) -> None:
    """
    处理移除障碍物事件。
    Args:
        data (Dict[str, Any]): 目标障碍物坐标。
    """
    logger.info(f"移除障碍物: ({data.get('x')}, {data.get('y')})")
    success: bool
    msg: str
    success, msg = crane_service.remove_obstacle_near(data)

    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": msg})

        path, stats, _ = crane_service.plan_path()
        if path:
            emit("update_path", path)
            emit("planning_stats", stats)
    else:
        emit("operation_failed", {"message": msg})


@socketio.on("sync_mission_coordinates")
def handle_sync_mission(data: Dict[str, Any]) -> None:
    """
    处理任务坐标同步事件。
    Args:
        data (Dict[str, Any]): 任务坐标数据。
    """
    crane_service.update_mission_state(data)
    socketio.emit("sync_mission_coordinates", data, include_self=False)


if __name__ == "__main__":
    logger.info(f"Web 服务器启动： http://{DEFAULT_SERVER_HOST}:{DEFAULT_SERVER_PORT}")
    socketio.run(app, host=DEFAULT_SERVER_HOST, port=DEFAULT_SERVER_PORT, debug=False)
