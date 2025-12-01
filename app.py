import eventlet

# 必须在导入其他库之前打补丁，以支持协程
eventlet.monkey_patch()

import logging
from typing import Dict, Any, Optional

from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

from smart_crane.core.config import settings
from smart_crane.core.crane_service import CraneService
from smart_crane.core.constants import MSG_PLAN_SUCCESS, MSG_PLAN_FAIL


# ==============================================================================
# 1. 自定义智能日志系统
# ==============================================================================
class SmartLogFormatter(logging.Formatter):
    """
    【智能日志格式化器】

    功能：
    1. 自动对齐：确保日志等级和名称列宽固定，视觉整洁。
    2. 智能缩写：支持无限嵌套的 Logger 名称显示。
       当名称过长时，优先保留最具体的末尾部分（组件名）。
    """

    def __init__(self, fmt: Optional[str] = None, datefmt: Optional[str] = None):
        super().__init__(fmt, datefmt)
        self.width_level = 6
        # 增加宽度以容纳常见的嵌套层级 (例如 "Planner.Grid")
        self.width_name = 24

    def _smart_truncate_name(self, name: str, max_width: int) -> str:
        """智能缩短日志名称算法。

        Args:
            name: 原始 Logger 名称 (如 "TrajectoryPlanner.GridAdapter")
            max_width: 最大允许显示宽度

        Returns:
            str: 缩写后的名称
        """
        if len(name) <= max_width:
            return name

        parts = name.split(".")

        # 策略1：尝试只保留父级首字母
        # 效果: "TrajectoryPlanner.GridAdapter" -> "T.GridAdapter"
        short_parts = [p[0] for p in parts[:-1]] + [parts[-1]]
        short_name = ".".join(short_parts)

        if len(short_name) <= max_width:
            return short_name

        # 策略2：如果缩写后依然太长 (说明层级极深，或末尾名字本身就超长)
        # 优先保留最右侧的字符（最具体的组件名），在左侧截断并添加提示
        # 效果: "A.B.C.D.E.F.G.GridAdapter" -> "..GridAdapter"
        # 这样用户永远能看到日志是“谁”发出的，而不是看到一堆父级前缀 "A.B.C.D..."
        return ".." + short_name[-(max_width - 2) :]

    def format(self, record: logging.LogRecord) -> str:
        # 1. 备份原始属性
        orig_levelname = record.levelname
        orig_name = record.name

        # 2. 格式化 Level (左对齐)
        record.levelname = f"{orig_levelname[:self.width_level]:<{self.width_level}}"

        # 3. 格式化 Name (智能缩写 + 填充)
        display_name = self._smart_truncate_name(orig_name, self.width_name)
        record.name = f"{display_name:<{self.width_name}}"

        # 4. 执行父类格式化
        formatted_msg = super().format(record)

        # 5. 还原现场 (防止影响其他 Handler)
        record.levelname = orig_levelname
        record.name = orig_name

        return formatted_msg


class SocketIOLogHandler(logging.Handler):
    """将日志消息实时推送到前端 SocketIO 的处理器"""

    def emit(self, record: logging.LogRecord) -> None:
        try:
            # 获取格式化后的文本（包含对齐后的 name 和 level）
            log_text = self.format(record)

            if "socketio" in globals() and socketio is not None:
                socketio.emit(
                    "server_log",
                    {
                        "level": record.levelname.strip(),  # 前端可能需要原始等级颜色
                        "msg": log_text,
                        "time": record.created,
                    },
                )
        except Exception:
            self.handleError(record)


# ==============================================================================
# 2. 应用与日志初始化
# ==============================================================================
app = Flask(__name__, template_folder="templates")
app.config["SECRET_KEY"] = settings.app.secret_key

socketio = SocketIO(app, async_mode="eventlet", cors_allowed_origins="*")

# --- 日志系统配置 ---
log_level_name = settings.app.log_level.upper()
log_level = getattr(logging, log_level_name, logging.INFO)

# 清除默认 Handlers
logging.getLogger().handlers = []

# 定义格式：时间 | 等级 | 模块(智能对齐) | 消息
LOG_FORMAT_STR = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"
DATE_FORMAT_STR = "%H:%M:%S"

smart_formatter = SmartLogFormatter(fmt=LOG_FORMAT_STR, datefmt=DATE_FORMAT_STR)

# 1. 控制台输出
console_handler = logging.StreamHandler()
console_handler.setFormatter(smart_formatter)
console_handler.setLevel(log_level)

# 2. WebSocket 输出
web_log_handler = SocketIOLogHandler()
web_log_handler.setFormatter(smart_formatter)
web_log_handler.setLevel(log_level)

# 3. 绑定到 Root Logger
root_logger = logging.getLogger()
root_logger.setLevel(log_level)
root_logger.addHandler(console_handler)
root_logger.addHandler(web_log_handler)

# 初始化主 Logger
logger = logging.getLogger("APP Server")
logger.info(f"系统日志服务已就绪 (Level: {log_level_name})")


# ==============================================================================
# 3. 实例化业务服务
# ==============================================================================
crane_service = CraneService(settings=settings, logger=root_logger)


# ==============================================================================
# 4. HTTP 路由
# ==============================================================================
@app.route("/")
def index() -> str:
    return render_template("main.html")


# ==============================================================================
# 5. SocketIO 事件处理
# ==============================================================================
@socketio.on("connect")
def handle_connect() -> None:
    logger.info(f"客户端接入 (SID: {request.sid})")
    state = crane_service.get_full_state()
    emit("update_map_state", state)

    if crane_service.last_calculated_path:
        emit("update_path", crane_service.last_calculated_path)
        if crane_service.last_stats:
            emit("planning_stats", crane_service.last_stats)


@socketio.on("update_settings")
def handle_update_settings(data: Dict[str, Any]) -> None:
    logger.info("收到配置更新指令")
    success, msg = crane_service.update_configuration(data)

    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": msg})
    else:
        logger.error(f"配置更新异常: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("request_path")
def handle_request_path(data: Dict[str, Any]) -> None:
    if "start" in data or "end" in data:
        crane_service.update_mission_state(data)
        socketio.emit("sync_mission_coordinates", data, include_self=False)

    logger.info("启动路径规划引擎...")
    path, stats, msg = crane_service.plan_path()

    if path:
        emit("update_path", path)
        emit("planning_stats", stats)
        # 格式化消息中不再需要手动加前缀
        logger.info(MSG_PLAN_SUCCESS.format(count=len(path)))
        emit("operation_success", {"message": f"规划成功 ({len(path)} 节点)"})
    else:
        emit("update_path", [])
        emit("planning_stats", stats)
        logger.warning(MSG_PLAN_FAIL.format(reason=msg))
        emit("operation_failed", {"message": msg})


@socketio.on("add_obstacle")
def handle_add_obstacle(data: Dict[str, Any]) -> None:
    logger.info(f"新增障碍物: ({data.get('x')}, {data.get('y')})")
    success, msg = crane_service.add_obstacle(data)

    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": msg})

        # 自动触发重规划
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
    logger.info(f"移除障碍物: ({data.get('x')}, {data.get('y')})")
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
    crane_service.update_mission_state(data)
    socketio.emit("sync_mission_coordinates", data, include_self=False)


if __name__ == "__main__":
    logger.info("Web服务器启动: http://0.0.0.0:5000")
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)
