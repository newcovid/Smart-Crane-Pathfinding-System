import eventlet

eventlet.monkey_patch()

import logging
import uuid
from typing import Dict, Any
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

from core.config import Config
from core.crane_service import CraneService


# =========================================================================
# 1. 自定义日志处理器 (Log Streaming)
# =========================================================================
class SocketIOLogHandler(logging.Handler):
    """将后端日志实时推送到 WebSocket 客户端"""

    def emit(self, record):
        try:
            log_entry = self.format(record)
            # 使用 socketio.emit 广播日志，namespace='/'
            # 注意: 这里需要在一个应用上下文中或者是广播模式
            if "socketio" in globals():
                socketio.emit(
                    "server_log",
                    {
                        "level": record.levelname,
                        "msg": log_entry,
                        "time": record.created,
                    },
                )
        except Exception:
            self.handleError(record)


# =========================================================================
# 2. 应用初始化
# =========================================================================
app = Flask(__name__, template_folder="templates")
app.config.from_object(Config)

# 初始化 SocketIO
socketio = SocketIO(app, async_mode="eventlet", cors_allowed_origins="*")

# 配置日志系统
# 清除默认 handler，避免重复
logging.getLogger().handlers = []

# 格式化器
formatter = logging.Formatter(
    "%(asctime)s [%(levelname)s] %(name)s: %(message)s", "%H:%M:%S"
)

# 1. 控制台输出
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)
console_handler.setLevel(logging.INFO)

# 2. Web 端输出
socket_handler = SocketIOLogHandler()
socket_handler.setFormatter(formatter)
socket_handler.setLevel(logging.INFO)

# 挂载到 Root Logger
root_logger = logging.getLogger()
root_logger.setLevel(logging.INFO)
root_logger.addHandler(console_handler)
root_logger.addHandler(socket_handler)

logger = app.logger
logger.info(">>> [System] 日志系统初始化完成 (Console + Web)")

# =========================================================================
# 3. 业务服务单例
# =========================================================================
crane_service = CraneService(config=Config, logger=logger)


# =========================================================================
# 4. 路由与事件
# =========================================================================
@app.route("/")
def index():
    return render_template("main.html")


@socketio.on("connect")
def handle_connect():
    logger.info(f"[Socket] Client connected: {request.sid}")
    state = crane_service.get_full_state()
    emit("update_map_state", state)

    if crane_service.last_calculated_path:
        emit("update_path", crane_service.last_calculated_path)
        if crane_service.last_stats:
            emit("planning_stats", crane_service.last_stats)


@socketio.on("update_settings")
def handle_update_settings(data):
    logger.info(f"[Config] Update request: {data}")
    success, msg = crane_service.update_configuration(data)
    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": "Settings updated"})
    else:
        logger.error(f"[Config] Update failed: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("request_path")
def handle_request_path(data):
    if "start" in data:
        crane_service.update_mission_state(data)
        socketio.emit("sync_mission_coordinates", data, include_self=False)

    logger.info("[Plan] Request received...")
    path, stats, msg = crane_service.plan_path()

    if path:
        emit("update_path", path)
        emit("planning_stats", stats)
        logger.info(f"[Plan] Success. Nodes: {len(path)}")
    else:
        emit("update_path", [])
        emit("planning_stats", stats)
        logger.warning(f"[Plan] Failed: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("add_obstacle")
def handle_add_obstacle(data):
    # 增加详细日志以调试障碍物添加流程
    logger.info(f"[Map] Adding obstacle request: {data}")
    success, msg = crane_service.add_obstacle(data)
    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
        logger.info("[Map] Obstacle added & Map broadcasted")
    else:
        logger.error(f"[Map] Add failed: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("remove_obstacle_near")
def handle_remove_obstacle(data):
    success, msg = crane_service.remove_obstacle_near(data)
    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
    else:
        emit("operation_failed", {"message": "No obstacle found here"})


@socketio.on("sync_mission_coordinates")
def handle_sync_mission(data):
    crane_service.update_mission_state(data)
    socketio.emit("sync_mission_coordinates", data, include_self=False)


if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
