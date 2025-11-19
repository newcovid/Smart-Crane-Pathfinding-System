import eventlet

# [关键] 执行 Monkey Patch，使标准库 socket 支持协程，这对 Flask-SocketIO 至关重要
eventlet.monkey_patch()

import logging
from typing import Dict, Any
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

from config import Config
from crane_service import CraneService

# =========================================================================
# 1. 应用初始化与配置 (App Setup)
# =========================================================================
print(">>> [Boot] 正在启动 Flask 应用框架...")

app = Flask(__name__, template_folder="templates")
app.config.from_object(Config)

# 配置日志格式
# 工业级日志通常包含：时间戳、日志级别、模块名、具体消息
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
app.logger.setLevel(logging.INFO)

# 初始化 SocketIO
# async_mode="eventlet" 是生产环境的最佳实践，性能远高于默认的 threading 模式
socketio = SocketIO(app, async_mode="eventlet", cors_allowed_origins="*")

# =========================================================================
# 2. 依赖注入 (Dependency Injection)
# =========================================================================
# 实例化核心业务服务 CraneService (单例模式)
# 它将持有整个应用的业务状态 (地图、规划器等)
crane_service = CraneService(config=Config, logger=app.logger)


# =========================================================================
# 3. HTTP 路由控制器 (Controllers)
# =========================================================================
@app.route("/")
def index() -> str:
    """前端入口页面"""
    return render_template("index.html")


# =========================================================================
# 4. WebSocket 事件处理器 (SocketIO Handlers)
# =========================================================================


def broadcast_state_update() -> None:
    """[工具函数] 向所有连接的客户端广播最新的地图状态"""
    state = crane_service.get_full_state()
    socketio.emit("update_map_state", state)


@socketio.on("connect")
def handle_connect() -> None:
    """客户端连接事件"""
    sid = request.sid
    app.logger.info(f"[Socket] Client Connected: {sid}")
    # 立即发送一次当前状态，实现"首屏直出"
    state = crane_service.get_full_state()
    emit("update_map_state", state)


@socketio.on("disconnect")
def handle_disconnect() -> None:
    """客户端断开事件"""
    pass  # 实际业务中可能需要清理用户特定的资源，此处暂不需要


@socketio.on("update_settings")
def handle_update_settings(data: Dict[str, Any]) -> None:
    """
    [核心交互] 前端请求更新系统配置 (车间尺寸、算法参数等)
    """
    sid = request.sid
    app.logger.info(f"[Setting] 收到配置更新请求 (From {sid})")

    success, message = crane_service.update_configuration(data)

    if success:
        emit("operation_success", {"message": message})
        # 配置改变可能导致地图重建，必须广播给所有客户端以刷新视图
        broadcast_state_update()
    else:
        emit("operation_failed", {"message": message})


@socketio.on("request_path")
def handle_request_path(data: Dict[str, Any]) -> None:
    """路径规划请求"""
    path, message = crane_service.plan_path(data)
    if path:
        emit("update_path", path)
    else:
        emit("operation_failed", {"message": message})


@socketio.on("add_obstacle")
def handle_add_obstacle(data: Dict[str, Any]) -> None:
    """添加障碍物请求"""
    success, message = crane_service.add_obstacle(data)
    if success:
        # 只有成功才广播，避免无效刷新
        broadcast_state_update()
    else:
        emit("operation_failed", {"message": message})


@socketio.on("remove_obstacle_near")
def handle_remove_obstacle_near(data: Dict[str, Any]) -> None:
    """移除障碍物请求"""
    success, message = crane_service.remove_obstacle_near(data)
    if success:
        broadcast_state_update()
    else:
        emit("operation_failed", {"message": message})


# =========================================================================
# 5. 启动入口
# =========================================================================
if __name__ == "__main__":
    host = "127.0.0.1"
    port = 5000

    app.logger.info(f"服务启动中... 访问 http://{host}:{port}")

    # 使用 socketio.run 替代 app.run
    socketio.run(app, host=host, port=port, debug=True)
