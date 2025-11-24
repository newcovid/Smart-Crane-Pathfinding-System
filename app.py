# ==============================================================================
# 0. 基础环境设置 (Monkey Patching)
# ==============================================================================
# 什么是 monkey_patch?
# Python 默认是同步的（一件事做完再做下一事）。但 Web 服务器需要同时处理很多人。
# eventlet 是一个并发库，monkey_patch 会把 Python 标准库里的 socket、time 等
# 模块替换成支持并发的版本。
# 注意：这行代码必须写在所有其他 import 之前，否则会报错！
import eventlet

eventlet.monkey_patch()

import logging
import uuid
from typing import Dict, Any, Optional

# Flask 是 Web 框架 (提供网页服务)
# SocketIO 是实时通信库 (提供 WebSocket 双向通道)
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

# 导入我们的核心业务逻辑
from core.config import Config
from core.crane_service import CraneService


# ==============================================================================
# 1. 自定义日志处理器 (Log Streaming to Frontend)
# ==============================================================================
class SocketIOLogHandler(logging.Handler):
    """
    【SocketIO】

    作用:
    Python 的 print 或 logging 默认只打印在控制台里。
    这个类的作用是拦截这些日志，通过 WebSocket 发送给浏览器。
    在网页右下角的"系统终端"里可以看到后端的运行情况。
    """

    def emit(self, record: logging.LogRecord):
        """
        当有新日志生成时，会自动调用这个方法。
        """
        try:
            # 1. 格式化日志 (把时间、等级、内容拼成字符串)
            log_entry = self.format(record)

            # 2. 检查 socketio 是否已初始化
            # globals() 获取当前全局变量字典
            if "socketio" in globals() and socketio is not None:
                # 3. 广播日志给所有连接的客户端
                # event 名为 "server_log"，前端监听这个事件就能收到日志
                socketio.emit(
                    "server_log",
                    {
                        "level": record.levelname,  # 日志等级 (INFO, ERROR...)
                        "msg": log_entry,  # 日志内容
                        "time": record.created,  # 时间戳
                    },
                )
        except Exception:
            # 如果发送日志出错 (比如网络断了)，使用默认的错误处理，防止程序崩溃
            self.handleError(record)


# ==============================================================================
# 2. 应用初始化 (App Factory)
# ==============================================================================

# 初始化 Flask 应用
app = Flask(__name__, template_folder="templates")
# 加载配置
app.config.from_object(Config)

# 初始化 SocketIO
# async_mode="eventlet": 指定使用 eventlet 作为异步模式，性能最好
# cors_allowed_origins="*": 允许跨域 (允许任何网址访问，开发调试方便)
socketio = SocketIO(app, async_mode="eventlet", cors_allowed_origins="*")

# --- 日志系统配置 (Logging Setup) ---

# 动态获取配置中的日志等级
# 从 app.config 中读取 LOG_LEVEL，默认为 'INFO'
log_level_name = app.config.get("LOG_LEVEL", "INFO").upper()

# 将字符串转换为 logging 模块的常量
# getattr(logging, "DEBUG") -> 10 (logging.DEBUG)
# getattr(logging, "INFO")  -> 20 (logging.INFO)
log_level = getattr(logging, log_level_name, logging.INFO)
# 1. 清除 Python 默认的日志处理器 (防止日志重复打印)
logging.getLogger().handlers = []

# 2. 定义日志格式
# 格式: 时间 [等级] 模块名: 消息内容
formatter = logging.Formatter(
    "%(asctime)s [%(levelname)s] %(name)s: %(message)s", "%H:%M:%S"
)

# 3. 处理器 A: 控制台输出 (Stdout)
# 让开发者在 PyCharm/VSCode 的终端里能看到日志
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)
console_handler.setLevel(log_level)

# 4. 处理器 B: 网页端输出 (WebSocket)
# 让用户在网页上能看到日志
web_log_handler = SocketIOLogHandler()
web_log_handler.setFormatter(formatter)
web_log_handler.setLevel(log_level)

# 5. 获取根记录器 (Root Logger) 并挂载处理器
root_logger = logging.getLogger()
root_logger.setLevel(log_level)
root_logger.addHandler(console_handler)
root_logger.addHandler(web_log_handler)

# 获取一个专属的 logger 给 app.py 使用
logger = logging.getLogger("Server")
logger.info(">>> [System] 系统初始化完成 (Console + Web Logs Ready)")


# ==============================================================================
# 3. 实例化业务服务 (Business Layer)
# ==============================================================================
# CraneService 是业务层，它持有 MapManager 和 Planner
# 这里的 logger 传给它，它内部会用来打印日志
crane_service = CraneService(config=Config, logger=root_logger)


# ==============================================================================
# 4. HTTP 路由 (Web Page Routes)
# ==============================================================================
@app.route("/")
def index():
    """
    首页路由。
    当用户访问 http://localhost:5000/ 时，返回 main.html 网页文件。
    """
    return render_template("main.html")


# ==============================================================================
# 5. SocketIO 事件处理 (Event Handlers)
# ==============================================================================


@socketio.on("connect")
def handle_connect():
    """
    [事件] 客户端连接。
    当网页打开或刷新时触发。
    """
    # request.sid 是每个客户端唯一的会话 ID (Session ID)
    logger.info(f"[Socket] 客户端已连接 (SID: {request.sid})")

    # 1. 获取当前的全量状态 (地图、障碍物、配置)
    state = crane_service.get_full_state()

    # 2. 发送给当前连接的客户端
    emit("update_map_state", state)

    # 3. 如果之前计算过路径，顺便把旧路径也发过去，避免刷新后路径消失
    if crane_service.last_calculated_path:
        emit("update_path", crane_service.last_calculated_path)
        if crane_service.last_stats:
            emit("planning_stats", crane_service.last_stats)


@socketio.on("update_settings")
def handle_update_settings(data: Dict[str, Any]):
    """
    [事件] 更新设置。
    当用户在左侧侧边栏修改了参数并点击"应用"时触发。
    """
    logger.info(f"[Config] 收到配置更新请求: {data}")

    # 调用服务层更新配置
    success, msg = crane_service.update_configuration(data)

    if success:
        # 如果更新成功 (比如地图大小变了)，需要把新状态广播给所有客户端
        # broadcast=True 表示发给所有人，不仅仅是当前操作的人
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": "配置已更新"})
    else:
        logger.error(f"[Config] 更新失败: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("request_path")
def handle_request_path(data: Dict[str, Any]):
    """
    [事件] 请求路径规划。
    当用户点击"开始规划路径"按钮时触发。
    """
    # 1. 允许用户只传终点，起点默认为上次的位置
    if "start" in data:
        crane_service.update_mission_state(data)
        # 同步给其他客户端，让大家的起点终点图标一致
        socketio.emit("sync_mission_coordinates", data, include_self=False)

    logger.info("[Plan] 收到规划请求，正在调用核心算法...")

    # 2. 调用核心业务去算路
    # 这里的 plan_path 内部已经集成了 MapManager 和 Planner
    path, stats, msg = crane_service.plan_path()

    if path:
        # 成功: 发送路径数据和性能统计
        emit("update_path", path)
        emit("planning_stats", stats)
        logger.info(f"[Plan] 规划成功! 路径节点数: {len(path)}")
        emit("operation_success", {"message": f"路径规划成功 ({len(path)} 节点)"})
    else:
        # 失败: 发送空路径(清空画布)和失败消息
        emit("update_path", [])
        emit("planning_stats", stats)
        logger.warning(f"[Plan] 规划失败: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("add_obstacle")
def handle_add_obstacle(data: Dict[str, Any]):
    """
    [事件] 添加障碍物。
    当用户在地图上拖拽生成矩形时触发。
    """
    logger.info(f"[Map] 收到添加障碍物请求: Pos=({data.get('x')}, {data.get('y')})")

    # 1. 调用服务层添加障碍
    success, msg = crane_service.add_obstacle(data)

    if success:
        # 2. 如果成功，立即把新地图广播给所有人
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": f"已添加障碍物: {msg}"})
        logger.info("[Map] 障碍物添加成功，触发自动重规划 (Auto-Replan)...")

        # 3. 自动触发一次重规划
        # 这样用户加完障碍物，路径会自动避开，体验更好
        path, stats, _ = crane_service.plan_path()
        if path:
            emit("update_path", path)
            emit("planning_stats", stats)
        else:
            emit("update_path", [])  # 如果路被堵死了，清空路径
            emit("planning_stats", stats)
    else:
        logger.error(f"[Map] 添加失败: {msg}")
        emit("operation_failed", {"message": msg})


@socketio.on("remove_obstacle_near")
def handle_remove_obstacle(data: Dict[str, Any]):
    """
    [事件] 移除障碍物。
    当用户使用"移除"工具点击地图时触发。
    """

    logger.info(f"[Map] 请求移除障碍物: 点击位置 ({data.get('x')}, {data.get('y')})")

    success, msg = crane_service.remove_obstacle_near(data)

    if success:
        socketio.emit("update_map_state", crane_service.get_full_state())
        emit("operation_success", {"message": f"已移除障碍物: {msg}"})
        logger.info("[Map] 移除成功，触发自动重规划...")

        # 自动重规划
        path, stats, _ = crane_service.plan_path()
        if path:
            emit("update_path", path)
            emit("planning_stats", stats)
    else:
        emit("operation_failed", {"message": "点击位置附近没有找到障碍物"})


@socketio.on("sync_mission_coordinates")
def handle_sync_mission(data: Dict[str, Any]):
    """
    [事件] 同步任务坐标。
    当用户拖拽起点或终点图标时触发。
    """
    # 这里的 include_self=False：
    # 意思是不要发回给发送者自己，因为发送者已经拖拽到位了，
    # 只发给别的正在观看的客户端，实现多人协同。
    crane_service.update_mission_state(data)
    socketio.emit("sync_mission_coordinates", data, include_self=False)


# ==============================================================================
# 6. 程序入口 (Main Entry)
# ==============================================================================
if __name__ == "__main__":
    logger.info(">>> [Boot] 服务器正在启动: http://0.0.0.0:5000")

    # 启动 SocketIO 服务器
    # debug=True: 代码修改后自动重启，方便开发
    # host='0.0.0.0': 允许局域网内其他电脑访问
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)
