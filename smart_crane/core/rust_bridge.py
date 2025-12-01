import logging
import importlib
from typing import Optional, Any, Type

# =============================================================================
# 1. 模块级初始化 (Module Level Initialization)
# =============================================================================

# 使用模块全名作为 Logger 名称 (e.g., "smart_crane.core.rust_bridge")
# 这样可以自动继承父级 Logger (如 "smart_crane") 的配置
logger = logging.getLogger(__name__)

# 全局标记: 尝试加载底层二进制扩展
HAS_RUST_CORE = False
_rust_module: Optional[Any] = None

try:
    # 尝试导入编译好的 Rust 扩展 (.pyd / .so)
    import smart_crane_core

    _rust_module = smart_crane_core
    HAS_RUST_CORE = True
    logger.info("Rust 高性能核心扩展 (smart_crane_core) 加载成功。")

except ImportError:
    # 这是一个预期内的行为（用户可能没编译 Rust 或处于纯 Python 开发环境）
    logger.info(
        "未检测到原生 'smart_crane_core' 扩展。系统将降级使用纯 Python 模式运行。"
    )
except Exception as e:
    # 加载过程中发生非预期错误（如依赖缺失、版本不兼容）
    logger.error(f"加载 'smart_crane_core' 时发生严重错误: {e}", exc_info=True)


class RustBackend:
    """Rust 后端代理 (Rust Backend Proxy)。

    作为 Python 业务层与底层 Rust 二进制扩展之间的唯一访问网关。
    负责屏蔽底层模块的存在性检查，提供安全的类获取接口。

    设计模式:
        静态代理 (Static Proxy) / 门面模式 (Facade) 的简化版。
    """

    @staticmethod
    def is_available() -> bool:
        """检查 Rust 核心扩展是否可用。

        Returns:
            bool: True 表示 Rust 模块已加载，可以使用高性能算法。
        """
        return HAS_RUST_CORE

    # =========================================================================
    # 寻路相关 (Pathfinding)
    # =========================================================================

    @staticmethod
    def get_astar_class() -> Optional[Type[Any]]:
        """获取 Rust 版 A* 规划器类。

        Returns:
            Optional[Type[Any]]: 如果可用，返回 `RustAStarPlanner` 类；否则返回 None。
        """
        if HAS_RUST_CORE and _rust_module and hasattr(_rust_module, "RustAStarPlanner"):
            return _rust_module.RustAStarPlanner
        return None

    @staticmethod
    def get_dslite_class() -> Optional[Type[Any]]:
        """获取 Rust 版 D* Lite 规划器类。

        Returns:
            Optional[Type[Any]]: 如果可用，返回 `RustDLitePlanner` 类；否则返回 None。
        """
        if HAS_RUST_CORE and _rust_module and hasattr(_rust_module, "RustDLitePlanner"):
            return _rust_module.RustDLitePlanner
        return None

    # =========================================================================
    # 地图/网格相关 (Map Core) - 预留接口
    # =========================================================================

    @staticmethod
    def get_map_processor_class() -> Optional[Type[Any]]:
        """[Future] 获取 Rust 版网格计算器。

        用于替代 Python 层的 GridFactory，将大规模矩阵运算下沉至 Rust。

        Returns:
            Optional[Type[Any]]: `RustMapProcessor` 类或 None。
        """
        if HAS_RUST_CORE and _rust_module and hasattr(_rust_module, "RustMapProcessor"):
            return _rust_module.RustMapProcessor
        return None

    # =========================================================================
    # 后处理相关 (Post Processing) - 预留接口
    # =========================================================================

    @staticmethod
    def get_post_processor_functions() -> Optional[Any]:
        """[Future] 获取 Rust 版路径平滑算法库。

        Returns:
            Optional[Any]: 包含 `greedy_shortcut` 或 `bezier_smooth` 的模块/对象。
        """
        if HAS_RUST_CORE and _rust_module and hasattr(_rust_module, "post_processing"):
            return _rust_module.post_processing
        return None
