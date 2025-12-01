import math
import logging
from typing import List, Tuple, Dict, Optional, Any, Union, TYPE_CHECKING

# [类型注解优化] 仅在类型检查阶段导入 MapManager，避免运行时循环引用
if TYPE_CHECKING:
    from smart_crane.core.map_manager import WorkshopMapManager

# 尝试导入科学计算库，用于高性能矩阵运算
try:
    import numpy as np
    from scipy.ndimage import distance_transform_edt, maximum_filter

    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# 类型别名定义
Grid2D = List[List[int]]
Grid3D = List[List[List[int]]]


class GridFactory:
    """网格计算工厂 (Grid Computation Factory)。

    负责处理高密度的网格生成和数学运算，特别是涉及障碍物膨胀（Inflation）
    和 3D 体素化（Voxelization）的繁重计算任务。
    此类设计为无状态工具类，主要依赖 NumPy 和 SciPy 提升性能。
    """

    @staticmethod
    def create_inflated_grid_2d(
        base_grid: Grid2D,
        xy_margin: float,
        logger: Optional[logging.Logger] = None,
    ) -> Grid2D:
        """使用欧几里得距离变换 (EDT) 创建 2D 膨胀网格。

        通过计算每个空闲网格到最近障碍物的距离，将距离小于安全边距的网格标记为障碍物。

        Args:
            base_grid (Grid2D): 原始的 2D 网格数据 (0=空闲, 1=障碍)。
            xy_margin (float): 水平方向的安全膨胀半径 (单位: 网格数)。
            logger (Optional[logging.Logger]): 父级日志记录器。

        Returns:
            Grid2D: 膨胀处理后的 2D 网格。
                    如果未安装 SciPy，则返回原始网格并打印警告。
        """
        # 1. 建立日志上下文
        if logger:
            local_logger = logger.getChild("GridFactory")
        else:
            local_logger = logging.getLogger("GridFactory")

        rows = len(base_grid)
        cols = len(base_grid[0]) if rows > 0 else 0

        if rows == 0:
            return []

        if not HAS_SCIPY:
            local_logger.warning("未安装 SciPy 库，无法执行膨胀计算！将返回原始网格。")
            return base_grid

        local_logger.debug(
            f"开始 2D 膨胀计算. Grid: {rows}x{cols}, Margin: {xy_margin:.2f}"
        )

        # 2. 转换为 Numpy 数组
        np_grid = np.array(base_grid, dtype=np.int8)

        # 3. 墙壁膨胀处理 (Padding)
        # 在网格四周填充一圈障碍物，使距离变换算法认为边界也是墙壁，从而实现边界的安全斥力
        np_grid_padded = np.pad(
            np_grid, pad_width=1, mode="constant", constant_values=1
        )

        # 4. 计算距离场 (EDT - Euclidean Distance Transform)
        # 计算每个非障碍点(0)到最近障碍点(1)的欧氏距离
        feature_mask = (np_grid_padded == 0).astype(int)
        dist_map = distance_transform_edt(feature_mask)

        # 5. 切除 Padding，还原原始尺寸
        dist_map = dist_map[1:-1, 1:-1]

        # 6. 应用膨胀阈值
        # +0.5 是为了补偿网格离散化带来的精度损失，确保圆形膨胀能覆盖边缘
        inflated_mask = dist_map <= (xy_margin + 0.5)
        inflated_np = inflated_mask.astype(int)

        # 7. 合并原始障碍物 (取并集)
        # 即使膨胀半径为0，原始障碍物也不能丢
        final_grid = np.maximum(inflated_np, np_grid)

        return final_grid.tolist()

    @staticmethod
    def create_3d_voxel_grid(
        map_dims: Tuple[int, int, int],
        resolution: float,
        obstacles: List[Dict[str, Any]],
        margins: Tuple[float, float, float],
        is_infinite: bool,
        coord_converter: "WorkshopMapManager",
        logger: Optional[logging.Logger] = None,
    ) -> Grid3D:
        """生成 3D 体素网格 (Voxel Grid)。

        结合高度图 (Height Map) 和垂直膨胀策略，生成完整的三维障碍物分布。

        Args:
            map_dims (Tuple[int, int, int]): 网格维度 (rows, cols, layers)。
            resolution (float): 网格分辨率 (米/格)。
            obstacles (List[Dict[str, Any]]): 障碍物数据列表。
            margins (Tuple[float, float, float]): 安全边距元组。
                包含 (xy_margin, z_margin_obs, z_margin_ceil)。
            is_infinite (bool): 是否开启无限高障碍物模式。
            coord_converter (WorkshopMapManager): 坐标转换器实例 (MapManager)。
                使用字符串前向引用避免运行时循环依赖。
            logger (Optional[logging.Logger]): 父级日志记录器。

        Returns:
            Grid3D: 3D 体素网格列表。
        """
        # 1. 建立日志上下文
        if logger:
            local_logger = logger.getChild("GridFactory")
        else:
            local_logger = logging.getLogger("GridFactory")

        rows, cols, layers = map_dims
        xy_margin, z_margin_obs, z_margin_ceil = margins
        height_m = layers * resolution

        if not HAS_SCIPY:
            local_logger.warning("未安装 SciPy 库，无法生成 3D 网格！返回空列表。")
            return []

        local_logger.debug(
            f"开始 3D 体素化. Dim: {map_dims}, Obs: {len(obstacles)}, "
            f"Margins: (XY={xy_margin:.1f}, Z_Obs={z_margin_obs:.1f})"
        )

        # 2. 构建高度图 (Height Map)
        # 这是一个 2D 矩阵，存储每个 (x, y) 位置的最高障碍物高度
        height_map = np.zeros((rows, cols), dtype=np.float32)

        for o in obstacles:
            # 获取障碍物在网格中的覆盖范围
            r_s, c_s, _ = coord_converter.world_to_grid(o["x_m"], o["y_m"])
            # 使用 -1e-4 避免浮点数恰好在边界时多占一格
            r_e, c_e, _ = coord_converter.world_to_grid(
                o["x_m"] + o["w_m"] - 1e-4, o["y_m"] + o["h_m"] - 1e-4
            )

            if is_infinite:
                # 无限高模式：高度设为地图顶端以上
                obs_occupy_z = height_m + 1.0
            else:
                # 有限高模式：物体高度 + 垂直安全边距
                obs_occupy_z = o.get("z_m", 100.0) + z_margin_obs

            # 限制索引范围，防止越界
            r_s, r_e = max(0, r_s), min(rows, r_e + 1)
            c_s, c_e = max(0, c_s), min(cols, c_e + 1)

            # 更新高度图：取当前高度与新障碍物高度的最大值
            if r_s < r_e and c_s < c_e:
                height_map[r_s:r_e, c_s:c_e] = np.maximum(
                    height_map[r_s:r_e, c_s:c_e], obs_occupy_z
                )

        # 3. 高度图平面膨胀 (XY Inflation)
        # 使用最大值滤波器 (Maximum Filter) 对高度图进行形态学膨胀
        if xy_margin > 0:
            # 构造圆形卷积核
            search_radius = int(math.ceil(xy_margin + 0.5))
            y, x = np.ogrid[
                -search_radius : search_radius + 1, -search_radius : search_radius + 1
            ]
            mask = x**2 + y**2 <= (xy_margin + 0.5) ** 2

            # 执行滤波：这会让“高”的区域向四周扩散，模拟 3D 柱体的变粗效果
            height_map = maximum_filter(
                height_map,
                footprint=mask,
                mode="constant",
                cval=height_m + 1.0,  # 边界外视为无限高墙
            )

        # 4. 3D 体素化 (Voxelization via Broadcasting)
        # 利用 NumPy 的广播机制快速生成 3D 矩阵

        # z_coords: (1, 1, Layers) -> 表示每层体素中心的物理高度
        z_coords = (np.arange(layers) + 0.5) * resolution
        z_coords = z_coords.reshape(1, 1, -1)

        # height_map: (Rows, Cols, 1) -> 扩展维度以匹配 z_coords
        h_map_3d = height_map.reshape(rows, cols, 1)

        # 判定 1: 障碍物占据 (当前层高度 < 障碍物高度)
        is_obstacle = z_coords < h_map_3d

        # 判定 2: 天花板限制 (吊具自身高度导致无法贴顶飞行)
        # 如果当前层高度 + 吊具半高 > 地图限高，则该层不可用
        is_ceiling_hit = (z_coords + z_margin_ceil) > height_m

        # 判定 3: 地板限制 (防止吊具钻入地下)
        # 如果当前层高度 - 安全边距 < 0，则该层不可用
        is_floor_hit = (z_coords - z_margin_obs) < 0

        # 合并所有阻挡条件
        final_grid_mask = is_obstacle | is_ceiling_hit | is_floor_hit

        # 转换为 int 列表返回
        return final_grid_mask.astype(np.int8).tolist()
