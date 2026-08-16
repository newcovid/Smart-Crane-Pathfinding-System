"""C-Space 膨胀网格的正确性测试。

覆盖两条容易被破坏的不变量：

1. **静态层缓存不改变结果**——把同一批障碍物拆成静态 + 动态登记，
   与全部登记为静态，必须得到逐格相同的网格。
2. **两条膨胀分支等价**——障碍物数量跨过 ``OBSTACLE_COUNT_THRESHOLD``
   会在"逐障碍绘制"与"全局 EDT"之间切换，两者结果必须一致，
   否则同一场景的安全边界会随障碍物增减而突变。

这两条都属于安全性质：一旦膨胀层变窄，规划器就会允许吊具靠得更近。
"""

from __future__ import annotations

import logging
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from smart_crane.core.map_manager import WorkshopMapManager
from smart_crane.core.rust_bridge import RustBackend

SEED = 20260817
MARGIN = 2.0


def _build(n_static, n_dynamic, *, split, size=120.0, resolution=1.0, seed=SEED):
    """构造场景。

    split=True  -> 动态障碍物登记为 dynamic（走静态层缓存 + 动态叠加）
    split=False -> 全部登记为 static（等价于不拆分的合并计算）
    """
    rng = random.Random(seed)
    mm = WorkshopMapManager(
        width_m=size, length_m=size, height_m=20.0, resolution_m=resolution
    )
    for i in range(n_static):
        mm.add_static_obstacle(
            f"s{i}", x=rng.uniform(0, size - 6), y=rng.uniform(0, size - 6),
            w=4.0, h=4.0, z=5.0,
        )
    for i in range(n_dynamic):
        x, y = rng.uniform(0, size - 4), rng.uniform(0, size - 4)
        add = mm.update_dynamic_obstacle if split else mm.add_static_obstacle
        add(f"d{i}", x=x, y=y, w=2.0, h=2.0, z=5.0)
    return mm


def _diff(grid_a, grid_b):
    """返回 (a 多占的格数, a 少占的格数)。"""
    more = less = 0
    for ra, rb in zip(grid_a, grid_b):
        for x, y in zip(ra, rb):
            if x and not y:
                more += 1
            elif y and not x:
                less += 1
    return more, less


class _GridTestBase(unittest.TestCase):
    def setUp(self):
        logging.disable(logging.CRITICAL)
        self.addCleanup(logging.disable, logging.NOTSET)


class TestStaticLayerCache(_GridTestBase):
    """静态层缓存必须是纯粹的性能优化，不得改变任何一格。"""

    SCENARIOS = ((10, 4), (40, 6), (80, 4), (60, 20), (120, 8))

    def _assert_identical(self, engine_label):
        for n_static, n_dynamic in self.SCENARIOS:
            with self.subTest(engine=engine_label, static=n_static, dynamic=n_dynamic):
                split = _build(n_static, n_dynamic, split=True).get_2d_projection_grid(
                    xy_margin=MARGIN
                )
                merged = _build(n_static, n_dynamic, split=False).get_2d_projection_grid(
                    xy_margin=MARGIN
                )
                more, less = _diff(split, merged)
                self.assertEqual(
                    less, 0,
                    f"拆分后有 {less} 格由占据变为空闲——膨胀层变窄，放宽了安全边界",
                )
                self.assertEqual(
                    (more, less), (0, 0),
                    f"拆分与合并结果不一致（多占 {more}，少占 {less}）",
                )

    def test_python_engine(self):
        with RustBackend.disabled():
            self._assert_identical("python")

    @unittest.skipUnless(RustBackend.is_extension_loaded(), "未构建 Rust 扩展")
    def test_rust_engine(self):
        self._assert_identical("rust")


class TestInflationBranchConsistency(_GridTestBase):
    """跨越密度阈值时，两条膨胀分支必须给出相同的膨胀层。

    Rust 侧在障碍物数 > OBSTACLE_COUNT_THRESHOLD(50) 时切换到全局 EDT，
    否则逐障碍绘制。两者度量的都应当是"格心到栅格化种子格心"的距离；
    早期实现中绘制分支量的是到连续矩形的距离，系统性偏小，
    导致同一场景在阈值两侧的安全边界不同。
    """

    @unittest.skipUnless(RustBackend.is_extension_loaded(), "未构建 Rust 扩展")
    def test_rust_branches_agree_with_python_edt(self):
        # 48 与 56 分别落在阈值 50 的两侧
        for n in (48, 56):
            with self.subTest(obstacles=n):
                rust_grid = _build(n, 0, split=False).get_2d_projection_grid(
                    xy_margin=MARGIN
                )
                with RustBackend.disabled():
                    py_grid = _build(n, 0, split=False).get_2d_projection_grid(
                        xy_margin=MARGIN
                    )
                more, less = _diff(rust_grid, py_grid)
                self.assertEqual(
                    less, 0,
                    f"{n} 个障碍物时 Rust 的膨胀层比 Python 窄 {less} 格",
                )

    def test_consistent_across_resolutions(self):
        """非 1.0 分辨率下拆分同样不得改变结果。

        回归用例：xy_margin 的单位是网格数，一旦某条分支误按米处理，
        分辨率偏离 1.0 时就会暴露。
        """
        for res in (0.5, 1.0, 2.0):
            with self.subTest(resolution=res):
                with RustBackend.disabled():
                    split = _build(30, 5, split=True, resolution=res).get_2d_projection_grid(
                        xy_margin=1.0 / res
                    )
                    merged = _build(30, 5, split=False, resolution=res).get_2d_projection_grid(
                        xy_margin=1.0 / res
                    )
                more, less = _diff(split, merged)
                self.assertEqual((more, less), (0, 0), f"分辨率 {res} m 下拆分改变了结果")


if __name__ == "__main__":
    unittest.main(verbosity=2)
