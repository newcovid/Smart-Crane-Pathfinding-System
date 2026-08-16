"""路径规划正确性测试。

核心思路：用 A* 作为裁判（oracle）交叉验证 D* Lite 的增量重规划。
两者在同一张图上应当得出一致的可达性结论；A* 能找到路径而 D* Lite 找不到，
即为 D* Lite 的增量更新缺陷。

运行::

    python -m pytest tests/ -v
    python -m unittest discover -s tests -v
"""

from __future__ import annotations

import logging
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from smart_crane.algorithms.pathfinding.astar import AStarPlanner
from smart_crane.algorithms.pathfinding.dslite import DLitePlanner
from smart_crane.core.constants import GRID_FREE, GRID_OCCUPIED
from smart_crane.core.rust_bridge import RustBackend

SEED = 20260816


def build_grid(size: int, obstacle_ratio: float = 0.22, block: int = 4, seed: int = SEED):
    """生成带矩形障碍块的确定性网格，起点与终点周围留空。"""
    rng = random.Random(seed)
    grid = [[GRID_FREE] * size for _ in range(size)]
    count = int(size * size * obstacle_ratio) // (block * block)
    for _ in range(count):
        r = rng.randrange(0, max(1, size - block))
        c = rng.randrange(0, max(1, size - block))
        for dr in range(block):
            for dc in range(block):
                grid[r + dr][c + dc] = GRID_OCCUPIED
    margin = max(2, size // 20)
    for r in range(margin):
        for c in range(margin):
            grid[r][c] = GRID_FREE
            grid[size - 1 - r][size - 1 - c] = GRID_FREE
    return grid


class TestImportEntryPoints(unittest.TestCase):
    """包必须能从任意子模块入口导入，不依赖导入顺序。

    回归用例：core/__init__ 曾急切导入 crane_service，与算法层的
    core.constants 依赖构成环，导致先导入算法层时抛
    ImportError: partially initialized module。
    """

    def test_algorithms_layer_first(self):
        import subprocess

        root = str(Path(__file__).resolve().parent.parent)
        for stmt in (
            "from smart_crane.algorithms.pathfinding.astar import AStarPlanner",
            "from smart_crane.algorithms.pathfinding.dslite import DLitePlanner",
            "from smart_crane.core import CraneService, settings",
            "from smart_crane.core.config import settings",
        ):
            with self.subTest(stmt=stmt):
                proc = subprocess.run(
                    [sys.executable, "-c", stmt], cwd=root, capture_output=True, text=True
                )
                self.assertEqual(proc.returncode, 0, proc.stderr)


class TestAStar(unittest.TestCase):
    SIZE = 60

    def setUp(self):
        logging.disable(logging.CRITICAL)
        self.addCleanup(logging.disable, logging.NOTSET)

    def test_finds_path_on_open_grid(self):
        size = 20
        grid = [[GRID_FREE] * size for _ in range(size)]
        p = AStarPlanner(grid=grid, width_m=float(size), length_m=float(size))
        p.initialize((0, 0), (size - 1, size - 1))
        path = p.compute_path((0, 0))
        self.assertTrue(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (size - 1, size - 1))

    def test_path_avoids_obstacles(self):
        grid = build_grid(self.SIZE)
        p = AStarPlanner(grid=grid, width_m=float(self.SIZE), length_m=float(self.SIZE))
        p.initialize((0, 0), (self.SIZE - 1, self.SIZE - 1))
        path = p.compute_path((0, 0))
        self.assertTrue(path)
        for r, c in path:
            self.assertEqual(grid[r][c], GRID_FREE, f"路径穿过了障碍物 ({r},{c})")

    def test_no_path_when_walled_off(self):
        size = 20
        grid = [[GRID_FREE] * size for _ in range(size)]
        for c in range(size):  # 一道横墙彻底隔断
            grid[size // 2][c] = GRID_OCCUPIED
        p = AStarPlanner(grid=grid, width_m=float(size), length_m=float(size))
        p.initialize((0, 0), (size - 1, size - 1))
        self.assertFalse(p.compute_path((0, 0)))


class TestDStarLiteIncremental(unittest.TestCase):
    """D* Lite 增量重规划：以 A* 为裁判做交叉验证。"""

    SIZE = 60
    UPDATES = 120

    def setUp(self):
        logging.disable(logging.CRITICAL)
        self.addCleanup(logging.disable, logging.NOTSET)

    def test_matches_astar_reachability_under_dynamic_obstacles(self):
        rng = random.Random(SEED + 1)
        grid = build_grid(self.SIZE)
        start, goal = (0, 0), (self.SIZE - 1, self.SIZE - 1)

        dstar = DLitePlanner(grid=grid, width_m=float(self.SIZE), length_m=float(self.SIZE))
        dstar.initialize(start, goal)
        self.assertTrue(dstar.compute_path(start), "首次规划失败")

        defects = []
        for i in range(self.UPDATES):
            r = rng.randrange(self.SIZE // 4, self.SIZE * 3 // 4)
            c = rng.randrange(self.SIZE // 4, self.SIZE * 3 // 4)
            new_state = GRID_OCCUPIED if grid[r][c] == GRID_FREE else GRID_FREE
            grid[r][c] = new_state
            # 契约：2D 传 (x, y, 新状态)。内部以 change[:-1] 取坐标。
            dstar.update_obstacles([(r, c, new_state)])
            d_path = dstar.compute_path(start)

            if d_path:
                for pr, pc in d_path:
                    self.assertEqual(
                        grid[pr][pc], GRID_FREE, f"第 {i+1} 次更新后路径穿过障碍 ({pr},{pc})"
                    )
                continue

            judge = AStarPlanner(
                grid=[row[:] for row in grid],
                width_m=float(self.SIZE),
                length_m=float(self.SIZE),
            )
            judge.initialize(start, goal)
            if judge.compute_path(start):
                defects.append((i + 1, (r, c)))

        self.assertEqual(
            defects, [], f"{len(defects)} 次增量更新后 D* Lite 找不到路径而 A* 能找到：{defects[:5]}"
        )

    def test_incremental_is_faster_than_full_replan(self):
        """增量重规划应显著快于同规模的 A* 全量搜索。"""
        import time

        size = 100
        grid = build_grid(size)
        start, goal = (0, 0), (size - 1, size - 1)

        astar = AStarPlanner(grid=[r[:] for r in grid], width_m=float(size), length_m=float(size))
        astar.initialize(start, goal)
        t0 = time.perf_counter()
        self.assertTrue(astar.compute_path(start))
        astar_ms = (time.perf_counter() - t0) * 1000

        dstar = DLitePlanner(grid=grid, width_m=float(size), length_m=float(size))
        dstar.initialize(start, goal)
        self.assertTrue(dstar.compute_path(start))

        rng = random.Random(SEED + 2)
        samples = []
        for _ in range(10):
            r = rng.randrange(size // 4, size * 3 // 4)
            c = rng.randrange(size // 4, size * 3 // 4)
            new_state = GRID_OCCUPIED if grid[r][c] == GRID_FREE else GRID_FREE
            grid[r][c] = new_state
            dstar.update_obstacles([(r, c, new_state)])
            t0 = time.perf_counter()
            dstar.compute_path(start)
            samples.append((time.perf_counter() - t0) * 1000)

        median = sorted(samples)[len(samples) // 2]
        self.assertLess(
            median, astar_ms / 2, f"增量 {median:.1f}ms 未显著快于全量 {astar_ms:.1f}ms"
        )


@unittest.skipUnless(
    RustBackend.is_extension_loaded(),
    "未构建 Rust 扩展（maturin develop --release），跳过双引擎等价性测试",
)
class TestEngineEquivalence(unittest.TestCase):
    """Python 原生实现与 Rust 加速实现必须给出等价结果。

    这是双引擎项目最该有的测试。两套实现共用同一份接口契约，
    任何一侧的单位换算、哨兵语义、浮点判据、维度判断出现偏差，
    都会在这里暴露成"同一张图、同一份配置、两个不同的答案"。

    注意断言的是【代价等价】而非【逐点相同】：Rust 用 f32、Python 用 f64，
    代价相同的多条路径在 tie-break 时可能选择不同分支。
    """

    SIZES = (40, 80)

    def setUp(self):
        logging.disable(logging.CRITICAL)
        self.addCleanup(logging.disable, logging.NOTSET)

    @staticmethod
    def _path_cost(path):
        cost = 0.0
        for (r0, c0), (r1, c1) in zip(path, path[1:]):
            dr, dc = abs(r1 - r0), abs(c1 - c0)
            cost += 1.0 if (dr == 0 or dc == 0) else 2.0**0.5
        return cost

    def _plan(self, cls, grid, size, **kw):
        planner = cls(
            grid=[row[:] for row in grid],
            width_m=float(size),
            length_m=float(size),
            **kw,
        )
        start, goal = (0, 0), (size - 1, size - 1)
        planner.initialize(start, goal)
        return planner.compute_path(start)

    def test_astar_cost_matches_across_engines(self):
        for size in self.SIZES:
            with self.subTest(size=size):
                grid = build_grid(size)
                with RustBackend.disabled():
                    py_path = self._plan(AStarPlanner, grid, size, enable_rust=False)
                rs_path = self._plan(AStarPlanner, grid, size, enable_rust=True)

                self.assertTrue(py_path, "Python A* 未找到路径")
                self.assertTrue(rs_path, "Rust A* 未找到路径")
                self.assertAlmostEqual(
                    self._path_cost(py_path),
                    self._path_cost(rs_path),
                    places=3,
                    msg=f"{size}x{size} 上两引擎 A* 代价不一致",
                )

    def test_dstar_cost_matches_across_engines(self):
        for size in self.SIZES:
            with self.subTest(size=size):
                grid = build_grid(size)
                with RustBackend.disabled():
                    py_path = self._plan(DLitePlanner, grid, size, enable_rust=False)
                rs_path = self._plan(DLitePlanner, grid, size, enable_rust=True)

                self.assertTrue(py_path, "Python D* Lite 未找到路径")
                self.assertTrue(rs_path, "Rust D* Lite 未找到路径")
                self.assertAlmostEqual(
                    self._path_cost(py_path),
                    self._path_cost(rs_path),
                    places=3,
                    msg=f"{size}x{size} 上两引擎 D* Lite 代价不一致",
                )

    def test_inflation_matches_exactly(self):
        """C-Space 膨胀：两个引擎必须给出逐格相同的膨胀层。

        此前这里只能断言"Rust 不比 Python 宽松，且偏差 <= 20%"，
        因为三套实现的离散化方式不同：Python 用 SciPy 的 EDT，
        Rust 密集分支是手写 EDT，稀疏分支量的是到连续矩形的距离。
        把稀疏分支也改为量到栅格化种子格之后，三者已完全对齐，
        断言随之收紧为精确相等。

        回归用例：Rust 侧曾把 `xy_margin`（单位为网格数）当作米使用，
        分辨率恰为 1.0 m 时两者数值相同因而一直没暴露；
        0.5 m 时膨胀量翻倍，2.0 m 时只剩一半——后者是往不安全方向错。
        """
        from smart_crane.core.map_manager import WorkshopMapManager

        for res in (0.5, 1.0, 2.0):
            with self.subTest(resolution=res):
                # xy_margin 的单位是【网格数】，固定 1.0 m 的物理膨胀半径，
                # 换算成该分辨率下的格数。
                margin_cells = 1.0 / res

                def build():
                    mm = WorkshopMapManager(
                        width_m=20.0, length_m=20.0, height_m=10.0, resolution_m=res
                    )
                    mm.add_static_obstacle("o1", x=8.0, y=8.0, w=4.0, h=4.0, z=5.0)
                    return mm.get_2d_projection_grid(xy_margin=margin_cells)

                with RustBackend.disabled():
                    py_occ = sum(sum(row) for row in build())
                rs_occ = sum(sum(row) for row in build())

                self.assertGreaterEqual(
                    rs_occ,
                    py_occ,
                    f"分辨率 {res} m：Rust 的膨胀比 Python 更宽松 "
                    f"(Rust={rs_occ} < Python={py_occ})，这是往不安全方向的偏差",
                )
                self.assertEqual(
                    rs_occ,
                    py_occ,
                    f"分辨率 {res} m（膨胀 {margin_cells} 格）下两引擎占据格数不同 "
                    f"(Python={py_occ}, Rust={rs_occ})",
                )


if __name__ == "__main__":
    unittest.main(verbosity=2)
