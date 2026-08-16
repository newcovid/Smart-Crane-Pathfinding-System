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


if __name__ == "__main__":
    unittest.main(verbosity=2)
