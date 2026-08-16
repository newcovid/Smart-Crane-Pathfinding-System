"""路径规划基准测试。

对比 Python 原生实现与 Rust 加速实现在不同网格规模下的表现。
Rust 扩展不可用时自动跳过对应列，只输出 Python 侧数据。

用法::

    python benchmarks/bench.py
    python benchmarks/bench.py --sizes 100 250 500 --repeat 5

地图使用固定随机种子生成，因此不同机器、不同引擎之间的结果可比。
"""

from __future__ import annotations

import argparse
import random
import statistics
import sys
import time
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from smart_crane.algorithms.pathfinding.astar import AStarPlanner  # noqa: E402
from smart_crane.algorithms.pathfinding.dslite import DLitePlanner  # noqa: E402
from smart_crane.core.constants import GRID_FREE, GRID_OCCUPIED  # noqa: E402
from smart_crane.core.rust_bridge import HAS_RUST_CORE, RustBackend  # noqa: E402

Grid = List[List[int]]
SEED = 20260816
OBSTACLE_RATIO = 0.22
BLOCK_SIZE = 4


def build_grid(size: int, seed: int = SEED) -> Grid:
    """生成带矩形障碍块的网格。

    使用固定种子，保证同一 size 在任何机器上得到完全相同的地图。
    起点 (0,0) 与终点 (size-1,size-1) 周围留出空地，确保路径存在。
    """
    rng = random.Random(seed)
    grid: Grid = [[GRID_FREE] * size for _ in range(size)]

    count = int(size * size * OBSTACLE_RATIO) // (BLOCK_SIZE * BLOCK_SIZE)
    for _ in range(count):
        r = rng.randrange(0, max(1, size - BLOCK_SIZE))
        c = rng.randrange(0, max(1, size - BLOCK_SIZE))
        for dr in range(BLOCK_SIZE):
            for dc in range(BLOCK_SIZE):
                grid[r + dr][c + dc] = GRID_OCCUPIED

    margin = max(2, size // 20)
    for r in range(margin):
        for c in range(margin):
            grid[r][c] = GRID_FREE
            grid[size - 1 - r][size - 1 - c] = GRID_FREE
    return grid


def _time_ms(fn: Callable[[], object]) -> Tuple[float, object]:
    t0 = time.perf_counter()
    result = fn()
    return (time.perf_counter() - t0) * 1000.0, result


def bench_astar(size: int, repeat: int) -> Tuple[Optional[float], int]:
    """A* 全局规划：每次重建规划器，测量一次完整搜索。"""
    samples: List[float] = []
    steps = 0
    for _ in range(repeat):
        grid = build_grid(size)
        planner = AStarPlanner(grid=grid, width_m=float(size), length_m=float(size))
        start, goal = (0, 0), (size - 1, size - 1)
        planner.initialize(start, goal)
        ms, path = _time_ms(lambda: planner.compute_path(start))
        if not path:
            return None, 0
        samples.append(ms)
        steps = len(path)
    return statistics.median(samples), steps


def bench_dstar_incremental(size: int, repeat: int, updates: int = 20) -> Optional[float]:
    """D* Lite 增量重规划：测量首次规划之后，每次障碍物变更的重规划耗时。

    这是 D* Lite 相对 A* 的核心价值所在——只重算受影响节点，不做全量搜索。
    """
    rng = random.Random(SEED + 1)
    samples: List[float] = []

    for _ in range(repeat):
        grid = build_grid(size)
        planner = DLitePlanner(grid=grid, width_m=float(size), length_m=float(size))
        start, goal = (0, 0), (size - 1, size - 1)
        planner.initialize(start, goal)
        if not planner.compute_path(start):  # 首次规划不计入
            return None

        for _ in range(updates):
            r = rng.randrange(size // 4, size * 3 // 4)
            c = rng.randrange(size // 4, size * 3 // 4)
            new_state = GRID_OCCUPIED if grid[r][c] == GRID_FREE else GRID_FREE
            grid[r][c] = new_state
            # 注意 update_obstacles 的契约：2D 传 (x, y, 新状态)，3D 传 (x, y, z, 新状态)。
            # 内部用 change[:-1] 取坐标，少传一位会被解析成非法节点并静默跳过。
            # update_obstacles 与 compute_path 一起计时：一次环境变化的
            # 真实成本是"通知规划器 + 取出新路径"之和，只计后者会偏乐观约 8%。
            def _replan():
                planner.update_obstacles([(r, c, new_state)])
                return planner.compute_path(start)

            ms, _ = _time_ms(_replan)
            samples.append(ms)

    return statistics.median(samples) if samples else None


def fmt(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:,.1f}"


def _run_table(sizes: Sequence[int], repeat: int, updates: int, label: str) -> None:
    print(f"\n=== {label} ===")
    header = (
        f"{'网格':>12} | {'A* 全局 (ms)':>14} | "
        f"{'D* Lite 增量 (ms)':>18} | {'加速比':>7} | {'路径步数':>8}"
    )
    print(header)
    print("-" * len(header))
    for size in sizes:
        astar_ms, steps = bench_astar(size, repeat)
        dstar_ms = bench_dstar_incremental(size, repeat, updates)
        ratio = (
            f"{astar_ms / dstar_ms:.0f}x" if (astar_ms and dstar_ms) else "n/a"
        )
        print(
            f"{size}x{size:<7} | {fmt(astar_ms):>14} | "
            f"{fmt(dstar_ms):>18} | {ratio:>7} | {steps:>8}"
        )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sizes", type=int, nargs="+", default=[100, 200, 400])
    parser.add_argument("--repeat", type=int, default=3, help="每项取中位数的重复次数")
    parser.add_argument("--updates", type=int, default=20, help="D* Lite 增量更新次数")
    parser.add_argument(
        "--engine",
        choices=("auto", "python", "rust", "both"),
        default="auto",
        help=(
            "auto=按扩展可用性自动选择；python=强制纯 Python；"
            "rust=强制 Rust（缺扩展则报错）；both=同一次运行内两者都跑，输出对照表"
        ),
    )
    args = parser.parse_args(argv)

    print(f"障碍占比 {OBSTACLE_RATIO:.0%}，{BLOCK_SIZE}x{BLOCK_SIZE} 矩形块，种子 {SEED}")
    print(f"重复 {args.repeat} 次取中位数，每次 {args.updates} 轮增量更新")
    print(f"Rust 扩展: {'已加载' if HAS_RUST_CORE else '未检测到'}")

    if args.engine in ("rust", "both") and not HAS_RUST_CORE:
        print("\n错误: 指定了 rust 引擎但扩展不可用。请先 `maturin develop --release`。")
        return 1

    if args.engine == "both":
        with RustBackend.disabled():
            _run_table(args.sizes, args.repeat, args.updates, "Python 原生")
        _run_table(args.sizes, args.repeat, args.updates, "Rust 加速")
    elif args.engine == "python":
        with RustBackend.disabled():
            _run_table(args.sizes, args.repeat, args.updates, "Python 原生")
    else:
        label = "Rust 加速" if RustBackend.is_available() else "Python 原生"
        _run_table(args.sizes, args.repeat, args.updates, label)

    if not HAS_RUST_CORE:
        print(
            "\n提示: 未检测到 Rust 扩展，以上为 Python 原生实现的数据。\n"
            "      构建后用 --engine both 可在同一次运行内得到双引擎对照表：\n"
            "        pip install maturin && maturin develop --release\n"
            "        python benchmarks/bench.py --engine both"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
