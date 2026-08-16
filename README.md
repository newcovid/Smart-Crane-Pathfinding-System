# Smart Crane Pathfinding System

工业天车（桥式起重机）的路径规划原型系统。Rust 算法核心 + Python 业务层 + Web 可视化，
在含静态设备与动态障碍物的车间环境中，实时解算无碰撞、符合运动学约束的运行轨迹。

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
![Rust](https://img.shields.io/badge/Rust-PyO3-000000?logo=rust&logoColor=white)
![Tests](https://img.shields.io/badge/tests-6%20passing-4c1)
![License](https://img.shields.io/badge/license-MIT-blue)

---

## 这个项目在解决什么

天车吊运的路径规划有三个绕不开的约束，任何一个用通用的网格寻路 Demo 都处理不了：

1. **吊具和货物有体积。** 不能把起重机当质点，必须按物理足迹（footprint）膨胀障碍物，
   在构型空间（C-Space）里规划。
2. **环境是动态的。** 叉车、人员、临时堆放物随时出现。每次变化都全量重算 A*，
   在 300×300 以上的栅格上就跟不上节拍了。
3. **重载设备不能急转。** 栅格搜索产生的锯齿路径直接给到变频器，
   会造成大车小车反复加减速冲击。

对应的三层解法就是这个仓库的主要内容。

---

## 核心设计

### 分层：按变更频率切分，不是按技术栈切分

| 层 | 实现 | 变更频率 | 为什么放这里 |
|---|---|---|---|
| 算法核心 | Rust（PyO3） | 低 | 重载设备算错路径的物理代价高，这一层要的是确定性和速度 |
| 业务与编排 | Python | **高** | 工业项目需求变更频繁，接口和策略要能快速迭代 |
| 可视化 | Vue + Three.js | 中 | 让不写代码的人能直接验证路径是否合理 |

算法层和业务层的变更频率差一个数量级，所以必须解耦。
这不是"用 Rust 显得高级"，是**把稳定的东西和易变的东西分开**。

### 算法链路

```
点云/栅格输入
   ↓  C-Space 膨胀（按足迹直径 + 安全边距）
   ↓  A*  全局规划          ── 静态最优路径
   ↓  D* Lite  增量重规划    ── 障碍物变化时只重算受影响节点
   ↓  贪婪捷径优化           ── 去掉栅格锯齿，提取关键转折点
   ↓  二阶贝塞尔平滑         ── 生成符合机械惯性的圆滑曲线
最终轨迹点序列
```

**D\* Lite 是这里的关键。** 障碍物移动时，它基于已有搜索树做增量修补，
而不是丢弃一切从头搜索。代价从"整张图"降到"受影响的局部"。

### Rust 与 Python 双引擎，优雅降级

`smart_crane/core/rust_bridge.py` 在导入时探测 Rust 扩展：

- 扩展可用 → 走 Rust 实现
- 扩展缺失 → 自动回落到功能完整的 Python 实现（不是桩，是等价实现）
- 也可用环境变量 `ENABLE_RUST_CORE=false` 强制走 Python

好处是：没有 Rust 工具链的人 `pip install -r requirements.txt` 之后能直接跑，
而两套实现共用同一组测试，可以互相验证。

---

## 性能实测

固定随机种子生成的地图（22% 障碍占比，4×4 矩形块），重复 3 次取中位数。
测量的是**同一张图上，A\* 全量搜索** 与 **D\* Lite 在一次障碍物变更后的增量重规划**。

**Python 原生实现**（Python 3.14，Windows x64）：

| 网格 | A* 全量 (ms) | D* Lite 增量 (ms) | 增量加速比 | 路径步数 |
|---:|---:|---:|---:|---:|
| 100×100 | 9.7 | 0.8 | **12×** | 114 |
| 200×200 | 30.5 | 1.9 | **16×** | 226 |
| 300×300 | 130.0 | 3.6 | **36×** | 351 |
| 400×400 | 134.8 | 4.5 | **30×** | 443 |

规模越大，增量重规划的优势越明显——这正是它在动态环境下的价值所在。

自行复现：

```bash
python benchmarks/bench.py --sizes 100 200 300 400 --repeat 3
```

脚本会自动探测 Rust 扩展；构建之后重跑即可得到 Rust 侧的对照数据。

> Rust 与 Python 的横向对比数据尚未在本机测得（缺 Rust 工具链），
> 因此本 README 不给出该比值。基准脚本已就绪，构建后一条命令即可产出。

---

## 快速开始

```bash
git clone https://github.com/newcovid/Smart-Crane-Pathfinding-System.git
cd Smart-Crane-Pathfinding-System
pip install -r requirements.txt
python app.py
```

浏览器打开 `http://127.0.0.1:5000`。前端资源已全部本地化（`static/vendor/`），无需联网。

### 启用 Rust 加速（可选）

```bash
pip install maturin
maturin develop --release
```

编译产物不入库——它绑定具体的 Python 小版本，换个版本就无法导入。
请本地构建，或从 Releases 取对应版本的 wheel。

### 运行测试

```bash
python -m unittest discover -s tests -v
```

---

## 项目结构

```text
.
├── app.py                          # Flask + Socket.IO 入口
├── smart_crane/
│   ├── core/
│   │   ├── config.py               # Pydantic Settings 配置
│   │   ├── constants.py            # 全局常量（避免魔法数字散落）
│   │   ├── crane_service.py        # 业务控制器，维护规划器生命周期
│   │   ├── map_manager.py          # 车间地图状态（线程安全）
│   │   ├── rust_bridge.py          # Rust 扩展探测与降级
│   │   └── components/
│   │       └── grid_factory.py     # C-Space 膨胀与栅格生成
│   └── algorithms/
│       ├── trajectory_planner.py   # 策略上下文，编排完整链路
│       ├── pathfinding/            # base / astar / dslite
│       ├── post_processing/        # base / greedy / bezier
│       └── components/             # planner_factory / safety_guard / grid_adapter
├── src/                            # Rust 核心（PyO3）
│   ├── pathfinding/                # astar.rs / dslite.rs
│   ├── post_processing/            # greedy.rs / bezier.rs
│   ├── map/                        # manager.rs / grid_factory.rs
│   └── components/                 # safety_guard.rs / grid_adapter.rs
├── static/                         # Vue + Three.js 前端（含本地化 vendor）
├── templates/main.html
├── benchmarks/bench.py             # 性能基准
└── tests/test_pathfinding.py       # 正确性测试（以 A* 为裁判交叉验证 D* Lite）
```

---

## 已实现 / 未实现

诚实划一下边界。这是一个**原型**，不是可以直接上产线的产品。

**已实现**

- A*、D* Lite（含增量更新）、贪婪捷径优化、二阶贝塞尔平滑
- 基于足迹的 C-Space 膨胀，支持 2D 与 3D 栅格
- Rust 核心 + Python 等价实现，运行时自动选择
- 碰撞守卫（起点/终点位于障碍内时拒绝任务）
- Web 端 2D 拓扑与 3D 场景双视图，Socket.IO 实时同步
- 配置热更新、异步日志与轮转

**预留接口但未实现**

- 前端雷达点云 → 栅格的转换接口
- 后端变频器控制接口
- **多车联动与防碰撞调度**——架构上预留了位置，未落地验证

项目因业务变更中止，上述三项停在接口定义阶段。

---

## 一个值得记录的坑

`update_obstacles()` 的入参契约是 **`(x, y, 新状态)`**（3D 为 `(x, y, z, 新状态)`），
内部以 `change[:-1]` 取坐标。

少传最后一位的话，坐标会被截断成非法节点，`is_valid()` 返回 False 后**静默跳过**——
栅格改了，但规划器从未收到通知。表现为搜索"认为自己已收敛"，
而 g 值场与实际地图脱节，梯度回溯随即在两点间震荡并触发环路检测。

写基准脚本时踩了这个坑，一度误判成 D* Lite 的增量更新有缺陷。
教训是：**这类静默跳过应该报错或至少打警告，而不是 `continue`。**
`tests/test_pathfinding.py` 里已用 A* 作为裁判把这条路径覆盖住了。

---

## 许可证

[MIT](LICENSE)

第三方前端资源位于 `static/vendor/`，各自遵循其原始许可证
（Vue、Three.js、Tailwind CSS、Socket.IO、Inter 字体、Phosphor Icons）。
