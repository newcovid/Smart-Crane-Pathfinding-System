# 第三方组件声明

本项目为支持内网隔离环境下的离线部署，将前端依赖全部本地化到 `static/vendor/`。
这些文件不属于本项目，各自遵循下列许可证。

`manage_deps.py` 负责抓取与更新这些依赖。

---

## 前端运行时（`static/vendor/`）

| 组件 | 版本来源 | 许可证 | 位置 |
|---|---|---|---|
| [Vue.js](https://github.com/vuejs/core) | 全局构建版 | MIT | `vue.global.js` |
| [Three.js](https://github.com/mrdoob/three.js) | ES Module 构建版 | MIT | `three.module.js`、`three/examples/jsm/` |
| [Socket.IO Client](https://github.com/socketio/socket.io-client) | 压缩版 | MIT | `socket.io.min.js` |
| [Tailwind CSS](https://github.com/tailwindlabs/tailwindcss) | 浏览器内 JIT 版 | MIT | `tailwindcss.js` |
| [Inter](https://github.com/rsms/inter) | v4.1 | SIL Open Font License 1.1 | `fonts/` |
| [Phosphor Icons](https://github.com/phosphor-icons/web) | Web 字体版 | MIT | `phosphor/` |

> **字体许可证提醒**：Inter 采用 SIL OFL 1.1，该协议要求随字体一同分发许可证全文，
> 且字体本身不得单独售卖。如需在衍生作品中重命名字体，请注意 OFL 的保留字体名称条款。

## Rust 依赖（`Cargo.toml`）

| Crate | 许可证 |
|---|---|
| [pyo3](https://github.com/PyO3/pyo3) | Apache-2.0 |
| [pyo3-log](https://github.com/vorner/pyo3-log) | Apache-2.0 OR MIT |
| [log](https://github.com/rust-lang/log) | Apache-2.0 OR MIT |
| [ordered-float](https://github.com/reem/rust-ordered-float) | MIT |

完整依赖树（含传递依赖）可用 `cargo tree` 查看，
或用 `cargo install cargo-about && cargo about generate` 生成完整清单。

## Python 依赖（`requirements.txt`）

| 包 | 许可证 |
|---|---|
| Flask | BSD-3-Clause |
| Flask-SocketIO | MIT |
| simple-websocket | MIT |
| python-dotenv | BSD-3-Clause |
| NumPy | BSD-3-Clause |
| SciPy | BSD-3-Clause |
| pydantic-settings | MIT |

---

## 本项目自身

`smart_crane/`、`src/`、`app.py`、`tests/`、`benchmarks/` 及 `static/`（`vendor/` 除外）
采用 [MIT 许可证](LICENSE)。

## 许可证全文

Vue、Three.js、Socket.IO 的压缩产物文件头自带版权与许可证声明。
Tailwind CSS 的浏览器构建版、Inter 与 Phosphor 的字体二进制不含内嵌声明，
其许可证全文请从上表链接的上游仓库获取。

将本项目用于对外分发（而非自用或演示）时，
**须先将各组件的 LICENSE 全文置于 `static/vendor/LICENSES/`**——
SIL OFL 1.1 对随附许可证全文是强制要求，MIT 亦要求保留版权声明。
