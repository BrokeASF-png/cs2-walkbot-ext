# CS2 WalkBot (External)

一个完全外部（无注入、无 Hook）的 Counter-Strike 2 路径机器人，附带 ImGui/DX11 可视化悬浮窗。
通过 `ReadProcessMemory` 读取 `cs2.exe` 游戏状态，依据录制好的航点图驱动合成键鼠输入完成巡逻 / 跟随 / 分支移动。

> ⚠️ 仅供学习、逆向研究与图形/算法实验。请勿用于在线对战或违反 Valve 服务条款的场景，风险自负。

---

## 功能概览

- **纯外部**：不注入 DLL、不 Hook 游戏函数，仅使用 `ReadProcessMemory` + `SendInput`。
- **航点系统**：支持普通 / 多分支（Sequential / Random / Cycle）航点，可录制、编辑、分支链接。
- **扩展航点字段**（向后兼容旧 `paths.json`）：
  - `flags` 位掩码：`CROUCH` / `JUMP` / `WALK` / `SNIPER` / `LADDER` / `PRECISE`
  - 每航点 `radius` / `wait_time` / `desired_speed` / `gaze_target`
- **三种瞄准后端**，运行时可切换：
  - **Legacy Smooth** — 标量 EMA，快速确定。
  - **Adaptive Servo** — 带 jerk 限制的 PID 伺服，带人化 cadence 包络。
  - **WindMouse + Fitts** — BenLand100 风噪声累加 + Fitts's Law 目标速度，天然的 start-glide-settle 曲线。
- **运动预测**：速度/加速度平滑，前瞄补偿移动目标。
- **紧急刹车**：停止时自动发反向键抵消滑行。
- **透明悬浮窗**：独立线程 `WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST`，基于状态签名节流重绘。

概念图见：[`docs/walkbot_concept.html`](docs/walkbot_concept.html)（单页可视化，浏览器直接打开）。

---

## 构建

需要：

- Visual Studio 2022（或更新版本，Toolset v145）
- Windows 10/11 SDK
- C++20
- 依赖已内置在 `include/`（`nlohmann/json.hpp` 精简版 + CS2 SDK 头）和 `CS2_WalkBot_Ext/imgui`

**命令行构建（Release x64）：**

```bash
msbuild CS2_WalkBot_Ext.slnx -p:Configuration=Release -p:Platform=x64 -m
```

或直接用 VS 打开 `CS2_WalkBot_Ext.slnx`。输出：`x64\Release\CS2_WalkBot_Ext.exe`。

链接库：`d3d11.lib`、`dxgi.lib`、`d3dcompiler.lib`、`dwmapi.lib`。

---

## 架构

```
cs2.exe memory ─┐
                ├─> interfaces ─> PathRunnerState ─> Aim Backend ─> SendInput
process/memory ─┘                       │
                                        └─> Overlay Thread ─> WorldToScreen
```

| 模块 | 文件 | 职责 |
|---|---|---|
| 进程/内存 | `process.*`, `memory.h` | 附加 `cs2.exe`、模式扫描、RIP 相对解析 |
| 接口层 | `interfaces.h`, `game_types.h`, `entities.*`, `sdk/` | 解析 LocalPlayer / EntityList / ViewMatrix |
| 航点 | `path.*`, `paths.json` | `PathManager` 维护图 + 分支游标持久化 |
| 运行时 | `walkbot_core.*`, `path_runner.*` | 目标选择、姿态决策、瞄准分发 |
| UI / 悬浮窗 | `main.cpp`, `walkbot_ui.*` | 控制窗 + 独立透明悬浮窗线程 |
| 工具 | `tools/*.py` | 从 demo / awpy 生成 `paths.json` |

**CS2 更新后通常的修复点**：
1. `interfaces.h::Initialize()` 中的模式扫描签名
2. `game_types.h` / `sdk/*.hpp` 里的 `MEM_PAD(offset)` 偏移

---

## 配置

`config.json` 与可执行文件同目录，UI 内几乎全部参数可持久化。
开/停默认热键 `F6` / `F7`（`start_hotkey_vk` / `stop_hotkey_vk`）。

**WindMouse + Fitts 参数**（新）：

| 键 | 默认 | 说明 |
|---|---|---|
| `wind_gravity` | 9.0 | 向 Fitts 目标速度收敛的指数速率 |
| `wind_force` | 80.0 | 风噪声强度（counts/s²） |
| `wind_max_step` | 1600 | 速度上限 |
| `wind_damping` | 12.0 | 该距离以内衰减噪声，保证末端平滑 |
| `wind_fitts_a` | 0.080 | Fitts a（基础延迟 s） |
| `wind_fitts_b` | 0.090 | Fitts b（每 bit 成本 s/bit） |
| `wind_fitts_tolerance` | 0.8 | Fitts W（目标容忍宽度） |

---

## 目录结构

```
CS/
├── CLAUDE.md               # Claude Code 用的项目指南
├── CS2_WalkBot_Ext.slnx    # VS 2022 解决方案
├── CS2_WalkBot_Ext/        # 源码 + 资源
│   ├── main.cpp
│   ├── walkbot_core.*      # PathRunnerState + 瞄准算法
│   ├── path_runner.cpp     # 每 tick 决策循环
│   ├── path.*              # PathManager / Waypoint
│   ├── walkbot_ui.*        # ImGui 控制面板
│   ├── interfaces.h        # 模式扫描 & 全局接口
│   ├── sdk/                # CS2 SDK 头
│   ├── imgui/              # 内置 ImGui
│   └── paths.json          # 录制好的航点图
├── docs/
│   └── walkbot_concept.html  # 架构 + 瞄准算法概念图
├── include/                # 外部依赖（json、CS2 SDK dump）
├── tools/                  # Python 路径生成脚本
└── x64/                    # 构建输出（已 gitignore）
```

---

## License

暂未指定。默认保留所有权利 —— 若需使用请先联系仓库所有者。
