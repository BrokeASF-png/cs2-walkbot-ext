# CS2 WalkBot (External)

**CS2 外部路径机器人 — 纯外部、无注入的 Counter-Strike 2 自动巡逻工具**

A fully external (no injection, no hooks) Counter-Strike 2 walkbot with humanized aim, waypoint path running, and a DX11 overlay.

---

## Overview / 概述

通过 `ReadProcessMemory` 读取 `cs2.exe` 游戏状态，沿录制好的航点图驱动合成键鼠输入（`SendInput`）完成巡逻、跟随、分支移动。程序不注入 DLL、不 Hook 游戏函数，只读内存 + 模拟输入。透明 DX11 悬浮窗实时显示航点和连线。

Reads `cs2.exe` game state via `ReadProcessMemory`, drives synthesized keyboard/mouse input (`SendInput`) along a recorded waypoint graph, and renders a transparent DX11 overlay. No DLL injection, no in-game hooks — memory is read-only, input is synthesized externally.

---

## Features / 功能

- **纯外部 / Fully external** — `ReadProcessMemory` + `SendInput`，不碰游戏进程内部
- **航点系统 / Waypoint graph** — 普通航点与多分支航点；分支选择 Sequential / Random / Cycle；录制、编辑、持久化到 `paths.json`
- **三种瞄准后端 / Three aim backends (runtime switchable):**
  - **Legacy Smooth** — 标量 EMA 平滑
  - **Adaptive Servo** — PID 伺服，带 jerk 限制，可调 response / damping / max acceleration / max jerk / max speed
  - **Humanized (WindMouse + Fitts)** — 风噪声 + Fitts's Law 目标速度，产生自然的 start-glide-settle 曲线，叠加 tremor / drift / 随机 kick
- **运动预测 / Movement prediction** — 速度加速度平滑 + 前瞄补偿（look-ahead）
- **紧急刹车 / Emergency brake** — 停止时自动反向键抵消滑行
- **敌人跟踪 / Enemy tracking** — 就近敌人跟踪、在最近敌人航点等待、可将队友视为敌人
- **透明悬浮窗 / Click-through overlay** — 独立线程 `WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST`，`WorldToScreen` 投影航点，状态签名节流重绘
- **Python 路径生成 / Path generation tools** — 从 demo、awpy 数据或手工布局生成 `paths.json`，附中心线简化器

概念图 / Concept diagram: [`docs/walkbot_concept.html`](docs/walkbot_concept.html)

---

## Tech Stack / 技术栈

- **C++20** — Windows 控制台应用，Unicode
- **Visual Studio 2022** — PlatformToolset v145, Windows 10 SDK
- **Dear ImGui + DX11** — `imgui_impl_dx11` / `imgui_impl_win32`
- **链接库 / Libs:** `d3d11.lib`, `dxgi.lib`, `d3dcompiler.lib`, `dwmapi.lib`
- **内置依赖 / Vendored:** nlohmann/json, CS2 SDK dump headers, LXGWWenKai font
- **工具脚本 / Scripts:** Python 3 (`tools/*.py`, deps: `demoparser2` / `awpy`)

Platforms: x64 / x86, Debug / Release.

---

## Project Structure / 项目结构

```
cs2-walkbot-ext/
├── CS2_WalkBot_Ext.slnx          # VS 2022 solution
├── CS2_WalkBot_Ext/              # 主源码 / main source
│   ├── main.cpp                  # 窗口/悬浮窗线程/热键/主循环
│   ├── walkbot_core.*            # PathRunnerState_t + 三种瞄准算法
│   ├── path_runner.cpp           # 每 tick 决策循环 (UpdatePathRunner)
│   ├── path.*                    # PathManager / Waypoint 图模型
│   ├── walkbot_ui.*              # ImGui 控制面板
│   ├── interfaces.h              # 模式扫描 & 接口解析
│   ├── process.*, memory.h       # 附加 cs2.exe / RPM / 签名扫描
│   ├── sdk/                      # CS2 SDK dump 头
│   ├── imgui/                    # Dear ImGui
│   ├── config.json               # 运行参数
│   └── paths.json                # 航点图数据
├── docs/walkbot_concept.html     # 架构概念图
├── include/                      # nlohmann/json, CS2 SDK dump
└── tools/                        # Python 路径生成脚本
```

---

## Architecture / 架构

```
cs2.exe memory ─┐
                ├─> interfaces ─> walkbot_core (state) ─> path_runner (decision) ─> SendInput
process/memory ─┘                        │
                                         └─> Overlay Thread ─> WorldToScreen
```

| Module | Files | Job |
|---|---|---|
| 进程/内存 | `process.*`, `memory.h` | 打开 `cs2.exe`，`ReadProcessMemory`，模块枚举，签名扫描（含 RIP 解析） |
| 接口层 | `interfaces.h`, `entities.*`, `sdk/` | 每 tick 解析 LocalPlayer / EntityList / ViewMatrix |
| 航点 | `path.*`, `paths.json` | Waypoint 图维护、分支游标、持久化 |
| 运行时 | `walkbot_core.*`, `path_runner.*` | 状态管理、目标选择、姿态决策、瞄准分发 |
| UI/悬浮窗 | `main.cpp`, `walkbot_ui.*` | DX11 控制窗 + 透明 overlay |
| 工具 | `tools/*.py` | demo/awpy 路径生成 |

**CS2 更新后的常见修复点:**
1. `interfaces.h::Initialize()` 中的模式扫描签名
2. `game_types.h` / `sdk/*.hpp` 的 `MEM_PAD(offset)` 偏移

---

## Getting Started / 快速开始

### Prerequisites / 前置

- Visual Studio 2022 (Toolset v145)
- Windows 10/11 SDK
- C++20
- 所有依赖已内置，无需包管理器

### Build / 构建

命令行 (Release x64):

```bash
msbuild CS2_WalkBot_Ext.slnx -p:Configuration=Release -p:Platform=x64 -m
```

或直接用 VS 打开 `CS2_WalkBot_Ext.slnx` 构建。

<!-- TODO: confirm exact output path/name -->

### Run / 运行

1. 启动 CS2，进入地图
2. 以管理员权限运行 `CS2_WalkBot_Ext.exe`（`ReadProcessMemory` 需要提升权限）<!-- TODO: confirm elevation requirement -->
3. 程序附加 `cs2.exe`，弹出 ImGui 控制窗 + 透明悬浮窗
4. 录制/编辑航点，或加载已有 `paths.json`
5. 热键：`F6` 启动，`F7` 停止

`config.json` 与 `paths.json` 需与可执行文件同目录。

---

## Configuration / 配置

配置持久化在 `config.json`（与 exe 同目录），`auto_save_config` 为 true 时改动即时保存。

**主要分组:**

- **录制** — `smart_record_path`, `min_record_distance`, `record_new_waypoint_on_angle_change`, `new_waypoint_type`
- **overlay** — `draw_point_indices` / `draw_point_labels` / `draw_point_rings` / `draw_heading_hints` / `draw_player_guide` / `highlight_current` / `highlight_nearest`
- **runner** — 全部运行时参数:
  - 瞄准: `aim_control_mode` (0=Legacy Smooth, 1=Servo/PID, else=Humanized), `mouse_yaw`/`mouse_pitch`, `aim_speed_multiplier` <!-- TODO: confirm exact aim_control_mode enum mapping in walkbot_core.h -->
  - 伺服: `servo_yaw_response`, `servo_pitch_response`, `servo_yaw_damping`, `servo_pitch_damping`, `servo_max_speed`, `servo_max_acceleration`, `servo_max_jerk`, `servo_deadzone`, `servo_error_curve`
  - 预测: `enable_movement_prediction`, `movement_prediction_gain`, `movement_prediction_look_ahead_seconds`, `movement_prediction_max_lead`, `movement_velocity_smoothing`
  - 刹车: `emergency_brake_on_stop`, `emergency_brake_max_seconds`, `emergency_brake_speed_threshold`
  - 敌人: `track_nearest_enemy`, `wait_at_enemy_nearest_waypoint`, `treat_teammates_as_enemies`
  - 跳点: `skip_to_next_on_angle_diff`, `skip_current_yaw_diff_threshold`, `skip_next_yaw_diff_threshold`, `skip_trigger_distance`, `waypoint_reach_distance`
- **热键** — `start_hotkey_vk` / `stop_hotkey_vk` (Windows virtual key codes)

<!-- TODO: confirm WindMouse params (wind_gravity/wind_force/wind_max_step/wind_damping/wind_fitts_a/wind_fitts_b/wind_fitts_tolerance) are present in the current config schema -->

---

## Path Tools / 路径生成工具

`tools/*.py` — 独立 Python 脚本，不参与 C++ 构建:

| Script | Description |
|---|---|
| `generate_paths_from_demo.py` | 从 CS2 demo 移动轨迹生成 `paths.json` (需 `demoparser2`) |
| `generate_mirage_path_from_awpy.py` | 用 awpy 导航区域数据生成 Mirage 中心线路径 |
| `generate_mirage_manual_path.py` | 手工生成 Mirage 路线图 |
| `simplify_paths_centerline.py` | 将 `paths.json` 简化为中心线路线 |

```bash
python tools/simplify_paths_centerline.py --input CS2_WalkBot_Ext/paths.json --output CS2_WalkBot_Ext/paths.json
```

<!-- TODO: confirm Python version / requirements install steps; no requirements.txt in repo -->

---

## Status / 状态

实验性项目。无测试、无 CI、无 linter。SDK 偏移随 CS2 更新失效需手动维护。

Experimental. No tests, CI, or linter. SDK offsets break on CS2 updates and require manual patching.

---

## License / 许可证

仓库未包含 LICENSE 文件。默认保留所有权利。

No LICENSE file present. All rights reserved by default.
