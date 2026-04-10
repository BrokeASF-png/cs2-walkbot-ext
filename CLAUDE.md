# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

External C++20 CS2 (Counter-Strike 2) "walkbot" — a Windows console app that attaches to `cs2.exe` from another process, reads game state via `ReadProcessMemory`, drives a recorded waypoint path, and sends synthesized keyboard/mouse input. Includes an ImGui/DX11 overlay for visualization and a multi-tab ImGui control panel. This is purely external (no DLL injection, no hooks inside the game).

## Build

Open `CS2_WalkBot_Ext.slnx` in Visual Studio 2022, or use MSBuild:

```
msbuild CS2_WalkBot_Ext.slnx /p:Configuration=Release /p:Platform=x64
```

Configurations: `Debug` / `Release`. Platforms: `x64` / `Win32`. Toolset `v145`, C++20, Unicode, `_CONSOLE` subsystem. Links against `d3d11.lib`, `dxgi.lib`, `d3dcompiler.lib`, `dwmapi.lib`. Include paths: project dir, `../include` (for `nlohmann/json.hpp` and the CS2 SDK headers in `include/*.hpp`), and `CS2_WalkBot_Ext/sdk`.

No tests, no linter, no package manager. New `.cpp`/`.h` files must be added to both `CS2_WalkBot_Ext.vcxproj` `<ItemGroup>`s and the `.filters` file.

## Architecture

Data flows: `cs2.exe memory → process/memory/interfaces → walkbot_core (state) → path_runner (decision loop) → synthetic input` plus `walkbot_core → main overlay renderer`.

**Process attach & memory (`process.*`, `memory.h`)**. `AttachCS2()` opens `cs2.exe` by name into the global `g_hProcess`. `memory.h` defines a `Memory` class (global `g_Memory`) wrapping `ReadProcessMemory`, module enumeration via `TlHelp32`, and `PatterScan()` for signature scanning with `SCAN_RESOLVE_RIP` for RIP-relative resolution. All game reads go through `g_Memory.ReadMemory<T>(addr)`.

**Interface layer (`interfaces.h`, `game_types.h`, `entities.*`, `sdk/*.hpp`)**. `CGlobals` (global `g_Globals`) holds resolved offsets (`CNetworkGameClient`, `LocalPlayerController`, `EntityList`, `EntitySystem`, `ViewMatrix`, `CSGOInput`). On each tick, `UpdateInterfaces()` / `UpdateLocalPlayer()` / `UpdateViewMatrix()` re-read the interface snapshots and the local player's controller + pawn + velocity. Game class layouts use `MEM_PAD(offset)` macros to skip to known field offsets from the CS2 SDK dumps under `include/*_dll.hpp` and `CS2_WalkBot_Ext/sdk/`. When updating for a new game build, re-run pattern scans and bump the `MEM_PAD` offsets — these are the typical points of breakage.

**Path model (`path.*`, `paths.json`)**. `PathManager` owns a `std::vector<Waypoint>`, each with position, angle, type (`Normal` or `MultiBranch`), a branch-selection mode (`Sequential` / `Random` / `Cycle`), and `nextIndices` for branching graphs. Persisted to `paths.json` next to the executable. `ResolveNextWaypointIndex()` is stateful (advances the cycle cursor); `PeekNextWaypointIndex()` is pure. `NormalizeWaypointLinks()` keeps graph indices consistent after insert/remove/swap.

**Runner state & humanized aim (`walkbot_core.*`, `path_runner.*`)**. `PathRunnerState_t` is a large flat struct holding every tunable: forward/backward/duck key state, aim-control mode (`LegacySmooth` vs `PID`), servo PID gains, per-axis `HumanAxisState_t` (tremor, drift, residual, cadence), movement prediction (velocity/acceleration smoothing, look-ahead), emergency brake, enemy tracking, skip-ahead thresholds. Three aim backends coexist and are selectable at runtime: `ComputeLegacySmoothedMouseDelta`, `ComputeMouseDeltaAdaptiveServo`, and `ComputeHumanizedMouseDeltaAdvanced` (the latter adds tremor, drift phases, random kicks). Input goes out via `SendRelativeMouseMove` / `SendKeyboardKey`. `UpdatePathRunner()` in `path_runner.cpp` is the per-tick decision function: picks target waypoint, handles enemy-nearest waiting, crouch-on-lower-target, and calls into the aim backend.

**UI & overlay (`main.cpp`, `walkbot_ui.*`)**. `main.cpp` owns two windows: a DX11-backed ImGui control window (where `walkbot_ui.cpp` draws tabs) and a separate **transparent click-through overlay window** running on its own thread (`OverlayThreadMain`) with `WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST`. The overlay projects waypoints/lines with `WorldToScreen` using `g_Globals.m_matViewMatrix` and throttles repaints via `kOverlayBuildIntervalMs` + a state signature (`CalculateOverlayRenderSignature`) so it only redraws when something changes. Overlay data handoff uses `g_OverlayDataMutex`.

**Config (`config.json`)**. UI settings and all runner tunables are serialized via `nlohmann::json`. `auto_save_config` persists on change. Start/stop hotkeys (`start_hotkey_vk` / `stop_hotkey_vk`) default to F6/F7.

**Path generation tools (`tools/*.py`)**. Standalone Python scripts (not part of the C++ build) that generate `paths.json` files from CS2 demo files via `demoparser2`, or from awpy, or hand-crafted mirage layouts, plus a centerline simplifier. These are one-off utilities — run them directly with Python, no build step.

## Notes for modification

- The `main.cpp` file is large (~1000+ lines) and mixes window setup, overlay thread, hotkey polling, and the main tick loop. Read it with `offset`/`limit`.
- When adding a new runner tunable: add the field to `PathRunnerState_t` in `walkbot_core.h`, serialize it in the config load/save in `main.cpp`, and expose it in `walkbot_ui.cpp`.
- Pattern-scan signatures in `interfaces.h::Initialize()` are the first thing to break after a CS2 update.
- Branch-record mode in the UI adds waypoints as children of a source waypoint rather than appending linearly — see `AddRecordedWaypoint` in `walkbot_core.*`.
