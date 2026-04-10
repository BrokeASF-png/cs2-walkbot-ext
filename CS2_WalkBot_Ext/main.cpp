#include <d3d11.h>
#include <dwmapi.h>
#include <dxgi.h>
#include <tchar.h>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <unordered_set>
#include <utility>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_win32.h"
#include "imgui/imgui_impl_dx11.h"
#include "font_lxgw.h"
#include "process.h"
#include "memory.h"
#include "interfaces.h"
#include "utilities.h"
#include "path.h"
#include "walkbot_core.h"
#include "path_runner.h"
#include "walkbot_ui.h"

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dwmapi.lib")
#pragma comment(lib, "dxgi.lib")

using json = nlohmann::json;

static ID3D11Device*            g_pd3dDevice        = nullptr;
static ID3D11DeviceContext*     g_pd3dDeviceContext = nullptr;
static IDXGISwapChain*          g_pSwapChain        = nullptr;
static ID3D11RenderTargetView*  g_mainRenderTargetView = nullptr;

static HWND                     g_hOverlayWindow    = nullptr;
static std::vector<OverlayPoint_t> g_OverlayPoints;
static std::vector<OverlayLine_t>  g_OverlayLines;
static std::mutex               g_OverlayDataMutex;
static HANDLE                   g_hOverlayThread = nullptr;
static HANDLE                   g_hOverlayReadyEvent = nullptr;
static bool                     g_bOverlayDrawPoints = true;
static bool                     g_bOverlayDrawPointIndices = true;
static bool                     g_bOverlayDrawLines = true;
static bool                     g_bOverlayDrawPointRings = true;
static bool                     g_bOverlayDrawPointLabels = true;
static bool                     g_bOverlayDrawHeadingHints = true;
static bool                     g_bOverlayDrawPlayerGuide = true;
static bool                     g_bOverlayHighlightCurrent = true;
static bool                     g_bOverlayHighlightNearest = true;
static int                      g_nOverlayNearestWaypointIndex = -1;
static int                      g_nOverlayCurrentWaypointIndex = -1;
static int                      g_nOverlayNextWaypointIndex = -1;
static bool                     g_bOverlayVisible = false;
static bool                     g_bOverlayNeedsRepaint = false;
static ULONGLONG                g_uOverlayLastBuildTick = 0;
static int                      g_nOverlayWidth = 0;
static int                      g_nOverlayHeight = 0;
static int                      g_nOverlayPosX = 0;
static int                      g_nOverlayPosY = 0;
static std::size_t              g_uOverlayStateSignature = 0;
static std::size_t              g_uOverlaySourceSignature = 0;
static constexpr ULONGLONG      kOverlayBuildIntervalMs = 16;

bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
LRESULT WINAPI OverlayWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
static void InitializeOverlayWindow(HWND hWnd);

static DWORD WINAPI OverlayThreadMain(LPVOID lpParameter)
{
    const HINSTANCE hInstance = reinterpret_cast<HINSTANCE>(lpParameter);
    WNDCLASSEXW overlayWindowClass = {
        sizeof(overlayWindowClass), CS_CLASSDC, OverlayWndProc, 0L, 0L,
        hInstance, nullptr, nullptr, nullptr, nullptr,
        L"CS2WalkBotExtOverlay", nullptr
    };
    RegisterClassExW(&overlayWindowClass);

    HWND hOverlay = CreateWindowExW(
        WS_EX_TOPMOST | WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOOLWINDOW,
        overlayWindowClass.lpszClassName,
        L"CS2 WalkBot Overlay",
        WS_POPUP,
        0, 0, 1280, 800,
        nullptr, nullptr, overlayWindowClass.hInstance, nullptr
    );

    g_hOverlayWindow = hOverlay;
    if (g_hOverlayWindow)
        InitializeOverlayWindow(g_hOverlayWindow);

    if (g_hOverlayReadyEvent)
        SetEvent(g_hOverlayReadyEvent);

    if (!g_hOverlayWindow)
        return 0;

    MSG msg{};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0)
    {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    if (g_hOverlayWindow && IsWindow(g_hOverlayWindow))
    {
        DestroyWindow(g_hOverlayWindow);
        g_hOverlayWindow = nullptr;
    }

    UnregisterClassW(overlayWindowClass.lpszClassName, overlayWindowClass.hInstance);
    return 0;
}

static void InitializeOverlayWindow(HWND hWnd)
{
    SetLayeredWindowAttributes(hWnd, RGB(0, 0, 0), 0, LWA_COLORKEY);

    const MARGINS margins = { -1 };
    DwmExtendFrameIntoClientArea(hWnd, &margins);
}

static void SyncOverlayToTargetWindow(HWND hOverlay, HWND hTarget)
{
    if (!hOverlay || !hTarget || !IsWindow(hTarget))
        return;

    RECT rect{};
    GetClientRect(hTarget, &rect);

    POINT topLeft{ rect.left, rect.top };
    POINT bottomRight{ rect.right, rect.bottom };
    ClientToScreen(hTarget, &topLeft);
    ClientToScreen(hTarget, &bottomRight);

    const int width = bottomRight.x - topLeft.x;
    const int height = bottomRight.y - topLeft.y;

    if (width <= 0 || height <= 0)
        return;

    const bool changed =
        g_nOverlayPosX != topLeft.x ||
        g_nOverlayPosY != topLeft.y ||
        g_nOverlayWidth != width ||
        g_nOverlayHeight != height;

    g_nOverlayPosX = topLeft.x;
    g_nOverlayPosY = topLeft.y;
    g_nOverlayWidth = width;
    g_nOverlayHeight = height;

    if (changed)
    {
        SetWindowPos(
            hOverlay,
            HWND_TOPMOST,
            topLeft.x,
            topLeft.y,
            width,
            height,
            SWP_NOACTIVATE
        );
        g_bOverlayNeedsRepaint = true;
    }
}

static std::vector<int> CollectEnemyCandidateWaypointIndices(
    const PathManager& pathManager,
    const LocalPlayer_t& localPlayer,
    bool includeTeammates)
{
    std::vector<int> candidateIndices;
    if (!localPlayer.m_pPlayerPawn)
        return candidateIndices;

    const std::uint8_t localTeam = localPlayer.m_pPlayerPawn->GetTeamNum();
    if (localTeam == 0)
        return candidateIndices;

    const auto& entities = g_EntityList.GetEntities();
    std::unordered_set<int> uniqueIndices;

    for (const EntityObject_t& entityObject : entities)
    {
        if (!entityObject.m_pEntity || entityObject.m_Type != EEntityType::ENTITY_PLAYER)
            continue;

        CCSPlayerController* pController = reinterpret_cast<CCSPlayerController*>(entityObject.m_pEntity);
        if (!pController || pController == localPlayer.m_pController)
            continue;

        C_CSPlayerPawn* pEnemyPawn = pController->GetPawnHandle().Get();
        if (!pEnemyPawn)
            continue;

        const std::uint8_t enemyTeam = pEnemyPawn->GetTeamNum();
        if (enemyTeam == 0)
            continue;

        if (!includeTeammates && enemyTeam == localTeam)
            continue;

        const Vector enemyPos = pEnemyPawn->GetAbsOrigin();
        if (!enemyPos.IsValid())
            continue;

        const int enemyWaypointIndex = FindNearestWaypointIndex(pathManager, enemyPos);
        if (enemyWaypointIndex < 0)
            continue;

        if (uniqueIndices.insert(enemyWaypointIndex).second)
            candidateIndices.push_back(enemyWaypointIndex);
    }

    return candidateIndices;
}

int main()
{
    std::srand(static_cast<unsigned int>(GetTickCount()));
    PathManager pathManager("paths.json");

    WNDCLASSEXW mainWindowClass = {
        sizeof(mainWindowClass), CS_CLASSDC, WndProc, 0L, 0L,
        GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr,
        L"CS2WalkBotExt", nullptr
    };
    RegisterClassExW(&mainWindowClass);

    HWND hwnd = CreateWindowW(
        mainWindowClass.lpszClassName, L"CS2 WalkBot Ext",
        WS_OVERLAPPEDWINDOW,
        100, 100, 1280, 800,
        nullptr, nullptr, mainWindowClass.hInstance, nullptr
    );

    g_hOverlayReadyEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    g_hOverlayThread = CreateThread(
        nullptr,
        0,
        OverlayThreadMain,
        mainWindowClass.hInstance,
        0,
        nullptr
    );
    if (!g_hOverlayThread && g_hOverlayReadyEvent)
        SetEvent(g_hOverlayReadyEvent);
    if (g_hOverlayReadyEvent)
    {
        WaitForSingleObject(g_hOverlayReadyEvent, 5000);
        CloseHandle(g_hOverlayReadyEvent);
        g_hOverlayReadyEvent = nullptr;
    }

    if (!CreateDeviceD3D(hwnd))
    {
        CleanupDeviceD3D();
        if (g_hOverlayWindow)
            PostMessageW(g_hOverlayWindow, WM_CLOSE, 0, 0);
        if (g_hOverlayThread)
        {
            WaitForSingleObject(g_hOverlayThread, 5000);
            CloseHandle(g_hOverlayThread);
            g_hOverlayThread = nullptr;
        }
        UnregisterClassW(mainWindowClass.lpszClassName, mainWindowClass.hInstance);
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWDEFAULT);
    UpdateWindow(hwnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    ImFontConfig font_cfg;
    font_cfg.FontDataOwnedByAtlas = false;
    io.Fonts->AddFontFromMemoryCompressedTTF(
        LXGWWenKai_compressed_data,
        LXGWWenKai_compressed_size,
        18.0f,
        &font_cfg,
        io.Fonts->GetGlyphRangesChineseSimplifiedCommon()
    );
    io.Fonts->Build();

    ImVec4 clear_color = ImVec4(0.1f, 0.1f, 0.1f, 1.0f);
    bool show_demo     = false;
    static char attach_status[256] = "Not attached";
    static char process_info[512] = "";
    static bool auto_record_path = false;
    static float min_record_distance = 20.0f;
    static bool smart_record_path = true;
    static float smart_record_straight_angle_deg = 6.0f;
    static float smart_record_probe_distance = 8.0f;
    static bool smart_record_has_last_sample = false;
    static Vector smart_record_last_sample_pos{};
    static QAngle smart_record_last_sample_ang{};
    static bool smart_record_has_prev_probe_sample = false;
    static Vector smart_record_prev_probe_sample_pos{};
    static bool smart_record_has_pending_terminal = false;
    static Vector smart_record_pending_terminal_pos{};
    static QAngle smart_record_pending_terminal_ang{};
    static bool smart_record_prev_auto_record = false;
    static bool record_waypoint_angle = true;
    static bool record_new_waypoint_on_angle_change = true;
    static bool allow_same_position_on_angle_change = true;
    static int new_waypoint_type = static_cast<int>(EWaypointType::Normal);
    static bool visualize_paths = true;
    static bool visualize_path_points = true;
    static bool visualize_path_lines = true;
    static std::vector<int> branch_target_selection;
    static int preview_waypoint_index = 0;
    static int selected_waypoint_index = -1;
    static bool waypoint_editor_dirty = false;
    static Vector waypoint_edit_pos{};
    static QAngle waypoint_edit_ang{};
    static bool branch_record_mode_active = false;
    static int branch_record_source_index = -1;
    static int branch_record_first_waypoint_index = -1;
    static bool start_hotkey_prev_down = false;
    static bool stop_hotkey_prev_down = false;
    static int start_hotkey_vk = VK_F6;
    static int stop_hotkey_vk = VK_F7;
    static PathRunnerState_t pathRunnerState;
    static Vector current_player_pos;
    static Vector current_player_velocity;
    static QAngle current_player_angle;
    static bool has_current_player_pos = false;
    static bool has_current_player_velocity = false;
    static bool auto_save_config = true;
    static char config_status[256] = "Config: not loaded";

    auto saveConfig = [&](const char* reason)
    {
        try
        {
            json j;
            j["auto_save_config"] = auto_save_config;
            j["auto_record_path"] = auto_record_path;
            j["min_record_distance"] = min_record_distance;
            j["smart_record_path"] = smart_record_path;
            j["smart_record_straight_angle_deg"] = smart_record_straight_angle_deg;
            j["smart_record_probe_distance"] = smart_record_probe_distance;
            j["record_waypoint_angle"] = record_waypoint_angle;
            j["record_new_waypoint_on_angle_change"] = record_new_waypoint_on_angle_change;
            j["allow_same_position_on_angle_change"] = allow_same_position_on_angle_change;
            j["new_waypoint_type"] = new_waypoint_type;
            j["visualize_paths"] = visualize_paths;
            j["visualize_path_points"] = visualize_path_points;
            j["visualize_path_lines"] = visualize_path_lines;
            j["start_hotkey_vk"] = start_hotkey_vk;
            j["stop_hotkey_vk"] = stop_hotkey_vk;

            j["overlay"]["draw_point_rings"] = g_bOverlayDrawPointRings;
            j["overlay"]["draw_point_labels"] = g_bOverlayDrawPointLabels;
            j["overlay"]["draw_point_indices"] = g_bOverlayDrawPointIndices;
            j["overlay"]["draw_heading_hints"] = g_bOverlayDrawHeadingHints;
            j["overlay"]["draw_player_guide"] = g_bOverlayDrawPlayerGuide;
            j["overlay"]["highlight_current"] = g_bOverlayHighlightCurrent;
            j["overlay"]["highlight_nearest"] = g_bOverlayHighlightNearest;

            j["runner"]["waypoint_reach_distance"] = pathRunnerState.waypointReachDistance;
            j["runner"]["direct_aim_on_start"] = pathRunnerState.directAimOnStart;
            j["runner"]["track_nearest_enemy"] = pathRunnerState.trackNearestEnemy;
            j["runner"]["treat_teammates_as_enemies"] = pathRunnerState.treatTeammatesAsEnemies;
            j["runner"]["wait_at_enemy_nearest_waypoint"] = pathRunnerState.waitAtEnemyNearestWaypoint;
            j["runner"]["move_yaw_threshold"] = pathRunnerState.moveYawThreshold;
            j["runner"]["move_pitch_threshold"] = pathRunnerState.movePitchThreshold;
            j["runner"]["auto_crouch_on_lower_target"] = pathRunnerState.autoCrouchOnLowerTarget;
            j["runner"]["crouch_height_delta"] = pathRunnerState.crouchHeightDelta;
            j["runner"]["emergency_brake_on_stop"] = pathRunnerState.emergencyBrakeOnStop;
            j["runner"]["emergency_brake_speed_threshold"] = pathRunnerState.emergencyBrakeSpeedThreshold;
            j["runner"]["emergency_brake_max_seconds"] = pathRunnerState.emergencyBrakeMaxSeconds;
            j["runner"]["allow_pitch_control"] = pathRunnerState.allowPitchControl;
            j["runner"]["set_pitch_on_initial_direct_aim"] = pathRunnerState.setPitchOnInitialDirectAim;
            j["runner"]["pitch_float_amplitude_deg"] = pathRunnerState.pitchFloatAmplitudeDeg;
            j["runner"]["pitch_float_frequency"] = pathRunnerState.pitchFloatFrequency;
            j["runner"]["pitch_float_yaw_gate_deg"] = pathRunnerState.pitchFloatYawGateDeg;
            j["runner"]["aim_speed_multiplier"] = pathRunnerState.aimSpeedMultiplier;
            j["runner"]["aim_control_mode"] = static_cast<int>(pathRunnerState.aimControlMode);
            j["runner"]["skip_to_next_on_large_yaw"] = pathRunnerState.skipToNextOnLargeYaw;
            j["runner"]["skip_trigger_distance"] = pathRunnerState.skipTriggerDistance;
            j["runner"]["skip_yaw_threshold"] = pathRunnerState.skipYawThreshold;
            j["runner"]["skip_to_next_on_angle_diff"] = pathRunnerState.skipToNextOnAngleDiff;
            j["runner"]["skip_current_yaw_diff_threshold"] = pathRunnerState.skipCurrentYawDiffThreshold;
            j["runner"]["skip_next_yaw_diff_threshold"] = pathRunnerState.skipNextYawDiffThreshold;
            j["runner"]["legacy_yaw_smooth"] = pathRunnerState.simpleYawSmooth;
            j["runner"]["legacy_pitch_smooth"] = pathRunnerState.simplePitchSmooth;
            j["runner"]["servo_yaw_response"] = pathRunnerState.servoYawResponse;
            j["runner"]["servo_yaw_damping"] = pathRunnerState.servoYawDamping;
            j["runner"]["servo_pitch_response"] = pathRunnerState.servoPitchResponse;
            j["runner"]["servo_pitch_damping"] = pathRunnerState.servoPitchDamping;
            j["runner"]["servo_deadzone"] = pathRunnerState.servoDeadzone;
            j["runner"]["servo_error_curve"] = pathRunnerState.servoErrorCurve;
            j["runner"]["servo_max_speed"] = pathRunnerState.servoMaxSpeed;
            j["runner"]["servo_max_acceleration"] = pathRunnerState.servoMaxAcceleration;
            j["runner"]["servo_max_jerk"] = pathRunnerState.servoMaxJerk;
            j["runner"]["wind_gravity"] = pathRunnerState.windGravity;
            j["runner"]["wind_force"] = pathRunnerState.windForce;
            j["runner"]["wind_max_step"] = pathRunnerState.windMaxStep;
            j["runner"]["wind_damping"] = pathRunnerState.windDamping;
            j["runner"]["wind_fitts_a"] = pathRunnerState.windFittsA;
            j["runner"]["wind_fitts_b"] = pathRunnerState.windFittsB;
            j["runner"]["wind_fitts_tolerance"] = pathRunnerState.windFittsTolerance;
            j["runner"]["mouse_pitch"] = pathRunnerState.mousePitch;
            j["runner"]["mouse_yaw"] = pathRunnerState.mouseYaw;
            j["runner"]["mouse_sensitivity"] = pathRunnerState.mouseSensitivity;
            j["runner"]["enable_movement_prediction"] = pathRunnerState.enableMovementPrediction;
            j["runner"]["movement_prediction_look_ahead_seconds"] = pathRunnerState.movementPredictionLookAheadSeconds;
            j["runner"]["movement_prediction_gain"] = pathRunnerState.movementPredictionGain;
            j["runner"]["movement_prediction_acceleration_gain"] = pathRunnerState.movementPredictionAccelerationGain;
            j["runner"]["movement_prediction_max_lead"] = pathRunnerState.movementPredictionMaxLead;
            j["runner"]["movement_velocity_smoothing"] = pathRunnerState.movementVelocitySmoothing;

            std::ofstream file("config.json");
            file << j.dump(2);
            snprintf(config_status, sizeof(config_status), "Config: saved (%s)", reason);
        }
        catch (const std::exception& e)
        {
            snprintf(config_status, sizeof(config_status), "Config save error: %s", e.what());
        }
    };

    auto loadConfig = [&]()
    {
        std::ifstream file("config.json");
        if (!file.is_open())
        {
            strcpy_s(config_status, sizeof(config_status), "Config: file not found, using defaults");
            return;
        }

        try
        {
            const json j = json::parse(file);

            auto loadBool = [&](const json& obj, const char* key, bool& target)
            {
                if (!obj.contains(key))
                    return;
                try
                {
                    target = obj[key].get<bool>();
                }
                catch (...)
                {
                }
            };
            auto loadInt = [&](const json& obj, const char* key, int& target)
            {
                if (!obj.contains(key))
                    return;
                try
                {
                    target = obj[key].get<int>();
                }
                catch (...)
                {
                }
            };
            auto loadFloat = [&](const json& obj, const char* key, float& target)
            {
                if (obj.contains(key) && obj[key].is_number()) target = obj[key].get<float>();
            };

            loadBool(j, "auto_save_config", auto_save_config);
            loadBool(j, "auto_record_path", auto_record_path);
            loadFloat(j, "min_record_distance", min_record_distance);
            loadBool(j, "smart_record_path", smart_record_path);
            loadFloat(j, "smart_record_straight_angle_deg", smart_record_straight_angle_deg);
            loadFloat(j, "smart_record_probe_distance", smart_record_probe_distance);
            loadBool(j, "record_waypoint_angle", record_waypoint_angle);
            loadBool(j, "record_new_waypoint_on_angle_change", record_new_waypoint_on_angle_change);
            loadBool(j, "allow_same_position_on_angle_change", allow_same_position_on_angle_change);
            loadInt(j, "new_waypoint_type", new_waypoint_type);
            loadBool(j, "visualize_paths", visualize_paths);
            loadBool(j, "visualize_path_points", visualize_path_points);
            loadBool(j, "visualize_path_lines", visualize_path_lines);
            loadInt(j, "start_hotkey_vk", start_hotkey_vk);
            loadInt(j, "stop_hotkey_vk", stop_hotkey_vk);

            if (j.contains("overlay") && j["overlay"].is_object())
            {
                const json& o = j["overlay"];
                loadBool(o, "draw_point_rings", g_bOverlayDrawPointRings);
                loadBool(o, "draw_point_labels", g_bOverlayDrawPointLabels);
                loadBool(o, "draw_point_indices", g_bOverlayDrawPointIndices);
                loadBool(o, "draw_heading_hints", g_bOverlayDrawHeadingHints);
                loadBool(o, "draw_player_guide", g_bOverlayDrawPlayerGuide);
                loadBool(o, "highlight_current", g_bOverlayHighlightCurrent);
                loadBool(o, "highlight_nearest", g_bOverlayHighlightNearest);
            }

            if (j.contains("runner") && j["runner"].is_object())
            {
                const json& r = j["runner"];
                loadFloat(r, "waypoint_reach_distance", pathRunnerState.waypointReachDistance);
                loadBool(r, "direct_aim_on_start", pathRunnerState.directAimOnStart);
                loadBool(r, "track_nearest_enemy", pathRunnerState.trackNearestEnemy);
                loadBool(r, "treat_teammates_as_enemies", pathRunnerState.treatTeammatesAsEnemies);
                loadBool(r, "wait_at_enemy_nearest_waypoint", pathRunnerState.waitAtEnemyNearestWaypoint);
                loadFloat(r, "move_yaw_threshold", pathRunnerState.moveYawThreshold);
                loadFloat(r, "move_pitch_threshold", pathRunnerState.movePitchThreshold);
                loadBool(r, "auto_crouch_on_lower_target", pathRunnerState.autoCrouchOnLowerTarget);
                loadFloat(r, "crouch_height_delta", pathRunnerState.crouchHeightDelta);
                loadBool(r, "emergency_brake_on_stop", pathRunnerState.emergencyBrakeOnStop);
                loadFloat(r, "emergency_brake_speed_threshold", pathRunnerState.emergencyBrakeSpeedThreshold);
                loadFloat(r, "emergency_brake_max_seconds", pathRunnerState.emergencyBrakeMaxSeconds);
                loadBool(r, "allow_pitch_control", pathRunnerState.allowPitchControl);
                loadBool(r, "set_pitch_on_initial_direct_aim", pathRunnerState.setPitchOnInitialDirectAim);
                loadFloat(r, "pitch_float_amplitude_deg", pathRunnerState.pitchFloatAmplitudeDeg);
                loadFloat(r, "pitch_float_frequency", pathRunnerState.pitchFloatFrequency);
                loadFloat(r, "pitch_float_yaw_gate_deg", pathRunnerState.pitchFloatYawGateDeg);
                loadFloat(r, "aim_speed_multiplier", pathRunnerState.aimSpeedMultiplier);
                if (r.contains("aim_control_mode") && r["aim_control_mode"].is_number())
                {
                    int rawAimMode = r["aim_control_mode"].get<int>();
                    if (rawAimMode < static_cast<int>(PathRunnerState_t::EAimControlMode::LegacySmooth))
                        rawAimMode = static_cast<int>(PathRunnerState_t::EAimControlMode::LegacySmooth);
                    if (rawAimMode > static_cast<int>(PathRunnerState_t::EAimControlMode::WindMouse))
                        rawAimMode = static_cast<int>(PathRunnerState_t::EAimControlMode::WindMouse);
                    pathRunnerState.aimControlMode = static_cast<PathRunnerState_t::EAimControlMode>(rawAimMode);
                }
                loadBool(r, "skip_to_next_on_large_yaw", pathRunnerState.skipToNextOnLargeYaw);
                loadFloat(r, "skip_trigger_distance", pathRunnerState.skipTriggerDistance);
                loadFloat(r, "skip_yaw_threshold", pathRunnerState.skipYawThreshold);
                loadBool(r, "skip_to_next_on_angle_diff", pathRunnerState.skipToNextOnAngleDiff);
                loadFloat(r, "skip_current_yaw_diff_threshold", pathRunnerState.skipCurrentYawDiffThreshold);
                loadFloat(r, "skip_next_yaw_diff_threshold", pathRunnerState.skipNextYawDiffThreshold);
                loadFloat(r, "legacy_yaw_smooth", pathRunnerState.simpleYawSmooth);
                loadFloat(r, "legacy_pitch_smooth", pathRunnerState.simplePitchSmooth);
                loadFloat(r, "servo_yaw_response", pathRunnerState.servoYawResponse);
                loadFloat(r, "servo_yaw_damping", pathRunnerState.servoYawDamping);
                loadFloat(r, "servo_pitch_response", pathRunnerState.servoPitchResponse);
                loadFloat(r, "servo_pitch_damping", pathRunnerState.servoPitchDamping);
                loadFloat(r, "servo_deadzone", pathRunnerState.servoDeadzone);
                loadFloat(r, "servo_error_curve", pathRunnerState.servoErrorCurve);
                loadFloat(r, "servo_max_speed", pathRunnerState.servoMaxSpeed);
                loadFloat(r, "servo_max_acceleration", pathRunnerState.servoMaxAcceleration);
                loadFloat(r, "servo_max_jerk", pathRunnerState.servoMaxJerk);
                loadFloat(r, "wind_gravity", pathRunnerState.windGravity);
                loadFloat(r, "wind_force", pathRunnerState.windForce);
                loadFloat(r, "wind_max_step", pathRunnerState.windMaxStep);
                loadFloat(r, "wind_damping", pathRunnerState.windDamping);
                loadFloat(r, "wind_fitts_a", pathRunnerState.windFittsA);
                loadFloat(r, "wind_fitts_b", pathRunnerState.windFittsB);
                loadFloat(r, "wind_fitts_tolerance", pathRunnerState.windFittsTolerance);
                loadFloat(r, "mouse_pitch", pathRunnerState.mousePitch);
                loadFloat(r, "mouse_yaw", pathRunnerState.mouseYaw);
                loadFloat(r, "mouse_sensitivity", pathRunnerState.mouseSensitivity);
                loadBool(r, "enable_movement_prediction", pathRunnerState.enableMovementPrediction);
                loadFloat(r, "movement_prediction_look_ahead_seconds", pathRunnerState.movementPredictionLookAheadSeconds);
                loadFloat(r, "movement_prediction_gain", pathRunnerState.movementPredictionGain);
                loadFloat(r, "movement_prediction_acceleration_gain", pathRunnerState.movementPredictionAccelerationGain);
                loadFloat(r, "movement_prediction_max_lead", pathRunnerState.movementPredictionMaxLead);
                loadFloat(r, "movement_velocity_smoothing", pathRunnerState.movementVelocitySmoothing);
            }

            ResetHumanizedAimState(pathRunnerState);
            ResetMovementPredictionState(pathRunnerState);
            strcpy_s(config_status, sizeof(config_status), "Config: loaded");
        }
        catch (const std::exception& e)
        {
            snprintf(config_status, sizeof(config_status), "Config load error: %s", e.what());
        }
    };

    loadConfig();

    bool done = false;
    while (!done)
    {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT)
                done = true;
        }
        if (done) break;

        auto startRun = [&]()
        {
            const int nearestWaypointIndex = has_current_player_pos
                ? FindNearestWaypointIndex(pathManager, current_player_pos)
                : -1;

            if (nearestWaypointIndex >= 0)
            {
                pathManager.ResetRuntimeState();
                pathRunnerState.currentWaypointIndex = nearestWaypointIndex;
                pathRunnerState.isRunning = true;
                pathRunnerState.pendingInitialDirectAim = pathRunnerState.directAimOnStart;
                ResetHumanizedAimState(pathRunnerState);
                ResetMovementPredictionState(pathRunnerState);
                snprintf(
                    pathRunnerState.status,
                    sizeof(pathRunnerState.status),
                    "Running: start from waypoint #%d",
                    nearestWaypointIndex
                );
            }
            else
            {
                strcpy_s(pathRunnerState.status, sizeof(pathRunnerState.status), "Start failed: no valid waypoint");
            }
        };

        auto stopRun = [&]()
        {
            StopPathRunner(pathRunnerState);
            strcpy_s(pathRunnerState.status, sizeof(pathRunnerState.status), "Stopped by user");
        };

        const bool startHotkeyDown = (GetAsyncKeyState(start_hotkey_vk) & 0x8000) != 0;
        const bool stopHotkeyDown = (GetAsyncKeyState(stop_hotkey_vk) & 0x8000) != 0;
        if (startHotkeyDown && !start_hotkey_prev_down)
            startRun();
        if (stopHotkeyDown && !stop_hotkey_prev_down)
            stopRun();
        start_hotkey_prev_down = startHotkeyDown;
        stop_hotkey_prev_down = stopHotkeyDown;

        HWND hGameWindow = GetCS2Window();
        if (hGameWindow && visualize_paths)
        {
            SyncOverlayToTargetWindow(g_hOverlayWindow, hGameWindow);
            if (g_hOverlayWindow)
                ShowWindow(g_hOverlayWindow, SW_SHOWNA);
            g_bOverlayVisible = true;
        }
        else
        {
            if (g_hOverlayWindow)
                ShowWindow(g_hOverlayWindow, SW_HIDE);
            g_bOverlayVisible = false;
            g_nOverlayWidth = 0;
            g_nOverlayHeight = 0;
            g_nOverlayPosX = 0;
            g_nOverlayPosY = 0;
            g_uOverlayStateSignature = 0;
            g_uOverlaySourceSignature = 0;
            g_uOverlayLastBuildTick = 0;
            g_bOverlayNeedsRepaint = false;
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        if (IsCS2Attached() && g_Memory.IsAttached())
        {
            CUtilities::UpdateGameState();
            g_Globals.UpdateViewMatrix();
            g_Globals.UpdateInterfaces();

            const GameState_t& gameState = CUtilities::GetGameState();
            if (gameState.m_bIsInGame)
            {
                g_Globals.UpdateLocalPlayer();
                g_EntityList.UpdateEntities();
            }
            else
            {
                g_Globals.m_LocalPlayer.m_pController = nullptr;
                g_Globals.m_LocalPlayer.m_pPlayerPawn = nullptr;
                g_Globals.m_LocalPlayer.m_vecVelocity = {};
                g_EntityList.Clear();
            }
        }
        else
        {
            CUtilities::UpdateGameState();
            g_Globals.m_LocalPlayer.m_pController = nullptr;
            g_Globals.m_LocalPlayer.m_pPlayerPawn = nullptr;
            g_Globals.m_LocalPlayer.m_vecVelocity = {};
            g_Globals.m_matViewMatrix = {};
            g_EntityList.Clear();
        }

        has_current_player_pos = false;
        current_player_pos = {};
        has_current_player_velocity = false;
        current_player_velocity = {};
        current_player_angle = {};

        if (g_Globals.m_LocalPlayer.m_pPlayerPawn)
        {
            current_player_pos = g_Globals.m_LocalPlayer.m_pPlayerPawn->GetAbsOrigin();
            has_current_player_pos = current_player_pos.IsValid();
            current_player_velocity = g_Globals.m_LocalPlayer.m_vecVelocity;
            has_current_player_velocity = current_player_velocity.IsValid();
        }

        current_player_angle = g_Globals.m_CSGOInput.m_angViewAngle;
        const std::vector<int> enemyCandidateWaypointIndices = pathRunnerState.trackNearestEnemy
            ? CollectEnemyCandidateWaypointIndices(
                pathManager,
                g_Globals.m_LocalPlayer,
                pathRunnerState.treatTeammatesAsEnemies
            )
            : std::vector<int>{};

        UpdatePathRunner(
            pathRunnerState,
            pathManager,
            has_current_player_pos,
            current_player_pos,
            has_current_player_velocity,
            current_player_velocity,
            current_player_angle,
            enemyCandidateWaypointIndices,
            hGameWindow
        );

        const QAngle recorded_angle = record_waypoint_angle ? current_player_angle : QAngle{};
        const EWaypointType recorded_type = (new_waypoint_type == static_cast<int>(EWaypointType::MultiBranch))
            ? EWaypointType::MultiBranch
            : EWaypointType::Normal;
        const auto& currentWaypoints = pathManager.GetWaypoints();

        if (smart_record_prev_auto_record && !auto_record_path)
        {
            const auto& waypoints = pathManager.GetWaypoints();
            if (smart_record_path &&
                smart_record_has_pending_terminal &&
                !waypoints.empty() &&
                waypoints.back().pos.Distance(smart_record_pending_terminal_pos) > 0.5f)
            {
                AddRecordedWaypoint(
                    pathManager,
                    smart_record_pending_terminal_pos,
                    record_waypoint_angle ? smart_record_pending_terminal_ang : QAngle{},
                    recorded_type,
                    branch_record_mode_active,
                    branch_record_source_index,
                    branch_record_first_waypoint_index
                );
            }

            smart_record_has_last_sample = false;
            smart_record_has_prev_probe_sample = false;
            smart_record_has_pending_terminal = false;
        }
        smart_record_prev_auto_record = auto_record_path;

        if (branch_record_source_index >= static_cast<int>(currentWaypoints.size()) ||
            branch_record_source_index < 0 ||
            (branch_record_source_index >= 0 &&
                currentWaypoints[static_cast<size_t>(branch_record_source_index)].type != EWaypointType::MultiBranch))
        {
            branch_record_mode_active = false;
            branch_record_source_index = -1;
            branch_record_first_waypoint_index = -1;
        }

        if (auto_record_path && has_current_player_pos)
        {
            const auto& waypoints = pathManager.GetWaypoints();
            bool should_record = waypoints.empty();
            if (smart_record_path)
            {
                if (waypoints.empty())
                {
                    should_record = true;
                    smart_record_has_last_sample = false;
                    smart_record_has_prev_probe_sample = false;
                    smart_record_has_pending_terminal = false;
                }
                else if (!smart_record_has_last_sample)
                {
                    smart_record_last_sample_pos = current_player_pos;
                    smart_record_last_sample_ang = current_player_angle;
                    smart_record_has_last_sample = true;
                    smart_record_has_prev_probe_sample = false;
                    should_record = false;
                }
                else
                {
                    const float probeDistance = (std::max)(0.1f, smart_record_probe_distance);
                    const Vector sampleDelta = current_player_pos - smart_record_last_sample_pos;
                    const float sampleDelta2DLen = sampleDelta.Length2D();
                    if (sampleDelta2DLen < probeDistance)
                    {
                        should_record = false;
                    }
                    else
                    {
                        const Vector fromLastWaypoint = smart_record_last_sample_pos - waypoints.back().pos;
                        const float fromLastWaypoint2DLen = fromLastWaypoint.Length2D();
                        const float straightThreshold = (std::max)(0.1f, smart_record_straight_angle_deg);
                        bool isStraight = true;
                        bool shouldForceRecordByAngle = false;
                        bool shouldForceRecordByDirection = false;

                        if (fromLastWaypoint2DLen > 0.01f)
                        {
                            const float dot2D =
                                fromLastWaypoint.m_flX * sampleDelta.m_flX +
                                fromLastWaypoint.m_flY * sampleDelta.m_flY;
                            const float denom = fromLastWaypoint2DLen * sampleDelta2DLen;
                            float cosValue = denom > 0.0f ? (dot2D / denom) : 1.0f;
                            cosValue = (std::max)(-1.0f, (std::min)(1.0f, cosValue));
                            const float turnAngleDeg = std::acos(cosValue) * (180.0f / 3.14159265358979323846f);
                            isStraight = turnAngleDeg <= straightThreshold;
                        }

                        // Local direction-change check from consecutive movement vectors.
                        if (smart_record_has_prev_probe_sample)
                        {
                            const Vector prevDelta = smart_record_last_sample_pos - smart_record_prev_probe_sample_pos;
                            const float prevDelta2DLen = prevDelta.Length2D();
                            if (prevDelta2DLen >= probeDistance * 0.35f)
                            {
                                const float dotLocal2D =
                                    prevDelta.m_flX * sampleDelta.m_flX +
                                    prevDelta.m_flY * sampleDelta.m_flY;
                                const float denomLocal = prevDelta2DLen * sampleDelta2DLen;
                                float cosLocal = denomLocal > 0.0f ? (dotLocal2D / denomLocal) : 1.0f;
                                cosLocal = (std::max)(-1.0f, (std::min)(1.0f, cosLocal));
                                const float localTurnDeg = std::acos(cosLocal) * (180.0f / 3.14159265358979323846f);
                                const float directionChangeThreshold = (std::max)(2.5f, straightThreshold * 0.60f);
                                if (localTurnDeg > directionChangeThreshold)
                                    shouldForceRecordByDirection = true;

                                const float localStraightThreshold = (std::max)(2.0f, straightThreshold * 0.75f);
                                if (isStraight && localTurnDeg > localStraightThreshold)
                                    isStraight = false;
                            }
                        }

                        // Side offset check: if current segment deviates laterally enough
                        // from the current straight baseline, break straight compression.
                        if (isStraight && fromLastWaypoint2DLen > probeDistance * 0.65f)
                        {
                            const float crossAbs2D = std::fabs(
                                fromLastWaypoint.m_flX * sampleDelta.m_flY -
                                fromLastWaypoint.m_flY * sampleDelta.m_flX);
                            const float sideOffset = crossAbs2D / fromLastWaypoint2DLen;
                            const float sideOffsetThreshold = (std::max)(1.2f, probeDistance * 0.42f);
                            if (sideOffset > sideOffsetThreshold)
                                isStraight = false;
                        }

                        if (isStraight &&
                            record_waypoint_angle &&
                            record_new_waypoint_on_angle_change)
                        {
                            const float yawDeltaDeg = std::fabs(std::remainderf(
                                current_player_angle.m_flYaw - smart_record_last_sample_ang.m_flYaw,
                                360.0f));
                            const float pitchDeltaDeg = std::fabs(std::remainderf(
                                current_player_angle.m_flPitch - smart_record_last_sample_ang.m_flPitch,
                                360.0f));
                            const float yawThresholdDeg = (std::max)(1.0f, straightThreshold * 0.55f);
                            const float pitchThresholdDeg = (std::max)(1.5f, straightThreshold * 0.75f);
                            shouldForceRecordByAngle =
                                yawDeltaDeg >= yawThresholdDeg || pitchDeltaDeg >= pitchThresholdDeg;
                        }

                        if (isStraight && !shouldForceRecordByAngle && !shouldForceRecordByDirection)
                        {
                            smart_record_pending_terminal_pos = current_player_pos;
                            smart_record_pending_terminal_ang = current_player_angle;
                            smart_record_has_pending_terminal = true;
                            should_record = false;
                        }
                        else
                        {
                            should_record = false;
                            if (smart_record_has_pending_terminal &&
                                waypoints.back().pos.Distance(smart_record_pending_terminal_pos) >
                                (std::max)(0.5f, min_record_distance * 0.35f))
                            {
                                AddRecordedWaypoint(
                                    pathManager,
                                    smart_record_pending_terminal_pos,
                                    record_waypoint_angle ? smart_record_pending_terminal_ang : QAngle{},
                                    recorded_type,
                                    branch_record_mode_active,
                                    branch_record_source_index,
                                    branch_record_first_waypoint_index
                                );
                            }
                            smart_record_has_pending_terminal = false;
                            if (shouldForceRecordByAngle || shouldForceRecordByDirection)
                                should_record = true;
                        }

                        smart_record_prev_probe_sample_pos = smart_record_last_sample_pos;
                        smart_record_has_prev_probe_sample = true;
                        smart_record_last_sample_pos = current_player_pos;
                        smart_record_last_sample_ang = current_player_angle;
                    }
                }
            }
            else if (!should_record)
            {
                if (record_waypoint_angle &&
                    record_new_waypoint_on_angle_change &&
                    waypoints.back().angle != current_player_angle &&
                    (allow_same_position_on_angle_change || waypoints.back().pos != current_player_pos))
                {
                    should_record = true;
                }
                else if (min_record_distance <= 0.0f)
                {
                    should_record = waypoints.back().pos != current_player_pos;
                }
                else
                {
                    should_record = waypoints.back().pos.Distance(current_player_pos) > min_record_distance;
                }
            }

            if (should_record)
            {
                AddRecordedWaypoint(
                    pathManager,
                    current_player_pos,
                    recorded_angle,
                    recorded_type,
                    branch_record_mode_active,
                    branch_record_source_index,
                    branch_record_first_waypoint_index
                );
                if (smart_record_path)
                {
                    smart_record_last_sample_pos = current_player_pos;
                    smart_record_last_sample_ang = current_player_angle;
                    smart_record_has_last_sample = true;
                    smart_record_has_prev_probe_sample = false;
                    smart_record_has_pending_terminal = false;
                }
            }
        }

        g_bOverlayDrawPoints = visualize_path_points;
        g_bOverlayDrawLines = visualize_path_lines;
        g_nOverlayNearestWaypointIndex = has_current_player_pos
            ? FindNearestWaypointIndex(pathManager, current_player_pos)
            : -1;
        g_nOverlayCurrentWaypointIndex =
            (pathRunnerState.isRunning && pathRunnerState.currentWaypointIndex >= 0)
            ? pathRunnerState.currentWaypointIndex
            : -1;
        g_nOverlayNextWaypointIndex = -1;
        if (g_nOverlayCurrentWaypointIndex >= 0)
        {
            const int resolvedNext = pathManager.PeekNextWaypointIndex(static_cast<size_t>(g_nOverlayCurrentWaypointIndex));
            if (resolvedNext >= 0)
                g_nOverlayNextWaypointIndex = resolvedNext;
        }

        std::size_t overlaySourceSignature = 0;
        auto mixOverlaySource = [&](int v)
        {
            overlaySourceSignature ^= static_cast<std::size_t>(v) + 0x9e3779b9u +
                (overlaySourceSignature << 6) + (overlaySourceSignature >> 2);
        };

        mixOverlaySource(visualize_paths ? 1 : 0);
        mixOverlaySource(g_bOverlayVisible ? 1 : 0);
        mixOverlaySource(g_nOverlayWidth);
        mixOverlaySource(g_nOverlayHeight);
        mixOverlaySource(g_nOverlayNearestWaypointIndex);
        mixOverlaySource(g_nOverlayCurrentWaypointIndex);
        mixOverlaySource(g_nOverlayNextWaypointIndex);
        mixOverlaySource(has_current_player_pos ? 1 : 0);
        if (has_current_player_pos)
        {
            mixOverlaySource(static_cast<int>(std::lround(current_player_pos.m_flX * 10.0f)));
            mixOverlaySource(static_cast<int>(std::lround(current_player_pos.m_flY * 10.0f)));
            mixOverlaySource(static_cast<int>(std::lround(current_player_pos.m_flZ * 10.0f)));
            mixOverlaySource(static_cast<int>(std::lround(current_player_angle.m_flYaw * 10.0f)));
            mixOverlaySource(static_cast<int>(std::lround(current_player_angle.m_flPitch * 10.0f)));
        }

        // Include the view-projection matrix so camera movement immediately triggers overlay rebuild.
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
                mixOverlaySource(static_cast<int>(std::lround(g_Globals.m_matViewMatrix[r][c] * 1000.0f)));
        }

        const bool overlaySourceChanged = (overlaySourceSignature != g_uOverlaySourceSignature);
        if (overlaySourceChanged)
            g_uOverlaySourceSignature = overlaySourceSignature;

        const ULONGLONG overlayNowTick = GetTickCount64();
        const bool shouldRebuildOverlay =
            g_bOverlayVisible &&
            visualize_paths &&
            (overlaySourceChanged ||
                (overlayNowTick - g_uOverlayLastBuildTick >= kOverlayBuildIntervalMs) ||
                g_bOverlayNeedsRepaint);
        bool overlayRebuilt = false;
        std::vector<OverlayPoint_t> nextOverlayPoints;
        std::vector<OverlayLine_t> nextOverlayLines;

        if (shouldRebuildOverlay)
        {
            const auto& waypoints = pathManager.GetWaypoints();
            if (!waypoints.empty())
            {
                branch_target_selection.resize(waypoints.size(), 0);
                nextOverlayPoints.reserve(waypoints.size());
                nextOverlayLines.reserve(waypoints.size() * 3);

                const ImVec2 displaySize(
                    static_cast<float>(g_nOverlayWidth),
                    static_cast<float>(g_nOverlayHeight)
                );
                if (displaySize.x > 0.0f && displaySize.y > 0.0f)
                {
                    const ViewMatrix_t& matWorldToScreen = g_Globals.m_matViewMatrix;
                    std::vector<bool> pointVisible(waypoints.size(), false);
                    std::vector<POINT> pointScreenPositions(waypoints.size());

                    for (size_t i = 0; i < waypoints.size(); ++i)
                    {
                        const int waypointIndex = static_cast<int>(i);
                        ImVec2 screenPos{};
                        const bool onScreen = WorldToScreen(waypoints[i].pos, screenPos, matWorldToScreen, displaySize);
                        if (!onScreen)
                            continue;

                        pointVisible[i] = true;
                        pointScreenPositions[i] = POINT{
                            static_cast<LONG>(screenPos.x),
                            static_cast<LONG>(screenPos.y)
                        };
                        nextOverlayPoints.push_back(OverlayPoint_t{
                            pointScreenPositions[i],
                            waypoints[i].type,
                            waypointIndex,
                            waypointIndex == g_nOverlayCurrentWaypointIndex,
                            waypointIndex == g_nOverlayNextWaypointIndex,
                            waypointIndex == g_nOverlayNearestWaypointIndex
                        });

                        if (g_bOverlayDrawHeadingHints)
                        {
                            const float yawRad = waypoints[i].angle.m_flYaw * 0.01745329252f;
                            const Vector hintWorld{
                                waypoints[i].pos.m_flX + std::cos(yawRad) * 28.0f,
                                waypoints[i].pos.m_flY + std::sin(yawRad) * 28.0f,
                                waypoints[i].pos.m_flZ
                            };
                            ImVec2 hintScreen{};
                            if (WorldToScreen(hintWorld, hintScreen, matWorldToScreen, displaySize))
                            {
                                nextOverlayLines.push_back(OverlayLine_t{
                                    pointScreenPositions[i],
                                    POINT{ static_cast<LONG>(hintScreen.x), static_cast<LONG>(hintScreen.y) },
                                    EOverlayLineStyle::HeadingHint
                                });
                            }
                        }
                    }

                    for (size_t i = 0; i < waypoints.size(); ++i)
                    {
                        if (!pointVisible[i])
                            continue;

                        const Waypoint& waypoint = waypoints[i];
                        if (waypoint.type == EWaypointType::MultiBranch && !waypoint.nextIndices.empty())
                        {
                            for (int nextIndex : waypoint.nextIndices)
                            {
                                if (nextIndex < 0 || nextIndex >= static_cast<int>(waypoints.size()))
                                    continue;
                                if (!pointVisible[static_cast<size_t>(nextIndex)])
                                    continue;

                                nextOverlayLines.push_back(OverlayLine_t{
                                    pointScreenPositions[i],
                                    pointScreenPositions[static_cast<size_t>(nextIndex)],
                                    (static_cast<int>(i) == g_nOverlayCurrentWaypointIndex &&
                                        nextIndex == g_nOverlayNextWaypointIndex)
                                        ? EOverlayLineStyle::ActivePath
                                        : EOverlayLineStyle::Branch
                                });
                            }
                        }
                        else if (i + 1 < waypoints.size() && pointVisible[i + 1])
                        {
                            const int nextIndex = static_cast<int>(i + 1);
                            nextOverlayLines.push_back(OverlayLine_t{
                                pointScreenPositions[i],
                                pointScreenPositions[i + 1],
                                (static_cast<int>(i) == g_nOverlayCurrentWaypointIndex &&
                                    nextIndex == g_nOverlayNextWaypointIndex)
                                    ? EOverlayLineStyle::ActivePath
                                    : EOverlayLineStyle::Normal
                            });
                        }
                    }

                    if (g_bOverlayDrawPlayerGuide &&
                        has_current_player_pos &&
                        g_nOverlayCurrentWaypointIndex >= 0 &&
                        g_nOverlayCurrentWaypointIndex < static_cast<int>(waypoints.size()))
                    {
                        ImVec2 playerScreen{};
                        if (WorldToScreen(current_player_pos, playerScreen, matWorldToScreen, displaySize))
                        {
                            const Waypoint& currentWp = waypoints[static_cast<size_t>(g_nOverlayCurrentWaypointIndex)];
                            ImVec2 targetScreen{};
                            if (WorldToScreen(currentWp.pos, targetScreen, matWorldToScreen, displaySize))
                            {
                                nextOverlayLines.push_back(OverlayLine_t{
                                    POINT{ static_cast<LONG>(playerScreen.x), static_cast<LONG>(playerScreen.y) },
                                    POINT{ static_cast<LONG>(targetScreen.x), static_cast<LONG>(targetScreen.y) },
                                    EOverlayLineStyle::PlayerGuide
                                });
                            }
                        }
                    }
                }
            }

            overlayRebuilt = true;
            g_uOverlayLastBuildTick = overlayNowTick;
        }

        {
            std::lock_guard<std::mutex> overlayLock(g_OverlayDataMutex);
            if (overlayRebuilt)
            {
                g_OverlayPoints.swap(nextOverlayPoints);
                g_OverlayLines.swap(nextOverlayLines);
            }

            std::size_t overlayStateSignature = 0;
            auto mixOverlayState = [&](int v)
            {
                overlayStateSignature ^= static_cast<std::size_t>(v) + 0x9e3779b9u +
                    (overlayStateSignature << 6) + (overlayStateSignature >> 2);
            };
            mixOverlayState(visualize_paths ? 1 : 0);
            mixOverlayState(g_bOverlayVisible ? 1 : 0);
            mixOverlayState(g_bOverlayDrawPoints ? 1 : 0);
            mixOverlayState(g_bOverlayDrawPointIndices ? 1 : 0);
            mixOverlayState(g_bOverlayDrawLines ? 1 : 0);
            mixOverlayState(g_bOverlayDrawPointRings ? 1 : 0);
            mixOverlayState(g_bOverlayDrawPointLabels ? 1 : 0);
            mixOverlayState(g_bOverlayDrawHeadingHints ? 1 : 0);
            mixOverlayState(g_bOverlayDrawPlayerGuide ? 1 : 0);
            mixOverlayState(g_bOverlayHighlightCurrent ? 1 : 0);
            mixOverlayState(g_bOverlayHighlightNearest ? 1 : 0);
            mixOverlayState(g_nOverlayNearestWaypointIndex);
            mixOverlayState(g_nOverlayCurrentWaypointIndex);
            mixOverlayState(g_nOverlayNextWaypointIndex);
            mixOverlayState(g_nOverlayWidth);
            mixOverlayState(g_nOverlayHeight);
            mixOverlayState(static_cast<int>(g_OverlayPoints.size()));
            mixOverlayState(static_cast<int>(g_OverlayLines.size()));

            if (overlayRebuilt || overlayStateSignature != g_uOverlayStateSignature)
            {
                g_uOverlayStateSignature = overlayStateSignature;
                g_bOverlayNeedsRepaint = true;
            }
        }

        if (g_hOverlayWindow && g_bOverlayVisible && g_bOverlayNeedsRepaint)
        {
            RedrawWindow(g_hOverlayWindow, nullptr, nullptr, RDW_INVALIDATE | RDW_NOERASE);
            g_bOverlayNeedsRepaint = false;
        }

        ImGui::Begin("CS2 WalkBot Ext", nullptr, ImGuiWindowFlags_NoMove);
        ImGui::SetWindowPos(ImVec2(0, 0));
        ImGui::SetWindowSize(ImGui::GetIO().DisplaySize);

        if (ImGui::BeginTabBar("MainTabs"))
        {
            if (ImGui::BeginTabItem("Process"))
            {
                ImGui::Text("Process Management");
                ImGui::Separator();

                if (ImGui::Button("Attach to CS2", ImVec2(120, 0)))
                {
                    if (AttachCS2())
                    {
                        g_Memory.AttachToProcess(g_hProcess, g_dwProcessId);
                        g_Globals.Initialize();

                        strcpy_s(attach_status, sizeof(attach_status), "Attached successfully");
                        snprintf(process_info, sizeof(process_info),
                            "PID: %lu\nHandle: %p", g_dwProcessId, g_hProcess);
                    }
                    else
                    {
                        strcpy_s(attach_status, sizeof(attach_status), "Attach failed - check console");
                        strcpy_s(process_info, sizeof(process_info), "");
                    }
                }

                ImGui::SameLine();
                if (IsCS2Attached())
                {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "ATTACHED");
                }
                else
                {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "NOT ATTACHED");
                }

                ImGui::Separator();
                ImGui::Text("Status: %s", attach_status);

                ImGui::Separator();
                ImGui::Text("Overlay Window: %s", g_bOverlayVisible ? "VISIBLE" : "HIDDEN");
                ImGui::Text("Overlay Size: %d x %d", g_nOverlayWidth, g_nOverlayHeight);

                if (strlen(process_info) > 0)
                {
                    ImGui::Separator();
                    ImGui::Text("Process Info:");
                    ImGui::TextWrapped("%s", process_info);
                }

                ImGui::Separator();
                ImGui::Text("Game State:");

                const GameState_t& gameState = CUtilities::GetGameState();

                if (gameState.m_bIsConnected)
                {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Connected: YES");
                }
                else
                {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Connected: NO");
                }

                if (gameState.m_bIsInGame)
                {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "In Game: YES");
                }
                else
                {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "In Game: NO");
                }

                if (gameState.m_bIsChangingLevel)
                {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Changing Level: YES");
                }
                else
                {
                    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), "Changing Level: NO");
                }

                ImGui::Text("Signon State: %d", gameState.m_nCurrentSignonState);

                ImGui::Separator();
                ImGui::Text("Local Player:");

                if (g_Globals.m_LocalPlayer.IsValid())
                {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Player: VALID");
                    ImGui::Text("Controller: %p", g_Globals.m_LocalPlayer.m_pController);
                    ImGui::Text("Pawn: %p", g_Globals.m_LocalPlayer.m_pPlayerPawn);
                    const Vector& localVelocity = g_Globals.m_LocalPlayer.m_vecVelocity;
                    const float speed2D = std::sqrt(
                        localVelocity.m_flX * localVelocity.m_flX +
                        localVelocity.m_flY * localVelocity.m_flY
                    );
                    ImGui::Text(
                        "Velocity: (%.2f, %.2f, %.2f)  Speed2D: %.2f",
                        localVelocity.m_flX,
                        localVelocity.m_flY,
                        localVelocity.m_flZ,
                        speed2D
                    );
                }
                else
                {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Player: INVALID");
                }

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Main"))
            {
                const MainTabActions_t mainTabActions = DrawMainRunnerTab(
                    pathRunnerState,
                    pathManager,
                    has_current_player_pos,
                    current_player_pos
                );
                if (mainTabActions.startRunRequested)
                    startRun();
                if (mainTabActions.stopRunRequested)
                    stopRun();

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Visual"))
            {
                ImGui::Text("Overlay Visuals");
                ImGui::Separator();

                ImGui::Checkbox("Enable Path Overlay", &visualize_paths);
                ImGui::Checkbox("Draw Waypoints", &visualize_path_points);
                ImGui::Checkbox("Draw Path Lines", &visualize_path_lines);
                ImGui::Checkbox("Draw Point Rings", &g_bOverlayDrawPointRings);
                ImGui::Checkbox("Draw Labels", &g_bOverlayDrawPointLabels);
                ImGui::Checkbox("Draw Point Indices", &g_bOverlayDrawPointIndices);
                ImGui::Checkbox("Draw Heading Hints", &g_bOverlayDrawHeadingHints);
                ImGui::Checkbox("Draw Player Guide Line", &g_bOverlayDrawPlayerGuide);
                ImGui::Checkbox("Highlight Current/Next", &g_bOverlayHighlightCurrent);
                ImGui::Checkbox("Highlight Nearest", &g_bOverlayHighlightNearest);

                ImGui::Separator();
                ImGui::Text("Overlay Runtime");
                ImGui::Text("Visible: %s", g_bOverlayVisible ? "YES" : "NO");
                ImGui::Text("Current Waypoint: %d", g_nOverlayCurrentWaypointIndex);
                ImGui::Text("Next Waypoint: %d", g_nOverlayNextWaypointIndex);
                ImGui::Text("Nearest Waypoint: %d", g_nOverlayNearestWaypointIndex);

                if (ImGui::Button("Force Overlay Repaint", ImVec2(170, 0)))
                    g_bOverlayNeedsRepaint = true;

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Settings"))
            {
                ImGui::Text("Configuration");
                ImGui::Separator();
                ImGui::Checkbox("Auto Save Config On Exit", &auto_save_config);
                ImGui::Text("Present Mode: Present(0,0) [Forced]");
                ImGui::TextWrapped("%s", config_status);

                if (ImGui::Button("Save Config", ImVec2(130, 0)))
                    saveConfig("manual");
                ImGui::SameLine();
                if (ImGui::Button("Load Config", ImVec2(130, 0)))
                    loadConfig();

                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Paths"))
            {
                ImGui::Text("Path Recorder");
                ImGui::Separator();

                if (has_current_player_pos)
                {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Current Position");
                    ImGui::Text("X: %.2f", current_player_pos.m_flX);
                    ImGui::Text("Y: %.2f", current_player_pos.m_flY);
                    ImGui::Text("Z: %.2f", current_player_pos.m_flZ);
                    ImGui::Text("Pitch: %.2f", current_player_angle.m_flPitch);
                    ImGui::Text("Yaw: %.2f", current_player_angle.m_flYaw);
                    ImGui::Text("Roll: %.2f", current_player_angle.m_flRoll);
                }
                else
                {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Current Position Unavailable");
                    ImGui::Text("Attach and enter a match to read pawn position");
                }

                ImGui::Separator();
                ImGui::Checkbox("Auto Record", &auto_record_path);
                ImGui::SetNextItemWidth(220.0f);
                ImGui::DragFloat("Min Record Distance", &min_record_distance, 1.0f, 0.0f, 1000.0f, "%.1f");
                ImGui::Checkbox("Smart Record (Straight Compression)", &smart_record_path);
                ImGui::BeginDisabled(!smart_record_path);
                ImGui::SetNextItemWidth(220.0f);
                ImGui::DragFloat("Smart Straight Angle (deg)", &smart_record_straight_angle_deg, 0.1f, 0.1f, 45.0f, "%.1f");
                ImGui::SetNextItemWidth(220.0f);
                ImGui::DragFloat("Smart Probe Distance", &smart_record_probe_distance, 0.5f, 0.1f, 200.0f, "%.1f");
                ImGui::EndDisabled();
                ImGui::Checkbox("Record Angle", &record_waypoint_angle);
                ImGui::BeginDisabled(!record_waypoint_angle);
                ImGui::Checkbox("New Point On Angle Change", &record_new_waypoint_on_angle_change);
                ImGui::BeginDisabled(!record_new_waypoint_on_angle_change);
                ImGui::Checkbox("Allow Same Pos On Angle Change", &allow_same_position_on_angle_change);
                ImGui::EndDisabled();
                ImGui::EndDisabled();
                ImGui::SetNextItemWidth(220.0f);
                ImGui::Combo("New Waypoint Type", &new_waypoint_type, "Normal\0Multi-Branch\0");
                ImGui::Separator();
                ImGui::Text("Branch Record");
                ImGui::Text(
                    "Status: %s",
                    branch_record_mode_active
                        ? "Recording from selected multi-branch point"
                        : "Inactive"
                );
                ImGui::SetNextItemWidth(220.0f);
                ImGui::DragInt(
                    "Branch Source",
                    &branch_record_source_index,
                    0.1f,
                    -1,
                    (std::max)(-1, static_cast<int>(pathManager.GetWaypointCount()) - 1)
                );

                const auto& branchRecordWaypoints = pathManager.GetWaypoints();
                const bool canStartBranchRecord =
                    branch_record_source_index >= 0 &&
                    branch_record_source_index < static_cast<int>(branchRecordWaypoints.size()) &&
                    branchRecordWaypoints[static_cast<size_t>(branch_record_source_index)].type == EWaypointType::MultiBranch;

                if (!canStartBranchRecord)
                    ImGui::BeginDisabled();
                if (ImGui::Button("Start Branch Record", ImVec2(170, 0)) && canStartBranchRecord)
                {
                    branch_record_mode_active = true;
                    branch_record_first_waypoint_index = -1;
                }
                if (!canStartBranchRecord)
                    ImGui::EndDisabled();

                ImGui::SameLine();
                if (ImGui::Button("Stop Branch Record", ImVec2(170, 0)))
                {
                    branch_record_mode_active = false;
                    branch_record_first_waypoint_index = -1;
                }

                if (ImGui::Button("Record Current Pos", ImVec2(150, 0)) && has_current_player_pos)
                {
                    AddRecordedWaypoint(
                        pathManager,
                        current_player_pos,
                        recorded_angle,
                        recorded_type,
                        branch_record_mode_active,
                        branch_record_source_index,
                        branch_record_first_waypoint_index
                    );
                }

                ImGui::SameLine();
                if (ImGui::Button("Reload Paths", ImVec2(120, 0)))
                {
                    pathManager.LoadFromFile();
                    branch_record_mode_active = false;
                    branch_record_source_index = -1;
                    branch_record_first_waypoint_index = -1;
                }

                ImGui::SameLine();
                if (ImGui::Button("Save Paths", ImVec2(120, 0)))
                    pathManager.SaveToFile();

                ImGui::SameLine();
                if (ImGui::Button("Clear Paths", ImVec2(120, 0)))
                {
                    pathManager.Clear();
                    branch_record_mode_active = false;
                    branch_record_source_index = -1;
                    branch_record_first_waypoint_index = -1;
                }

                ImGui::Separator();
                ImGui::Text("Waypoint Count: %zu", pathManager.GetWaypointCount());
                const size_t waypointCount = pathManager.GetWaypointCount();
                if (waypointCount > 0)
                {
                    if (preview_waypoint_index >= static_cast<int>(waypointCount))
                        preview_waypoint_index = static_cast<int>(waypointCount - 1);

                    ImGui::SetNextItemWidth(120.0f);
                    ImGui::DragInt("Preview From", &preview_waypoint_index, 0.1f, 0, static_cast<int>(waypointCount) - 1);
                    ImGui::SameLine();
                    if (ImGui::Button("Preview Next", ImVec2(110, 0)))
                    {
                        const int nextIndex = pathManager.ResolveNextWaypointIndex(static_cast<size_t>(preview_waypoint_index));
                        if (nextIndex >= 0)
                            preview_waypoint_index = nextIndex;
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Reset Branch State", ImVec2(140, 0)))
                        pathManager.ResetRuntimeState();

                    const int resolvedPreviewNext = pathManager.PeekNextWaypointIndex(static_cast<size_t>(preview_waypoint_index));
                    ImGui::Text("Resolved Next: %d", resolvedPreviewNext);
                }

                const auto& editableWaypoints = pathManager.GetWaypoints();
                if (!editableWaypoints.empty())
                {
                    if (selected_waypoint_index < 0 || selected_waypoint_index >= static_cast<int>(editableWaypoints.size()))
                    {
                        selected_waypoint_index = 0;
                        waypoint_editor_dirty = false;
                    }

                    ImGui::Separator();
                    ImGui::Text("Quick Edit");
                    ImGui::SetNextItemWidth(180.0f);
                    if (ImGui::DragInt("Selected Index", &selected_waypoint_index, 0.1f, 0, static_cast<int>(editableWaypoints.size()) - 1))
                    {
                        waypoint_editor_dirty = false;
                    }

                    const Waypoint& selectedWaypoint = editableWaypoints[static_cast<size_t>(selected_waypoint_index)];
                    if (!waypoint_editor_dirty)
                    {
                        waypoint_edit_pos = selectedWaypoint.pos;
                        waypoint_edit_ang = selectedWaypoint.angle;
                    }

                    bool localEdited = false;
                    ImGui::SetNextItemWidth(220.0f);
                    localEdited |= ImGui::DragFloat3("Edit Pos", &waypoint_edit_pos.m_flX, 0.5f, -50000.0f, 50000.0f, "%.2f");
                    ImGui::SetNextItemWidth(220.0f);
                    localEdited |= ImGui::DragFloat("Edit Pitch", &waypoint_edit_ang.m_flPitch, 0.2f, -89.0f, 89.0f, "%.2f");
                    ImGui::SetNextItemWidth(220.0f);
                    localEdited |= ImGui::DragFloat("Edit Yaw", &waypoint_edit_ang.m_flYaw, 0.2f, -180.0f, 180.0f, "%.2f");
                    if (localEdited)
                        waypoint_editor_dirty = true;

                    if (ImGui::Button("Apply Position/Angle", ImVec2(190, 0)))
                    {
                        pathManager.SetWaypointTransform(
                            static_cast<size_t>(selected_waypoint_index),
                            waypoint_edit_pos,
                            waypoint_edit_ang
                        );
                        waypoint_editor_dirty = false;
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Use Current View", ImVec2(140, 0)) && has_current_player_pos)
                    {
                        waypoint_edit_pos = selectedWaypoint.pos;
                        waypoint_edit_ang = current_player_angle;
                        waypoint_editor_dirty = true;
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Use Current Pos+View", ImVec2(170, 0)) && has_current_player_pos)
                    {
                        waypoint_edit_pos = current_player_pos;
                        waypoint_edit_ang = current_player_angle;
                        waypoint_editor_dirty = true;
                    }

                    ImGui::Separator();
                    ImGui::Text("Behavior Flags");
                    std::uint32_t wpFlags = selectedWaypoint.flags;
                    bool flagCrouch = (wpFlags & WPF_CROUCH) != 0u;
                    bool flagJump   = (wpFlags & WPF_JUMP)   != 0u;
                    bool flagWalk   = (wpFlags & WPF_WALK)   != 0u;
                    bool flagsChanged = false;
                    if (ImGui::Checkbox("Crouch", &flagCrouch)) flagsChanged = true;
                    ImGui::SameLine();
                    if (ImGui::Checkbox("Jump On Arrival", &flagJump)) flagsChanged = true;
                    ImGui::SameLine();
                    if (ImGui::Checkbox("Walk (shift)", &flagWalk)) flagsChanged = true;
                    if (flagsChanged)
                    {
                        std::uint32_t newFlags = 0;
                        if (flagCrouch) newFlags |= WPF_CROUCH;
                        if (flagJump)   newFlags |= WPF_JUMP;
                        if (flagWalk)   newFlags |= WPF_WALK;
                        pathManager.SetWaypointFlags(static_cast<size_t>(selected_waypoint_index), newFlags);
                    }

                    float wpRadius = selectedWaypoint.radius;
                    ImGui::SetNextItemWidth(220.0f);
                    if (ImGui::DragFloat("Reach Radius (0=global)", &wpRadius, 0.5f, 0.0f, 500.0f, "%.1f"))
                        pathManager.SetWaypointRadius(static_cast<size_t>(selected_waypoint_index), wpRadius);

                    float wpWait = selectedWaypoint.waitTime;
                    ImGui::SetNextItemWidth(220.0f);
                    if (ImGui::DragFloat("Wait On Arrival (s)", &wpWait, 0.05f, 0.0f, 60.0f, "%.2f"))
                        pathManager.SetWaypointWaitTime(static_cast<size_t>(selected_waypoint_index), wpWait);

                    float wpSpeed = selectedWaypoint.desiredSpeed;
                    ImGui::SetNextItemWidth(220.0f);
                    if (ImGui::DragFloat("Desired Speed (0..1, <0.75=walk)", &wpSpeed, 0.01f, 0.0f, 1.0f, "%.2f"))
                        pathManager.SetWaypointDesiredSpeed(static_cast<size_t>(selected_waypoint_index), wpSpeed);
                }

                if (ImGui::BeginChild("PathList", ImVec2(0, 0), true))
                {
                    const auto& waypoints = pathManager.GetWaypoints();
                    if (branch_target_selection.size() < waypoints.size())
                        branch_target_selection.resize(waypoints.size(), 0);

                    for (size_t i = 0; i < waypoints.size(); ++i)
                    {
                        const Waypoint& waypoint = waypoints[i];
                        const std::string connectionsLabel = GetWaypointConnectionsLabel(pathManager, i);
                        bool shouldBreakWaypointLoop = false;
                        ImGui::PushID(static_cast<int>(i));
                        ImGui::Separator();
                        char waypointLabel[256]{};
                        snprintf(
                            waypointLabel,
                            sizeof(waypointLabel),
                            "#%zu  [%s]  Pos(%.2f, %.2f, %.2f)  Ang(%.2f, %.2f, %.2f)",
                            i,
                            GetWaypointTypeLabel(waypoint.type),
                            waypoint.pos.m_flX,
                            waypoint.pos.m_flY,
                            waypoint.pos.m_flZ,
                            waypoint.angle.m_flPitch,
                            waypoint.angle.m_flYaw,
                            waypoint.angle.m_flRoll
                        );
                        const bool isSelectedWaypoint = selected_waypoint_index == static_cast<int>(i);
                        if (ImGui::Selectable(waypointLabel, isSelectedWaypoint))
                        {
                            selected_waypoint_index = static_cast<int>(i);
                            waypoint_editor_dirty = false;
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("Up") && i > 0)
                        {
                            pathManager.MoveWaypointUp(i);
                            if (selected_waypoint_index == static_cast<int>(i))
                                selected_waypoint_index -= 1;
                            shouldBreakWaypointLoop = true;
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("Dn") && i + 1 < waypoints.size())
                        {
                            pathManager.MoveWaypointDown(i);
                            if (selected_waypoint_index == static_cast<int>(i))
                                selected_waypoint_index += 1;
                            shouldBreakWaypointLoop = true;
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("Del"))
                        {
                            pathManager.RemoveWaypoint(i);
                            if (selected_waypoint_index >= static_cast<int>(waypoints.size()) - 1)
                                selected_waypoint_index = static_cast<int>((std::max)(0, static_cast<int>(waypoints.size()) - 2));
                            waypoint_editor_dirty = false;
                            shouldBreakWaypointLoop = true;
                        }
                        ImGui::Text("Next: %s", connectionsLabel.c_str());
                        if (waypoint.type == EWaypointType::MultiBranch)
                            ImGui::Text("Branch Mode: %s", GetBranchModeLabel(waypoint.branchMode));

                        if (ImGui::BeginPopupContextItem("WaypointContext"))
                        {
                            if (ImGui::MenuItem("Delete"))
                            {
                                pathManager.RemoveWaypoint(i);
                                shouldBreakWaypointLoop = true;
                            }
                            if (ImGui::MenuItem("Move Up", nullptr, false, i > 0))
                            {
                                pathManager.MoveWaypointUp(i);
                                shouldBreakWaypointLoop = true;
                            }
                            if (ImGui::MenuItem("Move Down", nullptr, false, i + 1 < waypoints.size()))
                            {
                                pathManager.MoveWaypointDown(i);
                                shouldBreakWaypointLoop = true;
                            }

                            if (ImGui::BeginMenu("Type"))
                            {
                                if (ImGui::MenuItem("Normal", nullptr, waypoint.type == EWaypointType::Normal))
                                {
                                    pathManager.SetWaypointType(i, EWaypointType::Normal);
                                    shouldBreakWaypointLoop = true;
                                }
                                if (ImGui::MenuItem("Multi-Branch", nullptr, waypoint.type == EWaypointType::MultiBranch))
                                {
                                    pathManager.SetWaypointType(i, EWaypointType::MultiBranch);
                                    shouldBreakWaypointLoop = true;
                                }
                                ImGui::EndMenu();
                            }

                            if (waypoint.type == EWaypointType::MultiBranch)
                            {
                                if (ImGui::MenuItem("Use As Branch Record Source"))
                                {
                                    branch_record_source_index = static_cast<int>(i);
                                    branch_record_mode_active = true;
                                    branch_record_first_waypoint_index = -1;
                                }

                                if (ImGui::BeginMenu("Branch Mode"))
                                {
                                    if (ImGui::MenuItem("Sequential", nullptr, waypoint.branchMode == EBranchSelectMode::Sequential))
                                    {
                                        pathManager.SetWaypointBranchMode(i, EBranchSelectMode::Sequential);
                                        shouldBreakWaypointLoop = true;
                                    }
                                    if (ImGui::MenuItem("Random", nullptr, waypoint.branchMode == EBranchSelectMode::Random))
                                    {
                                        pathManager.SetWaypointBranchMode(i, EBranchSelectMode::Random);
                                        shouldBreakWaypointLoop = true;
                                    }
                                    if (ImGui::MenuItem("Cycle", nullptr, waypoint.branchMode == EBranchSelectMode::Cycle))
                                    {
                                        pathManager.SetWaypointBranchMode(i, EBranchSelectMode::Cycle);
                                        shouldBreakWaypointLoop = true;
                                    }
                                    ImGui::EndMenu();
                                }

                                int& branchTargetIndex = branch_target_selection[i];
                                if (branchTargetIndex >= static_cast<int>(waypoints.size()))
                                    branchTargetIndex = static_cast<int>(waypoints.empty() ? 0 : waypoints.size() - 1);
                                if (branchTargetIndex == static_cast<int>(i) && waypoints.size() > 1)
                                    branchTargetIndex = (branchTargetIndex + 1) % static_cast<int>(waypoints.size());

                                ImGui::SetNextItemWidth(120.0f);
                                ImGui::DragInt("Route Target", &branchTargetIndex, 0.1f, 0, static_cast<int>(waypoints.size()) - 1);

                                const bool canAddBranch =
                                    branchTargetIndex >= 0 &&
                                    branchTargetIndex < static_cast<int>(waypoints.size()) &&
                                    branchTargetIndex != static_cast<int>(i);
                                if (!canAddBranch)
                                    ImGui::BeginDisabled();
                                if (ImGui::Button("Add Route", ImVec2(100, 0)) && canAddBranch)
                                {
                                    pathManager.AddBranchConnection(i, branchTargetIndex);
                                    shouldBreakWaypointLoop = true;
                                }
                                if (!canAddBranch)
                                    ImGui::EndDisabled();

                                for (int nextIndex : waypoint.nextIndices)
                                {
                                    ImGui::Text("Branch -> #%d", nextIndex);
                                    ImGui::SameLine();
                                    if (ImGui::Button(("Remove Route##" + std::to_string(nextIndex)).c_str(), ImVec2(110, 0)))
                                    {
                                        pathManager.RemoveBranchConnection(i, nextIndex);
                                        shouldBreakWaypointLoop = true;
                                        break;
                                    }
                                }
                            }

                            ImGui::EndPopup();
                        }

                        ImGui::PopID();
                        if (shouldBreakWaypointLoop)
                            break;
                    }
                }
                ImGui::EndChild();

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

        ImGui::End();

        if (show_demo)
            ImGui::ShowDemoWindow(&show_demo);

        ImGui::Render();
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, (float*)&clear_color);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        g_pSwapChain->Present(0, 0);
    }

    if (smart_record_path &&
        smart_record_has_pending_terminal &&
        auto_record_path &&
        pathManager.GetWaypointCount() > 0)
    {
        const auto& waypoints = pathManager.GetWaypoints();
        if (!waypoints.empty() &&
            waypoints.back().pos.Distance(smart_record_pending_terminal_pos) > 0.5f)
        {
            const EWaypointType recorded_type = (new_waypoint_type == static_cast<int>(EWaypointType::MultiBranch))
                ? EWaypointType::MultiBranch
                : EWaypointType::Normal;
            AddRecordedWaypoint(
                pathManager,
                smart_record_pending_terminal_pos,
                record_waypoint_angle ? smart_record_pending_terminal_ang : QAngle{},
                recorded_type,
                branch_record_mode_active,
                branch_record_source_index,
                branch_record_first_waypoint_index
            );
        }
    }

    if (auto_save_config)
        saveConfig("exit");

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    StopPathRunner(pathRunnerState);

    CleanupDeviceD3D();
    if (g_hOverlayWindow)
        PostMessageW(g_hOverlayWindow, WM_CLOSE, 0, 0);
    if (g_hOverlayThread)
    {
        WaitForSingleObject(g_hOverlayThread, 5000);
        CloseHandle(g_hOverlayThread);
        g_hOverlayThread = nullptr;
    }
    DestroyWindow(hwnd);
    UnregisterClassW(mainWindowClass.lpszClassName, mainWindowClass.hInstance);

    DetachCS2();

    return 0;
}

bool CreateDeviceD3D(HWND hWnd)
{
    DXGI_SWAP_CHAIN_DESC sd = {};
    sd.BufferCount                        = 2;
    sd.BufferDesc.Width                   = 0;
    sd.BufferDesc.Height                  = 0;
    sd.BufferDesc.Format                  = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator   = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags                              = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage                        = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow                       = hWnd;
    sd.SampleDesc.Count                   = 1;
    sd.SampleDesc.Quality                 = 0;
    sd.Windowed                           = TRUE;
    sd.SwapEffect                         = DXGI_SWAP_EFFECT_DISCARD;

    UINT createDeviceFlags = 0;
    D3D_FEATURE_LEVEL featureLevel;
    const D3D_FEATURE_LEVEL featureLevelArray[] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };

    HRESULT res = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
        createDeviceFlags, featureLevelArray, 2,
        D3D11_SDK_VERSION, &sd, &g_pSwapChain,
        &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext
    );
    if (res == DXGI_ERROR_UNSUPPORTED)
        res = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_WARP, nullptr,
            createDeviceFlags, featureLevelArray, 2,
            D3D11_SDK_VERSION, &sd, &g_pSwapChain,
            &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext
        );
    if (res != S_OK)
        return false;

    CreateRenderTarget();
    return true;
}

void CleanupDeviceD3D()
{
    CleanupRenderTarget();
    if (g_pSwapChain)        { g_pSwapChain->Release();        g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice)        { g_pd3dDevice->Release();        g_pd3dDevice = nullptr; }
}

void CreateRenderTarget()
{
    ID3D11Texture2D* pBackBuffer = nullptr;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    if (pBackBuffer)
    {
        g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
        pBackBuffer->Release();
    }
}

void CleanupRenderTarget()
{
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg)
    {
    case WM_SIZE:
        if (g_pd3dDevice != nullptr && wParam != SIZE_MINIMIZED)
        {
            CleanupRenderTarget();
            g_pSwapChain->ResizeBuffers(0, (UINT)LOWORD(lParam), (UINT)HIWORD(lParam), DXGI_FORMAT_UNKNOWN, 0);
            CreateRenderTarget();
        }
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU)
            return 0;
        break;
    case WM_DESTROY:
        if (hWnd == g_hOverlayWindow)
            g_hOverlayWindow = nullptr;
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

LRESULT WINAPI OverlayWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_ERASEBKGND:
        return 1;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    case WM_PAINT:
    {
        PAINTSTRUCT ps{};
        HDC hdc = BeginPaint(hWnd, &ps);
        std::vector<OverlayPoint_t> overlayPoints;
        std::vector<OverlayLine_t> overlayLines;
        bool drawLines = false;
        bool drawPoints = false;
        bool drawPointRings = false;
        bool drawPointIndices = false;
        bool drawPointLabels = false;
        bool highlightCurrent = false;
        bool highlightNearest = false;
        {
            std::lock_guard<std::mutex> overlayLock(g_OverlayDataMutex);
            overlayPoints = g_OverlayPoints;
            overlayLines = g_OverlayLines;
            drawLines = g_bOverlayDrawLines;
            drawPoints = g_bOverlayDrawPoints;
            drawPointRings = g_bOverlayDrawPointRings;
            drawPointIndices = g_bOverlayDrawPointIndices;
            drawPointLabels = g_bOverlayDrawPointLabels;
            highlightCurrent = g_bOverlayHighlightCurrent;
            highlightNearest = g_bOverlayHighlightNearest;
        }

        static HPEN hNormalLinePen = CreatePen(PS_SOLID, 2, RGB(80, 220, 120));
        static HPEN hBranchLinePen = CreatePen(PS_SOLID, 2, RGB(80, 180, 255));
        static HPEN hActiveLinePen = CreatePen(PS_SOLID, 3, RGB(255, 210, 80));
        static HPEN hGuideLinePen = CreatePen(PS_DOT, 2, RGB(255, 100, 180));
        static HPEN hHeadingLinePen = CreatePen(PS_SOLID, 1, RGB(220, 220, 220));
        static HPEN hRingPen = CreatePen(PS_SOLID, 1, RGB(255, 255, 255));
        static HBRUSH hNormalPointBrush = CreateSolidBrush(RGB(255, 180, 60));
        static HBRUSH hBranchPointBrush = CreateSolidBrush(RGB(80, 180, 255));
        static HBRUSH hCurrentPointBrush = CreateSolidBrush(RGB(255, 80, 80));
        static HBRUSH hNextPointBrush = CreateSolidBrush(RGB(255, 235, 120));
        static HBRUSH hNearestPointBrush = CreateSolidBrush(RGB(120, 255, 200));

        RECT rect{};
        GetClientRect(hWnd, &rect);
        HBRUSH hBackgroundBrush = CreateSolidBrush(RGB(0, 0, 0));
        FillRect(hdc, &rect, hBackgroundBrush);
        DeleteObject(hBackgroundBrush);

        SetBkMode(hdc, TRANSPARENT);

        if (drawLines && !overlayLines.empty())
        {
            HGDIOBJ hOldPen = SelectObject(hdc, hNormalLinePen);
            auto drawLineBatch = [&](EOverlayLineStyle style, HPEN pen)
            {
                SelectObject(hdc, pen);
                for (const OverlayLine_t& line : overlayLines)
                {
                    if (line.style != style)
                        continue;
                    MoveToEx(hdc, line.start.x, line.start.y, nullptr);
                    LineTo(hdc, line.end.x, line.end.y);
                }
            };

            drawLineBatch(EOverlayLineStyle::Normal, hNormalLinePen);
            drawLineBatch(EOverlayLineStyle::Branch, hBranchLinePen);
            drawLineBatch(EOverlayLineStyle::ActivePath, hActiveLinePen);
            drawLineBatch(EOverlayLineStyle::HeadingHint, hHeadingLinePen);
            drawLineBatch(EOverlayLineStyle::PlayerGuide, hGuideLinePen);
            SelectObject(hdc, hOldPen);
        }

        if (drawPoints && !overlayPoints.empty())
        {
            SetTextColor(hdc, RGB(255, 255, 255));
            HGDIOBJ hOldBrush = SelectObject(hdc, hNormalPointBrush);
            HGDIOBJ hOldPen = SelectObject(hdc, hRingPen);

            for (const OverlayPoint_t& overlayPoint : overlayPoints)
            {
                const POINT& pt = overlayPoint.point;
                const bool isMultiBranch = overlayPoint.waypointType == EWaypointType::MultiBranch;
                HBRUSH pointBrush = isMultiBranch ? hBranchPointBrush : hNormalPointBrush;
                if (highlightCurrent && overlayPoint.isCurrent)
                    pointBrush = hCurrentPointBrush;
                else if (highlightCurrent && overlayPoint.isNext)
                    pointBrush = hNextPointBrush;
                else if (highlightNearest && overlayPoint.isNearest)
                    pointBrush = hNearestPointBrush;

                SelectObject(hdc, pointBrush);
                const int radius = (overlayPoint.isCurrent || overlayPoint.isNext) ? 6 : 4;
                Ellipse(hdc, pt.x - radius, pt.y - radius, pt.x + radius, pt.y + radius);

                if (drawPointRings)
                {
                    SelectObject(hdc, GetStockObject(HOLLOW_BRUSH));
                    const int ringRadius = radius + 4;
                    Ellipse(hdc, pt.x - ringRadius, pt.y - ringRadius, pt.x + ringRadius, pt.y + ringRadius);
                    SelectObject(hdc, pointBrush);
                }

                if (drawPointIndices || drawPointLabels)
                {
                    std::string label;
                    if (drawPointIndices)
                    {
                        label = "#" + std::to_string(overlayPoint.waypointIndex);
                    }
                    if (drawPointLabels)
                    {
                        if (!label.empty())
                            label += " ";
                        if (overlayPoint.isCurrent)
                            label += "[CUR]";
                        else if (overlayPoint.isNext)
                            label += "[NEXT]";
                        else if (overlayPoint.isNearest)
                            label += "[NEAR]";
                    }
                    if (!label.empty())
                        TextOutA(hdc, pt.x + 8, pt.y - 10, label.c_str(), static_cast<int>(label.length()));
                }
            }

            SelectObject(hdc, hOldBrush);
            SelectObject(hdc, hOldPen);
        }

        EndPaint(hWnd, &ps);
        return 0;
    }
    }

    return DefWindowProcW(hWnd, msg, wParam, lParam);
}
