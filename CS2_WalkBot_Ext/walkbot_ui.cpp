#include "walkbot_ui.h"

#include <cmath>
#include "imgui/imgui.h"

MainTabActions_t DrawMainRunnerTab(
    PathRunnerState_t& pathRunnerState,
    const PathManager& pathManager,
    bool hasCurrentPlayerPos,
    const Vector& currentPlayerPos)
{
    MainTabActions_t actions{};
    static bool showAdvancedOptions = false;

    ImGui::Text("Path Runner");
    ImGui::Separator();
    ImGui::Text("Status: %s", pathRunnerState.status);
    ImGui::Text("Running: %s", pathRunnerState.isRunning ? "YES" : "NO");
    ImGui::Text("Current Waypoint: %d", pathRunnerState.currentWaypointIndex);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::Checkbox("Track Nearest Enemy", &pathRunnerState.trackNearestEnemy);
    ImGui::BeginDisabled(!pathRunnerState.trackNearestEnemy);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::Checkbox("Treat Teammates As Enemies", &pathRunnerState.treatTeammatesAsEnemies);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::Checkbox("Wait At Enemy Nearest Waypoint", &pathRunnerState.waitAtEnemyNearestWaypoint);
    ImGui::EndDisabled();
    if (pathRunnerState.trackNearestEnemy)
    {
        ImGui::Text(
            "Enemy Target: #%d (path %.1f)",
            pathRunnerState.trackedEnemyWaypointIndex,
            pathRunnerState.trackedEnemyPathDistance
        );
    }

    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Reach Distance", &pathRunnerState.waypointReachDistance, 0.1f, 1.0f, 200.0f, "%.1f");
    ImGui::SetNextItemWidth(220.0f);
    ImGui::Checkbox("Direct Aim On Start", &pathRunnerState.directAimOnStart);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Move Yaw Threshold", &pathRunnerState.moveYawThreshold, 0.1f, 0.0f, 180.0f, "%.1f");
    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Move Pitch Threshold", &pathRunnerState.movePitchThreshold, 0.1f, 0.0f, 180.0f, "%.1f");
    ImGui::SetNextItemWidth(220.0f);
    ImGui::Checkbox("Auto Crouch On Lower Target", &pathRunnerState.autoCrouchOnLowerTarget);
    ImGui::BeginDisabled(!pathRunnerState.autoCrouchOnLowerTarget);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Crouch Height Delta", &pathRunnerState.crouchHeightDelta, 0.5f, 0.0f, 80.0f, "%.1f");
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(220.0f);
    ImGui::Checkbox("Emergency Brake On Stop", &pathRunnerState.emergencyBrakeOnStop);
    ImGui::BeginDisabled(!pathRunnerState.emergencyBrakeOnStop);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Brake Speed Threshold", &pathRunnerState.emergencyBrakeSpeedThreshold, 1.0f, 0.0f, 300.0f, "%.0f");
    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Brake Max Seconds", &pathRunnerState.emergencyBrakeMaxSeconds, 0.005f, 0.00f, 0.30f, "%.3f");
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::Checkbox("Allow Pitch Control", &pathRunnerState.allowPitchControl))
    {
        ResetHumanizedAimState(pathRunnerState);
    }

    ImGui::BeginDisabled(pathRunnerState.allowPitchControl);
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::Checkbox("Set Pitch On Initial Direct Aim", &pathRunnerState.setPitchOnInitialDirectAim))
    {
        ResetHumanizedAimState(pathRunnerState);
    }
    ImGui::BeginDisabled(!pathRunnerState.setPitchOnInitialDirectAim);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Pitch Float Amplitude (deg)", &pathRunnerState.pitchFloatAmplitudeDeg, 0.01f, 0.0f, 5.0f, "%.2f");
    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Pitch Float Frequency", &pathRunnerState.pitchFloatFrequency, 0.05f, 0.1f, 8.0f, "%.2f");
    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Pitch Float Yaw Gate", &pathRunnerState.pitchFloatYawGateDeg, 0.1f, 0.0f, 60.0f, "%.1f");
    ImGui::EndDisabled();
    ImGui::EndDisabled();

    ImGui::SetNextItemWidth(220.0f);
    ImGui::DragFloat("Aim Speed Multiplier", &pathRunnerState.aimSpeedMultiplier, 0.01f, 0.10f, 5.00f, "%.2f");
    int aimMode = static_cast<int>(pathRunnerState.aimControlMode);
    const char* aimModeItems[] = { "Legacy Smooth", "Adaptive Servo", "WindMouse + Fitts" };
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::Combo("Aim Control Mode", &aimMode, aimModeItems, IM_ARRAYSIZE(aimModeItems)))
    {
        pathRunnerState.aimControlMode = static_cast<PathRunnerState_t::EAimControlMode>(aimMode);
        ResetHumanizedAimState(pathRunnerState);
    }

    ImGui::Checkbox("Show Advanced Options", &showAdvancedOptions);
    if (showAdvancedOptions)
    {
        ImGui::SetNextItemWidth(220.0f);
        ImGui::Checkbox("Skip To Next On Large Yaw", &pathRunnerState.skipToNextOnLargeYaw);
        ImGui::BeginDisabled(!pathRunnerState.skipToNextOnLargeYaw);
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Skip Trigger Distance", &pathRunnerState.skipTriggerDistance, 0.1f, 0.0f, 500.0f, "%.1f");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Skip Yaw Threshold", &pathRunnerState.skipYawThreshold, 0.5f, 0.0f, 180.0f, "%.1f");
        ImGui::EndDisabled();

        ImGui::SetNextItemWidth(220.0f);
        ImGui::Checkbox("Skip To Next On Angle Diff", &pathRunnerState.skipToNextOnAngleDiff);
        ImGui::BeginDisabled(!pathRunnerState.skipToNextOnAngleDiff);
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Skip Current Yaw Diff", &pathRunnerState.skipCurrentYawDiffThreshold, 0.5f, 0.0f, 180.0f, "%.1f");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Skip Next Yaw Diff", &pathRunnerState.skipNextYawDiffThreshold, 0.5f, 0.0f, 180.0f, "%.1f");
        ImGui::EndDisabled();

        if (pathRunnerState.aimControlMode == PathRunnerState_t::EAimControlMode::LegacySmooth)
        {
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Legacy Yaw Smooth", &pathRunnerState.simpleYawSmooth, 0.01f, 0.0f, 1.0f, "%.2f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Legacy Pitch Smooth", &pathRunnerState.simplePitchSmooth, 0.01f, 0.0f, 1.0f, "%.2f");
        }
        else if (pathRunnerState.aimControlMode == PathRunnerState_t::EAimControlMode::WindMouse)
        {
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Wind Gravity (slew)", &pathRunnerState.windGravity, 0.1f, 1.0f, 60.0f, "%.2f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Wind Force (noise)", &pathRunnerState.windForce, 1.0f, 0.0f, 600.0f, "%.1f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Wind Max Step", &pathRunnerState.windMaxStep, 10.0f, 80.0f, 8000.0f, "%.0f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Wind Damping (near)", &pathRunnerState.windDamping, 0.5f, 1.0f, 120.0f, "%.1f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Fitts a (base sec)", &pathRunnerState.windFittsA, 0.005f, 0.0f, 0.50f, "%.3f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Fitts b (sec/bit)", &pathRunnerState.windFittsB, 0.005f, 0.005f, 0.50f, "%.3f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Fitts W (tolerance)", &pathRunnerState.windFittsTolerance, 0.05f, 0.1f, 20.0f, "%.2f");
        }
        else
        {
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Yaw Response", &pathRunnerState.servoYawResponse, 0.05f, 0.5f, 40.0f, "%.2f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Yaw Damping", &pathRunnerState.servoYawDamping, 0.01f, 0.0f, 0.98f, "%.2f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Pitch Response", &pathRunnerState.servoPitchResponse, 0.05f, 0.5f, 40.0f, "%.2f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Pitch Damping", &pathRunnerState.servoPitchDamping, 0.01f, 0.0f, 0.98f, "%.2f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Deadzone", &pathRunnerState.servoDeadzone, 0.01f, 0.05f, 3.0f, "%.2f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Error Curve", &pathRunnerState.servoErrorCurve, 0.01f, 0.35f, 1.35f, "%.2f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Max Speed", &pathRunnerState.servoMaxSpeed, 10.0f, 80.0f, 6000.0f, "%.0f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Max Accel", &pathRunnerState.servoMaxAcceleration, 50.0f, 500.0f, 80000.0f, "%.0f");
            ImGui::SetNextItemWidth(220.0f);
            ImGui::DragFloat("Servo Max Jerk", &pathRunnerState.servoMaxJerk, 100.0f, 800.0f, 200000.0f, "%.0f");
        }

        ImGui::Separator();
        ImGui::Text("Movement Prediction");
        ImGui::Checkbox("Enable Prediction Compensation", &pathRunnerState.enableMovementPrediction);
        ImGui::BeginDisabled(!pathRunnerState.enableMovementPrediction);
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Prediction Look Ahead (s)", &pathRunnerState.movementPredictionLookAheadSeconds, 0.005f, 0.00f, 0.40f, "%.3f");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Prediction Gain", &pathRunnerState.movementPredictionGain, 0.05f, 0.00f, 3.00f, "%.2f");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Prediction Accel Gain", &pathRunnerState.movementPredictionAccelerationGain, 0.05f, 0.00f, 3.00f, "%.2f");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Prediction Max Lead", &pathRunnerState.movementPredictionMaxLead, 0.5f, 0.0f, 200.0f, "%.1f");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("Velocity Smoothing", &pathRunnerState.movementVelocitySmoothing, 0.01f, 0.01f, 1.00f, "%.2f");
        ImGui::EndDisabled();
        ImGui::Text(
            "Estimated Speed2D: %.1f u/s",
            std::sqrt(
                pathRunnerState.estimatedVelocity.m_flX * pathRunnerState.estimatedVelocity.m_flX +
                pathRunnerState.estimatedVelocity.m_flY * pathRunnerState.estimatedVelocity.m_flY
            )
        );

        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("m_pitch", &pathRunnerState.mousePitch, 0.0001f, 0.0001f, 1.0f, "%.6f");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("m_yaw", &pathRunnerState.mouseYaw, 0.0001f, 0.0001f, 1.0f, "%.6f");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::DragFloat("sensitivity", &pathRunnerState.mouseSensitivity, 0.01f, 0.01f, 100.0f, "%.6f");
    }
    ImGui::Text("Hotkeys: Start(F6) / Stop(F7)");

    if (ImGui::Button("Start Run", ImVec2(120, 0)))
        actions.startRunRequested = true;

    ImGui::SameLine();
    if (ImGui::Button("Stop Run", ImVec2(120, 0)))
        actions.stopRunRequested = true;

    if (hasCurrentPlayerPos)
    {
        const int nearestWaypointIndex = FindNearestWaypointIndex(pathManager, currentPlayerPos);
        ImGui::Text("Nearest Waypoint: %d", nearestWaypointIndex);
    }
    else
    {
        ImGui::Text("Nearest Waypoint: unavailable");
    }

    return actions;
}
