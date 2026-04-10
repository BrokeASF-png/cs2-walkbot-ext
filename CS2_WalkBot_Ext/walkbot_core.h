#pragma once

#include <Windows.h>
#include <cstddef>
#include <string>
#include <vector>
#include "imgui/imgui.h"
#include "interfaces.h"
#include "path.h"

struct OverlayPoint_t
{
    POINT point{};
    EWaypointType waypointType = EWaypointType::Normal;
    int waypointIndex = -1;
    bool isCurrent = false;
    bool isNext = false;
    bool isNearest = false;
};

enum class EOverlayLineStyle : int
{
    Normal = 0,
    Branch = 1,
    ActivePath = 2,
    PlayerGuide = 3,
    HeadingHint = 4
};

struct OverlayLine_t
{
    POINT start{};
    POINT end{};
    EOverlayLineStyle style = EOverlayLineStyle::Normal;
};

struct PathRunnerState_t
{
    enum class EAimControlMode
    {
        LegacySmooth = 0,
        PID = 1,
        WindMouse = 2
    };

    bool isRunning = false;
    bool isForwardPressed = false;
    bool isBackwardPressed = false;
    bool isDuckPressed = false;
    bool isWalkPressed = false;
    float waitRemainingSeconds = 0.0f;
    int lastProcessedArrivalWaypoint = -1;
    bool directAimOnStart = true;
    bool pendingInitialDirectAim = true;
    bool trackNearestEnemy = false;
    bool treatTeammatesAsEnemies = false;
    bool waitAtEnemyNearestWaypoint = true;
    bool autoCrouchOnLowerTarget = true;
    float crouchHeightDelta = 12.0f;
    int trackedEnemyWaypointIndex = -1;
    float trackedEnemyPathDistance = -1.0f;
    EAimControlMode aimControlMode = EAimControlMode::LegacySmooth;
    int currentWaypointIndex = -1;
    float waypointReachDistance = 12.0f;
    float mousePitch = 0.022f;
    float mouseYaw = 0.022f;
    float mouseSensitivity = 1.0f;
    bool allowPitchControl = false;
    bool setPitchOnInitialDirectAim = false;
    float pitchFloatAmplitudeDeg = 0.40f;
    float pitchFloatFrequency = 0.90f;
    float pitchFloatYawGateDeg = 2.0f;
    float pitchFloatPhase = 0.0f;
    float pitchFloatActivity = 0.0f;
    bool hasLockedPitch = false;
    float lockedPitchDeg = 0.0f;
    float aimSpeedMultiplier = 1.80f;
    float simpleYawSmooth = 0.16f;
    float simplePitchSmooth = 0.28f;
    float simpleYawJitter = 0.60f;
    float simplePitchJitter = 0.22f;
    bool skipToNextOnAngleDiff = false;
    float skipCurrentYawDiffThreshold = 55.0f;
    float skipNextYawDiffThreshold = 18.0f;
    bool skipToNextOnLargeYaw = true;
    float skipTriggerDistance = 20.0f;
    float skipYawThreshold = 95.0f;
    float moveYawThreshold = 24.0f;
    float movePitchThreshold = 90.0f;
    float servoYawResponse = 16.5f;
    float servoYawDamping = 0.84f;
    float servoPitchResponse = 12.5f;
    float servoPitchDamping = 0.88f;
    float servoDeadzone = 0.38f;
    float servoErrorCurve = 0.74f;
    float servoMaxSpeed = 2600.0f;
    float servoMaxAcceleration = 36000.0f;
    float servoMaxJerk = 95000.0f;
    float yawResponseGain = 45.0f;
    float pitchResponseGain = 13.0f;
    float yawMaxSpeed = 2300.0f;
    float pitchMaxSpeed = 520.0f;
    float yawMaxAcceleration = 26000.0f;
    float pitchMaxAcceleration = 7600.0f;
    float yawDeadzoneDeg = 0.22f;
    float pitchDeadzoneDeg = 0.18f;
    float yawSettleZoneDeg = 1.4f;
    float pitchSettleZoneDeg = 1.1f;
    float settleVelocity = 78.0f;
    float brakingStrength = 0.84f;
    float reversalDamping = 0.34f;
    float reversalWindowCounts = 22.0f;
    float yawTremor = 14.0f;
    float pitchTremor = 2.8f;
    float tremorFrequency = 8.0f;
    float randomKickChance = 0.85f;
    float randomKickStrength = 20.0f;
    float yawPitchCoupling = 0.08f;
    // WindMouse + Fitts humanized aim parameters
    float windGravity = 9.0f;          // velocity slew rate toward Fitts target (per-second)
    float windForce = 80.0f;           // noise strength (counts/sec^2)
    float windMaxStep = 1600.0f;       // adaptive velocity cap (counts/sec)
    float windDamping = 12.0f;         // distance (counts) below which noise decays
    float windFittsA = 0.080f;         // Fitts a: base latency (seconds)
    float windFittsB = 0.090f;         // Fitts b: cost per bit (sec/bit)
    float windFittsTolerance = 0.8f;   // Fitts W: tolerance (counts)
    float smoothedDeltaSeconds = 1.0f / 120.0f;
    bool  hasSmoothedDelta = false;
    LONGLONG lastControlCounter = 0;
    bool enableMovementPrediction = true;
    float movementPredictionLookAheadSeconds = 0.12f;
    float movementPredictionGain = 1.00f;
    float movementPredictionAccelerationGain = 0.55f;
    float movementPredictionMaxLead = 28.0f;
    float movementVelocitySmoothing = 0.22f;
    bool emergencyBrakeOnStop = true;
    float emergencyBrakeSpeedThreshold = 20.0f;
    float emergencyBrakeMaxSeconds = 0.14f;
    float emergencyBrakeRemainingSeconds = 0.0f;
    Vector estimatedVelocity{};
    Vector estimatedAcceleration{};
    Vector lastObservedPosition{};
    bool hasLastObservedPosition = false;

    struct HumanAxisState_t
    {
        float velocity = 0.0f;
        float residual = 0.0f;
        float acceleration = 0.0f;
        float integral = 0.0f;
        float previousError = 0.0f;
        float filteredError = 0.0f;
        float filteredDerivative = 0.0f;
        float driftPhasePrimary = 0.0f;
        float driftPhaseSecondary = 0.0f;
        float cadenceScale = 1.0f;
        float responseScale = 1.0f;
    };

    HumanAxisState_t pitchAxis{};
    HumanAxisState_t yawAxis{};
    char status[256] = "Idle";
};

const char* GetWaypointTypeLabel(EWaypointType type);
const char* GetBranchModeLabel(EBranchSelectMode branchMode);
std::string GetWaypointConnectionsLabel(const PathManager& pathManager, size_t index);
std::size_t CalculateOverlayRenderSignature(
    const std::vector<OverlayPoint_t>& points,
    const std::vector<OverlayLine_t>& lines,
    bool drawPoints,
    bool drawPointIndices,
    bool drawLines,
    bool drawPointRings,
    bool drawPointLabels,
    bool drawHeadingHints,
    bool drawPlayerGuide,
    bool highlightCurrent,
    bool highlightNearest,
    int nearestWaypointIndex,
    int currentWaypointIndex,
    int nextWaypointIndex,
    int width,
    int height);
float NormalizeAngle(float angle);
QAngle CalculateAngleToTarget(const Vector& from, const Vector& to);
int FindNearestWaypointIndex(const PathManager& pathManager, const Vector& position);
void SendKeyboardKey(WORD vk, bool keyDown);
void SetForwardKeyState(bool shouldPress, bool& isPressed);
void SetBackwardKeyState(bool shouldPress, bool& isPressed);
void SetDuckKeyState(bool shouldPress, bool& isPressed);
void SetWalkKeyState(bool shouldPress, bool& isPressed);
void PulseJumpKey();
void SendRelativeMouseMove(int deltaX, int deltaY);
float ClampUnit(float value);
float ComputeControlDeltaSeconds(PathRunnerState_t& runnerState);
void RandomizeHumanAxisState(PathRunnerState_t::HumanAxisState_t& axisState);
void ResetHumanizedAimState(PathRunnerState_t& runnerState);
void ResetMovementPredictionState(PathRunnerState_t& runnerState);
short ComputeHumanizedMouseDeltaAdvanced(
    float rawMouseDelta,
    float angleDeltaDeg,
    float mouseScaleDegPerCount,
    float responseGain,
    float maxSpeed,
    float maxAcceleration,
    float deadzoneDeg,
    float settleZoneDeg,
    float settleVelocity,
    float brakingStrength,
    float reversalDamping,
    float reversalWindowCounts,
    float tremorStrength,
    float tremorFrequency,
    float randomKickChance,
    float randomKickStrength,
    float dt,
    PathRunnerState_t::HumanAxisState_t& axisState,
    bool directAim);
short ComputeLegacySmoothedMouseDelta(float rawMouseDelta, float smooth, bool directAim);
short ComputeMouseDeltaWindMouse(
    float rawMouseDelta,
    float dt,
    float windGravity,
    float windForce,
    float windMaxStep,
    float windDamping,
    float fittsA,
    float fittsB,
    float fittsTolerance,
    PathRunnerState_t::HumanAxisState_t& axisState,
    bool directAim);
short ComputeMouseDeltaAdaptiveServo(
    float rawMouseDelta,
    float dt,
    float response,
    float damping,
    float deadzone,
    float errorCurve,
    float maxSpeed,
    float maxAcceleration,
    float maxJerk,
    PathRunnerState_t::HumanAxisState_t& axisState,
    bool directAim);
void StopPathRunner(PathRunnerState_t& runnerState);
int AddRecordedWaypoint(
    PathManager& pathManager,
    const Vector& pos,
    const QAngle& angle,
    EWaypointType type,
    bool branchRecordModeActive,
    int branchRecordSourceIndex,
    int& branchRecordFirstWaypointIndex);
bool WorldToScreen(const Vector& world, ImVec2& screen, const ViewMatrix_t& matrix, const ImVec2& display_size);
