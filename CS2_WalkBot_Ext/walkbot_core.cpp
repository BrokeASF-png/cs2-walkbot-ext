#include "walkbot_core.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace
{
    float RandomFloat01()
    {
        return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
    }

    std::size_t HashOverlayInt(std::size_t seed, int value)
    {
        seed ^= static_cast<std::size_t>(value) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
        return seed;
    }
}

const char* GetWaypointTypeLabel(EWaypointType type)
{
    switch (type)
    {
    case EWaypointType::MultiBranch:
        return "Multi-Branch";
    case EWaypointType::Normal:
    default:
        return "Normal";
    }
}

const char* GetBranchModeLabel(EBranchSelectMode branchMode)
{
    switch (branchMode)
    {
    case EBranchSelectMode::Random:
        return "Random";
    case EBranchSelectMode::Cycle:
        return "Cycle";
    case EBranchSelectMode::Sequential:
    default:
        return "Sequential";
    }
}

std::string GetWaypointConnectionsLabel(const PathManager& pathManager, size_t index)
{
    const std::vector<int> nextIndices = pathManager.GetResolvedNextIndices(index);
    if (nextIndices.empty())
        return "None";

    std::string label;
    for (size_t i = 0; i < nextIndices.size(); ++i)
    {
        if (!label.empty())
            label += ", ";

        label += "#";
        label += std::to_string(nextIndices[i]);
    }

    return label;
}

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
    int height)
{
    std::size_t signature = 0;
    signature = HashOverlayInt(signature, drawPoints ? 1 : 0);
    signature = HashOverlayInt(signature, drawPointIndices ? 1 : 0);
    signature = HashOverlayInt(signature, drawLines ? 1 : 0);
    signature = HashOverlayInt(signature, drawPointRings ? 1 : 0);
    signature = HashOverlayInt(signature, drawPointLabels ? 1 : 0);
    signature = HashOverlayInt(signature, drawHeadingHints ? 1 : 0);
    signature = HashOverlayInt(signature, drawPlayerGuide ? 1 : 0);
    signature = HashOverlayInt(signature, highlightCurrent ? 1 : 0);
    signature = HashOverlayInt(signature, highlightNearest ? 1 : 0);
    signature = HashOverlayInt(signature, nearestWaypointIndex);
    signature = HashOverlayInt(signature, currentWaypointIndex);
    signature = HashOverlayInt(signature, nextWaypointIndex);
    signature = HashOverlayInt(signature, width);
    signature = HashOverlayInt(signature, height);
    signature = HashOverlayInt(signature, static_cast<int>(points.size()));
    signature = HashOverlayInt(signature, static_cast<int>(lines.size()));

    for (const OverlayPoint_t& point : points)
    {
        signature = HashOverlayInt(signature, point.point.x);
        signature = HashOverlayInt(signature, point.point.y);
        signature = HashOverlayInt(signature, static_cast<int>(point.waypointType));
        signature = HashOverlayInt(signature, point.waypointIndex);
        signature = HashOverlayInt(signature, point.isCurrent ? 1 : 0);
        signature = HashOverlayInt(signature, point.isNext ? 1 : 0);
        signature = HashOverlayInt(signature, point.isNearest ? 1 : 0);
    }

    for (const OverlayLine_t& line : lines)
    {
        signature = HashOverlayInt(signature, line.start.x);
        signature = HashOverlayInt(signature, line.start.y);
        signature = HashOverlayInt(signature, line.end.x);
        signature = HashOverlayInt(signature, line.end.y);
        signature = HashOverlayInt(signature, static_cast<int>(line.style));
    }

    return signature;
}

float NormalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle < -180.0f)
        angle += 360.0f;

    return angle;
}

QAngle CalculateAngleToTarget(const Vector& from, const Vector& to)
{
    return from.ToAngles(to);
}

int FindNearestWaypointIndex(const PathManager& pathManager, const Vector& position)
{
    const auto& waypoints = pathManager.GetWaypoints();
    if (waypoints.empty())
        return -1;

    size_t nearestIndex = 0;
    float nearestDistance = position.Distance(waypoints[0].pos);

    for (size_t i = 1; i < waypoints.size(); ++i)
    {
        const float distance = position.Distance(waypoints[i].pos);
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestIndex = i;
        }
    }

    return static_cast<int>(nearestIndex);
}

void SendKeyboardKey(WORD vk, bool keyDown)
{
    INPUT input{};
    input.type = INPUT_KEYBOARD;
    input.ki.wVk = vk;
    input.ki.dwFlags = keyDown ? 0 : KEYEVENTF_KEYUP;
    SendInput(1, &input, sizeof(INPUT));
}

void SetForwardKeyState(bool shouldPress, bool& isPressed)
{
    if (shouldPress)
    {
        SendKeyboardKey('W', true);
        isPressed = true;
        return;
    }

    if (!isPressed)
        return;

    SendKeyboardKey('W', false);
    isPressed = false;
}

void SetBackwardKeyState(bool shouldPress, bool& isPressed)
{
    if (shouldPress)
    {
        SendKeyboardKey('S', true);
        isPressed = true;
        return;
    }

    if (!isPressed)
        return;

    SendKeyboardKey('S', false);
    isPressed = false;
}

void SetDuckKeyState(bool shouldPress, bool& isPressed)
{
    if (shouldPress)
    {
        SendKeyboardKey(VK_CONTROL, true);
        isPressed = true;
        return;
    }

    if (!isPressed)
        return;

    SendKeyboardKey(VK_CONTROL, false);
    isPressed = false;
}

void SetWalkKeyState(bool shouldPress, bool& isPressed)
{
    if (shouldPress)
    {
        SendKeyboardKey(VK_SHIFT, true);
        isPressed = true;
        return;
    }

    if (!isPressed)
        return;

    SendKeyboardKey(VK_SHIFT, false);
    isPressed = false;
}

void PulseJumpKey()
{
    SendKeyboardKey(VK_SPACE, true);
    SendKeyboardKey(VK_SPACE, false);
}

void SendRelativeMouseMove(int deltaX, int deltaY)
{
    if (deltaX == 0 && deltaY == 0)
        return;

    const int maxAbsDelta = (std::max)(std::abs(deltaX), std::abs(deltaY));
    const int packetCount = (std::max)(1, (std::min)(8, maxAbsDelta / 6 + 1));

    INPUT packets[8]{};
    int sentDx = 0;
    int sentDy = 0;
    for (int i = 0; i < packetCount; ++i)
    {
        const int remainSteps = packetCount - i;
        const int stepDx = (deltaX - sentDx) / remainSteps;
        const int stepDy = (deltaY - sentDy) / remainSteps;
        sentDx += stepDx;
        sentDy += stepDy;

        packets[i].type = INPUT_MOUSE;
        packets[i].mi.dwFlags = MOUSEEVENTF_MOVE;
        packets[i].mi.dx = stepDx;
        packets[i].mi.dy = stepDy;
    }

    SendInput(static_cast<UINT>(packetCount), packets, sizeof(INPUT));
}

float ClampUnit(float value)
{
    if (value < 0.0f)
        return 0.0f;
    if (value > 1.0f)
        return 1.0f;
    return value;
}

float ComputeControlDeltaSeconds(PathRunnerState_t& runnerState)
{
    static LARGE_INTEGER perfFrequency = {};
    if (perfFrequency.QuadPart == 0)
        QueryPerformanceFrequency(&perfFrequency);

    LARGE_INTEGER nowCounter{};
    QueryPerformanceCounter(&nowCounter);
    if (runnerState.lastControlCounter == 0)
    {
        runnerState.lastControlCounter = nowCounter.QuadPart;
        return runnerState.smoothedDeltaSeconds;
    }

    const double deltaCounts = static_cast<double>(nowCounter.QuadPart - runnerState.lastControlCounter);
    const double freq = static_cast<double>(perfFrequency.QuadPart > 0 ? perfFrequency.QuadPart : 1);
    const float rawDeltaSeconds = static_cast<float>(deltaCounts / freq);
    runnerState.lastControlCounter = nowCounter.QuadPart;

    const float clampedDelta = (std::max)(0.0025f, (std::min)(0.05f, rawDeltaSeconds));
    if (!runnerState.hasSmoothedDelta)
    {
        runnerState.smoothedDeltaSeconds = clampedDelta;
        runnerState.hasSmoothedDelta = true;
    }
    else
    {
        runnerState.smoothedDeltaSeconds += (clampedDelta - runnerState.smoothedDeltaSeconds) * 0.20f;
    }
    return runnerState.smoothedDeltaSeconds;
}

void RandomizeHumanAxisState(PathRunnerState_t::HumanAxisState_t& axisState)
{
    axisState = {};
    axisState.driftPhasePrimary = RandomFloat01() * 6.28318530718f;
    axisState.driftPhaseSecondary = RandomFloat01() * 6.28318530718f;
    axisState.cadenceScale = 0.90f + RandomFloat01() * 0.24f;
    axisState.responseScale = 0.90f + RandomFloat01() * 0.22f;
}

void ResetHumanizedAimState(PathRunnerState_t& runnerState)
{
    runnerState.smoothedDeltaSeconds = 1.0f / 120.0f;
    runnerState.hasSmoothedDelta = false;
    runnerState.lastControlCounter = 0;
    runnerState.hasLockedPitch = false;
    runnerState.lockedPitchDeg = 0.0f;
    runnerState.pitchFloatPhase = 0.0f;
    runnerState.pitchFloatActivity = 0.0f;
    RandomizeHumanAxisState(runnerState.pitchAxis);
    RandomizeHumanAxisState(runnerState.yawAxis);
}

void ResetMovementPredictionState(PathRunnerState_t& runnerState)
{
    runnerState.estimatedVelocity = {};
    runnerState.estimatedAcceleration = {};
    runnerState.lastObservedPosition = {};
    runnerState.hasLastObservedPosition = false;
}

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
    bool directAim)
{
    if (directAim)
    {
        RandomizeHumanAxisState(axisState);
        const float clampedDirect = (std::max)(-8192.0f, (std::min)(8192.0f, rawMouseDelta));
        return static_cast<short>(std::lround(clampedDirect));
    }

    const float safeScale = (std::max)(0.00001f, std::fabs(mouseScaleDegPerCount));
    const float deadzoneCounts = (std::max)(0.0f, deadzoneDeg) / safeScale;
    const float settleZoneCounts = (std::max)(0.0f, settleZoneDeg) / safeScale;
    const float clampedDt = (std::max)(0.002f, (std::min)(0.05f, dt));
    const float absError = std::fabs(rawMouseDelta);
    const float farErrorNorm = (std::min)(1.0f, absError / 150.0f);
    const float closeErrorNorm = 1.0f - farErrorNorm;
    float desiredVelocity = 0.0f;

    if (absError > deadzoneCounts)
    {
        const float responseScale = (std::max)(0.35f, responseGain) * axisState.responseScale;
        const float accelCurve = 0.40f + 0.60f * std::pow(farErrorNorm, 0.70f);
        desiredVelocity = rawMouseDelta * responseScale * accelCurve;

        const float velocityLimit = (std::max)(40.0f, maxSpeed);
        desiredVelocity = (std::max)(-velocityLimit, (std::min)(velocityLimit, desiredVelocity));

        const float brakeLimit =
            std::sqrt((std::max)(0.0f, 2.0f * (std::max)(100.0f, maxAcceleration) * absError))
            * (0.55f + 0.45f * (std::max)(0.1f, brakingStrength));
        desiredVelocity = (std::max)(-brakeLimit, (std::min)(brakeLimit, desiredVelocity));
    }

    const bool directionMismatch =
        (rawMouseDelta > 0.0f && axisState.velocity < 0.0f) ||
        (rawMouseDelta < 0.0f && axisState.velocity > 0.0f);
    if (directionMismatch)
    {
        const float reversalRange = (std::max)(1.0f, reversalWindowCounts);
        const float mismatchFactor = 1.0f - (std::min)(1.0f, absError / reversalRange);
        const float damping = 1.0f - mismatchFactor * (1.0f - ClampUnit(reversalDamping));
        axisState.velocity *= damping;
        desiredVelocity *= damping;
    }

    const float driftStrength =
        ((std::max)(0.0f, tremorStrength) * 0.12f + (std::max)(0.0f, randomKickStrength) * 0.03f)
        * (0.25f + 0.75f * closeErrorNorm);
    const float cadenceBase = (std::max)(0.2f, tremorFrequency);
    axisState.driftPhasePrimary += clampedDt * cadenceBase * axisState.cadenceScale * 6.28318530718f;
    axisState.driftPhaseSecondary += clampedDt * (cadenceBase * 0.43f + 0.7f) * axisState.cadenceScale * 6.28318530718f;
    if (axisState.driftPhasePrimary > 6.28318530718f)
        axisState.driftPhasePrimary -= 6.28318530718f;
    if (axisState.driftPhaseSecondary > 6.28318530718f)
        axisState.driftPhaseSecondary -= 6.28318530718f;

    const float driftWave =
        std::sin(axisState.driftPhasePrimary) * 0.72f +
        std::sin(axisState.driftPhaseSecondary + 1.37f) * 0.28f;
    desiredVelocity += driftWave * driftStrength;

    if (std::fabs(angleDeltaDeg) <= settleZoneDeg && std::fabs(axisState.velocity) <= settleVelocity)
    {
        desiredVelocity *= 0.55f;
    }

    const float accelerationLimit = (std::max)(100.0f, maxAcceleration) * clampedDt;
    const float velocityDelta = desiredVelocity - axisState.velocity;
    axisState.velocity += (std::max)(-accelerationLimit, (std::min)(accelerationLimit, velocityDelta));

    float frameMouseDelta = axisState.velocity * clampedDt;
    if (std::fabs(frameMouseDelta) > absError && absError <= settleZoneCounts * 1.8f)
    {
        frameMouseDelta = rawMouseDelta;
        axisState.velocity *= 0.45f;
    }

    axisState.residual += frameMouseDelta;
    float quantizedDelta = std::round(axisState.residual);
    axisState.residual -= quantizedDelta;

    if (quantizedDelta == 0.0f && absError > deadzoneCounts * 1.8f)
        quantizedDelta = rawMouseDelta > 0.0f ? 1.0f : -1.0f;

    if (absError <= deadzoneCounts && std::fabs(axisState.velocity) < settleVelocity * 0.6f)
    {
        quantizedDelta = 0.0f;
        axisState.residual = 0.0f;
    }

    quantizedDelta = (std::max)(-8192.0f, (std::min)(8192.0f, quantizedDelta));
    return static_cast<short>(quantizedDelta);
}

short ComputeLegacySmoothedMouseDelta(float rawMouseDelta, float smooth, bool directAim)
{
    if (directAim)
    {
        const float clampedDirect = (std::max)(-8192.0f, (std::min)(8192.0f, rawMouseDelta));
        return static_cast<short>(std::lround(clampedDirect));
    }

    const float clampedSmooth = ClampUnit(smooth);
    const float scaledDelta = rawMouseDelta * (1.0f - clampedSmooth);
    float quantizedDelta = std::round(scaledDelta);

    if (quantizedDelta == 0.0f && std::fabs(rawMouseDelta) >= 0.75f)
        quantizedDelta = rawMouseDelta > 0.0f ? 1.0f : -1.0f;

    quantizedDelta = (std::max)(-8192.0f, (std::min)(8192.0f, quantizedDelta));
    return static_cast<short>(quantizedDelta);
}

// WindMouse + Fitts humanized aim.
// Reuses HumanAxisState_t:
//   velocity       -> current velocity in counts/sec
//   residual       -> sub-count accumulator (quantization)
//   acceleration   -> wind noise accumulator (counts/sec^2)
// References: BenLand100 WindMouse, Fitts's Law (T = a + b*log2(D/W + 1))
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
    bool directAim)
{
    if (directAim)
    {
        RandomizeHumanAxisState(axisState);
        const float clampedDirect = (std::max)(-8192.0f, (std::min)(8192.0f, rawMouseDelta));
        return static_cast<short>(std::lround(clampedDirect));
    }

    const float clampedDt = (std::max)(0.002f, (std::min)(0.05f, dt));
    const float dist = rawMouseDelta;
    const float absDist = std::fabs(dist);
    const float sign = dist >= 0.0f ? 1.0f : -1.0f;

    // Deadzone: when essentially on target, bleed velocity and residual.
    if (absDist < 0.35f)
    {
        axisState.velocity *= 0.55f;
        axisState.acceleration *= 0.55f;
        float quiet = std::round(axisState.residual + dist);
        axisState.residual = (axisState.residual + dist) - quiet;
        quiet = (std::max)(-8192.0f, (std::min)(8192.0f, quiet));
        return static_cast<short>(quiet);
    }

    const float gravity = (std::max)(1.0f, windGravity);
    const float force = (std::max)(0.0f, windForce);
    const float maxStep = (std::max)(80.0f, windMaxStep);
    const float damping = (std::max)(1.0f, windDamping);
    const float W = (std::max)(0.5f, fittsTolerance);

    // Fitts's Law target velocity: want to cover remaining distance within mt seconds.
    const float index = std::log2((absDist / W) + 1.0f);
    const float mt = (std::max)(0.012f, fittsA + fittsB * index);
    float targetSpeed = absDist / mt;
    // Clamp cruise speed; close-range taper for smooth arrival.
    const float closeBlend = (std::min)(1.0f, absDist / (damping * 2.0f + 0.0001f));
    targetSpeed *= (0.35f + 0.65f * closeBlend);
    targetSpeed = (std::min)(maxStep, targetSpeed);
    const float targetVelocity = sign * targetSpeed;

    // WindMouse 1D noise accumulator. Decays near the target, keeping the end smooth.
    const float sqrt3 = 1.7320508f;
    const float sqrt5 = 2.2360679f;
    if (absDist >= damping)
    {
        const float noiseStep = force * clampedDt;
        axisState.acceleration = axisState.acceleration / sqrt3
            + (2.0f * RandomFloat01() - 1.0f) * noiseStep / sqrt5;
    }
    else
    {
        axisState.acceleration /= sqrt3;
    }
    const float noiseLimit = maxStep * 0.45f;
    if (axisState.acceleration > noiseLimit) axisState.acceleration = noiseLimit;
    if (axisState.acceleration < -noiseLimit) axisState.acceleration = -noiseLimit;

    // Integrate velocity toward Fitts target with exponential smoothing plus noise.
    const float alpha = 1.0f - std::exp(-gravity * clampedDt);
    axisState.velocity += (targetVelocity - axisState.velocity) * alpha;
    axisState.velocity += axisState.acceleration * clampedDt;

    // Velocity clamp.
    if (axisState.velocity > maxStep) axisState.velocity = maxStep;
    if (axisState.velocity < -maxStep) axisState.velocity = -maxStep;

    // Don't overshoot remaining distance in a single tick.
    const float maxStepThisTick = absDist / clampedDt;
    if (axisState.velocity > maxStepThisTick) axisState.velocity = maxStepThisTick;
    if (axisState.velocity < -maxStepThisTick) axisState.velocity = -maxStepThisTick;

    // Integrate and quantize via residual accumulator.
    const float stepCounts = axisState.velocity * clampedDt;
    axisState.residual += stepCounts;
    float quantized = std::round(axisState.residual);
    axisState.residual -= quantized;

    // Minimum nudge to avoid stalling on small fractional moves.
    if (quantized == 0.0f && absDist >= 0.75f)
    {
        quantized = sign;
        axisState.residual = 0.0f;
    }

    quantized = (std::max)(-8192.0f, (std::min)(8192.0f, quantized));
    return static_cast<short>(quantized);
}

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
    bool directAim)
{
    if (directAim)
    {
        RandomizeHumanAxisState(axisState);
        const float clampedDirect = (std::max)(-8192.0f, (std::min)(8192.0f, rawMouseDelta));
        return static_cast<short>(std::lround(clampedDirect));
    }

    const float clampedDt = (std::max)(0.002f, (std::min)(0.05f, dt));
    const float rawError = rawMouseDelta;
    const float absRawError = std::fabs(rawError);
    const float vMax = (std::max)(120.0f, maxSpeed);
    const float aMax = (std::max)(300.0f, maxAcceleration);
    const float jMax = (std::max)(800.0f, maxJerk);
    const float dzone = (std::max)(0.0f, deadzone);
    const float curve = (std::max)(0.35f, (std::min)(1.35f, errorCurve));
    const float resp = (std::max)(0.5f, response);
    const float damp = (std::max)(0.0f, (std::min)(0.98f, damping));

    const float errorFilterAlpha = (std::min)(1.0f, clampedDt * 26.0f);
    axisState.filteredError += (rawError - axisState.filteredError) * errorFilterAlpha;
    const float error = axisState.filteredError;
    const float absError = std::fabs(error);

    const float prevError = axisState.previousError;
    axisState.previousError = error;
    const bool crossedCenter = (error > 0.0f && prevError < 0.0f) || (error < 0.0f && prevError > 0.0f);
    const float rawDerivative = (error - prevError) / clampedDt;
    const float derivFilterAlpha = (std::min)(1.0f, clampedDt * 14.0f);
    axisState.filteredDerivative += (rawDerivative - axisState.filteredDerivative) * derivFilterAlpha;

    const float errorNorm = (std::min)(1.0f, absError / 220.0f);
    const float shaped = std::pow(errorNorm, (std::max)(0.30f, curve * 0.82f));
    const float derivativeNorm = (std::min)(1.0f, std::fabs(axisState.filteredDerivative) / 280.0f);
    const float baseResp = resp * axisState.responseScale *
        (1.0f + 1.35f * shaped + 0.45f * derivativeNorm);
    const float dynamicVMax = vMax * (0.92f + 1.35f * std::sqrt((std::max)(0.0f, errorNorm)));
    const float minCruise = 18.0f + 20.0f * errorNorm;
    const float desiredCruise = minCruise + (dynamicVMax - minCruise) * shaped;
    const float brakingVelocity = std::sqrt((std::max)(0.0f, 2.0f * aMax * absError));
    float targetSpeed = (std::min)(desiredCruise, brakingVelocity);
    if (absError < dzone * 3.0f)
        targetSpeed *= absError / (dzone * 3.0f + 0.0001f);
    float targetVelocity = error >= 0.0f ? targetSpeed : -targetSpeed;

    // Human-like cadence drift: continuous envelope changes (not random twitching).
    axisState.cadenceScale += ((0.86f + RandomFloat01() * 0.30f) - axisState.cadenceScale) * clampedDt * 0.75f;
    axisState.responseScale += ((0.90f + RandomFloat01() * 0.24f) - axisState.responseScale) * clampedDt * 0.65f;
    axisState.driftPhasePrimary += clampedDt * (6.0f + 3.5f * axisState.cadenceScale) * 6.28318530718f;
    axisState.driftPhaseSecondary += clampedDt * (2.5f + 2.2f * axisState.cadenceScale) * 6.28318530718f;
    if (axisState.driftPhasePrimary > 6.28318530718f)
        axisState.driftPhasePrimary -= 6.28318530718f;
    if (axisState.driftPhaseSecondary > 6.28318530718f)
        axisState.driftPhaseSecondary -= 6.28318530718f;

    const float cadenceWave =
        std::sin(axisState.driftPhasePrimary) * 0.70f +
        std::sin(axisState.driftPhaseSecondary + 1.11f) * 0.30f;
    const float cadenceInfluence = (0.10f + 0.24f * (1.0f - errorNorm));
    targetVelocity *= (1.0f + cadenceWave * cadenceInfluence);

    const float envelopeWave =
        std::sin(axisState.driftPhasePrimary * 0.63f + 0.52f) * 0.62f +
        std::sin(axisState.driftPhaseSecondary * 0.81f + 1.46f) * 0.38f;
    const float envelopeNorm = 0.5f + 0.5f * envelopeWave; // [0,1]
    const float accelEnvelope = 0.78f + 0.46f * envelopeNorm;
    const float dampingEnvelope = 0.86f + 0.22f * (1.0f - envelopeNorm);
    const float dynamicResp = baseResp * (0.88f + 0.30f * envelopeNorm);

    if (crossedCenter && absRawError < 10.0f)
    {
        axisState.velocity *= 0.30f;
        axisState.acceleration *= 0.25f;
        targetVelocity *= 0.35f;
    }

    const float speedAbs = std::fabs(axisState.velocity);
    const float stopDistance = (speedAbs * speedAbs) / (2.0f * (std::max)(200.0f, aMax));
    if (absError < 230.0f && stopDistance > absError * 0.86f)
    {
        targetVelocity *= 0.50f + 0.40f * (std::max)(0.0f, errorNorm);
        if ((axisState.velocity > 0.0f && error > 0.0f) || (axisState.velocity < 0.0f && error < 0.0f))
            targetVelocity -= axisState.velocity * 0.46f;
    }

    const bool signMismatch = (axisState.velocity > 0.0f && error < 0.0f) || (axisState.velocity < 0.0f && error > 0.0f);
    if (signMismatch && absError < (dzone + 3.2f))
    {
        axisState.velocity *= 0.22f;
        axisState.acceleration *= 0.18f;
        targetVelocity *= 0.20f;
    }

    // Coarse acquire phase: only for large errors, and with softer envelope to avoid lock-on feel.
    if (absRawError > (dzone + 150.0f))
    {
        const float farNorm = (std::min)(1.0f, absRawError / 1200.0f);
        const float acquireGain = (0.24f + 0.28f * farNorm) * (0.86f + 0.22f * envelopeNorm);
        const float acquireStepLimit = dynamicVMax * clampedDt * (0.85f + 1.05f * farNorm);
        float coarseDelta = rawError * acquireGain;
        coarseDelta = (std::max)(-acquireStepLimit, (std::min)(acquireStepLimit, coarseDelta));

        const float cadenceScale = 0.92f + 0.14f * envelopeWave;
        coarseDelta *= cadenceScale;

        const float prevVelocity = axisState.velocity;
        axisState.velocity = coarseDelta / clampedDt;
        axisState.acceleration = (axisState.velocity - prevVelocity) / clampedDt;
        axisState.residual += coarseDelta;

        float coarseQuantized = std::round(axisState.residual);
        axisState.residual -= coarseQuantized;
        coarseQuantized = (std::max)(-8192.0f, (std::min)(8192.0f, coarseQuantized));
        return static_cast<short>(coarseQuantized);
    }

    const float desiredAcceleration = (targetVelocity - axisState.velocity) * dynamicResp;
    const float aLimit = aMax * accelEnvelope;
    const float boundedDesiredAcceleration = (std::max)(-aLimit, (std::min)(aLimit, desiredAcceleration));
    const float accelDeltaLimit = jMax * clampedDt;
    const float accelDelta = boundedDesiredAcceleration - axisState.acceleration;
    axisState.acceleration += (std::max)(-accelDeltaLimit, (std::min)(accelDeltaLimit, accelDelta));
    axisState.acceleration *= (1.0f - 0.16f * damp * dampingEnvelope);
    axisState.acceleration = (std::max)(-aLimit, (std::min)(aLimit, axisState.acceleration));

    axisState.velocity += axisState.acceleration * clampedDt;
    axisState.velocity *= (1.0f - 0.045f * damp * dampingEnvelope);
    axisState.velocity = (std::max)(-dynamicVMax, (std::min)(dynamicVMax, axisState.velocity));

    float frameDelta = axisState.velocity * clampedDt;
    if (std::fabs(frameDelta) > absRawError && absRawError < 18.0f)
    {
        frameDelta = rawError;
        axisState.velocity *= 0.50f;
        axisState.acceleration *= 0.40f;
    }

    axisState.residual += frameDelta;
    float quantizedDelta = std::round(axisState.residual);
    axisState.residual -= quantizedDelta;

    const bool nearTarget = absRawError < (dzone + 0.95f);
    if (nearTarget)
    {
        if ((quantizedDelta > 0.0f && rawError < 0.0f) || (quantizedDelta < 0.0f && rawError > 0.0f))
            quantizedDelta = 0.0f;
        if (std::fabs(quantizedDelta) <= 1.0f && absRawError < (dzone + 0.45f))
            quantizedDelta = 0.0f;
        else if (std::fabs(quantizedDelta) == 0.0f && absRawError > (dzone + 0.20f) && RandomFloat01() < 0.18f)
            quantizedDelta = rawError > 0.0f ? 1.0f : -1.0f;
        axisState.residual *= 0.58f;
    }

    if (absRawError < (dzone + 0.35f) &&
        std::fabs(axisState.velocity) < 18.0f &&
        std::fabs(axisState.acceleration) < 120.0f)
    {
        quantizedDelta = 0.0f;
        axisState.residual = 0.0f;
        axisState.velocity *= 0.45f;
        axisState.acceleration *= 0.30f;
    }

    if (quantizedDelta == 0.0f && absRawError > (dzone + 1.6f))
        quantizedDelta = rawError > 0.0f ? 1.0f : -1.0f;

    quantizedDelta = (std::max)(-8192.0f, (std::min)(8192.0f, quantizedDelta));
    return static_cast<short>(quantizedDelta);
}

void StopPathRunner(PathRunnerState_t& runnerState)
{
    SetForwardKeyState(false, runnerState.isForwardPressed);
    SetBackwardKeyState(false, runnerState.isBackwardPressed);
    SetDuckKeyState(false, runnerState.isDuckPressed);
    SetWalkKeyState(false, runnerState.isWalkPressed);
    runnerState.emergencyBrakeRemainingSeconds = 0.0f;
    runnerState.waitRemainingSeconds = 0.0f;
    runnerState.lastProcessedArrivalWaypoint = -1;
    runnerState.isRunning = false;
    runnerState.currentWaypointIndex = -1;
    runnerState.trackedEnemyWaypointIndex = -1;
    runnerState.trackedEnemyPathDistance = -1.0f;
    runnerState.pendingInitialDirectAim = runnerState.directAimOnStart;
    ResetHumanizedAimState(runnerState);
    ResetMovementPredictionState(runnerState);
}

int AddRecordedWaypoint(
    PathManager& pathManager,
    const Vector& pos,
    const QAngle& angle,
    EWaypointType type,
    bool branchRecordModeActive,
    int branchRecordSourceIndex,
    int& branchRecordFirstWaypointIndex)
{
    pathManager.AddWaypoint(pos, angle, type);
    const int newWaypointIndex = static_cast<int>(pathManager.GetWaypointCount()) - 1;

    if (branchRecordModeActive &&
        branchRecordFirstWaypointIndex < 0 &&
        branchRecordSourceIndex >= 0)
    {
        pathManager.AddBranchConnection(
            static_cast<size_t>(branchRecordSourceIndex),
            newWaypointIndex
        );
        branchRecordFirstWaypointIndex = newWaypointIndex;
    }

    return newWaypointIndex;
}

bool WorldToScreen(const Vector& world, ImVec2& screen, const ViewMatrix_t& matrix, const ImVec2& display_size)
{
    const float flWidth =
        matrix[3][0] * world.m_flX +
        matrix[3][1] * world.m_flY +
        matrix[3][2] * world.m_flZ +
        matrix[3][3];

    if (flWidth < 0.001f)
        return false;

    const float flInverse = 1.0f / flWidth;
    screen.x = (
        matrix[0][0] * world.m_flX +
        matrix[0][1] * world.m_flY +
        matrix[0][2] * world.m_flZ +
        matrix[0][3]
    ) * flInverse;
    screen.y = (
        matrix[1][0] * world.m_flX +
        matrix[1][1] * world.m_flY +
        matrix[1][2] * world.m_flZ +
        matrix[1][3]
    ) * flInverse;

    screen.x = (display_size.x * 0.5f) + (screen.x * display_size.x) * 0.5f;
    screen.y = (display_size.y * 0.5f) - (screen.y * display_size.y) * 0.5f;
    return true;
}
