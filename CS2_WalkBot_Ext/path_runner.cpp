#include "path_runner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>

namespace
{
    constexpr float kMinControlDt = 0.002f;
    constexpr float kMaxControlDt = 0.050f;

    float Clamp01(float value)
    {
        return (std::max)(0.0f, (std::min)(1.0f, value));
    }

    float ClampDt(float dt)
    {
        return (std::max)(kMinControlDt, (std::min)(kMaxControlDt, dt));
    }

    float PlanarLength(const Vector& value)
    {
        return std::sqrt(value.m_flX * value.m_flX + value.m_flY * value.m_flY);
    }

    float ComputeForwardSpeed2D(const Vector& velocity, const QAngle& viewAngles)
    {
        const float yawRad = viewAngles.m_flYaw * (3.14159265358979323846f / 180.0f);
        const float forwardX = std::cos(yawRad);
        const float forwardY = std::sin(yawRad);
        return velocity.m_flX * forwardX + velocity.m_flY * forwardY;
    }

    void UpdateMovementVelocityEstimate(
        PathRunnerState_t& pathRunnerState,
        const Vector& currentPlayerPos,
        bool hasCurrentPlayerVelocity,
        const Vector& currentPlayerVelocity,
        float dt)
    {
        if (!currentPlayerPos.IsValid())
            return;

        if (!pathRunnerState.hasLastObservedPosition)
        {
            pathRunnerState.lastObservedPosition = currentPlayerPos;
            pathRunnerState.hasLastObservedPosition = true;
            pathRunnerState.estimatedVelocity = {};
            return;
        }

        const float safeDt = ClampDt(dt);
        const Vector frameVelocity = (currentPlayerPos - pathRunnerState.lastObservedPosition) / safeDt;
        pathRunnerState.lastObservedPosition = currentPlayerPos;

        Vector sampledVelocity = frameVelocity;
        if (hasCurrentPlayerVelocity && currentPlayerVelocity.IsValid())
            sampledVelocity = currentPlayerVelocity;

        const float smoothing = Clamp01(pathRunnerState.movementVelocitySmoothing);
        const Vector prevEstimatedVelocity = pathRunnerState.estimatedVelocity;
        pathRunnerState.estimatedVelocity = pathRunnerState.estimatedVelocity +
            (sampledVelocity - pathRunnerState.estimatedVelocity) * smoothing;

        const Vector sampledAcceleration = (pathRunnerState.estimatedVelocity - prevEstimatedVelocity) / safeDt;
        pathRunnerState.estimatedAcceleration = pathRunnerState.estimatedAcceleration +
            (sampledAcceleration - pathRunnerState.estimatedAcceleration) * smoothing;
    }

    Vector ComputeCompensatedTargetPosition(
        const PathRunnerState_t& pathRunnerState,
        const Vector& currentPlayerPos,
        const Waypoint& waypoint)
    {
        Vector compensated = waypoint.pos;
        if (!pathRunnerState.enableMovementPrediction || !pathRunnerState.hasLastObservedPosition)
            return compensated;

        const float distanceToWaypoint = currentPlayerPos.Distance(waypoint.pos);
        const float lookAhead = (std::max)(0.0f, (std::min)(0.40f, pathRunnerState.movementPredictionLookAheadSeconds));
        const float gain = (std::max)(0.0f, pathRunnerState.movementPredictionGain);
        const float accelGain = (std::max)(0.0f, pathRunnerState.movementPredictionAccelerationGain);
        const float maxLead = (std::max)(0.0f, pathRunnerState.movementPredictionMaxLead);

        const float speed2D = PlanarLength(pathRunnerState.estimatedVelocity);
        const float speedFactor = Clamp01(speed2D / 260.0f);
        const float nearSuppress = Clamp01((distanceToWaypoint - 6.0f) / 30.0f);
        const float effectiveLookAhead = lookAhead * (0.35f + 0.65f * speedFactor) * nearSuppress;

        Vector lead = pathRunnerState.estimatedVelocity * (effectiveLookAhead * gain);
        lead += pathRunnerState.estimatedAcceleration *
            (0.5f * effectiveLookAhead * effectiveLookAhead * accelGain);
        lead.m_flZ = 0.0f;

        const float leadLength = PlanarLength(lead);
        if (leadLength > maxLead && leadLength > 0.001f)
            lead *= maxLead / leadLength;

        compensated += lead;
        return compensated;
    }

    float PlanarDistance(const Vector& a, const Vector& b)
    {
        const float dx = a.m_flX - b.m_flX;
        const float dy = a.m_flY - b.m_flY;
        return std::sqrt(dx * dx + dy * dy);
    }

    struct DijkstraNode_t
    {
        float distance = 0.0f;
        int index = -1;
    };

    struct DijkstraNodeGreater_t
    {
        bool operator()(const DijkstraNode_t& lhs, const DijkstraNode_t& rhs) const
        {
            return lhs.distance > rhs.distance;
        }
    };

    bool BuildShortestPathTreeFrom(
        const PathManager& pathManager,
        int startIndex,
        std::vector<float>& outDistances,
        std::vector<int>& outPrevious)
    {
        const auto& waypoints = pathManager.GetWaypoints();
        const int waypointCount = static_cast<int>(waypoints.size());
        if (waypointCount <= 0 || startIndex < 0 || startIndex >= waypointCount)
            return false;

        outDistances.assign(static_cast<size_t>(waypointCount), std::numeric_limits<float>::infinity());
        outPrevious.assign(static_cast<size_t>(waypointCount), -1);

        std::priority_queue<DijkstraNode_t, std::vector<DijkstraNode_t>, DijkstraNodeGreater_t> queue;
        outDistances[static_cast<size_t>(startIndex)] = 0.0f;
        queue.push({ 0.0f, startIndex });

        while (!queue.empty())
        {
            const DijkstraNode_t cur = queue.top();
            queue.pop();

            if (cur.index < 0 || cur.index >= waypointCount)
                continue;

            if (cur.distance > outDistances[static_cast<size_t>(cur.index)])
                continue;

            const std::vector<int> nextIndices = pathManager.GetResolvedNextIndices(static_cast<size_t>(cur.index));
            for (int nextIndex : nextIndices)
            {
                if (nextIndex < 0 || nextIndex >= waypointCount)
                    continue;

                const float edgeCost = PlanarDistance(
                    waypoints[static_cast<size_t>(cur.index)].pos,
                    waypoints[static_cast<size_t>(nextIndex)].pos
                );
                const float candidateDistance = cur.distance + edgeCost;
                if (candidateDistance >= outDistances[static_cast<size_t>(nextIndex)])
                    continue;

                outDistances[static_cast<size_t>(nextIndex)] = candidateDistance;
                outPrevious[static_cast<size_t>(nextIndex)] = cur.index;
                queue.push({ candidateDistance, nextIndex });
            }
        }

        return true;
    }

    int ResolveNearestReachableCandidateByPath(
        const std::vector<int>& candidateIndices,
        const std::vector<float>& distances,
        float& outDistance)
    {
        int bestCandidate = -1;
        outDistance = -1.0f;

        for (int candidateIndex : candidateIndices)
        {
            if (candidateIndex < 0 || candidateIndex >= static_cast<int>(distances.size()))
                continue;

            const float distance = distances[static_cast<size_t>(candidateIndex)];
            if (!std::isfinite(distance))
                continue;

            if (bestCandidate < 0 || distance < outDistance)
            {
                bestCandidate = candidateIndex;
                outDistance = distance;
            }
        }

        return bestCandidate;
    }

    int ResolveNextHopFromPreviousTree(
        int startIndex,
        int targetIndex,
        const std::vector<int>& previous)
    {
        if (startIndex < 0 || targetIndex < 0 ||
            startIndex >= static_cast<int>(previous.size()) ||
            targetIndex >= static_cast<int>(previous.size()))
        {
            return -1;
        }

        if (startIndex == targetIndex)
            return targetIndex;

        int node = targetIndex;
        int parent = previous[static_cast<size_t>(node)];
        if (parent < 0)
            return -1;

        while (parent >= 0 && parent != startIndex)
        {
            node = parent;
            parent = previous[static_cast<size_t>(node)];
        }

        if (parent != startIndex)
            return -1;

        return node;
    }
}

void UpdatePathRunner(
    PathRunnerState_t& pathRunnerState,
    PathManager& pathManager,
    bool hasCurrentPlayerPos,
    const Vector& currentPlayerPos,
    bool hasCurrentPlayerVelocity,
    const Vector& currentPlayerVelocity,
    const QAngle& currentPlayerAngle,
    const std::vector<int>& enemyCandidateWaypointIndices,
    HWND hGameWindow)
{
    if (!pathRunnerState.isRunning)
    {
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetDuckKeyState(false, pathRunnerState.isDuckPressed);
        SetWalkKeyState(false, pathRunnerState.isWalkPressed);
        return;
    }

    const auto& waypoints = pathManager.GetWaypoints();
    std::vector<float> shortestDistances;
    std::vector<int> shortestPrevious;

    if (!hGameWindow || GetForegroundWindow() != hGameWindow)
    {
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetDuckKeyState(false, pathRunnerState.isDuckPressed);
        SetWalkKeyState(false, pathRunnerState.isWalkPressed);
        pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
        std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status), "Waiting: focus CS2 window");
        return;
    }

    if (!hasCurrentPlayerPos || waypoints.empty())
    {
        StopPathRunner(pathRunnerState);
        std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status), "Stopped: no player position or path");
        return;
    }

    if (pathRunnerState.currentWaypointIndex < 0 ||
        pathRunnerState.currentWaypointIndex >= static_cast<int>(waypoints.size()))
    {
        pathManager.ResetRuntimeState();
        pathRunnerState.currentWaypointIndex = FindNearestWaypointIndex(pathManager, currentPlayerPos);
        pathRunnerState.pendingInitialDirectAim = pathRunnerState.directAimOnStart;
        pathRunnerState.trackedEnemyWaypointIndex = -1;
        pathRunnerState.trackedEnemyPathDistance = -1.0f;
        pathRunnerState.waitRemainingSeconds = 0.0f;
        pathRunnerState.lastProcessedArrivalWaypoint = -1;
        ResetHumanizedAimState(pathRunnerState);
        ResetMovementPredictionState(pathRunnerState);
        std::snprintf(
            pathRunnerState.status,
            sizeof(pathRunnerState.status),
            "Running: start from waypoint #%d",
            pathRunnerState.currentWaypointIndex
        );
    }

    bool resolvedEndOfPath = false;
    bool waitingAtEnemyNearestWaypoint = false;
    while (pathRunnerState.currentWaypointIndex >= 0 &&
        pathRunnerState.currentWaypointIndex < static_cast<int>(waypoints.size()))
    {
        int chaseTargetIndex = -1;
        int chaseNextHopIndex = -1;
        if (pathRunnerState.trackNearestEnemy && !enemyCandidateWaypointIndices.empty())
        {
            if (BuildShortestPathTreeFrom(pathManager, pathRunnerState.currentWaypointIndex, shortestDistances, shortestPrevious))
            {
                float bestEnemyPathDistance = -1.0f;
                chaseTargetIndex = ResolveNearestReachableCandidateByPath(
                    enemyCandidateWaypointIndices,
                    shortestDistances,
                    bestEnemyPathDistance
                );
                if (chaseTargetIndex >= 0)
                {
                    chaseNextHopIndex = ResolveNextHopFromPreviousTree(
                        pathRunnerState.currentWaypointIndex,
                        chaseTargetIndex,
                        shortestPrevious
                    );
                    pathRunnerState.trackedEnemyWaypointIndex = chaseTargetIndex;
                    pathRunnerState.trackedEnemyPathDistance = bestEnemyPathDistance;
                }
                else
                {
                    pathRunnerState.trackedEnemyWaypointIndex = -1;
                    pathRunnerState.trackedEnemyPathDistance = -1.0f;
                }
            }
        }
        else
        {
            pathRunnerState.trackedEnemyWaypointIndex = -1;
            pathRunnerState.trackedEnemyPathDistance = -1.0f;
        }

        const Waypoint& currentWaypoint = waypoints[static_cast<size_t>(pathRunnerState.currentWaypointIndex)];
        const float distanceToWaypoint = currentPlayerPos.Distance(currentWaypoint.pos);
        const float effectiveReachDistance = currentWaypoint.radius > 0.0f
            ? currentWaypoint.radius
            : pathRunnerState.waypointReachDistance;
        if (distanceToWaypoint > effectiveReachDistance)
            break;

        // Arrival handling: fire one-shot jump and arm wait timer exactly once
        // per visit. lastProcessedArrivalWaypoint is cleared when the active
        // waypoint index changes, so re-entering the same waypoint re-arms.
        const int currentIdxForArrival = pathRunnerState.currentWaypointIndex;
        if (pathRunnerState.lastProcessedArrivalWaypoint != currentIdxForArrival)
        {
            pathRunnerState.lastProcessedArrivalWaypoint = currentIdxForArrival;
            if ((currentWaypoint.flags & WPF_JUMP) != 0u)
                PulseJumpKey();
            if (currentWaypoint.waitTime > 0.0f)
                pathRunnerState.waitRemainingSeconds = currentWaypoint.waitTime;
        }

        if (pathRunnerState.waitRemainingSeconds > 0.0f)
            break;

        int nextWaypointIndex = -1;
        if (chaseNextHopIndex >= 0 && chaseNextHopIndex != pathRunnerState.currentWaypointIndex)
        {
            nextWaypointIndex = chaseNextHopIndex;
        }
        else if (chaseTargetIndex >= 0 && chaseTargetIndex == pathRunnerState.currentWaypointIndex)
        {
            if (pathRunnerState.waitAtEnemyNearestWaypoint)
                waitingAtEnemyNearestWaypoint = true;
            break;
        }
        else
        {
            nextWaypointIndex = pathManager.ResolveNextWaypointIndex(
                static_cast<size_t>(pathRunnerState.currentWaypointIndex)
            );
        }

        if (nextWaypointIndex < 0 || nextWaypointIndex == pathRunnerState.currentWaypointIndex)
        {
            resolvedEndOfPath = true;
            break;
        }

        pathRunnerState.currentWaypointIndex = nextWaypointIndex;
        pathRunnerState.lastProcessedArrivalWaypoint = -1;
    }

    if (resolvedEndOfPath ||
        pathRunnerState.currentWaypointIndex < 0 ||
        pathRunnerState.currentWaypointIndex >= static_cast<int>(waypoints.size()))
    {
        StopPathRunner(pathRunnerState);
        std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status), "Finished: reached final waypoint");
        return;
    }

    if (pathRunnerState.waitRemainingSeconds > 0.0f &&
        pathRunnerState.currentWaypointIndex >= 0 &&
        pathRunnerState.currentWaypointIndex < static_cast<int>(waypoints.size()))
    {
        const Waypoint& holdWaypoint = waypoints[static_cast<size_t>(pathRunnerState.currentWaypointIndex)];
        const float crouchDelta = (std::max)(0.0f, pathRunnerState.crouchHeightDelta);
        const bool shouldDuck =
            ((holdWaypoint.flags & WPF_CROUCH) != 0u) ||
            (pathRunnerState.autoCrouchOnLowerTarget &&
             holdWaypoint.pos.m_flZ < (currentPlayerPos.m_flZ - crouchDelta));

        SetDuckKeyState(shouldDuck, pathRunnerState.isDuckPressed);
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetWalkKeyState(false, pathRunnerState.isWalkPressed);
        pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;

        const float dtWait = ComputeControlDeltaSeconds(pathRunnerState);
        pathRunnerState.waitRemainingSeconds =
            (std::max)(0.0f, pathRunnerState.waitRemainingSeconds - dtWait);

        std::snprintf(
            pathRunnerState.status,
            sizeof(pathRunnerState.status),
            "Waiting: pause at waypoint #%d (%.2fs left)",
            pathRunnerState.currentWaypointIndex,
            pathRunnerState.waitRemainingSeconds
        );
        return;
    }

    if (waitingAtEnemyNearestWaypoint)
    {
        const Waypoint& holdWaypoint = waypoints[static_cast<size_t>(pathRunnerState.currentWaypointIndex)];
        const float crouchDelta = (std::max)(0.0f, pathRunnerState.crouchHeightDelta);
        const bool shouldDuck =
            ((holdWaypoint.flags & WPF_CROUCH) != 0u) ||
            (pathRunnerState.autoCrouchOnLowerTarget &&
             holdWaypoint.pos.m_flZ < (currentPlayerPos.m_flZ - crouchDelta));

        SetDuckKeyState(shouldDuck, pathRunnerState.isDuckPressed);
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetWalkKeyState(false, pathRunnerState.isWalkPressed);
        pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
        std::snprintf(
            pathRunnerState.status,
            sizeof(pathRunnerState.status),
            "Waiting: reached enemy-nearest waypoint #%d",
            pathRunnerState.currentWaypointIndex
        );
        return;
    }

    const float dt = ComputeControlDeltaSeconds(pathRunnerState);
    UpdateMovementVelocityEstimate(
        pathRunnerState,
        currentPlayerPos,
        hasCurrentPlayerVelocity,
        currentPlayerVelocity,
        dt
    );

    int activeTargetIndex = pathRunnerState.currentWaypointIndex;
    const Waypoint& currentTargetWaypoint = waypoints[static_cast<size_t>(activeTargetIndex)];
    const float currentTargetDistance = currentPlayerPos.Distance(currentTargetWaypoint.pos);
    const Vector currentTargetAimPos = ComputeCompensatedTargetPosition(pathRunnerState, currentPlayerPos, currentTargetWaypoint);
    const QAngle currentTargetAngle = CalculateAngleToTarget(currentPlayerPos, currentTargetAimPos);
    const float currentTargetYawDelta =
        std::remainderf(currentTargetAngle.m_flYaw - currentPlayerAngle.m_flYaw, 360.0f);
    const float currentTargetYawDiff = std::fabs(currentTargetYawDelta);

    bool skippedToNextByYaw = false;
    bool skippedToNextByAngle = false;
    if (pathRunnerState.skipToNextOnLargeYaw &&
        currentTargetDistance <= (std::max)(0.0f, pathRunnerState.skipTriggerDistance) &&
        std::fabs(currentTargetYawDelta) >= (std::max)(0.0f, pathRunnerState.skipYawThreshold))
    {
        const int nextWaypointIndex = pathManager.PeekNextWaypointIndex(static_cast<size_t>(activeTargetIndex));
        if (nextWaypointIndex >= 0 &&
            nextWaypointIndex != activeTargetIndex &&
            nextWaypointIndex < static_cast<int>(waypoints.size()))
        {
            const Waypoint& nextWaypoint = waypoints[static_cast<size_t>(nextWaypointIndex)];
            const Vector nextAimPos = ComputeCompensatedTargetPosition(pathRunnerState, currentPlayerPos, nextWaypoint);
            const QAngle nextTargetAngle = CalculateAngleToTarget(currentPlayerPos, nextAimPos);
            const float nextPitchDelta =
                std::remainderf(nextTargetAngle.m_flPitch - currentPlayerAngle.m_flPitch, 360.0f);
            const float nextYawDelta =
                std::remainderf(nextTargetAngle.m_flYaw - currentPlayerAngle.m_flYaw, 360.0f);

            const float yawMoveThreshold = (std::max)(0.0f, pathRunnerState.moveYawThreshold);
            const float pitchMoveThreshold = (std::max)(0.0f, pathRunnerState.movePitchThreshold);
            const bool canWalkStraightToNext =
                std::fabs(nextYawDelta) <= yawMoveThreshold &&
                std::fabs(nextPitchDelta) <= pitchMoveThreshold;

            if (canWalkStraightToNext)
            {
                activeTargetIndex = nextWaypointIndex;
                pathRunnerState.currentWaypointIndex = nextWaypointIndex;
                skippedToNextByYaw = true;
            }
        }
    }

    if (!skippedToNextByYaw &&
        pathRunnerState.skipToNextOnAngleDiff &&
        currentTargetYawDiff >= (std::max)(0.0f, pathRunnerState.skipCurrentYawDiffThreshold))
    {
        const int nextWaypointIndex = pathManager.PeekNextWaypointIndex(static_cast<size_t>(activeTargetIndex));
        if (nextWaypointIndex >= 0 &&
            nextWaypointIndex != activeTargetIndex &&
            nextWaypointIndex < static_cast<int>(waypoints.size()))
        {
            const Waypoint& nextWaypoint = waypoints[static_cast<size_t>(nextWaypointIndex)];
            const Vector nextAimPos = ComputeCompensatedTargetPosition(pathRunnerState, currentPlayerPos, nextWaypoint);
            const QAngle nextTargetAngle = CalculateAngleToTarget(currentPlayerPos, nextAimPos);
            const float nextYawDelta =
                std::remainderf(nextTargetAngle.m_flYaw - currentPlayerAngle.m_flYaw, 360.0f);
            const float nextYawDiff = std::fabs(nextYawDelta);

            if (nextYawDiff <= (std::max)(0.0f, pathRunnerState.skipNextYawDiffThreshold))
            {
                activeTargetIndex = nextWaypointIndex;
                pathRunnerState.currentWaypointIndex = nextWaypointIndex;
                skippedToNextByAngle = true;
            }
        }
    }

    const Waypoint& targetWaypoint = waypoints[static_cast<size_t>(activeTargetIndex)];
    const float crouchDelta = (std::max)(0.0f, pathRunnerState.crouchHeightDelta);
    const bool shouldDuckForHeight =
        ((targetWaypoint.flags & WPF_CROUCH) != 0u) ||
        (pathRunnerState.autoCrouchOnLowerTarget &&
         targetWaypoint.pos.m_flZ < (currentPlayerPos.m_flZ - crouchDelta));
    SetDuckKeyState(shouldDuckForHeight, pathRunnerState.isDuckPressed);

    const bool shouldWalkForPacing =
        ((targetWaypoint.flags & WPF_WALK) != 0u) ||
        (targetWaypoint.desiredSpeed > 0.0f && targetWaypoint.desiredSpeed < 0.75f);
    SetWalkKeyState(shouldWalkForPacing, pathRunnerState.isWalkPressed);
    const Vector targetAimPos = ComputeCompensatedTargetPosition(pathRunnerState, currentPlayerPos, targetWaypoint);
    const QAngle targetAngle = CalculateAngleToTarget(currentPlayerPos, targetAimPos);
    const float pitchDelta = std::remainderf(targetAngle.m_flPitch - currentPlayerAngle.m_flPitch, 360.0f);
    const float yawDelta = std::remainderf(targetAngle.m_flYaw - currentPlayerAngle.m_flYaw, 360.0f);
    const float yawDeltaForAim = yawDelta;

    float sensitivity = pathRunnerState.mouseSensitivity;
    if (sensitivity <= 0.0f)
        sensitivity = 1.0f;
    const float speedMul = (std::max)(0.10f, pathRunnerState.aimSpeedMultiplier);

    const float pitchScale = sensitivity * pathRunnerState.mousePitch;
    const float yawScale = sensitivity * pathRunnerState.mouseYaw;
    const bool initialAcquirePhase = pathRunnerState.pendingInitialDirectAim;
    const float initialAcquireBoost = initialAcquirePhase ? 1.45f : 1.0f;
    const float initialAcquireDampingScale = initialAcquirePhase ? 0.82f : 1.0f;
    float pitchDeltaForAim = 0.0f;
    if (pathRunnerState.allowPitchControl)
    {
        pitchDeltaForAim = pitchDelta;
    }
    else if (pathRunnerState.setPitchOnInitialDirectAim)
    {
        if (initialAcquirePhase && !pathRunnerState.hasLockedPitch)
        {
            pathRunnerState.lockedPitchDeg = targetAngle.m_flPitch;
            pathRunnerState.hasLockedPitch = true;
            pitchDeltaForAim = pitchDelta;
        }
        else if (pathRunnerState.hasLockedPitch)
        {
            pitchDeltaForAim = std::remainderf(
                pathRunnerState.lockedPitchDeg - currentPlayerAngle.m_flPitch,
                360.0f
            );
        }

                        const float yawGate = (std::max)(0.0f, pathRunnerState.pitchFloatYawGateDeg);
                        const float yawAbs = std::fabs(yawDeltaForAim);
                        const float safeDt = (std::max)(0.001f, dt);
                        const float yawSpeedDegPerSec = yawAbs / safeDt;
                        const float yawAmpNorm = (std::max)(0.0f, yawAbs - yawGate) / (24.0f + yawGate);
                        const float yawSpeedNorm = yawSpeedDegPerSec / 300.0f;
                        const float activityTarget = (std::min)(1.0f, yawAmpNorm * 0.55f + yawSpeedNorm * 0.45f);
                        const float activityFollowRate = 1.2f + activityTarget * 8.5f;
                        pathRunnerState.pitchFloatActivity +=
                            (activityTarget - pathRunnerState.pitchFloatActivity) *
                            (std::min)(1.0f, safeDt * activityFollowRate);

                        const float floatFreqBase = (std::max)(0.10f, pathRunnerState.pitchFloatFrequency);
                        const float floatFreq =
                            floatFreqBase * (0.25f + 2.35f * pathRunnerState.pitchFloatActivity);
                        pathRunnerState.pitchFloatPhase += safeDt * floatFreq * 6.28318530718f;
                        if (pathRunnerState.pitchFloatPhase > 6.28318530718f)
                            pathRunnerState.pitchFloatPhase -= 6.28318530718f;

                        const float activityCurve = pathRunnerState.pitchFloatActivity * pathRunnerState.pitchFloatActivity;
                        const float floatAmplitude =
                            (std::max)(0.0f, pathRunnerState.pitchFloatAmplitudeDeg) *
                            (0.10f + 1.65f * activityCurve);
                        const float pitchFloat = std::sin(pathRunnerState.pitchFloatPhase) * floatAmplitude;
                        pitchDeltaForAim += pitchFloat;
                    }

    short mouseDeltaX = 0;
    short mouseDeltaY = 0;
    if (pathRunnerState.aimControlMode == PathRunnerState_t::EAimControlMode::LegacySmooth)
    {
        const float rawMouseDeltaX = pitchScale != 0.0f ? (pitchDeltaForAim / pitchScale) : 0.0f;
        const float rawMouseDeltaY = yawScale != 0.0f ? (-yawDeltaForAim / yawScale) : 0.0f;
        const float acquirePitchSmooth = initialAcquirePhase
            ? pathRunnerState.simplePitchSmooth * 0.42f
            : pathRunnerState.simplePitchSmooth;
        const float acquireYawSmooth = initialAcquirePhase
            ? pathRunnerState.simpleYawSmooth * 0.42f
            : pathRunnerState.simpleYawSmooth;

        mouseDeltaX = ComputeLegacySmoothedMouseDelta(
            rawMouseDeltaX,
            acquirePitchSmooth,
            false
        );
        mouseDeltaY = ComputeLegacySmoothedMouseDelta(
            rawMouseDeltaY,
            acquireYawSmooth,
            false
        );
    }
    else if (pathRunnerState.aimControlMode == PathRunnerState_t::EAimControlMode::WindMouse)
    {
        const float rawMouseDeltaX = pitchScale != 0.0f ? (pitchDeltaForAim / pitchScale) : 0.0f;
        const float rawMouseDeltaY = yawScale != 0.0f ? (-yawDeltaForAim / yawScale) : 0.0f;
        const float windMaxStep = (std::max)(80.0f, pathRunnerState.windMaxStep * speedMul * initialAcquireBoost);
        const float windGravity = (std::max)(1.0f, pathRunnerState.windGravity * speedMul);
        mouseDeltaX = ComputeMouseDeltaWindMouse(
            rawMouseDeltaX,
            dt,
            windGravity,
            pathRunnerState.windForce,
            windMaxStep,
            pathRunnerState.windDamping,
            pathRunnerState.windFittsA,
            pathRunnerState.windFittsB,
            pathRunnerState.windFittsTolerance,
            pathRunnerState.pitchAxis,
            false
        );
        mouseDeltaY = ComputeMouseDeltaWindMouse(
            rawMouseDeltaY,
            dt,
            windGravity,
            pathRunnerState.windForce,
            windMaxStep,
            pathRunnerState.windDamping,
            pathRunnerState.windFittsA,
            pathRunnerState.windFittsB,
            pathRunnerState.windFittsTolerance,
            pathRunnerState.yawAxis,
            false
        );
    }
    else
    {
        const float rawMouseDeltaX = pitchScale != 0.0f ? (pitchDeltaForAim / pitchScale) : 0.0f;
        const float rawMouseDeltaY = yawScale != 0.0f ? (-yawDeltaForAim / yawScale) : 0.0f;
        const float servoMaxSpeed = (std::max)(80.0f, pathRunnerState.servoMaxSpeed * speedMul * initialAcquireBoost);
        const float servoMaxAcceleration =
            (std::max)(500.0f, pathRunnerState.servoMaxAcceleration * speedMul * speedMul * initialAcquireBoost);
        const float servoMaxJerk =
            (std::max)(800.0f, pathRunnerState.servoMaxJerk * speedMul * speedMul * initialAcquireBoost);
        const float acquirePitchResponse = pathRunnerState.servoPitchResponse * initialAcquireBoost;
        const float acquireYawResponse = pathRunnerState.servoYawResponse * initialAcquireBoost;
        const float acquirePitchDamping = pathRunnerState.servoPitchDamping * initialAcquireDampingScale;
        const float acquireYawDamping = pathRunnerState.servoYawDamping * initialAcquireDampingScale;

        mouseDeltaX = ComputeMouseDeltaAdaptiveServo(
            rawMouseDeltaX,
            dt,
            acquirePitchResponse,
            acquirePitchDamping,
            pathRunnerState.servoDeadzone,
            pathRunnerState.servoErrorCurve,
            servoMaxSpeed,
            servoMaxAcceleration,
            servoMaxJerk,
            pathRunnerState.pitchAxis,
            false
        );
        mouseDeltaY = ComputeMouseDeltaAdaptiveServo(
            rawMouseDeltaY,
            dt,
            acquireYawResponse,
            acquireYawDamping,
            pathRunnerState.servoDeadzone,
            pathRunnerState.servoErrorCurve,
            servoMaxSpeed,
            servoMaxAcceleration,
            servoMaxJerk,
            pathRunnerState.yawAxis,
            false
        );
    }

    SendRelativeMouseMove(mouseDeltaY, mouseDeltaX);
    pathRunnerState.pendingInitialDirectAim = false;

    const float yawMoveThreshold = (std::max)(0.0f, pathRunnerState.moveYawThreshold);
    const float pitchMoveThreshold = (std::max)(0.0f, pathRunnerState.movePitchThreshold);
    const bool shouldMoveForward =
        std::fabs(yawDeltaForAim) <= yawMoveThreshold &&
        (!pathRunnerState.allowPitchControl || std::fabs(pitchDelta) <= pitchMoveThreshold);

    if (shouldMoveForward)
    {
        pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetForwardKeyState(true, pathRunnerState.isForwardPressed);
    }
    else
    {
        const bool wasForwardPressed = pathRunnerState.isForwardPressed;
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);

        bool shouldBrakeBackward = false;
        if (pathRunnerState.emergencyBrakeOnStop)
        {
            if (wasForwardPressed)
            {
                pathRunnerState.emergencyBrakeRemainingSeconds =
                    (std::max)(0.0f, pathRunnerState.emergencyBrakeMaxSeconds);
            }

            const float forwardSpeed = ComputeForwardSpeed2D(pathRunnerState.estimatedVelocity, currentPlayerAngle);
            const float speedThreshold = (std::max)(0.0f, pathRunnerState.emergencyBrakeSpeedThreshold);
            if (pathRunnerState.emergencyBrakeRemainingSeconds > 0.0f &&
                forwardSpeed > speedThreshold)
            {
                shouldBrakeBackward = true;
                pathRunnerState.emergencyBrakeRemainingSeconds =
                    (std::max)(0.0f, pathRunnerState.emergencyBrakeRemainingSeconds - dt);
            }
            else
            {
                pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
            }
        }
        else
        {
            pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
        }

        SetBackwardKeyState(shouldBrakeBackward, pathRunnerState.isBackwardPressed);
    }

    const float leadDistance = targetAimPos.Distance(targetWaypoint.pos);
    const bool trackingEnemyNow =
        pathRunnerState.trackNearestEnemy &&
        pathRunnerState.trackedEnemyWaypointIndex >= 0 &&
        pathRunnerState.trackedEnemyPathDistance >= 0.0f;
    std::snprintf(
        pathRunnerState.status,
        sizeof(pathRunnerState.status),
        "Running: target #%d dist %.2f yaw %.2f pitch %.2f move %s brake %s lead %.1f%s%s",
        activeTargetIndex,
        currentPlayerPos.Distance(targetWaypoint.pos),
        yawDelta,
        pitchDelta,
        shouldMoveForward ? "ON" : "OFF",
        pathRunnerState.isBackwardPressed ? "ON" : "OFF",
        leadDistance,
        skippedToNextByYaw ? " [SKIP]" :
            (skippedToNextByAngle ? " [ANGLE-SKIP]" : ""),
        trackingEnemyNow ? " [TRACK ENEMY]" : ""
    );
}
