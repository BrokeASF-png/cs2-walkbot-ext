#pragma once

#include <Windows.h>
#include <vector>
#include "walkbot_core.h"

void UpdatePathRunner(
    PathRunnerState_t& pathRunnerState,
    PathManager& pathManager,
    bool hasCurrentPlayerPos,
    const Vector& currentPlayerPos,
    bool hasCurrentPlayerVelocity,
    const Vector& currentPlayerVelocity,
    const QAngle& currentPlayerAngle,
    const std::vector<int>& enemyCandidateWaypointIndices,
    HWND hGameWindow);
