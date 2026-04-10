#pragma once

#include "walkbot_core.h"

struct MainTabActions_t
{
    bool startRunRequested = false;
    bool stopRunRequested = false;
};

MainTabActions_t DrawMainRunnerTab(
    PathRunnerState_t& pathRunnerState,
    const PathManager& pathManager,
    bool hasCurrentPlayerPos,
    const Vector& currentPlayerPos);
