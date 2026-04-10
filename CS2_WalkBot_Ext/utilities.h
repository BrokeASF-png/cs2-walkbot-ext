#pragma once
#include <cstdint>
#include "game_types.h"

class CUtilities
{
public:
    static bool IsChangingLevel();
    static bool IsInGame();
    static bool IsConnected();

    static void UpdateGameState();
    static const GameState_t& GetGameState() { return m_GameState; }

private:
    static GameState_t m_GameState;
};

inline GameState_t CUtilities::m_GameState;
