#pragma once
#include <cstdint>

// Game Signon States
enum SignonState_t : int
{
    SIGNONSTATE_NONE = 0,
    SIGNONSTATE_CHALLENGE = 1,
    SIGNONSTATE_CONNECTED = 2,
    SIGNONSTATE_NEW = 3,
    SIGNONSTATE_PRESPAWN = 4,
    SIGNONSTATE_SPAWN = 5,
    SIGNONSTATE_FULL = 6,
    SIGNONSTATE_CHANGELEVEL = 7
};

// Global game state
struct GameState_t
{
    bool m_bIsChangingLevel = false;
    bool m_bIsInGame = false;
    bool m_bIsConnected = false;
    SignonState_t m_nCurrentSignonState = SIGNONSTATE_NONE;
};
