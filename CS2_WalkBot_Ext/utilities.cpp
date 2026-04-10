#include "utilities.h"
#include "interfaces.h"

bool CUtilities::IsChangingLevel()
{
    return m_GameState.m_bIsChangingLevel;
}

bool CUtilities::IsInGame()
{
    return m_GameState.m_bIsInGame;
}

bool CUtilities::IsConnected()
{
    return m_GameState.m_bIsConnected;
}

void CUtilities::UpdateGameState()
{
    if (!g_Memory.IsAttached())
    {
        m_GameState = {};
        return;
    }

    const SignonState_t nSignonState = g_Globals.m_NetworkGameClient.m_nSignonState;
    m_GameState.m_nCurrentSignonState = nSignonState;

    // Update state flags
    m_GameState.m_bIsChangingLevel = (nSignonState == SIGNONSTATE_CHANGELEVEL);
    m_GameState.m_bIsInGame = (nSignonState == SIGNONSTATE_FULL);
    m_GameState.m_bIsConnected = (nSignonState >= SIGNONSTATE_CONNECTED);
}
