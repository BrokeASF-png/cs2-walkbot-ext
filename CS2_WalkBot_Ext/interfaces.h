#pragma once
#include <cstdint>
#include <cstdio>
#include "math_types.h"
#include "memory.h"
#include "game_types.h"
#include "entities.h"

// Forward declarations
class CNetWorkGameClient;

// Offsets structure
struct COffsets
{
    std::uintptr_t m_uNetworkGameClient = 0;
    std::uintptr_t m_uLocalPlayerController = 0;
    std::uintptr_t m_uEntityList = 0;
    std::uintptr_t m_uEntitySystem = 0;
    std::uintptr_t m_uViewMatrix = 0;
    std::uintptr_t m_uCSGOInput = 0;
};

// Local player structure
struct LocalPlayer_t
{
    CCSPlayerController* m_pController = nullptr;
    C_CSPlayerPawn* m_pPlayerPawn = nullptr;
    Vector m_vecVelocity{};

    bool IsValid() const { return m_pController != nullptr && m_pPlayerPawn != nullptr; }
};

// Network Game Client Interface
class CNetWorkGameClient
{
public:
    MEM_PAD(0x230);
    SignonState_t m_nSignonState;           // 0x0230
};

class CCSGOInput
{
public:
    MEM_PAD(0x688);
    QAngle m_angViewAngle;                  // 0x0688
};

class CGameEntitySystem
{
public:
    MEM_PAD(0x210);
    CEntityIdentity* m_pFirst;              // 0x0210
};

// Alternative: if you want to access members directly via padding
// class CNetWorkGameClientEx
// {
// public:
//     MEM_PAD(0x240)
//     SignonState_t m_nSignonState;
// };

// Global singleton for all interfaces
class CGlobals
{
public:
    CGlobals() = default;
    ~CGlobals() = default;

    // Initialize all offsets via pattern scanning
    void Initialize()
    {
        if (!g_Memory.IsAttached())
        {
            m_Offsets = {};
            m_LocalPlayer = {};
            m_uEntityList = 0;
            m_matViewMatrix = {};
            m_CSGOInput = {};
            m_NetworkGameClient = {};
            m_GameEntitySystem = {};
            return;
        }

        // Scan for CNetworkGameClient
        m_Offsets.m_uNetworkGameClient = g_Memory.PatterScan(
            "engine2.dll",
            "48 89 3D ? ? ? ? FF 87",
            SCAN_RESOLVE_RIP,
            0x3,
            0x7
        );

        if (m_Offsets.m_uNetworkGameClient)
        {
            printf("[+] Found CNetworkGameClient at: 0x%llX\n", m_Offsets.m_uNetworkGameClient);
        }
        else
        {
            printf("[!] Failed to find CNetworkGameClient\n");
        }

        // Scan for LocalPlayerController
        m_Offsets.m_uLocalPlayerController = g_Memory.PatterScan(
            "client.dll",
            "48 8B 05 ? ? ? ? 41 89 BE",
            SCAN_RESOLVE_RIP,
            0x3,
            0x7
        );

        if (m_Offsets.m_uLocalPlayerController)
        {
            printf("[+] Found LocalPlayerController at: 0x%llX\n", m_Offsets.m_uLocalPlayerController);
        }
        else
        {
            printf("[!] Failed to find LocalPlayerController\n");
        }

        m_Offsets.m_uEntityList = g_Memory.PatterScan(
            "client.dll",
            X("48 8B 0D ? ? ? ? 48 89 7C 24 ?? 8B FA C1 EB"),
            SCAN_RESOLVE_RIP,
            0x3,
            0x7
        );

        if (m_Offsets.m_uEntityList)
        {
            printf("[+] Found EntityList at: 0x%llX\n", m_Offsets.m_uEntityList);
        }
        else
        {
            printf("[!] Failed to find EntityList\n");
        }

        m_uEntityList = g_Memory.ReadMemory<std::uintptr_t>(m_Offsets.m_uEntityList);
        if (m_uEntityList)
        {
            printf("[+] Resolved EntityList pointer at: 0x%llX\n", m_uEntityList);
        }
        else
        {
            printf("[!] Failed to resolve EntityList pointer\n");
        }

        m_Offsets.m_uEntitySystem = g_Memory.PatterScan(
            "client.dll",
            X("48 8B 0D ? ? ? ? 8B D3 E8 ? ? ? ? 48 8B F0"),
            SCAN_RESOLVE_RIP,
            0x3,
            0x7
        );

        if (m_Offsets.m_uEntitySystem)
        {
            printf("[+] Found EntitySystem at: 0x%llX\n", m_Offsets.m_uEntitySystem);
        }
        else
        {
            printf("[!] Failed to find EntitySystem\n");
        }

        m_Offsets.m_uViewMatrix = g_Memory.PatterScan(
            "client.dll",
            X("48 8D 0D ? ? ? ? 48 C1 E0 06"),
            SCAN_RESOLVE_RIP,
            0x3,
            0x7
        );

        if (m_Offsets.m_uViewMatrix)
        {
            printf("[+] Found ViewMatrix at: 0x%llX\n", m_Offsets.m_uViewMatrix);
        }
        else
        {
            printf("[!] Failed to find ViewMatrix\n");
        }

        m_Offsets.m_uCSGOInput = g_Memory.PatterScan(
            "client.dll",
            X("48 8B 0D ? ? ? ? 4C 8B C6 8B 10 E8"),
            SCAN_RESOLVE_RIP,
            0x3,
            0x7
        );

        if (m_Offsets.m_uCSGOInput)
        {
            printf("[+] Found CSGOInput at: 0x%llX\n", m_Offsets.m_uCSGOInput);
        }
        else
        {
            printf("[!] Failed to find CSGOInput\n");
        }

        UpdateViewMatrix();
        UpdateInterfaces();

        UpdateLocalPlayer();
    }

    // Update local player pointers
    void UpdateLocalPlayer()
    {
        if (!g_Memory.IsAttached())
        {
            m_LocalPlayer = {};
            m_uEntityList = 0;
            m_GameEntitySystem = {};
            return;
        }

        // Only update when in game
        if (!m_Offsets.m_uLocalPlayerController)
        {
            m_LocalPlayer.m_pController = nullptr;
            m_LocalPlayer.m_pPlayerPawn = nullptr;
            m_LocalPlayer.m_vecVelocity = {};
            return;
        }

        // Read controller pointer
        CCSPlayerController* pController = g_Memory.ReadMemory<CCSPlayerController*>(m_Offsets.m_uLocalPlayerController);

        if (!pController)
        {
            m_LocalPlayer.m_pController = nullptr;
            m_LocalPlayer.m_pPlayerPawn = nullptr;
            m_LocalPlayer.m_vecVelocity = {};
            return;
        }

        m_LocalPlayer.m_pController = pController;

        // Get pawn handle and resolve it
        CHandle<C_CSPlayerPawn> hPawn = pController->GetPawnHandle();
        m_LocalPlayer.m_pPlayerPawn = hPawn.Get();
        m_LocalPlayer.m_vecVelocity = m_LocalPlayer.m_pPlayerPawn
            ? m_LocalPlayer.m_pPlayerPawn->GetAbsVelocity()
            : Vector{};
    }

    void UpdateViewMatrix()
    {
        if (!g_Memory.IsAttached() || !m_Offsets.m_uViewMatrix)
        {
            m_matViewMatrix = {};
            return;
        }

        m_matViewMatrix = g_Memory.ReadMemory<ViewMatrix_t>(m_Offsets.m_uViewMatrix);
    }

    void UpdateInterfaces()
    {
        if (!g_Memory.IsAttached())
        {
            m_CSGOInput = {};
            m_NetworkGameClient = {};
            m_GameEntitySystem = {};
            return;
        }

        if (m_Offsets.m_uCSGOInput)
        {
            const std::uintptr_t uCSGOInput = g_Memory.ReadMemory<std::uintptr_t>(m_Offsets.m_uCSGOInput);
            m_CSGOInput = uCSGOInput ? g_Memory.ReadMemory<CCSGOInput>(uCSGOInput) : CCSGOInput{};
        }
        else
        {
            m_CSGOInput = {};
        }

        if (m_Offsets.m_uNetworkGameClient)
        {
            const std::uintptr_t uNetworkGameClient = g_Memory.ReadMemory<std::uintptr_t>(m_Offsets.m_uNetworkGameClient);
            m_NetworkGameClient = uNetworkGameClient ? g_Memory.ReadMemory<CNetWorkGameClient>(uNetworkGameClient) : CNetWorkGameClient{};
        }
        else
        {
            m_NetworkGameClient = {};
        }

        if (m_Offsets.m_uEntitySystem)
        {
            const std::uintptr_t uEntitySystem = g_Memory.ReadMemory<std::uintptr_t>(m_Offsets.m_uEntitySystem);
            m_GameEntitySystem = uEntitySystem ? g_Memory.ReadMemory<CGameEntitySystem>(uEntitySystem) : CGameEntitySystem{};
        }
        else
        {
            m_GameEntitySystem = {};
        }
    }

    // Get network game client pointer
    std::uintptr_t GetNetworkGameClientPtr()
    {
        if (!g_Memory.IsAttached())
            return 0;

        if (!m_Offsets.m_uNetworkGameClient)
            return 0;

        std::uintptr_t uAddress = g_Memory.ReadMemory<std::uintptr_t>(m_Offsets.m_uNetworkGameClient);
        return uAddress;
    }

    // Get network game client
    CNetWorkGameClient* GetNetworkGameClient()
    {
        std::uintptr_t uPtr = GetNetworkGameClientPtr();
        if (!uPtr)
            return nullptr;

        return reinterpret_cast<CNetWorkGameClient*>(uPtr);
    }

    // Public members
    COffsets m_Offsets;
    std::uintptr_t m_uEntityList = 0;
    ViewMatrix_t m_matViewMatrix{};
    CCSGOInput m_CSGOInput{};
    CNetWorkGameClient m_NetworkGameClient{};
    CGameEntitySystem m_GameEntitySystem{};
    LocalPlayer_t m_LocalPlayer;

private:
};

inline CGlobals g_Globals;
