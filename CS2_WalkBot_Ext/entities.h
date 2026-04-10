#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "math_types.h"
#include "sdk/client_dll.hpp"
#include "memory.h"

#define INVALID_EHANDLE_INDEX 0xFFFFFFFF
#define ENT_ENTRY_MASK 0x7FFF
#define NUM_SERIAL_NUM_SHIFT_BITS 15

// Macro for offset-based member access with ReadMemory
#define OFFSET(TYPE, NAME, OFFSET_VAL) \
[[nodiscard]] __forceinline TYPE NAME() const noexcept { \
    return g_Memory.ReadMemory<TYPE>(reinterpret_cast<std::uintptr_t>(this) + OFFSET_VAL); \
};

// Forward declarations
class CEntityIdentity;
class CEntityInstance;
class CGameSceneNode;
class C_BaseEntity;
class C_CSPlayerPawn;
class CCSPlayerController;

// Entity Handle
class CBaseHandle
{
public:
    CBaseHandle() noexcept :
        m_uIndex(INVALID_EHANDLE_INDEX) {}

    CBaseHandle(const std::uint32_t uIndex) noexcept :
        m_uIndex(uIndex) {}

    CBaseHandle(const int nEntry, const int nSerial) noexcept
    {
        m_uIndex = nEntry | (nSerial << NUM_SERIAL_NUM_SHIFT_BITS);
    }

    [[nodiscard]] bool IsValid() const
    {
        return m_uIndex != INVALID_EHANDLE_INDEX;
    }

    [[nodiscard]] int GetEntryIndex() const
    {
        return static_cast<int>(m_uIndex & ENT_ENTRY_MASK);
    }

    [[nodiscard]] int GetSerialNumber() const
    {
        return static_cast<int>(m_uIndex >> NUM_SERIAL_NUM_SHIFT_BITS);
    }

    [[nodiscard]] C_BaseEntity* Get() const;

    [[nodiscard]] bool operator==(const CBaseHandle& other) const
    {
        return m_uIndex == other.m_uIndex;
    }

    [[nodiscard]] bool operator!=(const CBaseHandle& other) const
    {
        return !(*this == other);
    }

    std::uint32_t m_uIndex;
};

// Typed handle
template<typename T>
class CHandle : public CBaseHandle
{
public:
    CHandle() = default;
    CHandle(std::uint32_t uIndex) : CBaseHandle(uIndex) {}

    T* Get() const
    {
        return reinterpret_cast<T*>(CBaseHandle::Get());
    }
};

class CEntityIdentity
{
public:
    OFFSET(std::uint32_t, m_flags, cs2_dumper::schemas::client_dll::CEntityIdentity::m_flags);
    OFFSET(CEntityIdentity*, m_pNext, cs2_dumper::schemas::client_dll::CEntityIdentity::m_pNext);

    // CEntityIdentity::m_pInstance (CEntityInstance*) is located at the first pointer slot.
    OFFSET(CEntityInstance*, m_pInstance, 0x0);

    [[nodiscard]] int GetEntryIndex() const noexcept
    {
        const std::uint32_t uHandle = g_Memory.ReadMemory<std::uint32_t>(reinterpret_cast<std::uintptr_t>(this) + 0x10);
        return static_cast<int>(uHandle & ENT_ENTRY_MASK);
    }

    [[nodiscard]] int GetSerialNumber() const noexcept
    {
        const std::uint32_t uHandle = g_Memory.ReadMemory<std::uint32_t>(reinterpret_cast<std::uintptr_t>(this) + 0x10);
        return static_cast<int>(uHandle >> NUM_SERIAL_NUM_SHIFT_BITS);
    }
};

class CEntityInstance
{
public:
    [[nodiscard]] CBaseHandle GetRefEHandle() const
    {
        CEntityIdentity* pIdentity = m_pEntity();
        if (!pIdentity)
            return CBaseHandle();

        return CBaseHandle(pIdentity->GetEntryIndex(), pIdentity->GetSerialNumber() - (pIdentity->m_flags() & 1));
    }

    [[nodiscard]] std::string GetSchemaName() const
    {
        std::uintptr_t uSchemaNameAddress = g_Memory.ReadMemory(
            reinterpret_cast<std::uintptr_t>(this) + 0x10,
            { 0x8, 0x78, 0x8 }
        );
        if (uSchemaNameAddress == 0U)
            return {};

        std::string strSchemaName = g_Memory.ReadMemoryString(uSchemaNameAddress);
        if (strSchemaName.empty())
            return {};

        return strSchemaName;
    }

    OFFSET(CEntityIdentity*, m_pEntity, 0x10)
};

class CGameSceneNode
{
public:
    OFFSET(Vector, GetAbsOrigin, cs2_dumper::schemas::client_dll::CGameSceneNode::m_vecAbsOrigin);
    OFFSET(QAngle, GetAbsRotation, cs2_dumper::schemas::client_dll::CGameSceneNode::m_angAbsRotation);
};

// Base entity
class C_BaseEntity : public CEntityInstance
{
public:
    [[nodiscard]] static C_BaseEntity* GetBaseEntity(int nIdx) noexcept;

    OFFSET(CGameSceneNode*, GetGameSceneNode, cs2_dumper::schemas::client_dll::C_BaseEntity::m_pGameSceneNode);
    OFFSET(std::uint8_t, GetTeamNum, cs2_dumper::schemas::client_dll::C_BaseEntity::m_iTeamNum);
    OFFSET(Vector, GetAbsVelocity, cs2_dumper::schemas::client_dll::C_BaseEntity::m_vecAbsVelocity);

    [[nodiscard]] Vector GetAbsOrigin() const
    {
        CGameSceneNode* pSceneNode = GetGameSceneNode();
        if (!pSceneNode)
            return {};

        return pSceneNode->GetAbsOrigin();
    }

    [[nodiscard]] QAngle GetAbsRotation() const
    {
        CGameSceneNode* pSceneNode = GetGameSceneNode();
        if (!pSceneNode)
            return {};

        return pSceneNode->GetAbsRotation();
    }
};

// Player pawn (from SDK offset cs2_dumper::schemas::client_dll::C_CSPlayerPawnBase::m_hOriginalController = 0x1648)
class C_CSPlayerPawn : public C_BaseEntity
{
public:
    OFFSET(CHandle<CCSPlayerController>, GetOriginalController, cs2_dumper::schemas::client_dll::C_CSPlayerPawnBase::m_hOriginalController);
};

// Player controller (from SDK offset cs2_dumper::schemas::client_dll::CBasePlayerController::m_hPawn = 0x6C4)
class CCSPlayerController : public C_BaseEntity
{
public:
    OFFSET(CHandle<C_CSPlayerPawn>, GetPawnHandle, cs2_dumper::schemas::client_dll::CBasePlayerController::m_hPawn);

    CHandle<C_CSPlayerPawn> m_hPawn;
};

enum class EEntityType : std::uint8_t
{
    ENTITY_UNKNOWN = 0,
    ENTITY_PLAYER = 1
};

struct EntityObject_t
{
    C_BaseEntity* m_pEntity = nullptr;
    int m_nEntryIndex = -1;
    EEntityType m_Type = EEntityType::ENTITY_UNKNOWN;
};

class EntityList
{
public:
    void UpdateEntities();
    void Clear()
    {
        m_vecEntities.clear();
    }

    [[nodiscard]] const std::vector<EntityObject_t>& GetEntities() const
    {
        return m_vecEntities;
    }

private:
    std::vector<EntityObject_t> m_vecEntities;
};

inline EntityList g_EntityList;
