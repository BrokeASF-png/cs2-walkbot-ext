#include "entities.h"
#include "memory.h"
#include "interfaces.h"

C_BaseEntity* C_BaseEntity::GetBaseEntity(int nIdx) noexcept
{
    const std::uintptr_t uEntityList = g_Globals.m_uEntityList;
    if (!uEntityList)
        return nullptr;

    const std::uintptr_t uListEntry = g_Memory.ReadMemory<std::uintptr_t>(uEntityList + (0x8 * ((nIdx & 0x7FFF) >> 0x9)) + 0x10);
    if (!uListEntry)
        return nullptr;

    return g_Memory.ReadMemory<C_BaseEntity*>(uListEntry + 0x70 * (nIdx & 0x1FF));
}

C_BaseEntity* CBaseHandle::Get() const
{
    if (!IsValid())
        return nullptr;

    C_BaseEntity* pEntity = C_BaseEntity::GetBaseEntity(GetEntryIndex());
    if (!pEntity || pEntity->GetRefEHandle() != *this)
        return nullptr;

    return pEntity;
}

void EntityList::UpdateEntities()
{
    m_vecEntities.clear();
    static const FNV1A_t kPlayerControllerHash = FNV1A::Hash("CCSPlayerController");

    CEntityIdentity* pEntityIdentity = g_Globals.m_GameEntitySystem.m_pFirst;
    if (!pEntityIdentity)
        return;

    // Reasonable upfront reserve to reduce reallocations on full-map scans.
    m_vecEntities.reserve(128);

    for (; pEntityIdentity != nullptr; pEntityIdentity = pEntityIdentity->m_pNext())
    {
        CEntityInstance* pInstance = pEntityIdentity->m_pInstance();
        if (!pInstance)
            continue;

        C_BaseEntity* pBaseEntity = reinterpret_cast<C_BaseEntity*>(pInstance);
        const std::string schemaName = pBaseEntity->GetSchemaName();
        if (schemaName.empty())
            continue;

        const FNV1A_t schemaHash = FNV1A::Hash(schemaName.c_str());
        if (schemaHash == kPlayerControllerHash)
        {
            m_vecEntities.push_back(EntityObject_t{
                pBaseEntity,
                pBaseEntity->GetRefEHandle().GetEntryIndex(),
                EEntityType::ENTITY_PLAYER
            });
        }
    }
}
