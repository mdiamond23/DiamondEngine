#include "Scene/TransformSystem.h"
#include "Scene/Components.h"

// ---- public -----------------------------------------------------------------

void TransformSystem::Update(entt::registry& reg)
{
    if (m_Dirty) {
        Rebuild(reg);
        m_Dirty = false;
    }

    const int32_t count = static_cast<int32_t>(m_Entities.size());

    // Pass 1: copy local matrices from TransformComponents.
    // Reads are scattered (EnTT storage order != topological order) but writes
    // to m_Local are sequential — prefetcher handles the write side cleanly.
    for (int32_t i = 0; i < count; ++i)
        m_Local[i] = reg.get<TransformComponent>(m_Entities[i]).GetLocalMatrix();

    // Pass 2: compute world matrices.
    // Both m_Local and m_World are accessed sequentially.
    // m_World[m_ParentIndex[i]] was written earlier in the same pass (parent < i),
    // so it's almost always in L1/L2 cache.
    for (int32_t i = 0; i < count; ++i) {
        m_World[i] = (m_ParentIndex[i] < 0)
                   ? m_Local[i]
                   : m_World[m_ParentIndex[i]] * m_Local[i];
    }
}

const glm::mat4& TransformSystem::GetWorldMatrix(entt::entity e) const
{
    static const glm::mat4 s_Identity(1.0f);
    auto it = m_EntityToIndex.find(e);
    return (it != m_EntityToIndex.end()) ? m_World[it->second] : s_Identity;
}

void TransformSystem::OnEntityDestroyed(entt::entity e)
{
    m_EntityToIndex.erase(e);
    m_Dirty = true;
}

void TransformSystem::Clear()
{
    m_Local.clear();
    m_World.clear();
    m_ParentIndex.clear();
    m_Entities.clear();
    m_EntityToIndex.clear();
    m_Dirty = true;
}

// ---- private ----------------------------------------------------------------

void TransformSystem::Rebuild(entt::registry& reg)
{
    m_Local.clear();
    m_World.clear();
    m_ParentIndex.clear();
    m_Entities.clear();
    m_EntityToIndex.clear();

    // DFS from every root (HierarchyComponent.parent == entt::null).
    // Entities without a HierarchyComponent are also treated as roots.
    for (auto entity : reg.view<TransformComponent>()) {
        bool isRoot = true;
        if (reg.all_of<HierarchyComponent>(entity))
            isRoot = (reg.get<HierarchyComponent>(entity).parent == entt::null);
        if (isRoot)
            DFS(entity, -1, reg);
    }
}

void TransformSystem::DFS(entt::entity e, int32_t parentIdx, entt::registry& reg)
{
    const int32_t myIdx = static_cast<int32_t>(m_Entities.size());
    m_EntityToIndex[e]  = myIdx;
    m_Entities.push_back(e);
    m_Local.push_back(glm::mat4(1.0f));    // filled by Update pass 1
    m_World.push_back(glm::mat4(1.0f));
    m_ParentIndex.push_back(parentIdx);

    if (reg.all_of<HierarchyComponent>(e)) {
        for (entt::entity child : reg.get<HierarchyComponent>(e).children)
            if (reg.valid(child) && reg.all_of<TransformComponent>(child))
                DFS(child, myIdx, reg);
    }
}
