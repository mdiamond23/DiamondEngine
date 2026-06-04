#pragma once
#include <glm/glm.hpp>
#include <entt/entt.hpp>
#include <vector>
#include <unordered_map>

// Topologically-sorted flat-array world-transform cache.
//
// Layout invariant: for every index i, m_ParentIndex[i] < i.
// This lets the world-transform pass be a single forward loop with no
// tree traversal — parent world matrices are always already computed.
//
// Hot path (every frame, Update):
//   Pass 1 — copy local matrices from TransformComponents  (scattered read, sequential write)
//   Pass 2 — compute world matrices linearly               (sequential read+write)
//
// Rebuild (only when hierarchy structure changes):
//   DFS pre-order from each root, assigning indices that preserve the invariant.
class TransformSystem
{
public:
    // Call once per frame before reading any world matrices.
    void Update(entt::registry& reg);

    // O(1) lookup. Returns identity for unknown entities.
    const glm::mat4& GetWorldMatrix(entt::entity e) const;

    // Scene calls these on structural changes.
    void MarkDirty()                      { m_Dirty = true; }
    void OnEntityCreated(entt::entity e)  { (void)e; m_Dirty = true; }
    void OnEntityDestroyed(entt::entity e);
    void Clear();

private:
    void    Rebuild(entt::registry& reg);
    void    DFS(entt::entity e, int32_t parentIdx, entt::registry& reg);

    // Flat parallel arrays — all indexed the same way.
    std::vector<glm::mat4>    m_Local;        // local TRS matrix
    std::vector<glm::mat4>    m_World;        // computed world matrix
    std::vector<int32_t>      m_ParentIndex;  // array index of parent, -1 = root
    std::vector<entt::entity> m_Entities;     // entity handle at index i

    std::unordered_map<entt::entity, int32_t> m_EntityToIndex;

    bool m_Dirty = true;
};
