#pragma once
#include <entt/entt.hpp>
#include <unordered_map>
#include <string>
#include <string_view>
#include "Components.h"
#include "TransformSystem.h"

class Scene
{
public:
    // Creates an entity with TransformComponent, IDComponent, and HierarchyComponent.
    entt::entity CreateEntity(std::string_view name);

    // Removes the entity, detaches it from its parent, and recursively destroys children.
    void DestroyEntity(entt::entity entity);

    // Destroys all entities and clears scene state.
    void Clear();

    // Display name stored in the name map.
    const std::string& GetEntityName(entt::entity entity) const;
    void               SetEntityName(entt::entity entity, std::string_view name);

    // ---- Hierarchy ---------------------------------------------------------
    // Always use these instead of mutating HierarchyComponent directly.
    // Both keep the TransformSystem dirty flag and the bidirectional links in sync.
    void SetParent(entt::entity child, entt::entity parent);
    void UnsetParent(entt::entity child);   // makes child a scene root

    // Cycle guard: returns true if `potentialDescendant` lives inside `entity`'s subtree.
    bool IsAncestorOf(entt::entity entity, entt::entity potentialDescendant) const;

    // ---- Accessors ---------------------------------------------------------
    TransformSystem&       GetTransformSystem()       { return m_TransformSystem; }
    const TransformSystem& GetTransformSystem() const { return m_TransformSystem; }

    entt::registry&       GetRegistry()       { return m_Registry; }
    const entt::registry& GetRegistry() const { return m_Registry; }

    const std::unordered_map<entt::entity, std::string>& GetEntityNames() const { return m_EntityNames; }

private:
    entt::registry   m_Registry;
    TransformSystem  m_TransformSystem;
    std::unordered_map<entt::entity, std::string> m_EntityNames;
};
