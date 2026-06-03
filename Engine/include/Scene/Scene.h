#pragma once
#include <entt/entt.hpp>
#include <unordered_map>
#include <string>
#include <string_view>
#include "Components.h"

class Scene
{
public:
    // Creates an entity with a TransformComponent and IDComponent attached by default.
    entt::entity CreateEntity(std::string_view name);

    // Removes the entity and its name from the scene entirely.
    void DestroyEntity(entt::entity entity);

    // Destroys all entities and clears the name map.
    void Clear();

    // Returns the display name stored in the name map. Undefined if entity doesn't exist.
    const std::string& GetEntityName(entt::entity entity) const;
    void               SetEntityName(entt::entity entity, std::string_view name);

    // Raw registry access — use this for component queries and system iteration.
    entt::registry&       GetRegistry()       { return m_Registry; }
    const entt::registry& GetRegistry() const { return m_Registry; }

    // Used by HierarchyPanel to list all entities and their display names.
    const std::unordered_map<entt::entity, std::string>& GetEntityNames() const { return m_EntityNames; }

private:
    entt::registry m_Registry;
    std::unordered_map<entt::entity, std::string> m_EntityNames;  // entity -> display name
};