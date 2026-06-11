#pragma once
#include <entt/entt.hpp>
#include <unordered_map>
#include <string>
#include <string_view>
#include <memory>
#include <vector>
#include "Components.h"
#include "TransformSystem.h"
#include "Scripting.h"

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

    // ---- Component access --------------------------------------------------
    template<typename... Ts>
    auto View() { return m_Registry.view<Ts...>(); }

    template<typename... Ts>
    auto View() const { return m_Registry.view<Ts...>(); }

    template<typename... Ts>
    bool Has(entt::entity entity) const { return m_Registry.all_of<Ts...>(entity); }

    template<typename T>
    T& Get(entt::entity entity)
    {
        if constexpr (std::is_empty_v<T>) { static T t{}; m_Registry.get<T>(entity); return t; }
        else return m_Registry.get<T>(entity);
    }

    template<typename T>
    const T& Get(entt::entity entity) const
    {
        if constexpr (std::is_empty_v<T>) { static T t{}; m_Registry.get<T>(entity); return t; }
        else return m_Registry.get<T>(entity);
    }

    template<typename T, typename... Args>
    T& Add(entt::entity entity, Args&&... args)
    {
        if constexpr (std::is_empty_v<T>) { m_Registry.emplace<T>(entity, std::forward<Args>(args)...); static T t{}; return t; }
        else return m_Registry.emplace<T>(entity, std::forward<Args>(args)...);
    }

    template<typename T>
    void Remove(entt::entity entity) { m_Registry.remove<T>(entity); }

    // ---- Play mode ---------------------------------------------------------
    void StartPlay();
    void StopPlay();
    void UpdateSystems(float dt);
    bool IsPlaying() const { return m_Playing; }

    // ---- Accessors ---------------------------------------------------------
    TransformSystem&       GetTransformSystem()       { return m_TransformSystem; }
    const TransformSystem& GetTransformSystem() const { return m_TransformSystem; }

    entt::registry&       GetRegistry()       { return m_Registry; }
    const entt::registry& GetRegistry() const { return m_Registry; }

    entt::entity GetPrimaryCamera() const;

    const std::unordered_map<entt::entity, std::string>& GetEntityNames() const { return m_EntityNames; }
private:
    entt::registry   m_Registry;
    TransformSystem  m_TransformSystem;
    std::unordered_map<entt::entity, std::string> m_EntityNames;

    std::vector<std::unique_ptr<GameSystem>> m_Systems;
    bool                                     m_Playing = false;
};
