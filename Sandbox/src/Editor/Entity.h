#pragma once
#include <entt/entt.hpp>
#include <string>
#include "Scene/Scene.h"

// Thin entity layer for editor only
class Entity 
{
public:
    Entity() = default;
    Entity(entt::entity handle, Scene* scene) : m_Handle(handle), m_Scene(scene) {}

    operator entt::entity() const { return m_Handle; }

    template<typename T>
    T& Get() { return m_Scene->GetRegistry().get<T>(m_Handle); }

    template<typename T>
    bool Has() const { return m_Scene->GetRegistry().all_of<T>(m_Handle); }

    bool IsValid() const { return m_Handle != entt::null && m_Scene != nullptr; }

    const std::string& GetName() const { return m_Scene->GetEntityName(m_Handle); }

private:
    entt::entity m_Handle = entt::null;
    Scene*       m_Scene  = nullptr;
};