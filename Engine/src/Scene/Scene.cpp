#include "Scene/Scene.h"
#include "UUIDGenerator.h"

entt::entity Scene::CreateEntity(std::string_view name)
{
    entt::entity entity = m_Registry.create();
    m_Registry.emplace<IDComponent>(entity, GenerateUUID());
    m_Registry.emplace<TransformComponent>(entity);
    m_EntityNames[entity] = name;
    return entity;
}

void Scene::DestroyEntity(entt::entity entity)
{
    m_EntityNames.erase(entity);
    m_Registry.destroy(entity);
}

const std::string& Scene::GetEntityName(entt::entity entity) const
{
    return m_EntityNames.at(entity); 
}

void Scene::SetEntityName(entt::entity entity, std::string_view name)
{
    m_EntityNames[entity] = name;
}

