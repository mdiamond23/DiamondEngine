#include "Scene/Scene.h"
#include "UUIDGenerator.h"
#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_decompose.hpp>

// ---- entity lifetime --------------------------------------------------------

entt::entity Scene::CreateEntity(std::string_view name)
{
    entt::entity entity = m_Registry.create();
    m_Registry.emplace<IDComponent>(entity, GenerateUUID());
    m_Registry.emplace<TransformComponent>(entity);
    m_Registry.emplace<HierarchyComponent>(entity);   // root by default
    m_EntityNames[entity] = name;
    m_TransformSystem.OnEntityCreated(entity);
    return entity;
}

void Scene::DestroyEntity(entt::entity entity)
{
    if (!m_Registry.valid(entity)) return;

    // Detach from parent first so the parent's children list stays clean.
    UnsetParent(entity);

    // Recursively destroy children (copy list — DestroyEntity mutates it).
    if (m_Registry.all_of<HierarchyComponent>(entity)) {
        auto children = m_Registry.get<HierarchyComponent>(entity).children;
        for (entt::entity child : children)
            DestroyEntity(child);
    }

    m_TransformSystem.OnEntityDestroyed(entity);
    m_EntityNames.erase(entity);
    m_Registry.destroy(entity);
}

void Scene::Clear()
{
    m_Registry.clear();
    m_EntityNames.clear();
    m_TransformSystem.Clear();
}

// ---- names ------------------------------------------------------------------

const std::string& Scene::GetEntityName(entt::entity entity) const
{
    return m_EntityNames.at(entity);
}

void Scene::SetEntityName(entt::entity entity, std::string_view name)
{
    m_EntityNames[entity] = name;
}

// ---- hierarchy --------------------------------------------------------------

void Scene::SetParent(entt::entity child, entt::entity parent)
{
    if (!m_Registry.valid(child) || !m_Registry.valid(parent)) return;
    if (child == parent) return;

    // Prevent cycles: parent must not already be inside child's subtree.
    if (IsAncestorOf(child, parent)) return;

    // Detach from current parent.
    UnsetParent(child);

    auto& childHC  = m_Registry.get<HierarchyComponent>(child);
    auto& parentHC = m_Registry.get<HierarchyComponent>(parent);

    childHC.parent = parent;
    parentHC.children.push_back(child);

    m_TransformSystem.MarkDirty();
}

void Scene::UnsetParent(entt::entity child)
{
    if (!m_Registry.valid(child)) return;
    if (!m_Registry.all_of<HierarchyComponent>(child)) return;

    auto& childHC = m_Registry.get<HierarchyComponent>(child);
    if (childHC.parent == entt::null) return;

    // Bake the current world transform into local so the entity stays in place.
    if (m_Registry.all_of<TransformComponent>(child)) {
        glm::mat4 worldMat = m_TransformSystem.GetWorldMatrix(child);
        auto& tc = m_Registry.get<TransformComponent>(child);

        glm::vec3 pos, scale, skew;
        glm::vec4 persp;
        glm::quat rot;
        glm::decompose(worldMat, scale, rot, pos, skew, persp);

        tc.position     = pos;
        tc.rotation     = rot;
        tc.eulerDegrees = glm::degrees(glm::eulerAngles(rot));
        tc.scale        = scale;
    }

    if (m_Registry.valid(childHC.parent) &&
        m_Registry.all_of<HierarchyComponent>(childHC.parent))
    {
        auto& siblings = m_Registry.get<HierarchyComponent>(childHC.parent).children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), child), siblings.end());
    }

    childHC.parent = entt::null;
    m_TransformSystem.MarkDirty();
}

bool Scene::IsAncestorOf(entt::entity entity, entt::entity potentialDescendant) const
{
    if (!m_Registry.all_of<HierarchyComponent>(entity)) return false;
    for (entt::entity child : m_Registry.get<HierarchyComponent>(entity).children) {
        if (child == potentialDescendant) return true;
        if (IsAncestorOf(child, potentialDescendant)) return true;
    }
    return false;
}

// ---- play mode --------------------------------------------------------------

void Scene::StartPlay()
{
    if (m_Playing) return;
    m_Playing = true;
    m_Systems = SystemRegistry::Get().CreateSystems();
    for (auto& system : m_Systems)
        system->OnStart(*this);
}

void Scene::StopPlay()
{
    if (!m_Playing) return;
    for (auto& system : m_Systems)
        system->OnDestroy(*this);
    m_Systems.clear();
    m_Playing = false;
}

void Scene::UpdateSystems(float dt)
{
    if (!m_Playing) return;
    for (auto& system : m_Systems)
        system->OnUpdate(*this, dt);
}
