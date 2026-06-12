#include "SceneSerializer.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <unordered_map>

#include "Scene/Components.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/Collision.h"
#include "Scene/Physics/Rigidbody.h"
#include "Assets/ModelImporter.h"
#include "PhysicsMaterialAsset.h"
#include "Assets/ImageLoader.h"
#include "Renderer/MeshData.h"
#include "Renderer/TextureData.h"

using json = nlohmann::json;
using namespace Diamond;

// ---- helpers ----------------------------------------------------------------

static json JVec3(const glm::vec3& v) { return { v.x, v.y, v.z }; }
static json JQuat(const glm::quat& q) { return { q.w, q.x, q.y, q.z }; }
static glm::vec3 ToVec3(const json& j) { return { j[0], j[1], j[2] }; }
static glm::quat ToQuat(const json& j) { return glm::quat(float(j[0]), float(j[1]), float(j[2]), float(j[3])); }

static std::shared_ptr<Texture> LoadCached(
    const std::string& path,
    std::unordered_map<std::string, std::shared_ptr<Texture>>& cache)
{
    if (path.empty()) return nullptr;
    auto [it, inserted] = cache.emplace(path, nullptr);
    if (inserted) it->second = Texture::Create(path, false);
    return it->second;
}

static const std::vector<MeshData>& LoadMeshCached(
    const std::string& path,
    std::unordered_map<std::string, std::vector<MeshData>>& cache)
{
    auto [it, inserted] = cache.emplace(path, std::vector<MeshData>{});
    if (inserted) it->second = ModelImporter::Load(path);
    return it->second;
}

// ---- shared JSON core -------------------------------------------------------

static json ToJson(Scene& scene)
{
    json entities = json::array();

    for (auto& [entity, name] : scene.GetEntityNames()) {
        auto& reg = scene.GetRegistry();
        json ej;
        ej["name"] = name;

        if (reg.all_of<IDComponent>(entity))
            ej["uuid"] = reg.get<IDComponent>(entity).uuid;

        if (reg.all_of<HierarchyComponent>(entity)) {
            entt::entity parent = reg.get<HierarchyComponent>(entity).parent;
            if (parent != entt::null && reg.all_of<IDComponent>(parent))
                ej["parentUuid"] = reg.get<IDComponent>(parent).uuid;
        }

        if (reg.all_of<TransformComponent>(entity)) {
            auto& tc = reg.get<TransformComponent>(entity);
            ej["transform"] = {
                { "position",     JVec3(tc.position)     },
                { "rotation",     JQuat(tc.rotation)     },
                { "eulerDegrees", JVec3(tc.eulerDegrees) },
                { "scale",        JVec3(tc.scale)        }
            };
        }

        if (reg.all_of<MeshComponent>(entity)) {
            auto& mc = reg.get<MeshComponent>(entity);
            json mj = {
                { "meshPath",        mc.meshPath        },
                { "meshSubIndex",    mc.meshSubIndex    },
                { "materialPath",    mc.materialPath    },
                { "visible",         mc.visible         },
                { "castsShadow",     mc.castsShadow     },
                { "receivesShadow",  mc.receivesShadow  }
            };
            if (mc.material) {
                mj["albedoPath"]       = mc.material->AlbedoPath;
                mj["normalPath"]       = mc.material->NormalPath;
                mj["metallicPath"]     = mc.material->MetallicPath;
                mj["roughnessPath"]    = mc.material->RoughnessPath;
                mj["aoPath"]           = mc.material->AOPath;
                mj["emissivePath"]     = mc.material->EmissivePath;
                mj["emissiveStrength"] = mc.material->EmissiveStrength;
                mj["uvScale"]          = mc.material->UVScale;
            }
            ej["mesh"] = mj;
        }

        if (reg.all_of<LightComponent>(entity)) {
            auto& lc = reg.get<LightComponent>(entity);
            ej["light"] = {
                { "type",           (int)lc.type          },
                { "color",          JVec3(lc.color)       },
                { "intensity",      lc.intensity          },
                { "radius",         lc.radius             },
                { "innerConeAngle", lc.innerConeAngle     },
                { "outerConeAngle", lc.outerConeAngle     }
            };
        }

        if (reg.all_of<CameraComponent>(entity)) {
            auto& cc = reg.get<CameraComponent>(entity);
            ej["camera"] = {
                { "isPrimary", cc.isPrimary },
                { "fov",       cc.fov       },
                { "nearClip",  cc.nearClip  },
                { "farClip",   cc.farClip   }
            };
        }

        if (reg.all_of<ColliderComponent>(entity)) {
            auto& col = reg.get<ColliderComponent>(entity);
            json cj = {
                { "shapeType",     (int)col.shapeType      },
                { "halfExtents",   JVec3(col.halfExtents)  },
                { "radius",        col.radius              },
                { "halfHeight",    col.halfHeight          },
                { "meshPath",      col.meshPath            },
                { "physMatPath",   col.physMatPath         },
                { "localOffset",   JVec3(col.localOffset)  },
                { "localRotation", JQuat(col.localRotation) },
                { "isTrigger",     col.isTrigger           }
            };
            // Inline material block only saved when there is no asset path (legacy / manual)
            if (col.physMatPath.empty() && col.material) {
                cj["material"] = {
                    { "staticFriction",  col.material->staticFriction  },
                    { "dynamicFriction", col.material->dynamicFriction },
                    { "restitution",     col.material->restitution     }
                };
            }
            ej["collider"] = cj;
        }

        if (reg.all_of<RigidBodyComponent>(entity)) {
            auto& rb = reg.get<RigidBodyComponent>(entity);
            ej["rigidbody"] = {
                { "bodyType",       (int)rb.bodyType       },
                { "mass",           rb.mass                },
                { "linearDamping",  rb.linearDamping       },
                { "angularDamping", rb.angularDamping      },
                { "gravityScale",   rb.gravityScale        },
                { "lockRotX",       rb.lockRotX            },
                { "lockRotY",       rb.lockRotY            },
                { "lockRotZ",       rb.lockRotZ            }
            };
        }

        // Script components registered via DECLARE_COMPONENT
        json sc = json::object();
        for (auto& desc : ComponentRegistry::Get().GetAll()) {
            if (desc.serialize && desc.has(scene, entity)) {
                try { sc[desc.name] = json::parse(desc.serialize(scene, entity)); }
                catch (...) {}
            }
        }
        if (!sc.empty())
            ej["scriptComponents"] = sc;

        entities.push_back(ej);
    }

    return json{ { "entities", entities } };
}

static bool FromJson(Scene& scene, const json& root)
{
    scene.Clear();

    std::unordered_map<std::string, std::shared_ptr<Texture>>  texCache;
    std::unordered_map<std::string, std::vector<MeshData>>     meshCache;

    std::unordered_map<uint64_t, entt::entity> uuidToEntity;
    for (const auto& ej : root.at("entities")) {
        std::string  name = ej.value("name", "Entity");
        entt::entity e    = scene.CreateEntity(name);
        if (ej.contains("uuid")) {
            uint64_t uuid = ej["uuid"].get<uint64_t>();
            scene.GetRegistry().get<IDComponent>(e).uuid = uuid;
            uuidToEntity[uuid] = e;
        }
    }

    for (const auto& ej : root.at("entities")) {
        entt::entity e = entt::null;
        if (ej.contains("uuid")) {
            auto it = uuidToEntity.find(ej["uuid"].get<uint64_t>());
            if (it != uuidToEntity.end()) e = it->second;
        }
        if (e == entt::null) continue;

        auto& reg = scene.GetRegistry();

        if (ej.contains("transform")) {
            auto& tc = reg.get<TransformComponent>(e);
            auto& tj = ej["transform"];
            tc.position = ToVec3(tj["position"]);
            tc.rotation = ToQuat(tj["rotation"]);
            tc.scale    = ToVec3(tj["scale"]);
            if (tj.contains("eulerDegrees"))
                tc.eulerDegrees = ToVec3(tj["eulerDegrees"]);
            else
                tc.eulerDegrees = glm::degrees(glm::eulerAngles(tc.rotation));
        }

        if (ej.contains("mesh")) {
            const auto& mj     = ej["mesh"];
            std::string mpath  = mj.value("meshPath", "");
            int         subIdx = mj.value("meshSubIndex", 0);

            std::shared_ptr<Mesh> mesh;
            AABB bounds;

            if (mpath == "__sphere") {
                auto md = MeshData::UVSphere();
                bounds  = md.ComputeAABB();
                mesh    = Mesh::Create(md);
            } else if (mpath == "__cube") {
                auto md = MeshData::UnitCube();
                bounds  = md.ComputeAABB();
                mesh    = Mesh::Create(md);
            } else if (!mpath.empty()) {
                const auto& subMeshes = LoadMeshCached(mpath, meshCache);
                if (subIdx < (int)subMeshes.size()) {
                    bounds = subMeshes[subIdx].ComputeAABB();
                    mesh   = Mesh::Create(subMeshes[subIdx]);
                }
            }

            auto mat = std::make_shared<PBRMaterial>();
            mat->AlbedoPath       = mj.value("albedoPath",       "");
            mat->NormalPath       = mj.value("normalPath",       "");
            mat->MetallicPath     = mj.value("metallicPath",     "");
            mat->RoughnessPath    = mj.value("roughnessPath",    "");
            mat->AOPath           = mj.value("aoPath",           "");
            mat->EmissivePath     = mj.value("emissivePath",     "");
            mat->EmissiveStrength = mj.value("emissiveStrength", 0.0f);
            mat->UVScale          = mj.value("uvScale",          1.0f);

            mat->Albedo    = LoadCached(mat->AlbedoPath,    texCache);
            mat->Normal    = LoadCached(mat->NormalPath,    texCache);
            mat->Metallic  = LoadCached(mat->MetallicPath,  texCache);
            mat->Roughness = LoadCached(mat->RoughnessPath, texCache);
            mat->AO        = LoadCached(mat->AOPath,        texCache);
            mat->Emissive  = LoadCached(mat->EmissivePath,  texCache);

            auto& mc          = reg.emplace<MeshComponent>(e, mesh, mat, bounds);
            mc.meshPath       = mpath;
            mc.meshSubIndex   = subIdx;
            mc.materialPath   = mj.value("materialPath",   "");
            mc.visible        = mj.value("visible",        true);
            mc.castsShadow    = mj.value("castsShadow",    true);
            mc.receivesShadow = mj.value("receivesShadow", true);
        }

        if (ej.contains("light")) {
            const auto& lj = ej["light"];
            auto& lc          = reg.emplace<LightComponent>(e);
            lc.type           = (LightType)lj.value("type", 1);
            lc.color          = ToVec3(lj["color"]);
            lc.intensity      = lj.value("intensity",      100.0f);
            lc.radius         = lj.value("radius",         10.0f);
            lc.innerConeAngle = lj.value("innerConeAngle", 15.0f);
            lc.outerConeAngle = lj.value("outerConeAngle", 30.0f);
        }

        if (ej.contains("camera")) {
            const auto& cj = ej["camera"];
            auto& cc    = reg.emplace<CameraComponent>(e);
            cc.isPrimary = cj.value("isPrimary", true);
            cc.fov       = cj.value("fov",       60.0f);
            cc.nearClip  = cj.value("nearClip",  0.1f);
            cc.farClip   = cj.value("farClip",   1000.0f);
        }

        if (ej.contains("collider")) {
            const auto& cj = ej["collider"];
            auto& col = reg.emplace<ColliderComponent>(e);
            col.shapeType  = (CollisionShape)cj.value("shapeType", 0);
            col.radius     = cj.value("radius",    0.5f);
            col.halfHeight = cj.value("halfHeight", 0.5f);
            col.meshPath   = cj.value("meshPath",   "");
            col.physMatPath = cj.value("physMatPath", "");
            col.isTrigger  = cj.value("isTrigger",  false);
            if (cj.contains("halfExtents"))   col.halfExtents   = ToVec3(cj["halfExtents"]);
            if (cj.contains("localOffset"))   col.localOffset   = ToVec3(cj["localOffset"]);
            if (cj.contains("localRotation")) col.localRotation = ToQuat(cj["localRotation"]);
            if (!col.physMatPath.empty()) {
                col.physMatPath = NormalizePhysMatPath(col.physMatPath);
                col.material = LoadPhysicsMaterial(col.physMatPath);
            } else if (cj.contains("material")) {
                auto mat = std::make_shared<PhysicsMaterial>();
                mat->staticFriction  = cj["material"].value("staticFriction",  0.5f);
                mat->dynamicFriction = cj["material"].value("dynamicFriction", 0.5f);
                mat->restitution     = cj["material"].value("restitution",     0.3f);
                col.material = mat;
            }
        }

        if (ej.contains("rigidbody")) {
            const auto& rj = ej["rigidbody"];
            auto& rb = reg.emplace<RigidBodyComponent>(e);
            rb.bodyType       = (BodyType)rj.value("bodyType",       0);
            rb.mass           = rj.value("mass",           1.0f);
            rb.linearDamping  = rj.value("linearDamping",  0.05f);
            rb.angularDamping = rj.value("angularDamping", 0.05f);
            rb.gravityScale   = rj.value("gravityScale",   1.0f);
            rb.lockRotX       = rj.value("lockRotX",       false);
            rb.lockRotY       = rj.value("lockRotY",       false);
            rb.lockRotZ       = rj.value("lockRotZ",       false);
        }

        // Script components registered via DECLARE_COMPONENT
        if (ej.contains("scriptComponents")) {
            const auto& sc = ej["scriptComponents"];
            for (auto& desc : ComponentRegistry::Get().GetAll()) {
                if (desc.add && desc.deserialize && sc.contains(desc.name)) {
                    if (!desc.has(scene, e))
                        desc.add(scene, e);
                    try { desc.deserialize(scene, e, sc[desc.name].dump()); }
                    catch (...) {}
                }
            }
        }

        if (ej.contains("parentUuid")) {
            uint64_t parentUuid = ej["parentUuid"].get<uint64_t>();
            auto it = uuidToEntity.find(parentUuid);
            if (it != uuidToEntity.end())
                scene.SetParent(e, it->second);
        }
    }

    return true;
}

// ---- public API -------------------------------------------------------------

void SceneSerializer::Save(Scene& scene, const std::string& path)
{
    std::ofstream f(path);
    f << ToJson(scene).dump(2);
}

bool SceneSerializer::Load(Scene& scene, const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;
    json root;
    try { root = json::parse(f); }
    catch (...) { return false; }
    return FromJson(scene, root);
}

std::string SceneSerializer::Stringify(Scene& scene)
{
    return ToJson(scene).dump();
}

bool SceneSerializer::FromString(Scene& scene, const std::string& data)
{
    json root;
    try { root = json::parse(data); }
    catch (...) { return false; }
    return FromJson(scene, root);
}
