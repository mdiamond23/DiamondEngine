#include "SceneSerializer.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <unordered_map>

#include "Scene/Components.h"
#include "Assets/ModelImporter.h"
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

// Loads a texture once and returns the cached result on subsequent calls.
static std::shared_ptr<Texture> LoadCached(
    const std::string& path,
    std::unordered_map<std::string, std::shared_ptr<Texture>>& cache)
{
    if (path.empty()) return nullptr;
    auto [it, inserted] = cache.emplace(path, nullptr);
    if (inserted) it->second = Texture::Create(path, false);
    return it->second;
}

// Loads all sub-meshes for a file once and caches them.
static const std::vector<MeshData>& LoadMeshCached(
    const std::string& path,
    std::unordered_map<std::string, std::vector<MeshData>>& cache)
{
    auto [it, inserted] = cache.emplace(path, std::vector<MeshData>{});
    if (inserted) it->second = ModelImporter::Load(path);
    return it->second;
}

// ---- Save -------------------------------------------------------------------

void SceneSerializer::Save(Scene& scene, const std::string& path)
{
    json entities = json::array();

    for (auto& [entity, name] : scene.GetEntityNames()) {
        auto& reg = scene.GetRegistry();
        json ej;
        ej["name"] = name;

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

        entities.push_back(ej);
    }

    std::ofstream f(path);
    f << json{ { "entities", entities } }.dump(2);
}

// ---- Load -------------------------------------------------------------------

bool SceneSerializer::Load(Scene& scene, const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;

    json root;
    try { root = json::parse(f); }
    catch (...) { return false; }

    scene.Clear();

    std::unordered_map<std::string, std::shared_ptr<Texture>>  texCache;
    std::unordered_map<std::string, std::vector<MeshData>>     meshCache;

    for (const auto& ej : root.at("entities")) {
        std::string name = ej.value("name", "Entity");
        entt::entity e   = scene.CreateEntity(name);
        auto& reg        = scene.GetRegistry();

        // Transform
        if (ej.contains("transform")) {
            auto& tc = reg.get<TransformComponent>(e);
            auto& tj = ej["transform"];
            tc.position = ToVec3(tj["position"]);
            tc.rotation = ToQuat(tj["rotation"]);
            tc.scale    = ToVec3(tj["scale"]);
            // Restore the editor Euler cache; fall back to deriving from quat for
            // older scene files that don't have the eulerDegrees field.
            if (tj.contains("eulerDegrees"))
                tc.eulerDegrees = ToVec3(tj["eulerDegrees"]);
            else
                tc.eulerDegrees = glm::degrees(glm::eulerAngles(tc.rotation));
        }

        // Mesh
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
            mat->AlbedoPath    = mj.value("albedoPath",    "");
            mat->NormalPath    = mj.value("normalPath",    "");
            mat->MetallicPath  = mj.value("metallicPath",  "");
            mat->RoughnessPath = mj.value("roughnessPath", "");
            mat->AOPath        = mj.value("aoPath",        "");
            mat->EmissivePath  = mj.value("emissivePath",  "");
            mat->EmissiveStrength = mj.value("emissiveStrength", 0.0f);
            mat->UVScale          = mj.value("uvScale", 1.0f);

            mat->Albedo    = LoadCached(mat->AlbedoPath,    texCache);
            mat->Normal    = LoadCached(mat->NormalPath,    texCache);
            mat->Metallic  = LoadCached(mat->MetallicPath,  texCache);
            mat->Roughness = LoadCached(mat->RoughnessPath, texCache);
            mat->AO        = LoadCached(mat->AOPath,        texCache);
            mat->Emissive  = LoadCached(mat->EmissivePath,  texCache);

            auto& mc           = reg.emplace<MeshComponent>(e, mesh, mat, bounds);
            mc.meshPath        = mpath;
            mc.meshSubIndex    = subIdx;
            mc.materialPath    = mj.value("materialPath", "");
            mc.visible         = mj.value("visible",        true);
            mc.castsShadow     = mj.value("castsShadow",    true);
            mc.receivesShadow  = mj.value("receivesShadow", true);
        }

        // Light
        if (ej.contains("light")) {
            const auto& lj = ej["light"];
            auto& lc     = reg.emplace<LightComponent>(e);
            lc.type           = (LightType)lj.value("type", 1);
            lc.color          = ToVec3(lj["color"]);
            lc.intensity      = lj.value("intensity",      100.0f);
            lc.radius         = lj.value("radius",         10.0f);
            lc.innerConeAngle = lj.value("innerConeAngle", 15.0f);
            lc.outerConeAngle = lj.value("outerConeAngle", 30.0f);
        }
    }

    return true;
}
