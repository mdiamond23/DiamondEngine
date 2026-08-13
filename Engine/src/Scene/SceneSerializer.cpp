#include "Scene/SceneSerializer.h"
#include "Assets/AssetPathUtils.h"
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <unordered_map>
#include <unordered_set>

#include "Scene/Components.h"
#include "Scene/ComponentRegistry.h"
#include "Scene/Physics/Collision.h"
#include "Scene/Physics/Rigidbody.h"
#include "Scene/Physics/Constraint.h"
#include "Scene/Physics/RagdollComponent.h"
#include "Animation/IKComponent.h"
#include "Assets/ModelImporter.h"
#include "Assets/GltfImporter.h"
#include "Animation/AnimationComponents.h"
#include "Assets/PhysicsMaterialAsset.h"
#include "Assets/AnimStateMachineAsset.h"
#include "Assets/MaterialAsset.h"
#include "AssetPipeline/AssetRegistry.h"
#include "Assets/RagdollAsset.h"
#include "Assets/ImageLoader.h"
#include "Renderer/MeshData.h"
#include "Renderer/TextureData.h"
#include "Renderer/Font.h"

using json = nlohmann::json;
using namespace Diamond;

// ---- helpers ----------------------------------------------------------------

static json JVec2(const glm::vec2& v) { return { v.x, v.y }; }
static json JVec3(const glm::vec3& v) { return { v.x, v.y, v.z }; }
static json JVec4(const glm::vec4& v) { return { v.x, v.y, v.z, v.w }; }
static json JQuat(const glm::quat& q) { return { q.w, q.x, q.y, q.z }; }
static glm::vec2 ToVec2(const json& j) { return { j[0], j[1] }; }
static glm::vec3 ToVec3(const json& j) { return { j[0], j[1], j[2] }; }
static glm::vec4 ToVec4(const json& j) { return { j[0], j[1], j[2], j[3] }; }
static glm::quat ToQuat(const json& j) { return glm::quat(float(j[0]), float(j[1]), float(j[2]), float(j[3])); }

// ---- prefab path / uuid helpers ----------------------------------------------

// Prefab source paths appear with mixed separators and casing (absolute
// Windows paths written by different call sites) — canonicalize before storing
// or comparing so an instance always finds its file and its siblings.
static std::string NormalizePrefabPath(const std::string& p)
{
    if (p.empty()) return p;
    std::error_code ec;
    auto canon = std::filesystem::weakly_canonical(std::filesystem::path(p), ec);
    return (ec ? std::filesystem::path(p).lexically_normal() : canon).generic_string();
}

static bool SamePrefabPath(const std::string& a, const std::string& b)
{
    std::string na = NormalizePrefabPath(a), nb = NormalizePrefabPath(b);
    if (na.size() != nb.size()) return false;
    for (size_t i = 0; i < na.size(); ++i)
        if (std::tolower((unsigned char)na[i]) != std::tolower((unsigned char)nb[i]))
            return false;
    return true;
}

// Deterministic per-instance UUID for prefab children: the same instance root
// always derives the same id for the same prefab-local entity, so references
// into an instance's interior survive scene reloads (splitmix64 finalizer).
static uint64_t DerivePrefabChildUuid(uint64_t rootUuid, uint64_t localUuid)
{
    uint64_t x = rootUuid ^ (localUuid * 0x9E3779B97F4A7C15ull);
    x ^= x >> 30; x *= 0xBF58476D1CE4E5B9ull;
    x ^= x >> 27; x *= 0x94D049BB133111EBull;
    x ^= x >> 31;
    return x ? x : 1;   // 0 means "none" everywhere uuids are consumed
}

// Rewrites every "targetUuid" found in a serialized entity/component tree
// through `map` (constraint targets, IK chain targets — at any nesting depth,
// so registry script components are covered too). Uuids not in the map are
// references outside the prefab and pass through untouched.
static void RemapTargetUuids(nlohmann::json& j, const std::unordered_map<uint64_t, uint64_t>& map)
{
    if (j.is_object()) {
        for (auto& [k, v] : j.items()) {
            if (k == "targetUuid" && v.is_number_integer()) {
                auto it = map.find(v.get<uint64_t>());
                if (it != map.end()) v = it->second;
            } else {
                RemapTargetUuids(v, map);
            }
        }
    } else if (j.is_array()) {
        for (auto& v : j) RemapTargetUuids(v, map);
    }
}

static std::shared_ptr<Font> LoadFontCached(
    const std::string& path,
    std::unordered_map<std::string, std::shared_ptr<Font>>& cache)
{
    if (path.empty()) return nullptr;
    auto [it, inserted] = cache.emplace(path, nullptr);
    if (inserted) it->second = Font::Create(path, kUIFontBakeHeight);
    return it->second;
}

// ---- shared JSON core -------------------------------------------------------
// Scene and prefab files share the same per-entity schema; the entry points
// below serialize/deserialize one entity so both formats reuse them.

static json SerializeEntity(Scene& scene, entt::entity entity)
{
    auto& reg = scene.GetRegistry();
    json ej;
    ej["name"] = scene.GetEntityName(entity);

    if (reg.all_of<IDComponent>(entity))
        ej["uuid"] = reg.get<IDComponent>(entity).uuid;

    if (reg.all_of<HierarchyComponent>(entity)) {
        entt::entity parent = reg.get<HierarchyComponent>(entity).parent;
        if (parent != entt::null && reg.all_of<IDComponent>(parent))
            ej["parentUuid"] = reg.get<IDComponent>(parent).uuid;
    }

    if (reg.all_of<PrefabInstanceComponent>(entity)) {
        auto& pi = reg.get<PrefabInstanceComponent>(entity);
        if (!pi.sourcePath.empty())
            ej["prefabInstance"] = { { "sourcePath", pi.sourcePath } };
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
            { "visible",            mc.visible            },
            { "castsShadow",        mc.castsShadow        },
            { "staticShadowCaster", mc.staticShadowCaster },
            { "receivesShadow",     mc.receivesShadow     },
            { "transparent",        mc.transparent        }
        };
        // Inline material is only serialized when the mesh has no .mat asset;
        // an asset-backed material lives in its own file, referenced by path.
        if (mc.materialPath.empty() && mc.material) {
            mj["albedoPath"]       = mc.material->AlbedoPath;
            mj["normalPath"]       = mc.material->NormalPath;
            mj["metallicPath"]     = mc.material->MetallicPath;
            mj["roughnessPath"]    = mc.material->RoughnessPath;
            mj["aoPath"]           = mc.material->AOPath;
            mj["emissivePath"]     = mc.material->EmissivePath;
            mj["emissiveStrength"] = mc.material->EmissiveStrength;
            mj["uvScale"]          = mc.material->UVScale;
            mj["baseColorFactor"]  = std::array<float, 4>{ mc.material->BaseColorFactor.x, mc.material->BaseColorFactor.y,
                                                            mc.material->BaseColorFactor.z, mc.material->BaseColorFactor.w };
            mj["alphaMode"]        = AlphaModeToString(mc.material->Mode);
            mj["alphaCutoff"]      = mc.material->AlphaCutoff;
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

    if (reg.all_of<SkyLightComponent>(entity)) {
        auto& sl = reg.get<SkyLightComponent>(entity);
        // environmentPath rides the generic MakePortable/ResolveAll sweep over
        // the whole tree — nothing extra to hook here.
        ej["skylight"] = {
            { "environmentPath", sl.environmentPath },
            { "intensity",       sl.intensity       }
        };
    }

    if (reg.all_of<DDGIVolumeComponent>(entity)) {
        auto& dv = reg.get<DDGIVolumeComponent>(entity);
        ej["ddgiVolume"] = {
            { "extent",            JVec3(dv.extent)     },
            { "probeCountX",       dv.probeCounts.x     },
            { "probeCountY",       dv.probeCounts.y     },
            { "probeCountZ",       dv.probeCounts.z     },
            { "raysPerProbe",      dv.raysPerProbe      },
            { "hysteresis",        dv.hysteresis        },
            { "normalBias",        dv.normalBias        },
            { "viewBias",          dv.viewBias          },
            { "energy",            dv.energy            },
            { "maxRayDistance",    dv.maxRayDistance    },
            { "backfaceThreshold", dv.backfaceThreshold },
            { "showProbes",        dv.showProbes        },
            { "probeRadius",       dv.probeRadius       }
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

    if (reg.all_of<CanvasComponent>(entity)) {
        auto& cv = reg.get<CanvasComponent>(entity);
        ej["canvas"] = {
            { "scaleMode",           (int)cv.scaleMode             },
            { "referenceResolution", JVec2(cv.referenceResolution) },
            { "matchWidthHeight",    cv.matchWidthHeight           },
            { "sortOrder",           cv.sortOrder                  }
        };
    }

    if (reg.all_of<RectTransformComponent>(entity)) {
        auto& rt = reg.get<RectTransformComponent>(entity);
        ej["rectTransform"] = {
            { "anchorMin", JVec2(rt.anchorMin) },
            { "anchorMax", JVec2(rt.anchorMax) },
            { "pivot",     JVec2(rt.pivot)     },
            { "position",  JVec2(rt.position)  },
            { "size",      JVec2(rt.size)      },
            { "zOrder",    rt.zOrder           }
        };
    }

    if (reg.all_of<UIImageComponent>(entity)) {
        auto& im = reg.get<UIImageComponent>(entity);
        ej["uiImage"] = {
            { "texturePath", im.texturePath },
            { "tint",        JVec4(im.tint)  },
            { "uvMin",       JVec2(im.uvMin) },
            { "uvMax",       JVec2(im.uvMax) }
        };
    }

    if (reg.all_of<UITextComponent>(entity)) {
        auto& tx = reg.get<UITextComponent>(entity);
        ej["uiText"] = {
            { "text",        tx.text         },
            { "fontPath",    tx.fontPath     },
            { "sizePx",      tx.sizePx       },
            { "color",       JVec4(tx.color) },
            { "hAlign",      (int)tx.hAlign  },
            { "vAlign",      (int)tx.vAlign  },
            { "lineSpacing", tx.lineSpacing  },
            { "wrap",        tx.wrap         },
            { "underline",   tx.underline    }
        };
    }

    if (reg.all_of<UIProgressBarComponent>(entity)) {
        auto& pb = reg.get<UIProgressBarComponent>(entity);
        ej["uiProgressBar"] = {
            { "progress",        pb.progress                },
            { "backgroundColor", JVec4(pb.backgroundColor)  },
            { "fillColor",       JVec4(pb.fillColor)        },
            { "fillTexturePath", pb.fillTexturePath         },
            { "direction",       (int)pb.direction          }
        };
    }

    if (reg.all_of<UIButtonComponent>(entity)) {
        auto& bt = reg.get<UIButtonComponent>(entity);
        // onClick is behavior, not scene data — assigned from scripts, never saved.
        ej["uiButton"] = {
            { "normalTint",  JVec4(bt.normalTint)  },
            { "hoverTint",   JVec4(bt.hoverTint)   },
            { "pressedTint", JVec4(bt.pressedTint) }
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
            { "isTrigger",     col.isTrigger           },
            { "collisionGroup", col.collisionGroup     }
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
            { "lockRotZ",       rb.lockRotZ            },
            { "continuousCollision", rb.continuousCollision }
        };
    }

    if (reg.all_of<ConstraintComponent>(entity)) {
        auto& cc = reg.get<ConstraintComponent>(entity);
        ej["constraint"] = {
            { "type",       (int)cc.type   },
            { "targetUuid", cc.targetUuid  },
            { "anchor",     JVec3(cc.anchor) },
            { "axis",       JVec3(cc.axis)   },
            { "hasLimits",  cc.hasLimits   },
            { "limitMin",   cc.limitMin    },
            { "limitMax",   cc.limitMax    },
            { "swingNormalDeg", cc.swingNormalDeg },
            { "swingPlaneDeg",  cc.swingPlaneDeg  },
            { "twistMinDeg",    cc.twistMinDeg    },
            { "twistMaxDeg",    cc.twistMaxDeg    },
            { "motorMode",      (int)cc.motorMode },
            { "motorTarget",    cc.motorTarget    },
            { "motorMaxTorque", cc.motorMaxTorque },
            { "motorFrequency", cc.motorFrequency },
            { "motorDamping",   cc.motorDamping   },
            { "motorTargetEuler", JVec3(cc.motorTargetEuler) }
        };
    }

    if (reg.all_of<RagdollComponent>(entity)) {
        auto& rag = reg.get<RagdollComponent>(entity);
        ej["ragdoll"] = {
            { "assetPath", rag.assetPath },
            { "mode",      (int)rag.mode }   // serialized initial mode (usually Animated)
        };
    }

    if (reg.all_of<IKComponent>(entity)) {
        auto& ik = reg.get<IKComponent>(entity);
        json chainsJ = json::array();
        for (const auto& ch : ik.chains) {
            json c = {
                { "endEffectorBone", ch.endEffectorBone     },
                { "boneCount",       ch.boneCount           },
                { "solver",          (int)ch.solver         },
                { "targetOffset",    JVec3(ch.targetOffset) },
                { "poleOffset",      JVec3(ch.poleOffset)   },
                { "weight",          ch.weight              },
                { "easeTime",        ch.easeTime            },
                { "isFootChain",     ch.isFootChain         },
                { "castOffset",      ch.castOffset          },
                { "maxStepHeight",   ch.maxStepHeight       },
                { "ankleHeight",     ch.ankleHeight         },
                { "swingThreshold",  ch.swingThreshold      },
                { "tiltToNormal",    ch.tiltToNormal        },
                { "tiltWeight",      ch.tiltWeight          }
            };
            // Foot chains overwrite targetWorldPos every frame from the ground
            // raycast, so it's runtime scratch — only persist the authored target
            // (explicit world point / followed entity) for non-foot chains.
            if (!ch.isFootChain) {
                c["targetWorldPos"] = JVec3(ch.targetWorldPos);
                if (ch.targetEntity != entt::null && reg.valid(ch.targetEntity) &&
                    reg.all_of<IDComponent>(ch.targetEntity))
                    c["targetUuid"] = reg.get<IDComponent>(ch.targetEntity).uuid;
            }
            chainsJ.push_back(c);
        }
        ej["ik"] = {
            { "chains",         chainsJ           },
            { "pelvisBone",     ik.pelvisBone     },
            { "maxPelvisDrop",  ik.maxPelvisDrop  },
            { "pelvisEaseTime", ik.pelvisEaseTime }
        };
    }

    if (reg.all_of<SkinnedMeshComponent>(entity)) {
        auto& smc = reg.get<SkinnedMeshComponent>(entity);
        json sj = {
            { "meshPath",    smc.meshPath    },
            { "visible",     smc.visible     },
            { "castsShadow", smc.castsShadow }
        };
        // Material overrides. glTF embeds its textures (no file paths), so an
        // empty path here means "keep what the .glb ships with" on reload; only
        // a non-empty path is a user-assigned external texture.
        if (smc.material) {
            sj["albedoPath"]       = smc.material->AlbedoPath;
            sj["normalPath"]       = smc.material->NormalPath;
            sj["metallicPath"]     = smc.material->MetallicPath;
            sj["roughnessPath"]    = smc.material->RoughnessPath;
            sj["aoPath"]           = smc.material->AOPath;
            sj["emissivePath"]     = smc.material->EmissivePath;
            sj["emissiveStrength"] = smc.material->EmissiveStrength;
            sj["uvScale"]          = smc.material->UVScale;
            sj["baseColorFactor"]  = std::array<float, 4>{ smc.material->BaseColorFactor.x, smc.material->BaseColorFactor.y,
                                                            smc.material->BaseColorFactor.z, smc.material->BaseColorFactor.w };
            sj["alphaMode"]        = AlphaModeToString(smc.material->Mode);
            sj["alphaCutoff"]      = smc.material->AlphaCutoff;
        }
        ej["skinnedMesh"] = sj;
    }

    if (reg.all_of<AnimatorComponent>(entity)) {
        auto& anim = reg.get<AnimatorComponent>(entity);
        ej["animator"] = {
            { "clip",    anim.clip    },
            { "time",    anim.time    },
            { "speed",   anim.speed   },
            { "loop",    anim.loop    },
            { "playing", anim.playing }
        };
    }

    if (reg.all_of<AnimStateMachineComponent>(entity)) {
        auto& sm = reg.get<AnimStateMachineComponent>(entity);
        json smj;
        smj["assetPath"] = sm.assetPath;
        // Live parameter values (current overrides, not the asset defaults).
        // Triggers are transient and never serialized.
        json fj = json::object();
        for (auto& [k, v] : sm.floats) fj[k] = v;
        json bj = json::object();
        for (auto& [k, v] : sm.bools)  bj[k] = v;
        smj["floats"] = fj;
        smj["bools"]  = bj;
        ej["animStateMachine"] = smj;
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

    return ej;
}

// Per-load state shared across the entity list: the uuid→entity map remaps
// internal references, and the caches/pins live for the whole load pass.
// Textures/materials/models load through the asset registry (dedup across
// the whole editor, not just this load). The registry holds only weak_ptrs
// and mesh CPU data is dropped right after GPU upload, so pin every model
// asset until the load finishes — entities sharing a file then parse it
// once. Textures need no pin: the components/materials hold them strongly.
struct DeserializeCtx {
    std::unordered_map<uint64_t, entt::entity> uuidToEntity;
    std::unordered_map<std::string, std::shared_ptr<Font>> fontCache;
    std::vector<std::shared_ptr<void>> modelPins;
};

// Components are emplace_or_replace'd (not emplace'd): besides fresh loads,
// this also runs to apply prefab-instance overrides onto entities that already
// carry the component from instantiation.
static void DeserializeEntityComponents(Scene& scene, entt::entity e, const json& ej,
                                        DeserializeCtx& ctx)
{
    auto& reg = scene.GetRegistry();

    if (ej.contains("prefabInstance")) {
        auto& pi = reg.emplace_or_replace<PrefabInstanceComponent>(e);
        pi.sourcePath = ej["prefabInstance"].value("sourcePath", "");
    }

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
        // glTF-shipped material for this submesh (embedded textures, no file
        // paths) — the fallback when the entity has neither a .mat nor inline
        // texture paths.
        std::shared_ptr<PBRMaterial> shippedMat;

        auto isGltf = [](const std::string& p) {
            auto dot = p.find_last_of('.');
            if (dot == std::string::npos) return false;
            std::string ext = p.substr(dot + 1);
            for (auto& c : ext) c = (char)std::tolower((unsigned char)c);
            return ext == "gltf" || ext == "glb";
        };

        if (mpath == "__sphere") {
            auto md = MeshData::UVSphere();
            bounds  = md.ComputeAABB();
            mesh    = Mesh::Create(md);
        } else if (mpath == "__cube") {
            auto md = MeshData::UnitCube();
            bounds  = md.ComputeAABB();
            mesh    = Mesh::Create(md);
        } else if (isGltf(mpath)) {
            // The full scene import instead of the geometry-only MeshAsset:
            // same primitive order (so subIdx is interchangeable), but it also
            // carries the per-primitive materials.
            if (auto sceneAsset = Assets::Load<ImportedScene>(mpath)) {
                ctx.modelPins.push_back(sceneAsset);
                if (subIdx < (int)sceneAsset->meshes.size()) {
                    bounds = sceneAsset->meshes[subIdx].ComputeAABB();
                    mesh   = Mesh::Create(sceneAsset->meshes[subIdx]);
                    int mi = sceneAsset->primitiveMaterial[subIdx];
                    if (mi >= 0) shippedMat = sceneAsset->materials[mi];
                }
            }
        } else if (!mpath.empty()) {
            if (auto meshAsset = Assets::Load<Assets::MeshAsset>(mpath)) {
                ctx.modelPins.push_back(meshAsset);
                if (subIdx < (int)meshAsset->subMeshes.size()) {
                    bounds = meshAsset->subMeshes[subIdx].ComputeAABB();
                    mesh   = Mesh::Create(meshAsset->subMeshes[subIdx]);
                }
            }
        }

        std::string materialPath = mj.value("materialPath", "");

        std::shared_ptr<PBRMaterial> mat;
        if (!materialPath.empty()) {
            // Asset-backed: shared instance from the material library. Falls back
            // to an empty material if the .mat is missing so the mesh still draws.
            materialPath = NormalizeMaterialPath(materialPath);
            mat = MaterialLibrary::Get(materialPath);
            if (!mat) mat = std::make_shared<PBRMaterial>();
        } else {
            // Inline: start from what the glTF shipped (copied — the imported
            // scene is a shared asset), then apply only the overrides the user
            // set (non-empty texture paths). Same convention as skinned meshes.
            mat = shippedMat ? std::make_shared<PBRMaterial>(*shippedMat)
                             : std::make_shared<PBRMaterial>();
            auto applyTex = [&](const char* key, std::shared_ptr<Texture>& tex, std::string& path) {
                std::string p = mj.value(key, std::string{});
                if (!p.empty()) { path = p; tex = Assets::Load<Texture>(p); }
            };
            applyTex("albedoPath",    mat->Albedo,    mat->AlbedoPath);
            applyTex("normalPath",    mat->Normal,    mat->NormalPath);
            applyTex("metallicPath",  mat->Metallic,  mat->MetallicPath);
            applyTex("roughnessPath", mat->Roughness, mat->RoughnessPath);
            applyTex("aoPath",        mat->AO,        mat->AOPath);
            applyTex("emissivePath",  mat->Emissive,  mat->EmissivePath);
            mat->EmissiveStrength = mj.value("emissiveStrength", mat->EmissiveStrength);
            mat->UVScale          = mj.value("uvScale",          mat->UVScale);
            {
                std::array<float, 4> def{ mat->BaseColorFactor.x, mat->BaseColorFactor.y,
                                          mat->BaseColorFactor.z, mat->BaseColorFactor.w };
                auto bc = mj.value("baseColorFactor", def);
                mat->BaseColorFactor = glm::vec4(bc[0], bc[1], bc[2], bc[3]);
            }
            // Missing keys keep what the glTF shipped, like the fields above.
            mat->Mode        = AlphaModeFromString(mj.value("alphaMode", std::string{ AlphaModeToString(mat->Mode) }));
            mat->AlphaCutoff = mj.value("alphaCutoff", mat->AlphaCutoff);
        }

        auto& mc          = reg.emplace_or_replace<MeshComponent>(e, mesh, mat, bounds);
        mc.meshPath       = mpath;
        mc.meshSubIndex   = subIdx;
        mc.materialPath   = materialPath;
        mc.visible            = mj.value("visible",            true);
        mc.castsShadow        = mj.value("castsShadow",        true);
        mc.staticShadowCaster = mj.value("staticShadowCaster", true);
        mc.receivesShadow     = mj.value("receivesShadow",     true);
        mc.transparent        = mj.value("transparent",        false);
    }

    if (ej.contains("light")) {
        const auto& lj = ej["light"];
        auto& lc          = reg.emplace_or_replace<LightComponent>(e);
        lc.type           = (LightType)lj.value("type", 1);
        lc.color          = ToVec3(lj["color"]);
        lc.intensity      = lj.value("intensity",      100.0f);
        lc.radius         = lj.value("radius",         10.0f);
        lc.innerConeAngle = lj.value("innerConeAngle", 15.0f);
        lc.outerConeAngle = lj.value("outerConeAngle", 30.0f);
    }

    if (ej.contains("skylight")) {
        const auto& sj = ej["skylight"];
        auto& sl = reg.emplace_or_replace<SkyLightComponent>(e);
        sl.environmentPath = sj.value("environmentPath", std::string{});
        sl.intensity       = sj.value("intensity",       1.0f);
    }

    if (ej.contains("ddgiVolume")) {
        const auto& dj = ej["ddgiVolume"];
        auto& dv = reg.emplace_or_replace<DDGIVolumeComponent>(e);
        if (dj.contains("extent")) dv.extent = ToVec3(dj["extent"]);
        dv.probeCounts.x     = dj.value("probeCountX",       8);
        dv.probeCounts.y     = dj.value("probeCountY",       4);
        dv.probeCounts.z     = dj.value("probeCountZ",       8);
        dv.raysPerProbe      = dj.value("raysPerProbe",      64);
        dv.hysteresis        = dj.value("hysteresis",        0.97f);
        dv.normalBias        = dj.value("normalBias",        0.15f);
        dv.viewBias          = dj.value("viewBias",          0.10f);
        dv.energy            = dj.value("energy",            1.0f);
        dv.maxRayDistance    = dj.value("maxRayDistance",    40.0f);
        dv.backfaceThreshold = dj.value("backfaceThreshold", 0.25f);
        dv.showProbes        = dj.value("showProbes",        false);
        dv.probeRadius       = dj.value("probeRadius",       0.15f);
    }

    if (ej.contains("camera")) {
        const auto& cj = ej["camera"];
        auto& cc    = reg.emplace_or_replace<CameraComponent>(e);
        cc.isPrimary = cj.value("isPrimary", true);
        cc.fov       = cj.value("fov",       60.0f);
        cc.nearClip  = cj.value("nearClip",  0.1f);
        cc.farClip   = cj.value("farClip",   1000.0f);
    }

    if (ej.contains("canvas")) {
        const auto& cj = ej["canvas"];
        auto& cv = reg.emplace_or_replace<CanvasComponent>(e);
        cv.scaleMode = (CanvasComponent::ScaleMode)cj.value("scaleMode", 1);
        if (cj.contains("referenceResolution"))
            cv.referenceResolution = ToVec2(cj["referenceResolution"]);
        cv.matchWidthHeight = cj.value("matchWidthHeight", 0.5f);
        cv.sortOrder        = cj.value("sortOrder", 0);
    }

    if (ej.contains("rectTransform")) {
        const auto& rj = ej["rectTransform"];
        auto& rt = reg.emplace_or_replace<RectTransformComponent>(e);
        if (rj.contains("anchorMin")) rt.anchorMin = ToVec2(rj["anchorMin"]);
        if (rj.contains("anchorMax")) rt.anchorMax = ToVec2(rj["anchorMax"]);
        if (rj.contains("pivot"))     rt.pivot     = ToVec2(rj["pivot"]);
        if (rj.contains("position"))  rt.position  = ToVec2(rj["position"]);
        if (rj.contains("size"))      rt.size      = ToVec2(rj["size"]);
        rt.zOrder = rj.value("zOrder", 0);
    }

    if (ej.contains("uiImage")) {
        const auto& j = ej["uiImage"];
        auto& im = reg.emplace_or_replace<UIImageComponent>(e);
        im.texturePath = j.value("texturePath", "");
        im.texture     = Assets::Load<Texture>(im.texturePath);
        if (j.contains("tint"))  im.tint  = ToVec4(j["tint"]);
        if (j.contains("uvMin")) im.uvMin = ToVec2(j["uvMin"]);
        if (j.contains("uvMax")) im.uvMax = ToVec2(j["uvMax"]);
    }

    if (ej.contains("uiText")) {
        const auto& j = ej["uiText"];
        auto& tx = reg.emplace_or_replace<UITextComponent>(e);
        tx.text     = j.value("text", "");
        tx.fontPath = j.value("fontPath", "");
        tx.font     = LoadFontCached(tx.fontPath, ctx.fontCache);
        tx.sizePx   = j.value("sizePx", 24.0f);
        if (j.contains("color")) tx.color = ToVec4(j["color"]);
        tx.hAlign      = (UITextComponent::HAlign)j.value("hAlign", 0);
        tx.vAlign      = (UITextComponent::VAlign)j.value("vAlign", 0);
        tx.lineSpacing = j.value("lineSpacing", 1.0f);
        tx.wrap        = j.value("wrap", true);
        tx.underline   = j.value("underline", false);
    }

    if (ej.contains("uiProgressBar")) {
        const auto& j = ej["uiProgressBar"];
        auto& pb = reg.emplace_or_replace<UIProgressBarComponent>(e);
        pb.progress = j.value("progress", 0.5f);
        if (j.contains("backgroundColor")) pb.backgroundColor = ToVec4(j["backgroundColor"]);
        if (j.contains("fillColor"))       pb.fillColor       = ToVec4(j["fillColor"]);
        pb.fillTexturePath = j.value("fillTexturePath", "");
        pb.fillTexture     = Assets::Load<Texture>(pb.fillTexturePath);
        pb.direction       = (UIProgressBarComponent::Direction)j.value("direction", 0);
    }

    if (ej.contains("uiButton")) {
        const auto& j = ej["uiButton"];
        auto& bt = reg.emplace_or_replace<UIButtonComponent>(e);
        if (j.contains("normalTint"))  bt.normalTint  = ToVec4(j["normalTint"]);
        if (j.contains("hoverTint"))   bt.hoverTint   = ToVec4(j["hoverTint"]);
        if (j.contains("pressedTint")) bt.pressedTint = ToVec4(j["pressedTint"]);
    }

    if (ej.contains("collider")) {
        const auto& cj = ej["collider"];
        auto& col = reg.emplace_or_replace<ColliderComponent>(e);
        col.shapeType  = (CollisionShape)cj.value("shapeType", 0);
        col.radius     = cj.value("radius",    0.5f);
        col.halfHeight = cj.value("halfHeight", 0.5f);
        col.meshPath   = cj.value("meshPath",   "");
        col.physMatPath = cj.value("physMatPath", "");
        col.isTrigger  = cj.value("isTrigger",  false);
        col.collisionGroup = cj.value("collisionGroup", 0u);
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
        auto& rb = reg.emplace_or_replace<RigidBodyComponent>(e);
        rb.bodyType       = (BodyType)rj.value("bodyType",       0);
        rb.mass           = rj.value("mass",           1.0f);
        rb.linearDamping  = rj.value("linearDamping",  0.05f);
        rb.angularDamping = rj.value("angularDamping", 0.05f);
        rb.gravityScale   = rj.value("gravityScale",   1.0f);
        rb.lockRotX       = rj.value("lockRotX",       false);
        rb.lockRotY       = rj.value("lockRotY",       false);
        rb.lockRotZ       = rj.value("lockRotZ",       false);
        rb.continuousCollision = rj.value("continuousCollision", false);
    }

    if (ej.contains("constraint")) {
        const auto& cj = ej["constraint"];
        auto& cc = reg.emplace_or_replace<ConstraintComponent>(e);
        cc.type       = (ConstraintType)cj.value("type", 0);
        cc.targetUuid = cj.value("targetUuid", (uint64_t)0);
        // The target uuid must survive prefab instantiation, where every entity
        // gets a fresh id: remap through the load map. Identity on scene loads
        // (stored uuid == live uuid); targets outside the file pass through.
        if (auto it = ctx.uuidToEntity.find(cc.targetUuid); it != ctx.uuidToEntity.end())
            cc.targetUuid = reg.get<IDComponent>(it->second).uuid;
        if (cj.contains("anchor")) cc.anchor = ToVec3(cj["anchor"]);
        if (cj.contains("axis"))   cc.axis   = ToVec3(cj["axis"]);
        cc.hasLimits = cj.value("hasLimits", false);
        cc.limitMin  = cj.value("limitMin", -90.0f);
        cc.limitMax  = cj.value("limitMax",  90.0f);
        cc.swingNormalDeg = cj.value("swingNormalDeg", 45.0f);
        cc.swingPlaneDeg  = cj.value("swingPlaneDeg",  45.0f);
        cc.twistMinDeg    = cj.value("twistMinDeg",   -45.0f);
        cc.twistMaxDeg    = cj.value("twistMaxDeg",    45.0f);
        cc.motorMode      = (MotorMode)cj.value("motorMode", 0);
        cc.motorTarget    = cj.value("motorTarget",    0.0f);
        cc.motorMaxTorque = cj.value("motorMaxTorque", 50.0f);
        cc.motorFrequency = cj.value("motorFrequency", 2.0f);
        cc.motorDamping   = cj.value("motorDamping",   1.0f);
        if (cj.contains("motorTargetEuler")) cc.motorTargetEuler = ToVec3(cj["motorTargetEuler"]);
    }

    if (ej.contains("ragdoll")) {
        const auto& rj = ej["ragdoll"];
        auto& rag     = reg.emplace_or_replace<RagdollComponent>(e);
        rag.assetPath = rj.value("assetPath", std::string{});
        rag.mode      = (RagdollMode)rj.value("mode", 0);
        if (!rag.assetPath.empty()) {
            rag.assetPath = NormalizeRagdollPath(rag.assetPath);
            auto cfg = std::make_shared<RagdollConfig>();
            if (LoadRagdoll(rag.assetPath, *cfg)) rag.config = cfg;
            else spdlog::warn("Scene load: failed to load ragdoll '{}'", rag.assetPath);
        }
    }

    if (ej.contains("skinnedMesh")) {
        const auto& sj    = ej["skinnedMesh"];
        std::string mpath = sj.value("meshPath", "");
        if (!mpath.empty()) {
            auto model = Assets::Load<ImportedModel>(mpath);
            if (model && !model->skeleton.bones.empty() && !model->meshes.empty()) {
                ctx.modelPins.push_back(model);
                auto& smc       = reg.emplace_or_replace<SkinnedMeshComponent>(e);
                smc.skeleton    = model->skeleton;
                smc.clips       = model->animations;
                smc.meshPath    = mpath;
                smc.visible     = sj.value("visible",     true);
                smc.castsShadow = sj.value("castsShadow", true);

                AABB bounds;
                for (const auto& md : model->meshes) {
                    smc.meshes.push_back(Mesh::Create(md));
                    AABB b      = md.ComputeAABB();
                    bounds.min  = glm::min(bounds.min, b.min);
                    bounds.max  = glm::max(bounds.max, b.max);
                }
                smc.localBounds = bounds;

                // Material: start from what the .glb shipped (copied — the
                // imported model is a shared asset), then apply only the
                // overrides the user set (non-empty external texture paths).
                smc.material = model->material
                    ? std::make_shared<PBRMaterial>(*model->material)
                    : std::make_shared<PBRMaterial>();
                auto applyTex = [&](const char* key, std::shared_ptr<Texture>& tex, std::string& path) {
                    std::string p = sj.value(key, std::string{});
                    if (!p.empty()) { path = p; tex = Assets::Load<Texture>(p); }
                };
                applyTex("albedoPath",    smc.material->Albedo,    smc.material->AlbedoPath);
                applyTex("normalPath",    smc.material->Normal,    smc.material->NormalPath);
                applyTex("metallicPath",  smc.material->Metallic,  smc.material->MetallicPath);
                applyTex("roughnessPath", smc.material->Roughness, smc.material->RoughnessPath);
                applyTex("aoPath",        smc.material->AO,        smc.material->AOPath);
                applyTex("emissivePath",  smc.material->Emissive,  smc.material->EmissivePath);
                smc.material->EmissiveStrength = sj.value("emissiveStrength", smc.material->EmissiveStrength);
                smc.material->UVScale          = sj.value("uvScale",          smc.material->UVScale);
                {
                    std::array<float, 4> def{ smc.material->BaseColorFactor.x, smc.material->BaseColorFactor.y,
                                              smc.material->BaseColorFactor.z, smc.material->BaseColorFactor.w };
                    auto bc = sj.value("baseColorFactor", def);
                    smc.material->BaseColorFactor = glm::vec4(bc[0], bc[1], bc[2], bc[3]);
                }
                // Missing keys keep what the glTF shipped, like the fields above.
                smc.material->Mode        = AlphaModeFromString(sj.value("alphaMode", std::string{ AlphaModeToString(smc.material->Mode) }));
                smc.material->AlphaCutoff = sj.value("alphaCutoff", smc.material->AlphaCutoff);
            }
        }
    }

    if (ej.contains("animator")) {
        const auto& aj = ej["animator"];
        auto& anim   = reg.emplace_or_replace<AnimatorComponent>(e);
        anim.clip    = aj.value("clip",    0);
        anim.time    = aj.value("time",    0.0f);
        anim.speed   = aj.value("speed",   1.0f);
        anim.loop    = aj.value("loop",    true);
        anim.playing = aj.value("playing", true);
    }

    if (ej.contains("ik")) {
        const auto& ikj = ej["ik"];
        auto& ik = reg.emplace_or_replace<IKComponent>(e);
        ik.pelvisBone     = ikj.value("pelvisBone", std::string{});
        ik.maxPelvisDrop  = ikj.value("maxPelvisDrop",  0.5f);
        ik.pelvisEaseTime = ikj.value("pelvisEaseTime", 0.1f);
        if (ikj.contains("chains")) {
            for (const auto& c : ikj["chains"]) {
                IKChain ch;
                ch.endEffectorBone = c.value("endEffectorBone", std::string{});
                ch.boneCount       = c.value("boneCount", 2);
                ch.solver          = (IKSolverType)c.value("solver", 0);
                if (c.contains("targetOffset")) ch.targetOffset = ToVec3(c["targetOffset"]);
                if (c.contains("poleOffset"))   ch.poleOffset   = ToVec3(c["poleOffset"]);
                ch.weight          = c.value("weight",   1.0f);
                ch.easeTime        = c.value("easeTime", 0.15f);
                ch.isFootChain     = c.value("isFootChain",    false);
                ch.castOffset      = c.value("castOffset",     0.5f);
                ch.maxStepHeight   = c.value("maxStepHeight",  0.5f);
                ch.ankleHeight     = c.value("ankleHeight",    0.08f);
                ch.swingThreshold  = c.value("swingThreshold", 0.1f);
                ch.tiltToNormal    = c.value("tiltToNormal",   true);
                ch.tiltWeight      = c.value("tiltWeight",     0.8f);
                if (c.contains("targetWorldPos")) ch.targetWorldPos = ToVec3(c["targetWorldPos"]);
                if (c.contains("targetUuid")) {
                    auto it = ctx.uuidToEntity.find(c["targetUuid"].get<uint64_t>());
                    if (it != ctx.uuidToEntity.end()) ch.targetEntity = it->second;
                }
                ik.chains.push_back(ch);
            }
        }
    }

    if (ej.contains("animStateMachine")) {
        const auto& smj = ej["animStateMachine"];
        auto& sm = reg.emplace_or_replace<AnimStateMachineComponent>(e);
        sm.assetPath = smj.value("assetPath", "");
        if (!sm.assetPath.empty()) {
            sm.assetPath = NormalizeAnimSmPath(sm.assetPath);
            sm.machine   = LoadAnimStateMachine(sm.assetPath);
        }
        // Seed defaults from the asset, then apply the saved overrides.
        sm.SyncParams();
        if (smj.contains("floats"))
            for (auto& [k, v] : smj["floats"].items()) sm.floats[k] = v.get<float>();
        if (smj.contains("bools"))
            for (auto& [k, v] : smj["bools"].items())  sm.bools[k]  = v.get<bool>();
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
        auto it = ctx.uuidToEntity.find(parentUuid);
        if (it != ctx.uuidToEntity.end())
            scene.SetParent(e, it->second);
    }
}

// ---- prefab machinery used by scene save/load (defined in the prefab section) --

static json         SerializePrefabRef(Scene& scene, entt::entity root,
                                       std::unordered_map<std::string, json>& prefabCache);
static entt::entity InstantiateSceneRef(Scene& scene, const json& refJ, DeserializeCtx& sceneCtx);
static void         CollectSubtree(Scene& scene, entt::entity e, std::vector<entt::entity>& out);

// A "new-style" instance root serializes as a prefabRef node. Legacy roots
// (baked copies loaded from old scenes — instance link but no local-id stamp)
// keep serializing as plain entities so their tweaks aren't clobbered; broken
// placeholders serialize as refs so the link round-trips.
static bool IsInstanceRefRoot(entt::registry& reg, entt::entity e)
{
    if (!reg.all_of<PrefabInstanceComponent>(e)) return false;
    return reg.all_of<PrefabChildComponent>(e) || reg.get<PrefabInstanceComponent>(e).broken;
}

static bool HasInstanceRefAncestor(Scene& scene, entt::entity e)
{
    auto& reg = scene.GetRegistry();
    entt::entity p = reg.all_of<HierarchyComponent>(e)
                   ? reg.get<HierarchyComponent>(e).parent : entt::null;
    while (p != entt::null) {
        if (IsInstanceRefRoot(reg, p)) return true;
        p = reg.all_of<HierarchyComponent>(p)
          ? reg.get<HierarchyComponent>(p).parent : entt::null;
    }
    return false;
}

static json ToJson(Scene& scene)
{
    auto& reg = scene.GetRegistry();
    std::unordered_map<std::string, json> prefabCache;   // one parse per .prefab per save
    json entities = json::array();
    for (auto& [entity, name] : scene.GetEntityNames()) {
        bool covered = HasInstanceRefAncestor(scene, entity);
        if (IsInstanceRefRoot(reg, entity) && !covered)
            entities.push_back(SerializePrefabRef(scene, entity, prefabCache));
        else if (!covered || !reg.all_of<PrefabChildComponent>(entity))
            // Plain entity. The second clause keeps children the user parented
            // INTO an instance: they aren't part of the prefab, so they save as
            // plain entities (their parentUuid — a derived instance-child id —
            // is stable across loads).
            entities.push_back(SerializeEntity(scene, entity));
        // else: instance interior — reproduced by the ref node on load
    }
    return json{ { "entities", entities } };
}

static bool FromJson(Scene& scene, const json& root)
{
    scene.Clear();
    DeserializeCtx ctx;
    const json& entities = root.at("entities");

    // Prefab references first: their (deterministic) interior uuids land in
    // the remap map so plain entities can be parented under instance children.
    // The refs' own parents may be plain entities created later, so instance
    // parenting is deferred to the end.
    std::vector<std::pair<entt::entity, uint64_t>> refParents;
    for (const auto& ej : entities) {
        if (!ej.contains("prefabRef")) continue;
        const json& refJ = ej["prefabRef"];
        entt::entity instRoot = InstantiateSceneRef(scene, refJ, ctx);
        if (instRoot != entt::null && refJ.contains("parentUuid"))
            refParents.push_back({ instRoot, refJ["parentUuid"].get<uint64_t>() });
    }

    // Plain entities — two passes so parent/target references resolve
    // regardless of file order; scene loads restore the stored uuids so
    // cross-file references stay stable.
    std::vector<entt::entity> created;
    std::vector<const json*>  plainNodes;
    for (const auto& ej : entities) {
        if (ej.contains("prefabRef")) continue;
        entt::entity e = scene.CreateEntity(ej.value("name", "Entity"));
        created.push_back(e);
        plainNodes.push_back(&ej);
        if (ej.contains("uuid")) {
            uint64_t uuid = ej["uuid"].get<uint64_t>();
            scene.GetRegistry().get<IDComponent>(e).uuid = uuid;
            ctx.uuidToEntity[uuid] = e;
        }
    }
    for (size_t i = 0; i < plainNodes.size(); ++i)
        DeserializeEntityComponents(scene, created[i], *plainNodes[i], ctx);

    for (auto& [instRoot, parentUuid] : refParents) {
        auto it = ctx.uuidToEntity.find(parentUuid);
        if (it != ctx.uuidToEntity.end() && scene.GetRegistry().valid(it->second))
            scene.SetParent(instRoot, it->second);
    }
    return true;
}

// ---- public API -------------------------------------------------------------

void SceneSerializer::Save(Scene& scene, const std::string& path)
{
    json root = ToJson(scene);
    AssetPaths::MakePortable(root);
    std::ofstream f(path);
    f << root.dump(2);
}

bool SceneSerializer::Load(Scene& scene, const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;
    json root;
    try { root = json::parse(f); }
    catch (...) { return false; }
    AssetPaths::ResolveAll(root);
    return FromJson(scene, root);
}

std::string SceneSerializer::Stringify(Scene& scene)
{
    json root = ToJson(scene);
    AssetPaths::MakePortable(root);
    return root.dump();
}

bool SceneSerializer::FromString(Scene& scene, const std::string& data)
{
    json root;
    try { root = json::parse(data); }
    catch (...) { return false; }
    AssetPaths::ResolveAll(root);
    return FromJson(scene, root);
}

// ---- prefabs ----------------------------------------------------------------

static void CollectSubtree(Scene& scene, entt::entity e, std::vector<entt::entity>& out)
{
    out.push_back(e);
    auto& reg = scene.GetRegistry();
    if (reg.all_of<HierarchyComponent>(e))
        for (entt::entity child : reg.get<HierarchyComponent>(e).children)
            CollectSubtree(scene, child, out);
}

// Entity-json keys that are identity/structure rather than component data —
// excluded from the override diff (name is diffed separately).
static bool IsNonComponentKey(const std::string& k)
{
    return k == "name" || k == "uuid" || k == "parentUuid" || k == "prefabInstance";
}

// Parses a .prefab through a per-save-pass cache; returns nullptr when the
// file is missing/unreadable or has no prefab block.
static const json* LoadPrefabJson(const std::string& path,
                                  std::unordered_map<std::string, json>& cache)
{
    auto [it, inserted] = cache.emplace(NormalizePrefabPath(path), json{});
    if (inserted) {
        std::ifstream f(path);
        if (f.is_open()) {
            try { it->second = json::parse(f); }
            catch (...) { it->second = json{}; }
            AssetPaths::ResolveAll(it->second);
        }
    }
    const json& j = it->second;
    if (!j.contains("prefab") || !j["prefab"].contains("entities")) return nullptr;
    return &j;
}

bool PrefabSerializer::Save(Scene& scene, entt::entity root, const std::string& path)
{
    auto& reg = scene.GetRegistry();
    if (!reg.valid(root) || !reg.all_of<IDComponent>(root)) return false;

    std::vector<entt::entity> subtree;
    CollectSubtree(scene, root, subtree);

    // The file's uuids are prefab-local: a stamped entity keeps the local id it
    // was instantiated with, so re-saving from any instance leaves the id space
    // stable and other instances' override targets valid. Unstamped entities
    // (fresh subtree, children added in edit mode) adopt their live uuid as the
    // local id and are stamped with it — the saved subtree becomes the
    // canonical, zero-override instance of the file.
    std::unordered_map<uint64_t, uint64_t> liveToLocal;
    std::unordered_set<uint64_t>           usedLocal;
    for (entt::entity e : subtree) {
        uint64_t live  = reg.get<IDComponent>(e).uuid;
        uint64_t local = live;
        if (auto* pc = reg.try_get<PrefabChildComponent>(e); pc && pc->prefabUuid)
            local = pc->prefabUuid;
        // duplicated children carry a colliding stamp — fall back to the live id
        if (usedLocal.count(local)) local = live;
        usedLocal.insert(local);
        liveToLocal[live] = local;
        reg.emplace_or_replace<PrefabChildComponent>(e, local);
    }

    json entities = json::array();
    for (entt::entity e : subtree) {
        json ej = SerializeEntity(scene, e);
        ej["uuid"] = liveToLocal[reg.get<IDComponent>(e).uuid];
        if (ej.contains("parentUuid")) {
            auto it = liveToLocal.find(ej["parentUuid"].get<uint64_t>());
            if (it != liveToLocal.end()) ej["parentUuid"] = it->second;
        }
        RemapTargetUuids(ej, liveToLocal);
        if (e == root) {
            // The root's parent lives outside the prefab, and the instance link
            // is stamped fresh at instantiation — neither belongs in the file.
            ej.erase("parentUuid");
            ej.erase("prefabInstance");
        }
        entities.push_back(std::move(ej));
    }

    std::ofstream f(path);
    if (!f.is_open()) {
        spdlog::error("Prefab save: cannot open '{}'", path);
        return false;
    }
    json out{ { "prefab", {
        { "root",     liveToLocal[reg.get<IDComponent>(root).uuid] },
        { "entities", std::move(entities) }
    } } };
    AssetPaths::MakePortable(out);
    f << out.dump(2);
    return true;
}

// Core instantiation from a parsed prefab block. Ids are final before the
// component pass (constraint/IK remaps read live IDComponents): the root keeps
// its generated uuid unless `forcedRootUuid` is set (scene refs restoring a
// stored identity); children get deterministic ids derived from the root's.
// Fills `outLocalMap` with file-local uuid → live entity.
static entt::entity InstantiatePrefabJson(Scene& scene, const json& pj, const std::string& path,
                                          uint64_t forcedRootUuid,
                                          std::unordered_map<uint64_t, entt::entity>* outLocalMap)
{
    auto& reg = scene.GetRegistry();
    const json& entities = pj["entities"];

    DeserializeCtx ctx;
    std::vector<entt::entity> created;
    created.reserve(entities.size());
    for (const auto& ej : entities) {
        entt::entity e = scene.CreateEntity(ej.value("name", "Entity"));
        created.push_back(e);
        if (ej.contains("uuid"))
            ctx.uuidToEntity[ej["uuid"].get<uint64_t>()] = e;
    }
    if (created.empty()) return entt::null;

    // First entity is the fallback for legacy files without a root field —
    // Save always writes the root first.
    entt::entity rootEntity = created.front();
    if (pj.contains("root")) {
        auto it = ctx.uuidToEntity.find(pj["root"].get<uint64_t>());
        if (it != ctx.uuidToEntity.end()) rootEntity = it->second;
    }

    if (forcedRootUuid)
        reg.get<IDComponent>(rootEntity).uuid = forcedRootUuid;
    uint64_t rootUuid = reg.get<IDComponent>(rootEntity).uuid;

    size_t i = 0;
    for (const auto& ej : entities) {
        entt::entity e = created[i++];
        // legacy entities without a stored uuid stamp their generated one
        uint64_t local = ej.contains("uuid") ? ej["uuid"].get<uint64_t>()
                                             : reg.get<IDComponent>(e).uuid;
        if (e != rootEntity)
            reg.get<IDComponent>(e).uuid = DerivePrefabChildUuid(rootUuid, local);
        reg.emplace_or_replace<PrefabChildComponent>(e, local);
    }

    i = 0;
    for (const auto& ej : entities)
        DeserializeEntityComponents(scene, created[i++], ej, ctx);

    reg.emplace_or_replace<PrefabInstanceComponent>(rootEntity, path);
    if (outLocalMap) *outLocalMap = std::move(ctx.uuidToEntity);
    return rootEntity;
}

// Removes the component a serialized key refers to — override application
// needs this for components the instance removed relative to the prefab.
static void RemoveComponentByKey(Scene& scene, entt::entity e, const std::string& key)
{
    auto& reg = scene.GetRegistry();
    if      (key == "mesh")             reg.remove<MeshComponent>(e);
    else if (key == "light")            reg.remove<LightComponent>(e);
    else if (key == "skylight")         reg.remove<SkyLightComponent>(e);
    else if (key == "ddgiVolume")       reg.remove<DDGIVolumeComponent>(e);
    else if (key == "camera")           reg.remove<CameraComponent>(e);
    else if (key == "canvas")           reg.remove<CanvasComponent>(e);
    else if (key == "rectTransform")    reg.remove<RectTransformComponent>(e);
    else if (key == "uiImage")          reg.remove<UIImageComponent>(e);
    else if (key == "uiText")           reg.remove<UITextComponent>(e);
    else if (key == "uiProgressBar")    reg.remove<UIProgressBarComponent>(e);
    else if (key == "uiButton")         reg.remove<UIButtonComponent>(e);
    else if (key == "collider")         reg.remove<ColliderComponent>(e);
    else if (key == "rigidbody")        reg.remove<RigidBodyComponent>(e);
    else if (key == "constraint")       reg.remove<ConstraintComponent>(e);
    else if (key == "ragdoll")          reg.remove<RagdollComponent>(e);
    else if (key == "skinnedMesh")      reg.remove<SkinnedMeshComponent>(e);
    else if (key == "animator")         reg.remove<AnimatorComponent>(e);
    else if (key == "ik")               reg.remove<IKComponent>(e);
    else if (key == "animStateMachine") reg.remove<AnimStateMachineComponent>(e);
    else if (key == "scriptComponents")
        for (auto& desc : ComponentRegistry::Get().GetAll())
            if (desc.remove && desc.has(scene, e)) desc.remove(scene, e);
}

// Applies a ref node's override entries onto a freshly instantiated instance.
// Overrides are stored in prefab-local uuid space; target references inside
// them are remapped to this instance's live ids before deserializing.
static void ApplyRefOverrides(Scene& scene, const json& refJ,
                              const std::unordered_map<uint64_t, entt::entity>& localMap)
{
    if (!refJ.contains("overrides")) return;
    auto& reg = scene.GetRegistry();

    std::unordered_map<uint64_t, uint64_t> localToLive;
    DeserializeCtx ctx;   // keyed by live uuid so IK/constraint lookups resolve
    for (auto& [local, e] : localMap) {
        if (!reg.valid(e)) continue;
        uint64_t live = reg.get<IDComponent>(e).uuid;
        localToLive[local] = live;
        ctx.uuidToEntity[live] = e;
    }

    for (const auto& entry : refJ["overrides"]) {
        auto it = localMap.find(entry.value("target", (uint64_t)0));
        if (it == localMap.end() || !reg.valid(it->second)) continue;
        entt::entity e = it->second;

        if (entry.contains("name"))
            scene.SetEntityName(e, entry["name"].get<std::string>());

        if (entry.contains("removed"))
            for (const auto& k : entry["removed"])
                RemoveComponentByKey(scene, e, k.get<std::string>());

        if (entry.contains("components")) {
            json comps = entry["components"];
            RemapTargetUuids(comps, localToLive);
            // A scriptComponents override is authoritative for the whole set:
            // drop scripts the prefab added that the instance no longer has.
            if (comps.contains("scriptComponents"))
                for (auto& desc : ComponentRegistry::Get().GetAll())
                    if (desc.remove && desc.has(scene, e) &&
                        !comps["scriptComponents"].contains(desc.name))
                        desc.remove(scene, e);
            DeserializeEntityComponents(scene, e, comps, ctx);
        }
    }
}

// Serializes an instance root as a { "prefabRef": ... } node: source path, the
// root's identity/transform, entities deleted from the instance, and per-entity
// component blocks that differ from the .prefab. Reference fields are remapped
// into prefab-local uuid space so an edit like "constraint targets sibling"
// diffs clean and re-applies onto any future instantiation.
static json SerializePrefabRef(Scene& scene, entt::entity root,
                               std::unordered_map<std::string, json>& prefabCache)
{
    auto& reg = scene.GetRegistry();
    auto& pi  = reg.get<PrefabInstanceComponent>(root);

    json ref;
    ref["path"] = pi.sourcePath;
    ref["uuid"] = reg.get<IDComponent>(root).uuid;
    ref["name"] = scene.GetEntityName(root);
    if (reg.all_of<HierarchyComponent>(root)) {
        entt::entity parent = reg.get<HierarchyComponent>(root).parent;
        if (parent != entt::null && reg.all_of<IDComponent>(parent))
            ref["parentUuid"] = reg.get<IDComponent>(parent).uuid;
    }
    auto& tc = reg.get<TransformComponent>(root);
    ref["transform"] = {
        { "position",     JVec3(tc.position)     },
        { "rotation",     JQuat(tc.rotation)     },
        { "eulerDegrees", JVec3(tc.eulerDegrees) },
        { "scale",        JVec3(tc.scale)        }
    };

    // Broken placeholder: pass the overrides captured at load straight through.
    if (pi.broken) {
        if (!pi.pendingOverrides.empty()) {
            try {
                json pending = json::parse(pi.pendingOverrides);
                if (pending.contains("overrides"))       ref["overrides"]       = pending["overrides"];
                if (pending.contains("removedEntities")) ref["removedEntities"] = pending["removedEntities"];
            } catch (...) {}
        }
        return json{ { "prefabRef", std::move(ref) } };
    }

    const json* fileJ = LoadPrefabJson(pi.sourcePath, prefabCache);
    if (!fileJ) {
        spdlog::warn("Scene save: prefab '{}' unreadable — instance '{}' saved without override diff",
                     pi.sourcePath, scene.GetEntityName(root));
        return json{ { "prefabRef", std::move(ref) } };
    }

    std::unordered_map<uint64_t, const json*> prefabByLocal;
    for (const auto& ej : (*fileJ)["prefab"]["entities"])
        if (ej.contains("uuid")) prefabByLocal[ej["uuid"].get<uint64_t>()] = &ej;

    std::vector<entt::entity> subtree;
    CollectSubtree(scene, root, subtree);

    std::unordered_map<uint64_t, uint64_t> liveToLocal;
    std::unordered_set<uint64_t>           presentLocals;
    for (entt::entity e : subtree)
        if (auto* pc = reg.try_get<PrefabChildComponent>(e)) {
            liveToLocal[reg.get<IDComponent>(e).uuid] = pc->prefabUuid;
            presentLocals.insert(pc->prefabUuid);
        }

    json overrides = json::array();
    for (entt::entity e : subtree) {
        auto* pc = reg.try_get<PrefabChildComponent>(e);
        if (!pc) continue;                        // child added in-scene: saved as a plain entity
        auto pit = prefabByLocal.find(pc->prefabUuid);
        if (pit == prefabByLocal.end()) continue; // stamp not in the file (asset edited elsewhere)
        const json& fj = *pit->second;

        json lj = SerializeEntity(scene, e);
        RemapTargetUuids(lj, liveToLocal);

        json entry;
        json comps   = json::object();
        json removed = json::array();
        for (auto& [k, v] : lj.items()) {
            if (IsNonComponentKey(k)) continue;
            if (e == root && k == "transform") continue;   // stored at ref level
            if (!fj.contains(k) || fj[k] != v) comps[k] = v;
        }
        for (auto& [k, v] : fj.items()) {
            if (IsNonComponentKey(k)) continue;
            if (e == root && k == "transform") continue;
            if (!lj.contains(k)) removed.push_back(k);
        }
        if (e != root && scene.GetEntityName(e) != fj.value("name", ""))
            entry["name"] = scene.GetEntityName(e);
        if (!comps.empty())   entry["components"] = std::move(comps);
        if (!removed.empty()) entry["removed"]    = std::move(removed);
        if (!entry.empty()) {
            entry["target"] = pc->prefabUuid;
            overrides.push_back(std::move(entry));
        }
    }

    json removedEntities = json::array();
    for (auto& [localUuid, ej] : prefabByLocal)
        if (!presentLocals.count(localUuid)) removedEntities.push_back(localUuid);

    if (!overrides.empty())       ref["overrides"]       = std::move(overrides);
    if (!removedEntities.empty()) ref["removedEntities"] = std::move(removedEntities);
    return json{ { "prefabRef", std::move(ref) } };
}

// Instantiates a scene-level { "prefabRef": ... } node: prefab content first,
// then the stored root identity/transform, removed-entity deletions, and
// component overrides. Falls back to a placeholder entity that keeps the
// reference (and pending overrides) alive when the file is unreadable. The
// instance's live uuids are registered into `sceneCtx` so plain entities can
// resolve references into the instance's interior.
static entt::entity InstantiateSceneRef(Scene& scene, const json& refJ, DeserializeCtx& sceneCtx)
{
    auto& reg = scene.GetRegistry();
    std::string path       = refJ.value("path", "");
    uint64_t    storedUuid = refJ.value("uuid", (uint64_t)0);

    json fileJ;
    {
        std::ifstream f(path);
        if (f.is_open()) {
            try { fileJ = json::parse(f); }
            catch (...) { fileJ = json{}; }
            AssetPaths::ResolveAll(fileJ);
        }
    }

    entt::entity root = entt::null;
    std::unordered_map<uint64_t, entt::entity> localMap;
    if (fileJ.contains("prefab") && fileJ["prefab"].contains("entities"))
        root = InstantiatePrefabJson(scene, fileJ["prefab"], NormalizePrefabPath(path),
                                     storedUuid, &localMap);

    if (root == entt::null) {
        spdlog::error("Scene load: prefab '{}' missing or unreadable — created placeholder '{}'",
                      path, refJ.value("name", "Missing Prefab"));
        root = scene.CreateEntity(refJ.value("name", "Missing Prefab"));
        if (storedUuid) reg.get<IDComponent>(root).uuid = storedUuid;
        auto& pi      = reg.emplace_or_replace<PrefabInstanceComponent>(root);
        pi.sourcePath = path;
        pi.broken     = true;
        json pending;
        if (refJ.contains("overrides"))       pending["overrides"]       = refJ["overrides"];
        if (refJ.contains("removedEntities")) pending["removedEntities"] = refJ["removedEntities"];
        if (!pending.empty()) pi.pendingOverrides = pending.dump();
    }

    if (refJ.contains("name"))
        scene.SetEntityName(root, refJ["name"].get<std::string>());
    if (refJ.contains("transform")) {
        auto& tc       = reg.get<TransformComponent>(root);
        const auto& tj = refJ["transform"];
        tc.position = ToVec3(tj["position"]);
        tc.rotation = ToQuat(tj["rotation"]);
        tc.scale    = ToVec3(tj["scale"]);
        tc.eulerDegrees = tj.contains("eulerDegrees")
                        ? ToVec3(tj["eulerDegrees"])
                        : glm::degrees(glm::eulerAngles(tc.rotation));
    }

    if (!localMap.empty()) {
        if (refJ.contains("removedEntities"))
            for (const auto& lu : refJ["removedEntities"]) {
                auto it = localMap.find(lu.get<uint64_t>());
                // guard valid(): destroying a subtree already removed a child
                if (it != localMap.end() && it->second != root && reg.valid(it->second))
                    scene.DestroyEntity(it->second);
            }
        ApplyRefOverrides(scene, refJ, localMap);
    }

    for (auto& [local, e] : localMap)
        if (reg.valid(e))
            sceneCtx.uuidToEntity[reg.get<IDComponent>(e).uuid] = e;
    if (reg.valid(root))
        sceneCtx.uuidToEntity[reg.get<IDComponent>(root).uuid] = root;

    return root;
}

entt::entity PrefabSerializer::Instantiate(Scene& scene, const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        spdlog::error("Prefab instantiate: cannot open '{}'", path);
        return entt::null;
    }
    json root;
    try { root = json::parse(f); }
    catch (...) {
        spdlog::error("Prefab instantiate: '{}' is not valid JSON", path);
        return entt::null;
    }
    if (!root.contains("prefab") || !root["prefab"].contains("entities")) {
        spdlog::error("Prefab instantiate: '{}' has no prefab block", path);
        return entt::null;
    }
    AssetPaths::ResolveAll(root);
    return InstantiatePrefabJson(scene, root["prefab"], NormalizePrefabPath(path),
                                 /*forcedRootUuid=*/0, nullptr);
}

entt::entity PrefabSerializer::Instantiate(Scene& scene, const std::string& path,
                                           const glm::vec3& position)
{
    entt::entity root = Instantiate(scene, path);
    if (root != entt::null)
        scene.GetRegistry().get<TransformComponent>(root).position = position;
    return root;
}

int PrefabSerializer::SaveAndPropagate(Scene& scene, entt::entity root, const std::string& path)
{
    auto& reg = scene.GetRegistry();

    std::vector<entt::entity> targets;
    for (auto [e, pi] : scene.View<PrefabInstanceComponent>().each()) {
        if (e == root || pi.broken) continue;
        if (!SamePrefabPath(pi.sourcePath, path)) continue;
        if (!reg.all_of<PrefabChildComponent>(e)) {
            spdlog::warn("Prefab propagate: '{}' is a legacy baked instance — skipped "
                         "(Revert it to relink)", scene.GetEntityName(e));
            continue;
        }
        if (HasInstanceRefAncestor(scene, e)) continue;   // interior of an outer instance
        targets.push_back(e);
    }

    // Capture the siblings' override diffs BEFORE the file is overwritten:
    // diffing against the old content is what separates "inherited from the
    // previous prefab version" (must update) from "deliberate per-instance
    // edit" (must survive). Diffing after the save would freeze every stale
    // value in place as a spurious override.
    std::unordered_map<std::string, json> oldPrefabCache;
    std::vector<json>         refNodes;
    std::vector<entt::entity> parents;
    refNodes.reserve(targets.size());
    parents.reserve(targets.size());
    for (entt::entity e : targets) {
        refNodes.push_back(SerializePrefabRef(scene, e, oldPrefabCache));
        parents.push_back(reg.all_of<HierarchyComponent>(e)
                        ? reg.get<HierarchyComponent>(e).parent : entt::null);
    }

    if (!Save(scene, root, path)) return -1;

    int count = 0;
    for (size_t i = 0; i < targets.size(); ++i) {
        if (!reg.valid(targets[i])) continue;
        scene.DestroyEntity(targets[i]);
        DeserializeCtx ctx;
        entt::entity newRoot = InstantiateSceneRef(scene, refNodes[i]["prefabRef"], ctx);
        if (newRoot != entt::null && parents[i] != entt::null && reg.valid(parents[i]))
            scene.SetParent(newRoot, parents[i]);
        ++count;
    }
    return count;
}

entt::entity PrefabSerializer::Revert(Scene& scene, entt::entity instanceRoot)
{
    auto& reg = scene.GetRegistry();
    if (!reg.valid(instanceRoot) || !reg.all_of<PrefabInstanceComponent>(instanceRoot))
        return entt::null;

    // Minimal ref: no name/overrides, so the file's content (and root name)
    // comes back pristine; uuid and transform keep the instance's identity
    // and placement.
    json refJ;
    refJ["path"] = reg.get<PrefabInstanceComponent>(instanceRoot).sourcePath;
    refJ["uuid"] = reg.get<IDComponent>(instanceRoot).uuid;
    auto& tc = reg.get<TransformComponent>(instanceRoot);
    refJ["transform"] = {
        { "position",     JVec3(tc.position)     },
        { "rotation",     JQuat(tc.rotation)     },
        { "eulerDegrees", JVec3(tc.eulerDegrees) },
        { "scale",        JVec3(tc.scale)        }
    };
    entt::entity parent = reg.all_of<HierarchyComponent>(instanceRoot)
                        ? reg.get<HierarchyComponent>(instanceRoot).parent : entt::null;

    scene.DestroyEntity(instanceRoot);
    DeserializeCtx ctx;
    entt::entity newRoot = InstantiateSceneRef(scene, refJ, ctx);
    if (newRoot != entt::null && parent != entt::null && reg.valid(parent))
        scene.SetParent(newRoot, parent);
    return newRoot;
}

void PrefabSerializer::Unpack(Scene& scene, entt::entity instanceRoot)
{
    auto& reg = scene.GetRegistry();
    if (!reg.valid(instanceRoot)) return;
    std::vector<entt::entity> subtree;
    CollectSubtree(scene, instanceRoot, subtree);
    for (entt::entity e : subtree)
        reg.remove<PrefabChildComponent>(e);
    reg.remove<PrefabInstanceComponent>(instanceRoot);
}
