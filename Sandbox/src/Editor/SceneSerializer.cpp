#include "SceneSerializer.h"
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <fstream>
#include <unordered_map>

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
#include "PhysicsMaterialAsset.h"
#include "AnimStateMachineAsset.h"
#include "MaterialAsset.h"
#include "AssetPipeline/AssetRegistry.h"
#include "RagdollAsset.h"
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

        entities.push_back(ej);
    }

    return json{ { "entities", entities } };
}

static bool FromJson(Scene& scene, const json& root)
{
    scene.Clear();

    std::unordered_map<std::string, std::shared_ptr<Font>> fontCache;

    // Textures/materials/models load through the asset registry (dedup across
    // the whole editor, not just this load). The registry holds only weak_ptrs
    // and mesh CPU data is dropped right after GPU upload, so pin every model
    // asset until the load finishes — entities sharing a file then parse it
    // once. Textures need no pin: the components/materials hold them strongly.
    std::vector<std::shared_ptr<void>> modelPins;

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
                if (auto meshAsset = Assets::Load<Assets::MeshAsset>(mpath)) {
                    modelPins.push_back(meshAsset);
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
                // Inline: material stored directly in the scene (legacy / default).
                mat = std::make_shared<PBRMaterial>();
                mat->AlbedoPath       = mj.value("albedoPath",       "");
                mat->NormalPath       = mj.value("normalPath",       "");
                mat->MetallicPath     = mj.value("metallicPath",     "");
                mat->RoughnessPath    = mj.value("roughnessPath",    "");
                mat->AOPath           = mj.value("aoPath",           "");
                mat->EmissivePath     = mj.value("emissivePath",     "");
                mat->EmissiveStrength = mj.value("emissiveStrength", 0.0f);
                mat->UVScale          = mj.value("uvScale",          1.0f);

                mat->Albedo    = Assets::Load<Texture>(mat->AlbedoPath);
                mat->Normal    = Assets::Load<Texture>(mat->NormalPath);
                mat->Metallic  = Assets::Load<Texture>(mat->MetallicPath);
                mat->Roughness = Assets::Load<Texture>(mat->RoughnessPath);
                mat->AO        = Assets::Load<Texture>(mat->AOPath);
                mat->Emissive  = Assets::Load<Texture>(mat->EmissivePath);
            }

            auto& mc          = reg.emplace<MeshComponent>(e, mesh, mat, bounds);
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

        if (ej.contains("canvas")) {
            const auto& cj = ej["canvas"];
            auto& cv = reg.emplace<CanvasComponent>(e);
            cv.scaleMode = (CanvasComponent::ScaleMode)cj.value("scaleMode", 1);
            if (cj.contains("referenceResolution"))
                cv.referenceResolution = ToVec2(cj["referenceResolution"]);
            cv.matchWidthHeight = cj.value("matchWidthHeight", 0.5f);
            cv.sortOrder        = cj.value("sortOrder", 0);
        }

        if (ej.contains("rectTransform")) {
            const auto& rj = ej["rectTransform"];
            auto& rt = reg.emplace<RectTransformComponent>(e);
            if (rj.contains("anchorMin")) rt.anchorMin = ToVec2(rj["anchorMin"]);
            if (rj.contains("anchorMax")) rt.anchorMax = ToVec2(rj["anchorMax"]);
            if (rj.contains("pivot"))     rt.pivot     = ToVec2(rj["pivot"]);
            if (rj.contains("position"))  rt.position  = ToVec2(rj["position"]);
            if (rj.contains("size"))      rt.size      = ToVec2(rj["size"]);
            rt.zOrder = rj.value("zOrder", 0);
        }

        if (ej.contains("uiImage")) {
            const auto& j = ej["uiImage"];
            auto& im = reg.emplace<UIImageComponent>(e);
            im.texturePath = j.value("texturePath", "");
            im.texture     = Assets::Load<Texture>(im.texturePath);
            if (j.contains("tint"))  im.tint  = ToVec4(j["tint"]);
            if (j.contains("uvMin")) im.uvMin = ToVec2(j["uvMin"]);
            if (j.contains("uvMax")) im.uvMax = ToVec2(j["uvMax"]);
        }

        if (ej.contains("uiText")) {
            const auto& j = ej["uiText"];
            auto& tx = reg.emplace<UITextComponent>(e);
            tx.text     = j.value("text", "");
            tx.fontPath = j.value("fontPath", "");
            tx.font     = LoadFontCached(tx.fontPath, fontCache);
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
            auto& pb = reg.emplace<UIProgressBarComponent>(e);
            pb.progress = j.value("progress", 0.5f);
            if (j.contains("backgroundColor")) pb.backgroundColor = ToVec4(j["backgroundColor"]);
            if (j.contains("fillColor"))       pb.fillColor       = ToVec4(j["fillColor"]);
            pb.fillTexturePath = j.value("fillTexturePath", "");
            pb.fillTexture     = Assets::Load<Texture>(pb.fillTexturePath);
            pb.direction       = (UIProgressBarComponent::Direction)j.value("direction", 0);
        }

        if (ej.contains("uiButton")) {
            const auto& j = ej["uiButton"];
            auto& bt = reg.emplace<UIButtonComponent>(e);
            if (j.contains("normalTint"))  bt.normalTint  = ToVec4(j["normalTint"]);
            if (j.contains("hoverTint"))   bt.hoverTint   = ToVec4(j["hoverTint"]);
            if (j.contains("pressedTint")) bt.pressedTint = ToVec4(j["pressedTint"]);
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
            auto& rb = reg.emplace<RigidBodyComponent>(e);
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
            auto& cc = reg.emplace<ConstraintComponent>(e);
            cc.type       = (ConstraintType)cj.value("type", 0);
            cc.targetUuid = cj.value("targetUuid", (uint64_t)0);
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
            auto& rag     = reg.emplace<RagdollComponent>(e);
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
                    modelPins.push_back(model);
                    auto& smc       = reg.emplace<SkinnedMeshComponent>(e);
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
                }
            }
        }

        if (ej.contains("animator")) {
            const auto& aj = ej["animator"];
            auto& anim   = reg.emplace<AnimatorComponent>(e);
            anim.clip    = aj.value("clip",    0);
            anim.time    = aj.value("time",    0.0f);
            anim.speed   = aj.value("speed",   1.0f);
            anim.loop    = aj.value("loop",    true);
            anim.playing = aj.value("playing", true);
        }

        if (ej.contains("ik")) {
            const auto& ikj = ej["ik"];
            auto& ik = reg.emplace<IKComponent>(e);
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
                        auto it = uuidToEntity.find(c["targetUuid"].get<uint64_t>());
                        if (it != uuidToEntity.end()) ch.targetEntity = it->second;
                    }
                    ik.chains.push_back(ch);
                }
            }
        }

        if (ej.contains("animStateMachine")) {
            const auto& smj = ej["animStateMachine"];
            auto& sm = reg.emplace<AnimStateMachineComponent>(e);
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
