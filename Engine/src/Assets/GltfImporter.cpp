#include "Assets/GltfImporter.h"
#include "Assets/ImageLoader.h"
#include "Renderer/TextureData.h"

#include <cgltf.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <spdlog/spdlog.h>

#include <unordered_map>
#include <filesystem>
#include <cstring>
#include <array>

namespace Diamond {

// ---------------------------------------------------------------------------
// Parsing helpers
// ---------------------------------------------------------------------------

// Parses a glTF/GLB file and loads its buffers. Returns nullptr (and logs) on
// failure; the caller owns the result and must cgltf_free it.
static cgltf_data* ParseFile(const std::string& path)
{
    cgltf_options options = {};
    cgltf_data*   data    = nullptr;

    cgltf_result result = cgltf_parse_file(&options, path.c_str(), &data);
    if (result != cgltf_result_success) {
        spdlog::error("GltfImporter: failed to parse '{}' (code {})", path, (int)result);
        return nullptr;
    }
    // Without this the accessor reads would dereference null buffer data.
    result = cgltf_load_buffers(&options, data, path.c_str());
    if (result != cgltf_result_success) {
        spdlog::error("GltfImporter: failed to load buffers for '{}' (code {})", path, (int)result);
        cgltf_free(data);
        return nullptr;
    }
    return data;
}

// Decomposes an affine matrix to TRS. Mirrored rotations (negative scale) are
// not detected — same limitation the rest of the pipeline has.
static void DecomposeTRS(const glm::mat4& m, glm::vec3& T, glm::quat& R, glm::vec3& S)
{
    T = glm::vec3(m[3]);
    glm::vec3 c0(m[0]), c1(m[1]), c2(m[2]);
    S = glm::vec3(glm::length(c0), glm::length(c1), glm::length(c2));
    glm::mat3 rot(c0 / S.x, c1 / S.y, c2 / S.z);
    R = glm::quat_cast(rot);
}

// A node's local rest transform, decomposed to TRS. glTF nodes carry either
// explicit T/R/S (the common case for joints) or a baked matrix; handle both.
static void NodeLocalTRS(const cgltf_node* node, glm::vec3& T, glm::quat& R, glm::vec3& S)
{
    T = glm::vec3(0.0f);
    R = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    S = glm::vec3(1.0f);

    if (node->has_matrix) {
        DecomposeTRS(glm::make_mat4(node->matrix), T, R, S);
        return;
    }
    if (node->has_translation) T = glm::vec3(node->translation[0], node->translation[1], node->translation[2]);
    // glTF stores quaternions xyzw; glm::quat takes (w, x, y, z).
    if (node->has_rotation)    R = glm::quat(node->rotation[3], node->rotation[0], node->rotation[1], node->rotation[2]);
    if (node->has_scale)       S = glm::vec3(node->scale[0], node->scale[1], node->scale[2]);
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

// Generates per-vertex tangents/bitangents from positions, normals and UVs when a
// glTF omits the (optional) TANGENT attribute — mirrors Assimp's CalcTangentSpace.
// Without this, a zero tangent makes the shader's TBN degenerate (NaN normal →
// black shading). Standard Lengyel accumulation + Gram-Schmidt orthonormalization.
static void GenerateTangents(MeshData& data)
{
    std::vector<glm::vec3> tan(data.Vertices.size(), glm::vec3(0.0f));

    for (size_t i = 0; i + 2 < data.Indices.size(); i += 3) {
        uint32_t i0 = data.Indices[i], i1 = data.Indices[i + 1], i2 = data.Indices[i + 2];
        const Vertex& v0 = data.Vertices[i0];
        const Vertex& v1 = data.Vertices[i1];
        const Vertex& v2 = data.Vertices[i2];

        glm::vec3 e1 = v1.Position - v0.Position;
        glm::vec3 e2 = v2.Position - v0.Position;
        glm::vec2 d1 = v1.TexCoords - v0.TexCoords;
        glm::vec2 d2 = v2.TexCoords - v0.TexCoords;

        float denom = d1.x * d2.y - d2.x * d1.y;
        if (glm::abs(denom) < 1e-8f) continue;
        float r = 1.0f / denom;
        glm::vec3 t = (e1 * d2.y - e2 * d1.y) * r;
        tan[i0] += t; tan[i1] += t; tan[i2] += t;
    }

    for (size_t i = 0; i < data.Vertices.size(); ++i) {
        Vertex& v = data.Vertices[i];
        glm::vec3 n = v.Normal;
        glm::vec3 t = tan[i];
        // Gram-Schmidt; fall back to an arbitrary basis if the accumulated tangent
        // is degenerate (e.g. a vertex with no valid UV-bearing triangle).
        t = t - n * glm::dot(n, t);
        if (glm::dot(t, t) < 1e-12f)
            t = glm::abs(n.x) < 0.99f ? glm::cross(n, glm::vec3(1, 0, 0))
                                      : glm::cross(n, glm::vec3(0, 1, 0));
        v.Tangent   = glm::normalize(t);
        v.Bitangent = glm::cross(n, v.Tangent);
    }
}

// Pulls one primitive into a MeshData. When `jointRemap` is non-null, also reads
// JOINTS_0 / WEIGHTS_0, remapping the skin-local joint indices into final
// skeleton bone indices.
static MeshData ProcessPrimitive(const cgltf_primitive* prim,
                                 const std::vector<int>* jointRemap)
{
    MeshData data;
    bool hasTangent = false;

    cgltf_size vertexCount = 0;
    for (cgltf_size i = 0; i < prim->attributes_count; ++i) {
        if (prim->attributes[i].type == cgltf_attribute_type_position) {
            vertexCount = prim->attributes[i].data->count;
            break;
        }
    }
    if (vertexCount == 0) return data;
    data.Vertices.resize(vertexCount);

    for (cgltf_size a = 0; a < prim->attributes_count; ++a) {
        const cgltf_attribute& attr = prim->attributes[a];
        const cgltf_accessor*  acc  = attr.data;

        switch (attr.type) {
            case cgltf_attribute_type_position:
                for (cgltf_size i = 0; i < vertexCount; ++i)
                    cgltf_accessor_read_float(acc, i, &data.Vertices[i].Position.x, 3);
                break;
            case cgltf_attribute_type_normal:
                for (cgltf_size i = 0; i < vertexCount; ++i)
                    cgltf_accessor_read_float(acc, i, &data.Vertices[i].Normal.x, 3);
                break;
            case cgltf_attribute_type_texcoord:
                // glTF UVs are top-left origin and our embedded images are decoded
                // unflipped (row 0 → t=0), so the two already align — no V flip,
                // unlike the Assimp/FBX path which needs aiProcess_FlipUVs.
                if (attr.index == 0)
                    for (cgltf_size i = 0; i < vertexCount; ++i)
                        cgltf_accessor_read_float(acc, i, &data.Vertices[i].TexCoords.x, 2);
                break;
            case cgltf_attribute_type_tangent:
                // glTF tangents are vec4: xyz + w handedness. Bitangent = cross(N,T)*w.
                hasTangent = true;
                for (cgltf_size i = 0; i < vertexCount; ++i) {
                    glm::vec4 t(0.0f);
                    cgltf_accessor_read_float(acc, i, &t.x, 4);
                    Vertex& v   = data.Vertices[i];
                    v.Tangent   = glm::vec3(t);
                    v.Bitangent = glm::cross(v.Normal, glm::vec3(t)) * t.w;
                }
                break;
            case cgltf_attribute_type_joints:
                if (jointRemap && attr.index == 0) {
                    for (cgltf_size i = 0; i < vertexCount; ++i) {
                        cgltf_uint j[4] = { 0, 0, 0, 0 };
                        cgltf_accessor_read_uint(acc, i, j, 4);
                        glm::ivec4& ids = data.Vertices[i].BoneIDs;
                        for (int k = 0; k < 4; ++k)
                            ids[k] = (j[k] < jointRemap->size()) ? (*jointRemap)[j[k]] : 0;
                    }
                }
                break;
            case cgltf_attribute_type_weights:
                if (jointRemap && attr.index == 0)
                    for (cgltf_size i = 0; i < vertexCount; ++i)
                        cgltf_accessor_read_float(acc, i, &data.Vertices[i].BoneWeights.x, 4);
                break;
            default:
                break;
        }
    }

    if (prim->indices) {
        data.Indices.resize(prim->indices->count);
        for (cgltf_size i = 0; i < prim->indices->count; ++i)
            data.Indices[i] = (uint32_t)cgltf_accessor_read_index(prim->indices, i);
    } else {
        data.Indices.resize(vertexCount);
        for (cgltf_size i = 0; i < vertexCount; ++i)
            data.Indices[i] = (uint32_t)i;
    }

    // glTF tangents are optional; synthesize them when absent so the TBN is valid.
    if (!hasTangent)
        GenerateTangents(data);

    return data;
}

// ---------------------------------------------------------------------------
// Skeleton
// ---------------------------------------------------------------------------

// Builds a Skeleton from a glTF skin. Bones are topologically sorted (parent
// before child) so the animation system can compute world matrices in one pass.
// Fills:
//   jointRemap[skinJointIndex] -> final bone index  (for vertex JOINTS_0)
//   nodeToBone[node]           -> final bone index  (for animation channels)
static Skeleton BuildSkeleton(const cgltf_skin* skin, cgltf_data* data,
                              std::vector<int>& jointRemap,
                              std::unordered_map<const cgltf_node*, int>& nodeToBone)
{
    Skeleton skel;
    const cgltf_size n = skin->joints_count;
    if (n == 0) return skel;

    // First pass, in skin order: read each joint's rest TRS + inverse bind, and a
    // node->skinIndex lookup so we can resolve parents.
    std::unordered_map<const cgltf_node*, int> nodeToSkin;
    nodeToSkin.reserve(n);
    for (cgltf_size i = 0; i < n; ++i)
        nodeToSkin[skin->joints[i]] = (int)i;

    std::vector<Bone> skinOrder(n);
    std::vector<int>  parentSkin(n, -1);
    for (cgltf_size i = 0; i < n; ++i) {
        const cgltf_node* node = skin->joints[i];
        Bone& b = skinOrder[i];
        b.name  = node->name ? node->name : ("bone" + std::to_string(i));
        NodeLocalTRS(node, b.localT, b.localR, b.localS);

        if (skin->inverse_bind_matrices) {
            float m[16];
            cgltf_accessor_read_float(skin->inverse_bind_matrices, i, m, 16);
            b.inverseBind = glm::make_mat4(m);
        }
        auto pit = node->parent ? nodeToSkin.find(node->parent) : nodeToSkin.end();
        parentSkin[i] = (pit != nodeToSkin.end()) ? pit->second : -1;
    }

    // Topological sort: emit a joint only once its parent has been emitted.
    jointRemap.assign(n, 0);
    std::vector<int>  order;
    std::vector<bool> placed(n, false);
    order.reserve(n);
    while (order.size() < n) {
        bool progress = false;
        for (cgltf_size i = 0; i < n; ++i) {
            if (placed[i]) continue;
            int p = parentSkin[i];
            if (p == -1 || placed[p]) {
                jointRemap[i] = (int)order.size();
                order.push_back((int)i);
                placed[i] = true;
                progress = true;
            }
        }
        if (!progress) { // cycle / malformed — emit the rest as roots
            for (cgltf_size i = 0; i < n; ++i)
                if (!placed[i]) { jointRemap[i] = (int)order.size(); order.push_back((int)i); placed[i] = true; }
        }
    }

    // Emit bones in sorted order with remapped parent indices.
    skel.bones.resize(n);
    for (cgltf_size newIdx = 0; newIdx < n; ++newIdx) {
        int oldIdx          = order[newIdx];
        Bone b              = skinOrder[oldIdx];
        int  oldParent      = parentSkin[oldIdx];
        b.parent            = (oldParent == -1) ? -1 : jointRemap[oldParent];
        skel.bones[newIdx]  = b;
        skel.nameToIndex[b.name] = (int)newIdx;
        nodeToBone[skin->joints[oldIdx]] = (int)newIdx;
    }

    (void)data;
    return skel;
}

// ---------------------------------------------------------------------------
// Animation
// ---------------------------------------------------------------------------

static void ReadAnimations(cgltf_data* data,
                           const std::unordered_map<const cgltf_node*, int>& nodeToBone,
                           std::vector<AnimationClip>& out)
{
    for (cgltf_size ai = 0; ai < data->animations_count; ++ai) {
        const cgltf_animation& anim = data->animations[ai];

        AnimationClip clip;
        clip.name = anim.name ? anim.name : ("clip" + std::to_string(ai));

        // One BoneChannel per animated bone; channels merge T/R/S samplers.
        std::unordered_map<int, BoneChannel> byBone;

        for (cgltf_size ci = 0; ci < anim.channels_count; ++ci) {
            const cgltf_animation_channel& ch = anim.channels[ci];
            if (!ch.target_node || !ch.sampler) continue;

            auto bit = nodeToBone.find(ch.target_node);
            if (bit == nodeToBone.end()) continue; // not a skeleton bone
            int boneIndex = bit->second;

            const cgltf_accessor* input  = ch.sampler->input;  // times
            const cgltf_accessor* output = ch.sampler->output; // values
            if (!input || !output) continue;
            const cgltf_size keys = input->count;

            BoneChannel& bc = byBone[boneIndex];
            bc.boneIndex = boneIndex;

            for (cgltf_size k = 0; k < keys; ++k) {
                float t = 0.0f;
                cgltf_accessor_read_float(input, k, &t, 1);
                if (t > clip.duration) clip.duration = t;

                switch (ch.target_path) {
                    case cgltf_animation_path_type_translation: {
                        glm::vec3 v(0.0f);
                        cgltf_accessor_read_float(output, k, &v.x, 3);
                        bc.positions.push_back({ t, v });
                        break;
                    }
                    case cgltf_animation_path_type_rotation: {
                        float q[4] = { 0, 0, 0, 1 };
                        cgltf_accessor_read_float(output, k, q, 4);
                        bc.rotations.push_back({ t, glm::quat(q[3], q[0], q[1], q[2]) });
                        break;
                    }
                    case cgltf_animation_path_type_scale: {
                        glm::vec3 v(1.0f);
                        cgltf_accessor_read_float(output, k, &v.x, 3);
                        bc.scales.push_back({ t, v });
                        break;
                    }
                    default:
                        break; // weights (morph targets) unsupported for now
                }
            }
        }

        clip.channels.reserve(byBone.size());
        for (auto& [idx, bc] : byBone) clip.channels.push_back(std::move(bc));
        out.push_back(std::move(clip));
    }
}

// ---------------------------------------------------------------------------
// Materials / textures
// ---------------------------------------------------------------------------

// Decodes a glTF image to raw pixels. Images live either embedded in the GLB's
// binary buffer (buffer_view) or as an external file (uri). Decoded unflipped to
// match the engine's texture convention. data: URIs are not handled yet.
static ImageData DecodeImage(const cgltf_image* image, const std::string& baseDir)
{
    if (!image) return {};
    if (image->buffer_view) {
        const cgltf_buffer_view* bv = image->buffer_view;
        const uint8_t* base = static_cast<const uint8_t*>(bv->buffer->data);
        if (!base) return {};
        return ImageLoader::LoadFromMemory(base + bv->offset, bv->size, false);
    }
    if (image->uri) {
        if (std::strncmp(image->uri, "data:", 5) == 0) {
            spdlog::warn("GltfImporter: data: URI images not supported yet");
            return {};
        }
        return ImageLoader::Load(baseDir + "/" + image->uri, false);
    }
    return {};
}

static std::shared_ptr<Texture> TexFrom(const ImageData& img)
{
    if (img.Pixels.empty()) return nullptr;
    return Texture::CreateFromPixels(img.Pixels.data(), img.Width, img.Height, img.Channels);
}

// Extracts a single channel of a multi-channel image into a fresh R8 texture.
static std::shared_ptr<Texture> SingleChannel(const ImageData& img, int channel)
{
    if (img.Pixels.empty() || img.Channels <= channel) return nullptr;
    std::vector<uint8_t> out((size_t)img.Width * img.Height);
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = img.Pixels[i * img.Channels + channel];
    return Texture::CreateFromPixels(out.data(), img.Width, img.Height, 1);
}

// Per-import texture dedup: level files share images heavily across materials
// (Sponza reuses one normal map across a dozen materials), so upload each image
// once per import instead of once per material referencing it. Decoded pixels
// are NOT held for the import's duration — a 4K level's set would be GBs — only
// a single-slot decode cache, which covers the one back-to-back reuse pattern
// (roughness then metallic split from the same packed MR image).
struct ImportCache {
    const cgltf_image* lastImage = nullptr;
    ImageData          lastData;
    std::unordered_map<const cgltf_image*, std::shared_ptr<Texture>>                fullTex;
    std::unordered_map<const cgltf_image*, std::array<std::shared_ptr<Texture>, 4>> channelTex;
};

static const ImageData& CachedImage(const cgltf_image* img, const std::string& baseDir, ImportCache& c)
{
    if (c.lastImage != img) {
        c.lastData  = DecodeImage(img, baseDir);
        c.lastImage = img;
    }
    return c.lastData;
}

// The external-file source path of an image, empty for embedded/data: images.
static std::string ImageFilePath(const cgltf_image* img, const std::string& baseDir)
{
    if (!img || !img->uri || std::strncmp(img->uri, "data:", 5) == 0) return {};
    return baseDir + "/" + img->uri;
}

// Full-RGB(A) use of an image. External (uri) images go through
// Texture::Create so the cooked-BCn cache applies and the material keeps a
// real source path (outPath — enables hot reload and path-based
// serialization); embedded GLB images decode from the buffer.
static std::shared_ptr<Texture> CachedTex(const cgltf_image* img, const std::string& baseDir,
                                          ImportCache& c, std::string* outPath = nullptr)
{
    if (!img) return nullptr;
    std::string filePath = ImageFilePath(img, baseDir);

    auto it = c.fullTex.find(img);
    if (it == c.fullTex.end()) {
        auto tex = !filePath.empty()
            ? Texture::Create(filePath, /*flipVertically=*/false)
            : TexFrom(CachedImage(img, baseDir, c));
        it = c.fullTex.emplace(img, std::move(tex)).first;
    }
    // Recorded even when the GPU upload failed/was skipped — the path is the
    // serializable source of truth; a later load can retry from it.
    if (outPath && !filePath.empty())
        *outPath = filePath;
    return it->second;
}

// Single-channel use (packed metallic/roughness, occlusion): always needs the
// pixels, so this path can't ride the cooked cache.
static std::shared_ptr<Texture> CachedChannel(const cgltf_image* img, int channel,
                                              const std::string& baseDir, ImportCache& c)
{
    if (!img) return nullptr;
    auto& slots = c.channelTex[img];
    if (!slots[channel])
        slots[channel] = SingleChannel(CachedImage(img, baseDir, c), channel);
    return slots[channel];
}

static std::shared_ptr<PBRMaterial> LoadMaterial(const cgltf_material* mat, const std::string& baseDir,
                                                 ImportCache& cache)
{
    if (!mat) return nullptr;
    auto out = std::make_shared<PBRMaterial>();

    out->Mode = mat->alpha_mode == cgltf_alpha_mode_blend ? AlphaMode::Blend
              : mat->alpha_mode == cgltf_alpha_mode_mask  ? AlphaMode::Mask
                                                          : AlphaMode::Opaque;
    if (mat->alpha_mode == cgltf_alpha_mode_mask)
        out->AlphaCutoff = mat->alpha_cutoff;   // cgltf defaults it to 0.5 per spec

    if (mat->has_pbr_metallic_roughness) {
        const cgltf_pbr_metallic_roughness& pbr = mat->pbr_metallic_roughness;
        out->BaseColorFactor = glm::vec4(pbr.base_color_factor[0], pbr.base_color_factor[1],
                                         pbr.base_color_factor[2], pbr.base_color_factor[3]);
        if (pbr.base_color_texture.texture)
            out->Albedo = CachedTex(pbr.base_color_texture.texture->image, baseDir, cache,
                                    &out->AlbedoPath);

        // glTF packs roughness in G and metallic in B of one texture; our material
        // uses separate single-channel maps, so split it.
        if (pbr.metallic_roughness_texture.texture) {
            const cgltf_image* mr = pbr.metallic_roughness_texture.texture->image;
            out->Roughness = CachedChannel(mr, 1, baseDir, cache);
            out->Metallic  = CachedChannel(mr, 2, baseDir, cache);
        }
    }
    if (mat->normal_texture.texture)
        out->Normal = CachedTex(mat->normal_texture.texture->image, baseDir, cache,
                                &out->NormalPath);
    if (mat->occlusion_texture.texture)
        out->AO = CachedChannel(mat->occlusion_texture.texture->image, 0, baseDir, cache);
    if (mat->emissive_texture.texture) {
        out->Emissive         = CachedTex(mat->emissive_texture.texture->image, baseDir, cache,
                                          &out->EmissivePath);
        out->EmissiveStrength = mat->has_emissive_strength
            ? mat->emissive_strength.emissive_strength : 1.0f;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

std::vector<MeshData> GltfImporter::Load(const std::string& path)
{
    cgltf_data* data = ParseFile(path);
    if (!data) return {};

    std::vector<MeshData> meshes;
    for (cgltf_size m = 0; m < data->meshes_count; ++m) {
        const cgltf_mesh& mesh = data->meshes[m];
        for (cgltf_size p = 0; p < mesh.primitives_count; ++p) {
            const cgltf_primitive& prim = mesh.primitives[p];
            if (prim.type != cgltf_primitive_type_triangles) continue;
            MeshData md = ProcessPrimitive(&prim, nullptr);
            if (!md.Vertices.empty()) meshes.push_back(std::move(md));
        }
    }

    cgltf_free(data);
    return meshes;
}

ImportedModel GltfImporter::LoadModel(const std::string& path)
{
    ImportedModel model;
    cgltf_data* data = ParseFile(path);
    if (!data) return model;

    // Single skin assumption: characters typically have one. The skin defines the
    // joint index space used by both JOINTS_0 and the inverse-bind matrices.
    std::vector<int>                             jointRemap;
    std::unordered_map<const cgltf_node*, int>   nodeToBone;
    const cgltf_skin* skin = (data->skins_count > 0) ? &data->skins[0] : nullptr;
    if (skin)
        model.skeleton = BuildSkeleton(skin, data, jointRemap, nodeToBone);

    const std::vector<int>*  remapPtr     = skin ? &jointRemap : nullptr;
    const cgltf_material*    firstMaterial = nullptr;
    for (cgltf_size m = 0; m < data->meshes_count; ++m) {
        const cgltf_mesh& mesh = data->meshes[m];
        for (cgltf_size p = 0; p < mesh.primitives_count; ++p) {
            const cgltf_primitive& prim = mesh.primitives[p];
            if (prim.type != cgltf_primitive_type_triangles) continue;
            if (!firstMaterial && prim.material) firstMaterial = prim.material;
            MeshData md = ProcessPrimitive(&prim, remapPtr);
            if (!md.Vertices.empty()) model.meshes.push_back(std::move(md));
        }
    }

    // Single-material assumption: load the first material the primitives reference.
    if (firstMaterial) {
        std::string baseDir = std::filesystem::path(path).parent_path().string();
        ImportCache cache;
        model.material = LoadMaterial(firstMaterial, baseDir, cache);
    }

    if (skin)
        ReadAnimations(data, nodeToBone, model.animations);

    cgltf_free(data);
    return model;
}

// Depth-first walk accumulating world transforms; emits one instance per node
// that carries geometry. Geometry-less intermediate nodes contribute only
// their transform (flattened away — no empty entities for them).
static void WalkSceneNode(const cgltf_node* node, const glm::mat4& parentWorld,
                          const std::unordered_map<const cgltf_primitive*, int>& primFlatIndex,
                          std::vector<SceneNodeInstance>& out)
{
    glm::vec3 T; glm::quat R; glm::vec3 S;
    NodeLocalTRS(node, T, R, S);
    glm::mat4 world = parentWorld
        * glm::translate(glm::mat4(1.0f), T)
        * glm::mat4_cast(R)
        * glm::scale(glm::mat4(1.0f), S);

    if (node->mesh) {
        SceneNodeInstance inst;
        inst.name = node->name ? node->name
                  : (node->mesh->name ? node->mesh->name : "Node");
        DecomposeTRS(world, inst.position, inst.rotation, inst.scale);
        for (cgltf_size p = 0; p < node->mesh->primitives_count; ++p) {
            auto it = primFlatIndex.find(&node->mesh->primitives[p]);
            if (it != primFlatIndex.end()) inst.primitives.push_back(it->second);
        }
        if (!inst.primitives.empty()) out.push_back(std::move(inst));
    }

    for (cgltf_size c = 0; c < node->children_count; ++c)
        WalkSceneNode(node->children[c], world, primFlatIndex, out);
}

ImportedScene GltfImporter::LoadScene(const std::string& path)
{
    ImportedScene out;
    cgltf_data* data = ParseFile(path);
    if (!data) return out;

    // Flat primitive enumeration in file order. The skip rules (non-triangles,
    // empty geometry) MUST match Load() exactly — meshSubIndex refers to the
    // same submesh whichever path a MeshComponent was populated through.
    std::unordered_map<const cgltf_primitive*, int> primFlatIndex;
    for (cgltf_size m = 0; m < data->meshes_count; ++m) {
        const cgltf_mesh& mesh = data->meshes[m];
        for (cgltf_size p = 0; p < mesh.primitives_count; ++p) {
            const cgltf_primitive& prim = mesh.primitives[p];
            if (prim.type != cgltf_primitive_type_triangles) continue;
            MeshData md = ProcessPrimitive(&prim, nullptr);
            if (md.Vertices.empty()) continue;
            primFlatIndex[&prim] = (int)out.meshes.size();
            out.meshes.push_back(std::move(md));
            out.primitiveMaterial.push_back(
                prim.material ? (int)(prim.material - data->materials) : -1);
        }
    }

    // Load only materials a surviving primitive references; unreferenced slots
    // stay null so primitiveMaterial can index by file material order.
    out.materials.resize(data->materials_count);
    out.materialNames.resize(data->materials_count);
    out.materialTransparent.assign(data->materials_count, 0);
    std::string baseDir = std::filesystem::path(path).parent_path().string();
    ImportCache cache;
    for (int mi : out.primitiveMaterial) {
        if (mi < 0 || out.materials[mi]) continue;
        const cgltf_material& mat = data->materials[mi];
        out.materials[mi]           = LoadMaterial(&mat, baseDir, cache);
        out.materialNames[mi]       = mat.name ? mat.name : ("Material " + std::to_string(mi));
        // Blend only: MASK materials stay on the deferred path (the G-buffer
        // shader alpha-tests them via the material's AlphaCutoff) — routing
        // them through sorted blending is what caused decal sorting artifacts.
        out.materialTransparent[mi] = (mat.alpha_mode == cgltf_alpha_mode_blend) ? 1 : 0;
    }

    // Node instances from the default scene (or the first one, or loose root
    // nodes — exporters vary).
    const cgltf_scene* sc = data->scene ? data->scene
                          : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
    if (sc) {
        for (cgltf_size i = 0; i < sc->nodes_count; ++i)
            WalkSceneNode(sc->nodes[i], glm::mat4(1.0f), primFlatIndex, out.nodes);
    } else {
        for (cgltf_size i = 0; i < data->nodes_count; ++i)
            if (!data->nodes[i].parent)
                WalkSceneNode(&data->nodes[i], glm::mat4(1.0f), primFlatIndex, out.nodes);
    }

    // Mesh-library file with no scene graph: expose everything under one
    // identity-transform instance so the import still produces entities.
    if (out.nodes.empty() && !out.meshes.empty()) {
        SceneNodeInstance inst;
        inst.name = std::filesystem::path(path).stem().string();
        inst.primitives.resize(out.meshes.size());
        for (int i = 0; i < (int)out.meshes.size(); ++i) inst.primitives[i] = i;
        out.nodes.push_back(std::move(inst));
    }

    cgltf_free(data);
    return out;
}

bool GltfImporter::HasSkeleton(const std::string& path)
{
    // JSON-structure parse only — no cgltf_load_buffers, so this stays cheap
    // enough for the content browser to call per glTF file.
    cgltf_options options = {};
    cgltf_data*   data    = nullptr;
    if (cgltf_parse_file(&options, path.c_str(), &data) != cgltf_result_success)
        return false;
    bool skinned = data->skins_count > 0;
    cgltf_free(data);
    return skinned;
}

} // namespace Diamond
