// Headless validation tool for GltfImporter::LoadModel. Parsing is pure CPU (no
// OpenGL), so this runs without a window. Usage: GltfDump <file.glb> [...]
#include "Assets/GltfImporter.h"
#include "Animation/AnimationSampler.h"

#include <glm/glm.hpp>
#include <cstdio>
#include <cmath>

using namespace Diamond;

// Largest absolute deviation of any palette matrix from the identity.
static float MaxDeviationFromIdentity(const std::vector<glm::mat4>& palette)
{
    float worst = 0.0f;
    for (const auto& m : palette)
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r) {
                float expected = (c == r) ? 1.0f : 0.0f;
                worst = std::max(worst, std::fabs(m[c][r] - expected));
            }
    return worst;
}

// True if every matrix element is finite (no NaN/inf).
static bool AllFinite(const std::vector<glm::mat4>& palette)
{
    for (const auto& m : palette)
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                if (!std::isfinite(m[c][r])) return false;
    return true;
}

static void Dump(const char* path)
{
    printf("\n=== %s ===\n", path);
    ImportedModel model = GltfImporter::LoadModel(path);

    // Geometry
    size_t totalVerts = 0, totalTris = 0, skinnedVerts = 0;
    for (const auto& m : model.meshes) {
        totalVerts += m.Vertices.size();
        totalTris  += m.Indices.size() / 3;
        for (const auto& v : m.Vertices) {
            float w = v.BoneWeights.x + v.BoneWeights.y + v.BoneWeights.z + v.BoneWeights.w;
            if (w > 0.0001f) ++skinnedVerts;
        }
    }
    printf("Meshes: %zu  |  Vertices: %zu  |  Triangles: %zu  |  Skinned verts: %zu\n",
           model.meshes.size(), totalVerts, totalTris, skinnedVerts);

    // Skeleton
    printf("Bones: %zu\n", model.skeleton.bones.size());
    for (size_t i = 0; i < model.skeleton.bones.size(); ++i) {
        const Bone& b = model.skeleton.bones[i];
        // Verify the topological invariant: parent index must precede this bone.
        const char* warn = (b.parent >= (int)i) ? "  <-- PARENT NOT BEFORE CHILD!" : "";
        printf("  [%2zu] %-24s parent=%-3d%s\n", i, b.name.c_str(), b.parent, warn);
    }

    // Animations
    printf("Animations: %zu\n", model.animations.size());
    for (const auto& clip : model.animations) {
        size_t pos = 0, rot = 0, scl = 0;
        for (const auto& ch : clip.channels) { pos += ch.positions.size(); rot += ch.rotations.size(); scl += ch.scales.size(); }
        printf("  '%s'  duration=%.3fs  channels=%zu  keys(T/R/S)=%zu/%zu/%zu\n",
               clip.name.c_str(), clip.duration, clip.channels.size(), pos, rot, scl);
    }

    if (model.skeleton.bones.empty()) return;

    // Sampler validation. Bind pose must be ~identity (world == inverse of the
    // inverse-bind), and any sampled pose must stay finite.
    std::vector<glm::mat4> palette;
    ComputeBindPose(model.skeleton, palette);
    printf("Bind-pose max deviation from identity: %.6f  (%s)\n",
           MaxDeviationFromIdentity(palette),
           MaxDeviationFromIdentity(palette) < 1e-3f ? "OK" : "SUSPECT");

    for (const auto& clip : model.animations) {
        bool finite = true;
        for (int s = 0; s <= 8; ++s) { // sample across the clip
            SamplePose(model.skeleton, clip, clip.duration * (s / 8.0f), palette);
            finite = finite && AllFinite(palette);
        }
        printf("Sampled '%s' across duration: %s\n", clip.name.c_str(),
               finite ? "all finite OK" : "NON-FINITE!");
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) { printf("usage: %s <file.glb> [...]\n", argv[0]); return 1; }
    for (int i = 1; i < argc; ++i) Dump(argv[i]);
    return 0;
}
