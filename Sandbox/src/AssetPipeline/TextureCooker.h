#pragma once

#include <string>

// Offline BC7 cooker: shells out to texconv (DirectXTex) the same way the
// Vulkan backend's shader hot reload shells out to glslangValidator (see
// Engine/src/Platform/Vulkan/Passes/VulkanPassCommon.h). Sandbox-owned because
// it's an editor/dev-time tool, not something the engine needs at runtime.
// See Docs/asset-pipeline-design.md §4, slice 2.
namespace Diamond::TextureCooker {

// Filename-convention fallback classification for standalone images. CookAll
// uses exact glTF material-slot semantics when an image is referenced by a
// glTF — Color -> BC7, Normal -> BC5, scalar channels -> derived BC4.
enum class Usage { Color, Normal, Mask };

Usage ClassifyUsage(const std::string& sourcePath);

// Cooks one source image into Assets/Cache/.../Name.dds if the cooked file is
// missing or older than the source (mtime — same staleness rule
// DDSLoader::LoadCookedFor uses to read it back). Note: staleness is mtime-only
// — if a file was already cooked to a different format under an older cooker
// (e.g. a normal map hand-cooked as BC7 before this usage mapping existed),
// it reads as "fresh" and won't be recooked until its source changes. Delete
// Assets/Cache to force a full recook after a format-mapping change.
// Returns true only if a cook actually ran and texconv reported success.
bool CookOne(const std::string& sourcePath);

// Walks 'root' (Assets/ if empty), scans .gltf material slots, then cooks source
// images (.png/.tga/.jpg/.jpeg). Packed glTF roughness(G), metallic(B), and
// occlusion(R) become separately named BC4 DDS files. Returns the number of
// output files that were (re)cooked.
int CookAll(const std::string& root = {});

} // namespace Diamond::TextureCooker
