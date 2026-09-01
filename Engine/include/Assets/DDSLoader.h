#pragma once

#include "Renderer/RHI/RHIEnums.h"

#include <cstdint>
#include <string>
#include <vector>

namespace Diamond {

// A parsed cooked .dds: the block-compressed payload exactly as it uploads
// (every mip tightly packed, largest first) plus the geometry the RHI needs.
// See Docs/asset-pipeline-design.md §4.
struct DDSData {
    uint32_t             Width    = 0;
    uint32_t             Height   = 0;
    uint32_t             MipCount = 0;
    RHIFormat            Format   = RHIFormat::Undefined;
    std::vector<uint8_t> Payload;

    bool IsValid() const { return Format != RHIFormat::Undefined && !Payload.empty(); }
};

class DDSLoader {
public:
    // Parses a cooked .dds. Accepts only what TextureCooker's texconv shell-out
    // emits: DX10-header BC7/BC5/BC4_UNORM 2D single-layer textures (usage
    // picks the format — see TextureCooker::ClassifyUsage). Anything else
    // (legacy header, other formats, cubemaps/arrays) returns an invalid
    // DDSData and the caller falls back to the source-image path.
    static DDSData Load(const std::string& path);

    // Assets/X/Y.png -> Assets/Cache/X/Y.dds (works on the registry's absolute
    // normalized paths too: "Cache" is inserted after the last "Assets" path
    // component). Empty if the path has no Assets component -- only project
    // assets are cooked.
    static std::string CookedPathFor(const std::string& sourcePath);

    // Derived single-channel texture produced from a packed source image.
    // channel is 0/1/2/3 for R/G/B/A; e.g. Foo.png + 1 maps to
    // Assets/Cache/.../Foo.g.dds. These files let glTF metallic/roughness/AO
    // avoid decoding and splitting the source image at runtime.
    static std::string CookedChannelPathFor(const std::string& sourcePath, uint32_t channel);

    // One-call cooked lookup for a source image: resolves the cooked path,
    // requires it to be at least as new as the source (mtime -- a stale cooked
    // file is ignored until re-cooked, so hot reload composes: an edited PNG
    // loads uncooked immediately), then parses it. Invalid DDSData on any miss.
    static DDSData LoadCookedFor(const std::string& sourcePath);

    // Cooked-channel equivalent of LoadCookedFor. Freshness is checked against
    // the original packed source image.
    static DDSData LoadCookedChannelFor(const std::string& sourcePath, uint32_t channel);
};

} // namespace Diamond
