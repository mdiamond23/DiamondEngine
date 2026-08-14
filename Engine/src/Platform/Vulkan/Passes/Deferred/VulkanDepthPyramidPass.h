#pragma once

#include "Renderer/RHI/RHIRenderGraph.h"

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace Diamond {

class RHIDevice;
class RHIShader;
class RHIPipeline;
class RHIResourceSet;

// Linear view-depth pyramid over the G-buffer, built for GTAO's horizon march
// (and a ready input for a future HiZ screen trace).
//
// Level 0 extracts -gViewPos.z into R16F; each level after is a 2x2 MIN
// downsample of the previous one. Two wins over marching gViewPos directly:
// 2 bytes per tap instead of 8, and wide strides read a small cache-resident
// level instead of thrashing a full-res RGBA16F target.
//
// LEVELS ARE SEPARATE TEXTURES, not mips of one image: the RHI has no per-mip
// views for render targets (RHITextureDesc::generateMips is upload-only), and
// four extra bindings are far cheaper than adding mip plumbing to the RHI.
//
// One pipeline, one descriptor set per level — the extract and the downsample
// share a shader and differ only by a push-constant mode, so the binding layout
// is identical for every level.
class VulkanDepthPyramidPass {
public:
    // Levels beyond this stop helping: GTAO's stride caps at MAX_RADIUS_PIXELS
    // / steps, which lands inside level 3 for any sane step count.
    static constexpr int kLevels = 4;

    VulkanDepthPyramidPass(RHIDevice* device, const std::string& shaderDir,
                           uint32_t width, uint32_t height);
    ~VulkanDepthPyramidPass();

    // Dimensions of a level, halving each time and never dropping below 1.
    static uint32_t LevelSize(uint32_t base, int level) {
        uint32_t v = base >> level;
        return v > 0 ? v : 1u;
    }

    // Registers one compute pass per level. 'levels' must be caller-declared
    // R16F storage textures sized by LevelSize(); 'viewPos' is the G-buffer
    // position target level 0 is extracted from.
    void AddToGraph(RHIRenderGraph& graph, RGTextureHandle viewPos,
                    const std::array<RGTextureHandle, kLevels>& levels);

private:
    struct Push {
        glm::ivec2 dstSize;
        int        mode;   // 0 = extract from gViewPos, 1 = min-downsample
    };

    RHIDevice* m_Device;
    uint32_t   m_Width;
    uint32_t   m_Height;

    std::unique_ptr<RHIShader>   m_Comp;
    std::unique_ptr<RHIPipeline> m_Pipeline;
    std::array<std::unique_ptr<RHIResourceSet>, kLevels> m_Sets;
};

} // namespace Diamond
