#pragma once
#include <glad/gl.h>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace Diamond {

struct RGTextureHandle {
    uint32_t id = 0;
    bool IsValid() const { return id != 0; }
};

struct RGTextureDesc {
    int             width;
    int             height;
    GLenum          internalFormat;           // e.g. GL_RGBA16F, GL_RGBA8, GL_DEPTH_COMPONENT
    bool            needsDepth  = false;      // attach a depth renderbuffer (for passes that depth-test)
    RGTextureHandle mrtPrimary  = {};         // if valid, borrow primary's FBO and attach at mrtSlot
    int             mrtSlot     = 1;          // which COLOR_ATTACHMENT to use (1, 2, 3 ...)
};

struct RGPass {
    std::string                  name;
    std::vector<RGTextureHandle> reads;
    std::vector<RGTextureHandle> writes;
    std::function<void()>        execute;

    RGPass& Read(RGTextureHandle h)              { reads.push_back(h);      return *this; }
    RGPass& Write(RGTextureHandle h)             { writes.push_back(h);     return *this; }
    RGPass& SetExecute(std::function<void()> fn) { execute = std::move(fn); return *this; }
};

class RenderGraph {
public:
    // Setup phase — declare resources and passes
    RGTextureHandle DeclareTexture(std::string_view name, const RGTextureDesc& desc);
    RGPass&         AddPass(std::string_view name);

    // Compile phase — topological sort + cull dead passes
    void Compile();

    // Execute phase — allocate transient GL resources, run passes in order
    void Execute();

    // Reset — frees all GL resources and clears passes/textures
    void Clear();

    // Resource accessors — valid after Execute() has run the allocating pass
    uint32_t GetTexture(RGTextureHandle h) const;
    uint32_t GetFBO(RGTextureHandle h)     const;

private:
    struct TextureEntry {
        std::string   name;
        RGTextureDesc desc;
        uint32_t      glTexture      = 0;
        uint32_t      glFBO          = 0;
        uint32_t      glDepthRBO     = 0;    // non-zero when desc.needsDepth is true
        bool          isMRTSecondary = false; // FBO is borrowed from primary — do not delete
    };

    std::vector<RGPass>       m_Passes;
    std::vector<TextureEntry> m_Textures;
    std::vector<int>          m_SortedIndices;
};

} // namespace Diamond
