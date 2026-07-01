#pragma once
#include <glad/gl.h>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace Diamond {

// NOTE: these types are prefixed GLRG (GL render graph) because the RHI render
// graph (Renderer/RHI/RHIRenderGraph.h) defines its own RGPass/RGTextureHandle/
// RGTextureDesc with different layouts. Sharing the names was an ODR violation:
// any binary linking both graphs had the two RGPass definitions' identical-named
// implicit members (move ctor/dtor, vector<RGPass> machinery) COMDAT-folded to
// one layout's code, corrupting the other graph's passes.
struct GLRGTextureHandle {
    uint32_t id = 0;
    bool IsValid() const { return id != 0; }
};

struct GLRGTextureDesc {
    int             width;
    int             height;
    GLenum          internalFormat;           // e.g. GL_RGBA16F, GL_RGBA8, GL_DEPTH_COMPONENT
    bool            needsDepth  = false;      // attach a depth renderbuffer (for passes that depth-test)
    GLRGTextureHandle mrtPrimary  = {};         // if valid, borrow primary's FBO and attach at mrtSlot
    int             mrtSlot     = 1;          // which COLOR_ATTACHMENT to use (1, 2, 3 ...)
};

struct GLRGPass {
    std::string                  name;
    std::vector<GLRGTextureHandle> reads;
    std::vector<GLRGTextureHandle> writes;
    std::function<void()>        execute;

    GLRGPass& Read(GLRGTextureHandle h)              { reads.push_back(h);      return *this; }
    GLRGPass& Write(GLRGTextureHandle h)             { writes.push_back(h);     return *this; }
    GLRGPass& SetExecute(std::function<void()> fn) { execute = std::move(fn); return *this; }
};

class RenderGraph {
public:
    // Setup phase — declare resources and passes
    GLRGTextureHandle DeclareTexture(std::string_view name, const GLRGTextureDesc& desc);
    GLRGPass&         AddPass(std::string_view name);

    // Compile phase — topological sort + cull dead passes
    void Compile();

    // Execute phase — allocate transient GL resources, run passes in order
    void Execute();

    // Reset — frees all GL resources and clears passes/textures
    void Clear();

    // Resource accessors — valid after Execute() has run the allocating pass
    uint32_t GetTexture(GLRGTextureHandle h) const;
    uint32_t GetFBO(GLRGTextureHandle h)     const;

private:
    struct TextureEntry {
        std::string   name;
        GLRGTextureDesc desc;
        uint32_t      glTexture      = 0;
        uint32_t      glFBO          = 0;
        uint32_t      glDepthRBO     = 0;    // non-zero when desc.needsDepth is true
        bool          isMRTSecondary = false; // FBO is borrowed from primary — do not delete
    };

    std::vector<GLRGPass>       m_Passes;
    std::vector<TextureEntry> m_Textures;
    std::vector<int>          m_SortedIndices;
};

} // namespace Diamond
