#include "Renderer/RenderGraph.h"
#include <glad/gl.h>
#include <spdlog/spdlog.h>
#include <queue>

namespace Diamond {

static void deriveFormatAndType(GLenum internal, GLenum& baseFormat, GLenum& type)
{
    switch (internal) {
        case GL_R8:          baseFormat = GL_RED;            type = GL_UNSIGNED_BYTE; break;
        case GL_R16F:        baseFormat = GL_RED;            type = GL_FLOAT;         break;
        case GL_R32F:        baseFormat = GL_RED;            type = GL_FLOAT;         break;
        case GL_RG8:         baseFormat = GL_RG;             type = GL_UNSIGNED_BYTE; break;
        case GL_RG16F:       baseFormat = GL_RG;             type = GL_FLOAT;         break;
        case GL_RGB16F:      baseFormat = GL_RGB;            type = GL_FLOAT;         break;
        case GL_RGBA8:       baseFormat = GL_RGBA;           type = GL_UNSIGNED_BYTE; break;
        case GL_RGBA16F:     baseFormat = GL_RGBA;           type = GL_FLOAT;         break;
        case GL_RGBA32F:     baseFormat = GL_RGBA;           type = GL_FLOAT;         break;
        case GL_DEPTH_COMPONENT:
        case GL_DEPTH_COMPONENT24:
        case GL_DEPTH_COMPONENT32F:
                             baseFormat = GL_DEPTH_COMPONENT; type = GL_FLOAT;        break;
        default:             baseFormat = GL_RGBA;           type = GL_FLOAT;         break;
    }
}

GLRGTextureHandle RenderGraph::DeclareTexture(std::string_view name, const GLRGTextureDesc& desc)
{
    m_Textures.push_back(TextureEntry{ std::string(name), desc, 0, 0, 0 });
    // id is 1-based so that id=0 stays the "invalid" sentinel (matches IsValid())
    return GLRGTextureHandle{ static_cast<uint32_t>(m_Textures.size()) };
}

GLRGPass& RenderGraph::AddPass(std::string_view name)
{
    m_Passes.push_back(GLRGPass{ std::string(name) });
    return m_Passes.back();
}

uint32_t RenderGraph::GetTexture(GLRGTextureHandle h) const
{
    return m_Textures[h.id - 1].glTexture;
}

uint32_t RenderGraph::GetFBO(GLRGTextureHandle h) const
{
    return m_Textures[h.id - 1].glFBO;
}

// Compile phase — topological sort + cull dead passes
void RenderGraph::Compile()
{
    // 1. Build a write map- texture handle id and index of pass that writes to it

    // -1 means no passes write to this texture 
    std::vector<int> writerMap(m_Textures.size() + 1, -1);

    for (int i = 0; i < (int)m_Passes.size(); ++i)
    {
        for (const GLRGTextureHandle& h: m_Passes[i].writes)
        {
            writerMap[h.id] = i;
        }
    }

    // 2. Topological sort using Kahn's algorithm

    // in-degree[i] = number of passes that must run before pass i
    std::vector<int> inDegree(m_Passes.size(), 0);

    for (int i = 0; i < (int)m_Passes.size(); ++i)
    {
        for (const GLRGTextureHandle& h : m_Passes[i].reads)
        {
            if (writerMap[h.id] != -1) // someone writes this texture, so pass i depends on them
                inDegree[i]++;
        }
    }

    // Seed the queue with every pass that has no dependencies
    std::queue<int> ready;
    for (int i = 0; i < (int)m_Passes.size(); ++i)
    {
        if (inDegree[i] == 0)
            ready.push(i);
    }

    std::vector<int> sorted;
    sorted.reserve(m_Passes.size());

    while (!ready.empty())
    {
        int i = ready.front();
        ready.pop();
        sorted.push_back(i);

        // Pass i is done — decrement in-degree of every pass that reads what i writes
        for (const GLRGTextureHandle& written : m_Passes[i].writes)
        {
            for (int j = 0; j < (int)m_Passes.size(); ++j)
            {
                for (const GLRGTextureHandle& r : m_Passes[j].reads)
                {
                    if (r.id == written.id)
                    {
                        if (--inDegree[j] == 0)
                            ready.push(j); // j's last dependency is satisfied
                    }
                }
            }
        }
    }

    if ((int)sorted.size() != (int)m_Passes.size())
        spdlog::error("RenderGraph::Compile — cycle detected in pass dependencies!");

    // 3. Dead pass culling — walk sorted in reverse, mark alive passes
    // Sink passes (no writes) always alive — they write to the backbuffer
    std::vector<int> alive(m_Passes.size(), 0); // indexed by pass index, not sorted position

    for (int i = (int)sorted.size() - 1; i >= 0; --i)
    {
        int passIdx = sorted[i];

        if (m_Passes[passIdx].writes.empty())
            alive[passIdx] = 1; // sink pass — always alive

        if (alive[passIdx])
        {
            // Any pass that feeds an alive pass is also alive
            for (const GLRGTextureHandle& h : m_Passes[passIdx].reads)
            {
                int writerIdx = writerMap[h.id];
                if (writerIdx != -1)
                    alive[writerIdx] = 1;
            }
        }
    }

    // 4. Store final execution order — only alive passes, in sorted order
    m_SortedIndices.clear();
    for (int passIdx : sorted)
    {
        if (alive[passIdx])
            m_SortedIndices.push_back(passIdx);
    }

}

// Execute phase — allocate transient GL resources, run passes in order
void RenderGraph::Execute()
{
    // 1. Compute the last reader for each texture
    std::vector<int> last_reader(m_Textures.size() + 1, -1);

    for (int i = 0; i < (int)m_SortedIndices.size(); ++i)
    {
        int passIdx = m_SortedIndices[i];

        for (int j = 0; j < (int)m_Passes[passIdx].reads.size(); ++j)
        {
            last_reader[m_Passes[passIdx].reads[j].id] = passIdx; // keeps getting overwritten until the last reader
        }

    } 

    // 2. Run each alive pass in sorted order
    for (int i = 0; i < (int)m_SortedIndices.size(); ++i)
    {
        int passIdx = m_SortedIndices[i];
        GLRGPass& pass = m_Passes[passIdx];

        // Allocate GL texture + FBO for each write target that doesn't exist yet
        for (const GLRGTextureHandle& h : pass.writes)
        {
            TextureEntry& entry = m_Textures[h.id - 1]; // -1 because ids are 1-based
            if (entry.glTexture != 0)
                continue;

            // --- MRT secondary: borrow primary's FBO and attach at mrtSlot ---
            if (entry.desc.mrtPrimary.IsValid())
            {
                TextureEntry& primary = m_Textures[entry.desc.mrtPrimary.id - 1];

                GLenum baseFormat, dataType;
                deriveFormatAndType(entry.desc.internalFormat, baseFormat, dataType);

                glGenTextures(1, &entry.glTexture);
                glBindTexture(GL_TEXTURE_2D, entry.glTexture);
                glTexImage2D(GL_TEXTURE_2D, 0, entry.desc.internalFormat,
                             entry.desc.width, entry.desc.height,
                             0, baseFormat, dataType, nullptr);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                glBindFramebuffer(GL_FRAMEBUFFER, primary.glFBO);
                glFramebufferTexture2D(GL_FRAMEBUFFER,
                                       GL_COLOR_ATTACHMENT0 + entry.desc.mrtSlot,
                                       GL_TEXTURE_2D, entry.glTexture, 0);

                // Rebuild draw buffers to include all slots attached so far
                int maxSlot = entry.desc.mrtSlot;
                for (const TextureEntry& e : m_Textures)
                    if (e.desc.mrtPrimary.id == entry.desc.mrtPrimary.id && e.glTexture != 0)
                        maxSlot = std::max(maxSlot, e.desc.mrtSlot);

                std::vector<GLenum> drawBufs;
                drawBufs.reserve(maxSlot + 1);
                for (int s = 0; s <= maxSlot; ++s)
                    drawBufs.push_back(GL_COLOR_ATTACHMENT0 + s);
                glDrawBuffers(static_cast<GLsizei>(drawBufs.size()), drawBufs.data());

                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                entry.glFBO          = primary.glFBO; // borrowed — do not delete
                entry.isMRTSecondary = true;
                continue;
            }

            GLenum baseFormat, dataType;
            deriveFormatAndType(entry.desc.internalFormat, baseFormat, dataType);
            bool isDepth = (baseFormat == GL_DEPTH_COMPONENT);

            glGenTextures(1, &entry.glTexture);
            glBindTexture(GL_TEXTURE_2D, entry.glTexture);
            glTexImage2D(GL_TEXTURE_2D, 0, entry.desc.internalFormat,
                         entry.desc.width, entry.desc.height,
                         0, baseFormat, dataType, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

            glGenFramebuffers(1, &entry.glFBO);
            glBindFramebuffer(GL_FRAMEBUFFER, entry.glFBO);
            if (isDepth)
            {
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                       GL_TEXTURE_2D, entry.glTexture, 0);
                glDrawBuffer(GL_NONE);
                glReadBuffer(GL_NONE);
            }
            else
            {
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                       GL_TEXTURE_2D, entry.glTexture, 0);

                if (entry.desc.needsDepth)
                {
                    glGenRenderbuffers(1, &entry.glDepthRBO);
                    glBindRenderbuffer(GL_RENDERBUFFER, entry.glDepthRBO);
                    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24,
                                          entry.desc.width, entry.desc.height);
                    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                              GL_RENDERBUFFER, entry.glDepthRBO);
                }
            }
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        // Run the pass
        if (pass.execute)
            pass.execute();

        // 3. Free textures whose last reader was this pass
        for (int t = 0; t < (int)m_Textures.size(); ++t)
        {
            uint32_t id = static_cast<uint32_t>(t + 1);
            if (last_reader[id] != passIdx)
                continue;

            TextureEntry& entry = m_Textures[t];
            if (entry.glDepthRBO) { glDeleteRenderbuffers(1, &entry.glDepthRBO); entry.glDepthRBO = 0; }
            if (entry.glTexture)  { glDeleteTextures(1,      &entry.glTexture);  entry.glTexture  = 0; }
            if (entry.glFBO && !entry.isMRTSecondary) { glDeleteFramebuffers(1, &entry.glFBO); entry.glFBO = 0; }
            else entry.glFBO = 0;
        }
    }
}

// Reset — frees all GL resources and clears passes/textures
void RenderGraph::Clear()
{
    for (TextureEntry& entry : m_Textures)
    {
        if (entry.glDepthRBO) { glDeleteRenderbuffers(1, &entry.glDepthRBO); entry.glDepthRBO = 0; }
        if (entry.glTexture)  { glDeleteTextures(1,      &entry.glTexture);  entry.glTexture  = 0; }
        if (entry.glFBO && !entry.isMRTSecondary) { glDeleteFramebuffers(1, &entry.glFBO); entry.glFBO = 0; }
        else entry.glFBO = 0;
    }
    m_Passes.clear();
    m_Textures.clear();
    m_SortedIndices.clear();
}

} // namespace Diamond
