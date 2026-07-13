#include "Platform/OpenGL/Resources/OpenGLTexture.h"

#include <glad/gl.h>
#include "Assets/ImageLoader.h"
#include "Assets/DDSLoader.h"
#include "Profiling/GLRendererStats.h"
#include <spdlog/spdlog.h>

namespace Diamond {

namespace {

// Result of decoding + uploading a source file into a fresh GL texture. id==0
// means decode/upload failed — caller must not touch GL state with it.
struct GLUpload {
    uint32_t id       = 0;
    uint32_t width    = 0;
    uint32_t height   = 0;
    uint64_t byteSize = 0;
};

// Cooked BCn fast path (see Docs/asset-pipeline-design.md §4): uploads the
// pre-baked mip chain untouched with glCompressedTexImage2D, no decode, no
// glGenerateMipmap. BC7 (albedo/emissive) needs GL 4.2 BPTC; BC5/BC4
// (normals/masks) are RGTC, core since GL 3.0 — fine even on macOS's 4.1 cap.
// BPTC's macOS gap is a non-issue in practice since texconv is Windows-only
// and Assets/Cache is gitignored/per-machine, so no cooked BC7 files exist to
// hit that case there — the PNG path below stays load-bearing on macOS.
GLenum InternalFormatFor(RHIFormat format)
{
    switch (format) {
        case RHIFormat::BC7: return GL_COMPRESSED_RGBA_BPTC_UNORM;
        case RHIFormat::BC5: return GL_COMPRESSED_RG_RGTC2;
        case RHIFormat::BC4: return GL_COMPRESSED_RED_RGTC1;
        default:             return 0;
    }
}

GLUpload UploadDDS(const DDSData& dds)
{
    const GLenum internalFormat = InternalFormatFor(dds.Format);
    if (internalFormat == 0) {
        spdlog::warn("OpenGLTexture: cooked DDS has an unsupported format, falling back to source");
        return {};
    }

    GLUpload up;
    glGenTextures(1, &up.id);
    glBindTexture(GL_TEXTURE_2D, up.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, static_cast<GLint>(dds.MipCount) - 1);

    uint64_t offset = 0;
    for (uint32_t mip = 0; mip < dds.MipCount; ++mip) {
        const uint32_t w = dds.Width  >> mip ? dds.Width  >> mip : 1;
        const uint32_t h = dds.Height >> mip ? dds.Height >> mip : 1;
        const uint64_t levelSize = RHIFormatLevelSize(dds.Format, w, h);
        glCompressedTexImage2D(GL_TEXTURE_2D, static_cast<GLint>(mip), internalFormat,
                                static_cast<GLsizei>(w), static_cast<GLsizei>(h), 0,
                                static_cast<GLsizei>(levelSize), dds.Payload.data() + offset);
        offset += levelSize;
    }

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                     dds.MipCount > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    up.width    = dds.Width;
    up.height   = dds.Height;
    up.byteSize = offset;
    return up;
}

GLUpload UploadLDR(const std::string& path, bool flipVertically)
{
    ImageData img = ImageLoader::Load(path, flipVertically);
    if (img.Pixels.empty()) {
        spdlog::error("OpenGLTexture: no pixel data for '{}'", path);
        return {};
    }

    GLenum internalFormat = GL_RGB8;
    GLenum dataFormat     = GL_RGB;
    if (img.Channels == 1) { internalFormat = GL_R8;    dataFormat = GL_RED;  }
    if (img.Channels == 4) { internalFormat = GL_RGBA8; dataFormat = GL_RGBA; }

    GLUpload up;
    glGenTextures(1, &up.id);
    glBindTexture(GL_TEXTURE_2D, up.id);

    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat,
                 img.Width, img.Height, 0,
                 dataFormat, GL_UNSIGNED_BYTE, img.Pixels.data());
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
    spdlog::info("OpenGLTexture: loaded '{}' ({}x{}, {} ch)", path, img.Width, img.Height, img.Channels);

    up.width  = static_cast<uint32_t>(img.Width);
    up.height = static_cast<uint32_t>(img.Height);
    // Mipmapped, so ~4/3x the base level (geometric series of quarter-sized mips).
    up.byteSize = (uint64_t)up.width * up.height * GLStats::BytesPerTexel(internalFormat) * 4 / 3;
    return up;
}

GLUpload UploadHDR(const std::string& path, bool flipVertically)
{
    FloatImageData img = ImageLoader::LoadFloat(path, flipVertically);
    if (img.Pixels.empty()) {
        spdlog::error("OpenGLTexture: no float pixel data for '{}'", path);
        return {};
    }

    GLUpload up;
    glGenTextures(1, &up.id);
    glBindTexture(GL_TEXTURE_2D, up.id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F,
                 img.Width, img.Height, 0,
                 GL_RGB, GL_FLOAT, img.Pixels.data());

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
    spdlog::info("OpenGLTexture: loaded HDR '{}' ({}x{}, {} ch)", path, img.Width, img.Height, img.Channels);

    up.width  = static_cast<uint32_t>(img.Width);
    up.height = static_cast<uint32_t>(img.Height);
    // No mipmaps on this path.
    up.byteSize = (uint64_t)up.width * up.height * GLStats::BytesPerTexel(GL_RGB16F);
    return up;
}

// Shared cooked-first entry point for the constructor and Reload(). Cooked
// files are baked unflipped (matching every registry call site — see
// Docs/asset-pipeline-design.md §4), so a flipped caller always takes the
// source path. id==0 means "no fresh cooked file" (not an error) — callers
// fall through to UploadLDR.
GLUpload TryUploadCooked(const std::string& path, bool flipVertically)
{
    if (flipVertically) return {};
    DDSData dds = DDSLoader::LoadCookedFor(path);
    if (!dds.IsValid()) return {};
    GLUpload up = UploadDDS(dds);
    if (up.id) spdlog::info("OpenGLTexture: cooked load '{}' ({}, {} mips)", path, RHIFormatName(dds.Format), dds.MipCount);
    return up;
}

} // namespace

OpenGLTexture::OpenGLTexture(const std::string& path, bool flipVertically)
    : m_Path(path), m_FlipVertically(flipVertically), m_IsHDR(false)
{
    GLUpload up = TryUploadCooked(path, flipVertically);
    if (!up.id) up = UploadLDR(path, flipVertically);
    m_RendererID = up.id;
    m_Width      = up.width;
    m_Height     = up.height;
    m_ByteSize   = up.byteSize;
    if (m_RendererID) GLStats::RecordTextureAlloc(m_ByteSize);
}

OpenGLTexture::OpenGLTexture(const uint8_t* pixels, int width, int height, int channels)
{
    if (!pixels || width <= 0 || height <= 0) {
        spdlog::error("OpenGLTexture: invalid pixel data ({}x{}, {} ch)", width, height, channels);
        return;
    }

    m_Width  = static_cast<uint32_t>(width);
    m_Height = static_cast<uint32_t>(height);

    GLenum internalFormat = GL_RGB8;
    GLenum dataFormat     = GL_RGB;
    if (channels == 1) { internalFormat = GL_R8;    dataFormat = GL_RED;  }
    if (channels == 4) { internalFormat = GL_RGBA8; dataFormat = GL_RGBA; }

    glGenTextures(1, &m_RendererID);
    glBindTexture(GL_TEXTURE_2D, m_RendererID);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0,
                 dataFormat, GL_UNSIGNED_BYTE, pixels);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);

    m_ByteSize = (uint64_t)m_Width * m_Height * GLStats::BytesPerTexel(internalFormat) * 4 / 3;
    GLStats::RecordTextureAlloc(m_ByteSize);
}

OpenGLTexture::OpenGLTexture(const std::string& path, bool flipVertically, bool isHDR)
    : m_Path(path), m_FlipVertically(flipVertically), m_IsHDR(isHDR)
{
    GLUpload up = UploadHDR(path, flipVertically);
    m_RendererID = up.id;
    m_Width      = up.width;
    m_Height     = up.height;
    m_ByteSize   = up.byteSize;
    if (m_RendererID) GLStats::RecordTextureAlloc(m_ByteSize);
}

OpenGLTexture::~OpenGLTexture()
{
    GLStats::RecordTextureFree(m_ByteSize);
    glDeleteTextures(1, &m_RendererID);
}

void OpenGLTexture::Bind(uint32_t slot) const
{
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, m_RendererID);
}

bool OpenGLTexture::Reload()
{
    if (m_Path.empty()) return false;   // CreateFromPixels — no source file to reload from

    GLUpload up = m_IsHDR ? GLUpload{} : TryUploadCooked(m_Path, m_FlipVertically);
    if (!up.id)
        up = m_IsHDR ? UploadHDR(m_Path, m_FlipVertically) : UploadLDR(m_Path, m_FlipVertically);
    if (!up.id) return false;   // decode/upload failed (e.g. file mid-write) — keep the old texture

    glDeleteTextures(1, &m_RendererID);
    GLStats::RecordTextureFree(m_ByteSize);

    m_RendererID = up.id;
    m_Width      = up.width;
    m_Height     = up.height;
    m_ByteSize   = up.byteSize;
    GLStats::RecordTextureAlloc(m_ByteSize);
    return true;
}

} // namespace Diamond
