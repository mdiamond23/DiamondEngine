#pragma once
#include "Platform/OpenGL/Resources/OpenGLShader.h"

#include <filesystem>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>
#include <spdlog/spdlog.h>

namespace Diamond {

// Owns the engine's shaders by name so render code looks them up instead of
// holding ad-hoc handles, and supports hot-reload: ReloadChanged() recompiles
// any whose source files changed on disk (in place — see OpenGLShader::Reload),
// ReloadAll() forces a full recompile. Reloads happen in place, so references
// handed out by Load()/Get() stay valid across a reload.
class ShaderLibrary {
public:
    OpenGLShader& Load(const std::string& name, const std::string& vs, const std::string& fs) {
        return Store(name, std::make_shared<OpenGLShader>(vs, fs));
    }
    OpenGLShader& Load(const std::string& name, const std::string& vs,
                       const std::string& gs, const std::string& fs) {
        return Store(name, std::make_shared<OpenGLShader>(vs, gs, fs));
    }

    OpenGLShader& Get(const std::string& name) {
        auto it = m_Shaders.find(name);
        if (it == m_Shaders.end()) {
            // Programmer error — a name was requested that was never Load()ed.
            spdlog::critical("ShaderLibrary: shader '{}' was never loaded", name);
            std::abort();
        }
        return *it->second.shader;
    }

    // Force-recompile everything (e.g. a manual "Reload Shaders" hotkey).
    void ReloadAll() {
        for (auto& [name, e] : m_Shaders) {
            e.shader->Reload();
            RefreshTimes(e);
        }
    }

    // Recompile only shaders whose source files changed since the last check.
    // Cheap enough to poll on a throttle: a stat per watched file, no GL work
    // unless something actually changed.
    void ReloadChanged() {
        for (auto& [name, e] : m_Shaders) {
            if (Changed(e)) {
                e.shader->Reload();   // logs success/failure; keeps old program on failure
                RefreshTimes(e);      // one reload attempt per on-disk change
            }
        }
    }

private:
    struct Watched { std::string path; std::filesystem::file_time_type time; };
    struct Entry   { std::shared_ptr<OpenGLShader> shader; std::vector<Watched> files; };

    std::unordered_map<std::string, Entry> m_Shaders;

    OpenGLShader& Store(const std::string& name, std::shared_ptr<OpenGLShader> shader) {
        Entry e;
        e.shader = std::move(shader);
        Watch(e, e.shader->GetVertexPath());
        if (!e.shader->GetGeometryPath().empty()) Watch(e, e.shader->GetGeometryPath());
        Watch(e, e.shader->GetFragmentPath());
        auto [it, ok] = m_Shaders.insert_or_assign(name, std::move(e));
        return *it->second.shader;
    }

    static std::filesystem::file_time_type FileTime(const std::string& path) {
        std::error_code ec;
        auto t = std::filesystem::last_write_time(path, ec);
        return ec ? std::filesystem::file_time_type{} : t;
    }
    static void Watch(Entry& e, const std::string& path) { e.files.push_back({ path, FileTime(path) }); }
    static bool Changed(const Entry& e) {
        for (const auto& w : e.files) if (FileTime(w.path) != w.time) return true;
        return false;
    }
    static void RefreshTimes(Entry& e) {
        for (auto& w : e.files) w.time = FileTime(w.path);
    }
};

} // namespace Diamond
