#pragma once
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>
#include <cctype>

#ifndef ASSETS_DIR
#define ASSETS_DIR "Assets"
#endif

// Scene/prefab/material files are committed, so they must survive the repo
// moving between machines: nothing absolute may reach disk. Every path is
// written project-root-relative ("Assets/...", forward slashes) and resolved
// back to absolute at load. Files saved before this convention hold absolute
// paths from whatever root the repo had then; Resolve rescues those by
// rebasing their "Assets/..." suffix onto the current root.
namespace AssetPaths {

namespace fs = std::filesystem;

// Parent of the compile-time assets dir, e.g. "C:/Dev/DiamondEngine".
inline const fs::path& ProjectRoot()
{
    static const fs::path root = fs::path(ASSETS_DIR).parent_path();
    return root;
}

// Lowercase, forward-slash copy for case/separator-insensitive comparison
// (Windows paths in old files mix casing and separators freely).
inline std::string LowerGeneric(const std::string& s)
{
    std::string g = fs::path(s).generic_string();
    for (char& c : g) c = (char)std::tolower((unsigned char)c);
    return g;
}

// fs::is_absolute alone misses "C:\..." on non-Windows hosts; committed files
// can carry Windows-absolute paths regardless of the platform loading them.
inline bool IsAbsolutePath(const std::string& s)
{
    return fs::path(s).is_absolute()
        || (s.size() > 1 && s[1] == ':' && std::isalpha((unsigned char)s[0]));
}

// Absolute path -> portable "Assets/..." form. Non-absolute strings (names,
// "__sphere"-style sentinels, already-portable paths) pass through untouched,
// which is what makes this safe to run over every string in a JSON tree.
inline std::string ToPortable(const std::string& s)
{
    if (s.empty() || !IsAbsolutePath(s)) return s;

    const std::string low     = LowerGeneric(s);
    const std::string rootLow = LowerGeneric(ProjectRoot().string());
    if (!rootLow.empty() && low.size() > rootLow.size() + 1 &&
        low.compare(0, rootLow.size(), rootLow) == 0 && low[rootLow.size()] == '/')
        return fs::path(s).generic_string().substr(rootLow.size() + 1);

    // Absolute under some *other* root (stale machine) — keep the Assets/ tail.
    size_t pos = low.rfind("/assets/");
    if (pos != std::string::npos)
        return fs::path(s).generic_string().substr(pos + 1);

    return s;   // genuinely external path; nothing portable to write
}

// Portable or stale-absolute path -> absolute under the current root.
inline std::string Resolve(const std::string& s)
{
    if (s.empty()) return s;

    if (IsAbsolutePath(s)) {
        std::error_code ec;
        if (fs::exists(fs::path(s), ec)) return s;
        // Saved on a machine whose root no longer exists — rebase the tail.
        const std::string low = LowerGeneric(s);
        size_t pos = low.rfind("/assets/");
        if (pos != std::string::npos)
            return (ProjectRoot() / fs::path(s).generic_string().substr(pos + 1)).generic_string();
        return s;
    }

    // Only strings serialized by ToPortable start with "Assets/" — anything
    // else relative is a name/sentinel, not a path.
    if (LowerGeneric(s).rfind("assets/", 0) == 0)
        return (ProjectRoot() / fs::path(s)).generic_string();

    return s;
}

template<typename Fn>
inline void TransformStrings(nlohmann::json& j, const Fn& fn)
{
    if (j.is_object() || j.is_array()) {
        for (auto& v : j) TransformStrings(v, fn);
    } else if (j.is_string()) {
        const std::string& s = j.get_ref<const std::string&>();
        std::string t = fn(s);
        if (t != s) j = std::move(t);
    }
}

// Run over a whole serialized tree just before dump / just after parse.
inline void MakePortable(nlohmann::json& j) { TransformStrings(j, [](const std::string& s) { return ToPortable(s); }); }
inline void ResolveAll (nlohmann::json& j)  { TransformStrings(j, [](const std::string& s) { return Resolve(s); }); }

} // namespace AssetPaths
