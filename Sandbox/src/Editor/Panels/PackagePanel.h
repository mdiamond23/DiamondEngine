#pragma once
#include "Panels.h"
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Packager (Milestone 7): turns the open project into a shippable folder —
// builds the standalone Runtime target, walks the scene list's asset
// dependencies (text assets are scanned recursively for "Assets/..." refs,
// plus cooked-cache counterparts, model sidecars, and the default IBL HDR),
// copies everything exe-relative, and writes the boot.json the Runtime keys
// packaged mode off of.
//
// The explicit scene list is the source of truth (entry 0 = boot scene): the
// package contains exactly the listed scenes' dependency closure. Scenes that
// are referenced by walked assets but NOT listed produce a lint warning only —
// C++ script code can reference scenes unscannably, so reachability is
// advisory, never authoritative.
//
// The job runs on a background thread: it publishes progress through atomics
// and log lines through a mutexed queue the panel drains every frame. The UI
// stays live (and cancellable) while cmake builds and gigabytes copy.
class PackagePanel : public Panel {
public:
    PackagePanel();                      // loads ProjectSettings/Package.json
    ~PackagePanel() override;            // saves it back
    void OnImGuiRender() override;
    const char* GetName() const override { return "Packager"; }

private:
    // Everything the worker needs, copied by value at launch so the UI thread
    // can keep editing widgets while the job runs.
    struct Config {
        std::vector<std::string> scenes;   // portable "Assets/..." paths, [0] = boot
        std::string outputDir;             // absolute destination folder
        std::string title;
        std::string iconPath;              // portable if inside Assets/, else absolute
        int  width  = 1600;
        int  height = 900;
        bool debugBuild    = false;
        bool fullAssetCopy = false;        // whole Assets/ tree instead of the dep walk
    };

    enum class Stage : int { Idle, Building, Collecting, Cooking, Copying,
                             Finalizing, Done, Failed, Cancelled };

    void LoadSettings();
    void SaveSettings();
    void StartJob();
    void RunJob(Config cfg);               // worker-thread body
    bool RunBuild(bool debugBuild);        // shells out to cmake, streams output to the log
    void Log(std::string line);            // any thread
    void AddScene(const std::string& path);
    void DrawSceneList();
    void DrawStatusRow();

    // --- settings (UI thread only) ---
    std::vector<std::string> m_Scenes;     // portable "Assets/..." paths; [0] = boot scene
    char        m_OutputDir[512] = "";
    char        m_Title[128]     = "DiamondEngine";
    int         m_Width          = 1600;
    int         m_Height         = 900;
    std::string m_IconPath;
    bool        m_DebugBuild     = false;
    bool        m_FullAssetCopy  = false;

    // --- job state (worker writes, UI reads) ---
    std::thread           m_Worker;
    std::atomic<bool>     m_Running     { false };
    std::atomic<bool>     m_Cancel      { false };
    std::atomic<int>      m_Stage       { (int)Stage::Idle };
    std::atomic<int>      m_FilesDone   { 0 };
    std::atomic<int>      m_FilesTotal  { 0 };
    std::atomic<uint64_t> m_BytesCopied { 0 };

    std::mutex               m_LogMutex;
    std::vector<std::string> m_PendingLog;   // worker -> UI handoff
    std::vector<std::string> m_LogLines;     // drained on the UI thread, drawn clipped
};
