#pragma once
#include <memory>
#include <vector>
#include <functional>
#include <algorithm>

class Scene;

class GameSystem
{
public:
    virtual ~GameSystem() = default;
    virtual void OnStart(Scene& scene)            {}
    virtual void OnUpdate(Scene& scene, float dt) {}
    virtual void OnDestroy(Scene& scene)          {}

    // Label for this system's row in the editor profiler. DECLARE_SYSTEM
    // overrides it with the class name automatically. Must return a string
    // with permanent lifetime (the profiler keys scopes by the pointer).
    virtual const char* GetName() const { return "GameSystem"; }
};

class SystemRegistry
{
public:
    static SystemRegistry& Get()
    {
        static SystemRegistry instance;
        return instance;
    }

    using Factory = std::function<std::unique_ptr<GameSystem>()>;

    void Register(Factory factory, int priority)
    {
        m_entries.push_back({ priority, std::move(factory) });
        std::sort(m_entries.begin(), m_entries.end(),
            [](const Entry& a, const Entry& b) { return a.priority < b.priority; });
    }

    // Called at play mode start — returns a fresh sorted set of system instances
    std::vector<std::unique_ptr<GameSystem>> CreateSystems() const
    {
        std::vector<std::unique_ptr<GameSystem>> systems;
        systems.reserve(m_entries.size());
        for (const auto& e : m_entries)
            systems.push_back(e.factory());
        return systems;
    }

private:
    struct Entry
    {
        int     priority;
        Factory factory;
    };

    std::vector<Entry> m_entries;
};

// Place inside a GameSystem subclass to self-register with the given priority.
// The system is instantiated fresh each play session via CreateSystems().
// Also names the system for the profiler (Scene::UpdateSystems times each
// system's OnUpdate under its GetName()).
#define DECLARE_SYSTEM(ClassName, Priority)                               \
public:                                                                   \
    const char* GetName() const override { return #ClassName; }          \
private:                                                                  \
    static inline struct _Registrar_##ClassName {                         \
        _Registrar_##ClassName() {                                        \
            SystemRegistry::Get().Register(                               \
                []() { return std::make_unique<ClassName>(); }, Priority);\
        }                                                                 \
    } _registrar_##ClassName{};
