#pragma once
#include <string>
#include <vector>
#include <functional>
#include <entt/entt.hpp>

class Scene;

// Default no-op inspector draw. Specialize per component in your script file
// (after including imgui.h) to add inspector UI for that component type.
template<typename T>
inline void DrawComponentInspector(T&) {}

// Default no-op serialization. Specialize both in your script file (after
// including <nlohmann/json.hpp>) to persist component data across saves/loads.
template<typename T>
inline std::string SerializeComponent(const T&) { return "{}"; }

template<typename T>
inline void DeserializeComponent(T&, const std::string&) {}

struct ComponentDescriptor
{
    std::string                               name;
    std::function<void(Scene&, entt::entity)> add;
    std::function<bool(Scene&, entt::entity)> has;
    std::function<void(Scene&, entt::entity)> remove;
    std::function<void(Scene&, entt::entity)> drawInspector;
    std::function<std::string(Scene&, entt::entity)>              serialize   = nullptr;
    std::function<void(Scene&, entt::entity, const std::string&)> deserialize = nullptr;
};

class ComponentRegistry
{
public:
    static ComponentRegistry& Get()
    {
        static ComponentRegistry instance;
        return instance;
    }

    void Register(ComponentDescriptor desc)
    {
        m_Descriptors.push_back(std::move(desc));
    }

    const std::vector<ComponentDescriptor>& GetAll() const { return m_Descriptors; }

private:
    std::vector<ComponentDescriptor> m_Descriptors;
};

// Place at file scope, after the component struct AND its DrawComponentInspector
// specialization. C++17 inline variable ensures registration happens exactly once
// even if the header is included by multiple translation units.
#define DECLARE_COMPONENT(ClassName, DisplayName)                                                             \
    inline struct _CompReg_##ClassName {                                                                      \
        _CompReg_##ClassName() {                                                                              \
            ComponentRegistry::Get().Register({                                                               \
                DisplayName,                                                                                  \
                [](Scene& s, entt::entity e) { s.Add<ClassName>(e); },                                      \
                [](Scene& s, entt::entity e) -> bool { return s.Has<ClassName>(e); },                        \
                [](Scene& s, entt::entity e) { s.Remove<ClassName>(e); },                                    \
                [](Scene& s, entt::entity e) { DrawComponentInspector(s.Get<ClassName>(e)); },               \
                [](Scene& s, entt::entity e) -> std::string { return SerializeComponent(s.Get<ClassName>(e)); }, \
                [](Scene& s, entt::entity e, const std::string& d) { DeserializeComponent(s.Get<ClassName>(e), d); } \
            });                                                                                               \
        }                                                                                                     \
    } _comp_reg_##ClassName{};
