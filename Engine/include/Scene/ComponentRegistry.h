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

struct ComponentDescriptor
{
    std::string                               name;
    std::function<void(Scene&, entt::entity)> add;
    std::function<bool(Scene&, entt::entity)> has;
    std::function<void(Scene&, entt::entity)> remove;
    std::function<void(Scene&, entt::entity)> drawInspector;
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
#define DECLARE_COMPONENT(ClassName, DisplayName)                                              \
    inline struct _CompReg_##ClassName {                                                       \
        _CompReg_##ClassName() {                                                               \
            ComponentRegistry::Get().Register({                                                \
                DisplayName,                                                                   \
                [](Scene& s, entt::entity e) { s.Add<ClassName>(e); },                       \
                [](Scene& s, entt::entity e) -> bool { return s.Has<ClassName>(e); },         \
                [](Scene& s, entt::entity e) { s.Remove<ClassName>(e); },                     \
                [](Scene& s, entt::entity e) { DrawComponentInspector(s.Get<ClassName>(e)); } \
            });                                                                                \
        }                                                                                      \
    } _comp_reg_##ClassName{};
