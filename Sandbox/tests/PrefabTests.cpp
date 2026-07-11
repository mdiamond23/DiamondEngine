// Headless round-trip tests for the prefab reference pipeline: prefab save ->
// instantiate -> per-instance overrides -> scene save/load -> live-link
// propagation -> missing-file placeholder recovery. Uses only GPU-free
// components (transforms, lights, constraints) so it runs without a window.
//
// Build:  cmake --build build --target PrefabTests
// Run:    build/Sandbox/Debug/PrefabTests.exe   (exit code = failure count)

#include "Editor/SceneSerializer.h"
#include "Scene/Scene.h"
#include "Scene/Components.h"
#include "Scene/Physics/Constraint.h"
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

static int g_failures = 0;

#define CHECK(cond, msg)                                    \
    do {                                                    \
        if (cond) { std::printf("  ok    %s\n", msg); }     \
        else      { std::printf("  FAIL  %s\n", msg); ++g_failures; } \
    } while (0)

static entt::entity FindChildByName(Scene& s, entt::entity parent, const std::string& name)
{
    auto& reg = s.GetRegistry();
    if (parent == entt::null || !reg.all_of<HierarchyComponent>(parent)) return entt::null;
    for (auto c : reg.get<HierarchyComponent>(parent).children)
        if (s.GetEntityName(c) == name) return c;
    return entt::null;
}

int main()
{
    namespace fs = std::filesystem;
    fs::path dir = fs::temp_directory_path() / "diamond_prefab_tests";
    fs::remove_all(dir);
    fs::create_directories(dir);
    std::string prefabPath = (dir / "Rig.prefab").string();
    std::string scenePath  = (dir / "test.scene").string();
    std::string scenePath2 = (dir / "broken.scene").string();

    // ---- author a prefab: Rig { Lamp(light 5.0), Probe(constraint -> Lamp) } --
    std::printf("[authoring]\n");
    {
        Scene author;
        auto& reg = author.GetRegistry();
        entt::entity rig   = author.CreateEntity("Rig");
        entt::entity lamp  = author.CreateEntity("Lamp");
        entt::entity probe = author.CreateEntity("Probe");
        author.SetParent(lamp, rig);
        author.SetParent(probe, rig);
        reg.emplace<LightComponent>(lamp).intensity = 5.0f;
        reg.emplace<ConstraintComponent>(probe).targetUuid = reg.get<IDComponent>(lamp).uuid;

        CHECK(PrefabSerializer::Save(author, rig, prefabPath), "prefab file written");
        CHECK(reg.all_of<PrefabChildComponent>(rig) && reg.all_of<PrefabChildComponent>(lamp),
              "Save stamps the subtree with prefab-local ids");
    }

    // ---- two instances, one edited ------------------------------------------
    std::printf("[instances + overrides]\n");
    Scene scene;
    auto& reg = scene.GetRegistry();
    entt::entity r1 = PrefabSerializer::Instantiate(scene, prefabPath);
    entt::entity r2 = PrefabSerializer::Instantiate(scene, prefabPath);
    CHECK(r1 != entt::null && r2 != entt::null, "two instances created");

    entt::entity l1 = FindChildByName(scene, r1, "Lamp");
    entt::entity p1 = FindChildByName(scene, r1, "Probe");
    entt::entity p2 = FindChildByName(scene, r2, "Probe");
    CHECK(l1 != entt::null && p1 != entt::null && p2 != entt::null, "children instantiated");
    CHECK(reg.get<IDComponent>(r1).uuid != reg.get<IDComponent>(r2).uuid,
          "instances get distinct root uuids");
    CHECK(reg.get<ConstraintComponent>(p1).targetUuid == reg.get<IDComponent>(l1).uuid,
          "constraint target remapped into the instance");

    reg.get<TransformComponent>(r1).position = { 1.0f, 2.0f, 3.0f };  // ref-level transform
    reg.get<LightComponent>(l1).intensity    = 9.0f;                  // component override
    scene.SetEntityName(l1, "Lamp (bright)");                         // name override
    scene.DestroyEntity(p2);                                          // removed-entity override

    uint64_t r1Uuid = reg.get<IDComponent>(r1).uuid;
    uint64_t r2Uuid = reg.get<IDComponent>(r2).uuid;
    uint64_t l1Uuid = reg.get<IDComponent>(l1).uuid;

    SceneSerializer::Save(scene, scenePath);
    {
        std::ifstream f(scenePath);
        std::string txt((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        CHECK(txt.find("prefabRef") != std::string::npos, "scene file stores prefab references");
        CHECK(txt.find("\"Probe\"") == std::string::npos,
              "instance interiors are not baked into the scene file");
    }

    // ---- reload: overrides + deterministic interior uuids --------------------
    std::printf("[reload]\n");
    Scene loaded;
    CHECK(SceneSerializer::Load(loaded, scenePath), "scene loads");
    auto& lreg = loaded.GetRegistry();

    entt::entity R1 = loaded.FindByUuid(r1Uuid);
    entt::entity R2 = loaded.FindByUuid(r2Uuid);
    CHECK(R1 != entt::null && R2 != entt::null, "instance root uuids preserved");

    entt::entity L1 = FindChildByName(loaded, R1, "Lamp (bright)");
    CHECK(L1 != entt::null, "child name override applied");
    CHECK(L1 != entt::null && lreg.get<IDComponent>(L1).uuid == l1Uuid,
          "child uuid is deterministic across save/load");
    CHECK(L1 != entt::null && lreg.get<LightComponent>(L1).intensity == 9.0f,
          "component override applied");
    CHECK(lreg.get<TransformComponent>(R1).position == glm::vec3(1.0f, 2.0f, 3.0f),
          "instance root transform preserved");

    entt::entity P1 = FindChildByName(loaded, R1, "Probe");
    CHECK(P1 != entt::null && lreg.get<ConstraintComponent>(P1).targetUuid == l1Uuid,
          "interior constraint target still resolves after reload");

    CHECK(FindChildByName(loaded, R2, "Probe") == entt::null, "removed child stays removed");
    entt::entity L2 = FindChildByName(loaded, R2, "Lamp");
    CHECK(L2 != entt::null && lreg.get<LightComponent>(L2).intensity == 5.0f,
          "untouched instance matches the prefab");

    // ---- live link: push R1's state to the file, propagate to R2 -------------
    std::printf("[live link]\n");
    int n = PrefabSerializer::SaveAndPropagate(loaded, R1, prefabPath);
    CHECK(n == 1, "one sibling instance propagated");

    R2 = loaded.FindByUuid(r2Uuid);
    CHECK(R2 != entt::null, "propagated instance keeps its uuid");
    L2 = FindChildByName(loaded, R2, "Lamp (bright)");
    CHECK(L2 != entt::null && lreg.get<LightComponent>(L2).intensity == 9.0f,
          "sibling inherits the new prefab state");
    CHECK(FindChildByName(loaded, R2, "Probe") == entt::null,
          "sibling's own removed-child override survives propagation");

    // ---- missing file: placeholder keeps the reference alive -----------------
    std::printf("[missing file]\n");
    fs::rename(prefabPath, prefabPath + ".bak");
    Scene broken;
    CHECK(SceneSerializer::Load(broken, scenePath), "scene loads with the prefab file gone");
    auto& breg = broken.GetRegistry();
    entt::entity B1 = broken.FindByUuid(r1Uuid);
    CHECK(B1 != entt::null, "placeholder keeps the stored uuid");
    CHECK(B1 != entt::null && breg.all_of<PrefabInstanceComponent>(B1)
              && breg.get<PrefabInstanceComponent>(B1).broken,
          "placeholder is flagged broken");
    SceneSerializer::Save(broken, scenePath2);   // must round-trip the reference

    fs::rename(prefabPath + ".bak", prefabPath);
    Scene healed;
    CHECK(SceneSerializer::Load(healed, scenePath2), "re-saved scene loads after file returns");
    auto& hreg = healed.GetRegistry();
    entt::entity H1 = healed.FindByUuid(r1Uuid);
    entt::entity HL = FindChildByName(healed, H1, "Lamp (bright)");
    CHECK(H1 != entt::null && !hreg.get<PrefabInstanceComponent>(H1).broken,
          "instance heals once the file is back");
    CHECK(HL != entt::null && hreg.get<LightComponent>(HL).intensity == 9.0f,
          "overrides survive the broken round-trip");

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASSED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures;
}
