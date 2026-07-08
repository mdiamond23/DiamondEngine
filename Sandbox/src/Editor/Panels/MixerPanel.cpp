#include "MixerPanel.h"
#include "Audio/AudioAPI.h"
#include <imgui.h>
#include <cmath>

namespace {
    // The five buses in display order. Master leads, then the routable groups.
    struct BusEntry { Audio::Bus bus; const char* name; bool soloable; };
    constexpr BusEntry kBuses[] = {
        { Audio::Bus::Master,   "Master",   false },
        { Audio::Bus::Music,    "Music",    true  },
        { Audio::Bus::SFX,      "SFX",      true  },
        { Audio::Bus::UI,       "UI",       true  },
        { Audio::Bus::Ambience, "Ambience", true  },
    };

    // Linear gain -> dB string for the fader readout (-inf at silence).
    const char* GainToDb(float gain)
    {
        static char buf[16];
        if (gain <= 0.0001f) { return "-\xe2\x88\x9e dB"; }   // -∞ dB
        std::snprintf(buf, sizeof(buf), "%+.1f dB", 20.0f * std::log10(gain));
        return buf;
    }
}

void MixerPanel::OnImGuiRender()
{
    if (!m_Open) return;
    if (!ImGui::Begin("Mixer", &m_Open)) { ImGui::End(); return; }

    // First frame: adopt whatever mute state the engine already holds.
    if (!m_Initialized) {
        for (const auto& e : kBuses)
            m_UserMute[(int)e.bus] = Audio::IsBusMuted(e.bus);
        m_Initialized = true;
    }

    const bool anySolo = m_Solo[(int)Audio::Bus::Music] || m_Solo[(int)Audio::Bus::SFX]
                      || m_Solo[(int)Audio::Bus::UI]    || m_Solo[(int)Audio::Bus::Ambience];

    // Reconcile effective mute with the engine every frame: a soloed-elsewhere bus
    // is forced silent without disturbing the user's own mute toggle or its volume.
    for (const auto& e : kBuses) {
        const int  i   = (int)e.bus;
        const bool eff = m_UserMute[i] || (e.soloable && anySolo && !m_Solo[i]);
        if (Audio::IsBusMuted(e.bus) != eff)
            Audio::SetBusMuted(e.bus, eff);
    }

    // Strips laid out left-to-right, with a gap setting Master off from the groups.
    for (int s = 0; s < (int)IM_ARRAYSIZE(kBuses); ++s) {
        const BusEntry& e = kBuses[s];
        if (s > 0) {
            ImGui::SameLine(0.0f, s == 1 ? 24.0f : 8.0f);   // wider gap after Master
        }
        ImGui::PushID(s);
        DrawStrip(e.bus, e.name, e.soloable);
        ImGui::PopID();
    }

    // Voice meter — makes the one-shot voice cap (and its steal-oldest behavior)
    // observable rather than invisible.
    ImGui::Separator();
    const int active = Audio::ActiveVoiceCount();
    const int cap    = Audio::VoiceCap();
    ImGui::Text("Voices: %d / %d", active, cap);

    ImGui::End();
}

void MixerPanel::DrawStrip(Audio::Bus bus, const char* name, bool soloable)
{
    const int   i        = (int)bus;
    const float stripW   = 60.0f;
    const float faderH   = 140.0f;

    ImGui::BeginGroup();

    // Name header, centered over the strip.
    const float nameW = ImGui::CalcTextSize(name).x;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (stripW - nameW) * 0.5f);
    ImGui::TextUnformatted(name);

    // Vertical fader. The engine owns the value; we read it back each frame.
    float vol = Audio::GetBusVolume(bus);
    if (ImGui::VSliderFloat("##vol", ImVec2(stripW, faderH), &vol, 0.0f, 1.5f, ""))
        Audio::SetBusVolume(bus, vol);
    if (ImGui::IsItemHovered() || ImGui::IsItemActive())
        ImGui::SetTooltip("%.2f  (%s)", vol, GainToDb(vol));

    // dB readout, centered.
    const char* db    = GainToDb(vol);
    const float dbW   = ImGui::CalcTextSize(db).x;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (stripW - dbW) * 0.5f);
    ImGui::TextDisabled("%s", db);

    // Mute toggle — lit red when active.
    const bool muted = m_UserMute[i];
    if (muted) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.16f, 0.16f, 1.0f));
    if (ImGui::Button("M##mute", ImVec2(stripW, 0)))
        m_UserMute[i] = !m_UserMute[i];
    if (muted) ImGui::PopStyleColor();

    // Solo toggle — lit yellow when active. Master has none.
    if (soloable) {
        const bool solo = m_Solo[i];
        if (solo) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.78f, 0.66f, 0.10f, 1.0f));
        if (ImGui::Button("S##solo", ImVec2(stripW, 0)))
            m_Solo[i] = !m_Solo[i];
        if (solo) ImGui::PopStyleColor();
    } else {
        ImGui::Dummy(ImVec2(stripW, ImGui::GetFrameHeight()));
    }

    ImGui::EndGroup();
    ImGui::SameLine();
}
