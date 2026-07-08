#pragma once
#include "Panels.h"
#include "Audio/AudioTypes.h"

// DAW-style audio mixer. One vertical strip per bus (Master + the four routable
// groups Music/SFX/UI/Ambience), each with a volume fader, a dB readout, and
// mute/solo toggles. Reads and writes the live mix through the Audio:: facade, so
// it reflects (and drives) whatever the running engine is doing.
//
// Volume is owned by the engine (the panel just reads/writes it). Mute and solo
// are resolved here: the panel keeps the user's mute *intent* and the solo set,
// then each frame pushes the effective mute (userMute || any-solo-and-not-soloed)
// down to the engine. Solo never touches Master. Engine mute is lossless (volume
// is stored separately), so toggling solo can't lose a fader setting.
class MixerPanel : public Panel {
public:
    void OnImGuiRender() override;
    const char* GetName() const override { return "Mixer"; }

private:
    void DrawStrip(Audio::Bus bus, const char* name, bool soloable);

    // User intent, indexed by (int)Bus. Initialized from the engine on first draw.
    bool m_UserMute[5] = {};
    bool m_Solo[5]     = {};
    bool m_Initialized = false;
};
