#pragma once

class Panel {
public:
    virtual ~Panel() = default;
    virtual void OnImGuiRender() = 0;

    // Title shown in the "View" menu and passed to ImGui::Begin().
    virtual const char* GetName() const = 0;

    // Backing storage for the "View" menu checkbox and the window's own
    // close button (passed to ImGui::Begin() as p_open).
    bool& Open() { return m_Open; }

protected:
    bool m_Open = true;
};
