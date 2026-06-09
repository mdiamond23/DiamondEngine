#pragma once
#include "Panels.h"
#include "Core/Debug.h"

class ConsolePanel : public Panel {
public:
    void OnImGuiRender() override;
    void Log(LogLevel level, const std::string& message);
    void Clear();

private:
    bool m_AutoScroll = true;
};
