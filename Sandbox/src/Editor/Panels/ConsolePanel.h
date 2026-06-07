#pragma once
#include "Panels.h"
#include <string>
#include <vector>

enum class LogLevel { Info, Warning, Error };

struct LogEntry {
    LogLevel    level;
    std::string message;
};

class ConsolePanel : public Panel {
public:
    void OnImGuiRender() override;
    void Log(LogLevel level, const std::string& message);
    void Clear();

private:
    std::vector<LogEntry> m_Entries;
    bool m_AutoScroll = true;
};
