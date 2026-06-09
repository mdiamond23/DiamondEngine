#pragma once
#include <string>
#include <vector>

enum class LogLevel { Info, Warning, Error };

struct LogEntry
{
    LogLevel    level;
    std::string message;
};

class Debug
{
public:
    static void Log(const std::string& msg)   { s_Entries.push_back({LogLevel::Info,    msg}); }
    static void Warn(const std::string& msg)  { s_Entries.push_back({LogLevel::Warning, msg}); }
    static void Error(const std::string& msg) { s_Entries.push_back({LogLevel::Error,   msg}); }
    static void Clear()                        { s_Entries.clear(); }

    static const std::vector<LogEntry>& GetEntries() { return s_Entries; }

private:
    static inline std::vector<LogEntry> s_Entries;
};
