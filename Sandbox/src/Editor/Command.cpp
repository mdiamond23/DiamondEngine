#include "Command.h"
#include "Panels/ConsolePanel.h"

void CommandManager::ExecuteCommand(std::unique_ptr<Command> command)
{
    command->Execute();
    undoStack.push_back(std::move(command));
    if (undoStack.size() > maxDepth)
        undoStack.pop_front();
    redoStack.clear();
}

void CommandManager::Undo()
{
    if (undoStack.empty()) return;
    auto& command = undoStack.back();
    if (m_Console)
        m_Console->Log(LogLevel::Info, "[Undo] " + command->GetDescription());
    command->Undo();
    redoStack.push_back(std::move(command));
    undoStack.pop_back();
}

void CommandManager::Redo()
{
    if (redoStack.empty()) return;
    auto& command = redoStack.back();
    if (m_Console)
        m_Console->Log(LogLevel::Info, "[Redo] " + command->GetDescription());
    command->Execute();
    undoStack.push_back(std::move(command));
    redoStack.pop_back();
}

void CommandManager::RecordCommand(std::unique_ptr<Command> command)
{
    undoStack.push_back(std::move(command));
    if (undoStack.size() > maxDepth)
        undoStack.pop_front();
    redoStack.clear();
}

void CommandManager::Clear()
{
    undoStack.clear();
    redoStack.clear();
}
