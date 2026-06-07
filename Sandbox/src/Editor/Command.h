#pragma once
#include <deque>
#include <memory>
#include <functional>
#include <string>

class ConsolePanel;

class Command {
public:
    virtual ~Command() = default;
    virtual void Execute() = 0;
    virtual void Undo() = 0;
    virtual std::string GetDescription() const { return "Command"; }
};

class CommandManager {
    static constexpr int maxDepth = 50;
    std::deque<std::unique_ptr<Command>> undoStack;
    std::deque<std::unique_ptr<Command>> redoStack;

public:
    void SetConsole(ConsolePanel* console) { m_Console = console; }

    void ExecuteCommand(std::unique_ptr<Command> command); // performs Execute() then pushes
    void RecordCommand(std::unique_ptr<Command> command);  // pushes without calling Execute (action already done)
    void Undo();
    void Redo();
    void Clear();
    bool CanUndo() const { return !undoStack.empty(); }
    bool CanRedo() const { return !redoStack.empty(); }

private:
    ConsolePanel* m_Console = nullptr;
};

// For actions where execute and undo are arbitrary (add/remove component, etc.)
class FunctionCommand : public Command {
    std::function<void()> m_execute;
    std::function<void()> m_undo;
    std::string           m_description;
public:
    FunctionCommand(std::function<void()> execute, std::function<void()> undo, std::string desc = "")
        : m_execute(std::move(execute)), m_undo(std::move(undo)), m_description(std::move(desc)) {}

    void Execute() override { m_execute(); }
    void Undo()    override { m_undo(); }
    std::string GetDescription() const override { return m_description; }
};

template<typename T>
class ValueChangeCommand : public Command {
    std::function<void(const T&)> m_setter;
    T m_oldValue;
    T m_newValue;
    std::string m_description;
public:
    ValueChangeCommand(std::function<void(const T&)> setter, const T& oldValue, const T& newValue, std::string desc = "")
        : m_setter(std::move(setter)), m_oldValue(oldValue), m_newValue(newValue), m_description(std::move(desc)) {}

    void Execute() override { m_setter(m_newValue); }
    void Undo() override { m_setter(m_oldValue); }
    std::string GetDescription() const override { return m_description; }
};