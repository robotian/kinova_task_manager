#ifndef KINOVA_TASK_MANAGER__MANIPULATOR_COMMANDS_HPP_
#define KINOVA_TASK_MANAGER__MANIPULATOR_COMMANDS_HPP_

#include <string>
#include <unordered_map>

namespace kinova_task_manager {

enum class ManipulatorCommand {
    GO_STOW,
    GO_READY,
    MOVE_EEF,
    UNKNOWN
};

/**
 * @brief Helper to convert incoming Action Goal strings to Enums.
 * This centralizes the logic so both Client and Server speak the same language.
 */
inline ManipulatorCommand stringToCommand(const std::string& task_str) {
    static const std::unordered_map<std::string, ManipulatorCommand> command_map = {
        {"GO STOW",  ManipulatorCommand::GO_STOW},
        {"GO READY", ManipulatorCommand::GO_READY},
        {"MOVE EEF", ManipulatorCommand::MOVE_EEF}
    };

    auto it = command_map.find(task_str);
    if (it != command_map.end()) {
        return it->second;
    }
    return ManipulatorCommand::UNKNOWN;
}

}  // namespace kinova_task_manager

#endif  // KINOVA_TASK_MANAGER__MANIPULATOR_COMMANDS_HPP_