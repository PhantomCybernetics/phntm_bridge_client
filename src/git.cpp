#include "phntm_bridge/git.hpp"
#include "phntm_bridge/lib.hpp"
#include "rclcpp/rclcpp.hpp"

std::string executeGitCommand(const std::string& command, const std::string& repo_path) {
    std::array<char, 128> buffer;
    std::string result;

    // Prefix the command with "git -C <repoPath>"
    std::string full_command = "git -C " + repo_path + " " + command;

    FILE* pipe = popen(full_command.c_str(), "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }
    pclose(pipe);
    return result;
}

void readGitRepoHead(std::string repo_path, std::shared_ptr<BridgeConfig> config) {
   try {

        // Get the HEAD commit SHA
        std::string head_sha = executeGitCommand("rev-parse HEAD", repo_path);
        config->git_head_sha = trim(head_sha);

        // Get all tags pointing to the HEAD commit
        std::string tags = executeGitCommand("tag --points-at HEAD", repo_path);
        tags = trim(tags);

        // Determine the latest tag (if any)
        std::string latest_tag;
        if (!tags.empty()) {
            std::istringstream tagStream(tags);
            std::getline(tagStream, latest_tag); // Get the first tag (assuming sorted order)
            config->latest_git_tag = latest_tag;
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error reading Git HEAD & Tag %s", e.what());
        exit(1);
    }
}