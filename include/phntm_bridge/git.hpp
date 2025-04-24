#pragma once

#include "config.hpp"

namespace phntm {

    void readGitRepoHead(std::string repo_path, std::shared_ptr<BridgeConfig> config);
    
}