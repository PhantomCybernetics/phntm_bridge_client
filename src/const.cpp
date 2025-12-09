#include "phntm_bridge/const.hpp"
#include <cstdlib> 
#include <string.h>

namespace phntm {

    auto colorize_output = std::getenv("RCUTILS_COLORIZED_OUTPUT");
    auto colorize = colorize_output == NULL || strcmp(colorize_output, "1") == 0;
        
    std::string GRAY = colorize ? _GRAY : "";
    std::string RED = colorize ? _RED : "";
    std::string GREEN = colorize ? _GREEN : "";
    std::string LIME = colorize ? _LIME : "";
    std::string BLUE = colorize ? _BLUE : "";
    std::string YELLOW = colorize ? _YELLOW : "";
    std::string MAGENTA = colorize ? _MAGENTA : "";
    std::string CYAN = colorize ? _CYAN : "";
    std::string WHITE = colorize ? _WHITE : "";
    std::string CLR = colorize ? _CLR : "";
}