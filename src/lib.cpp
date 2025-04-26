#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/const.hpp"
#include <algorithm>
#include <iostream>

namespace phntm {

    void log(std::string msg, bool error, bool append_endl) {
        if (error)  {
            std::cerr << (RED + msg + CLR) + (append_endl ? "\n" : "");
        } else {
            std::cout << msg + (append_endl ? "\n" : "");
        }
    }

    std::string trim(const std::string str, const char *trim_chars) {
        size_t start = str.find_first_not_of(trim_chars);
        if (start == std::string::npos) return ""; // all whitespace
        size_t end = str.find_last_not_of(trim_chars);
        return str.substr(start, end - start + 1);
    }

    std::string replace (std::string str, const std::string& search, const std::string& replace) {
        size_t pos = 0;
        while ((pos = str.find(search, pos)) != std::string::npos) {
            str.replace(pos, search.length(), replace);
            pos += replace.length();
        }
        return  str;
    }

    std::vector<std::string> split(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(str);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    std::vector<std::string> rsplit(const std::string& str, const char delimiter, int maxsplit) {
        std::vector<std::string> result;
        
        std::string::size_type last_pos = str.length();
        std::string::size_type pos = str.rfind(delimiter);
        
        while (pos != std::string::npos && (maxsplit == -1 || static_cast<int>(result.size()) < maxsplit)) {
            result.push_back(str.substr(pos + 1, last_pos - pos - 1));
            last_pos = pos;
            pos = (pos == 0) ? std::string::npos : str.rfind(delimiter, pos - 1);
        }
        
        if (last_pos > 0) {
            result.push_back(str.substr(0, last_pos));
        } else if (last_pos == 0) {
            result.push_back("");
        }
        
        std::reverse(result.begin(), result.end());
        return result;
    }

    std::string join(const std::vector<std::string>& vec, const std::string separator) {
        if (vec.empty()) {
            return "";
        }

        std::string result;
        for (size_t i = 0; i < vec.size(); i++) {
            result += vec[i];
            if (i < vec.size() - 1) {
                result += separator;
            }
        }

        return result;
    }

    bool isImageOrVideoType(std::string msg_type) {
        return msg_type == IMAGE_MSG_TYPE ||
            msg_type == COMPRESSED_IMAGE_MSG_TYPE ||
            msg_type == VIDEO_STREAM_MSG_TYPE;
    }
        
    bool isEncodedVideoType(std::string msg_type) {
        return msg_type == VIDEO_STREAM_MSG_TYPE;
    }

     // receives CURL response data
     size_t CURLResponseCallback(void* contents, size_t size, size_t nmemb, std::string* out) {
        size_t totalSize = size * nmemb;
        out->append((char*)contents, totalSize);
        return totalSize;
    }
    
}