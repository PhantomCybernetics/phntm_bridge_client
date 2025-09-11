#pragma once

#include <string>
#include <json/json.h>

namespace phntm {

    std::string generateId(const size_t length);
    std::string trim(const std::string s, const char *trim_chars = " \t\n\r\f\v");
    std::string replace (std::string str, const std::string& search, const std::string& replace);
    std::string join(const std::vector<std::string>& vec, const std::string separator = ", ");
    std::string strToLower(std::string s);
    std::vector<std::string> split(const std::string& s, char delimiter);
    std::vector<std::string> rsplit(const std::string& str, const char delimiter, int maxsplit = -1);
    bool isImageOrVideoType(std::string msg_type);
    bool isEncodedVideoType(std::string msg_type);
    bool isImageType(std::string msg_type);
    bool isCompressedImageType(std::string msg_type);
    void log(std::string msg, bool error=false, bool append_endl=true);
    size_t CURLResponseCallback(void* contents, size_t size, size_t nmemb, std::string* out);
    std::string toHex(uint32_t num, bool uppercase = false);
    std::string getThreadId();
    uint64_t convertToRtpTimestamp(int32_t sec, uint32_t nanosec);
    uint64_t getCurrentRtpTimestamp();
    uint32_t getRandomUInt();

    struct CustomWidgetDef {
        std::string class_name;
        std::string url;
    };

    struct ServiceWidgetConfig {
        std::string service;
        std::string class_name;
        Json::Value data;
    };
}