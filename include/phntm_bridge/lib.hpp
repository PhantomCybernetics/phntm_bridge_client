#pragma once

#include <string>
#include <json/json.h>

std::string trim(const std::string s, const char *whitespace = " \t\n\r\f\v");
std::string replace (std::string str, const std::string& search, const std::string& replace);
std::string join(const std::vector<std::string>& vec, const std::string separator = ", ");
std::vector<std::string> split(const std::string& s, char delimiter);
std::vector<std::string> rsplit(const std::string& str, const char delimiter, int maxsplit = -1);

struct CustomWidgetDef {
    std::string class_name;
    std::string url;
};

struct ServiceWidgetConfig {
    std::string service;
    std::string class_name;
    Json::Value data;
};