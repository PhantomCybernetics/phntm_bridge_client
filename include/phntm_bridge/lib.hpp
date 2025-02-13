#pragma once

#include <string>
#include <json/json.h>

std::string trim(const std::string &s);

struct CustomWidgetDef {
    std::string class_name;
    std::string url;
};

struct ServiceWidgetConfig {
    std::string service;
    std::string class_name;
    Json::Value data;
};