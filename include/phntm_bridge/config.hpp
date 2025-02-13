#pragma once

#include <cstdio>
#include <vector>
#include <string>
#include <json/json.h>

#include "lib.hpp"

class BridgeConfig {
    public:
        std::string id_robot, auth_key, robot_name, maintainer_email;
        std::string ros_distro, git_head_sha, latest_git_tag;
        
        std::vector<std::string> extra_packages;

        bool use_cloud_ice_config;
        std::vector<std::string> ice_servers;
        std::vector<std::string> ice_servers_custom;
        std::string ice_username, ice_secret;

        std::vector<std::string> blacklist_topics, blacklist_services, blacklist_msg_types;

        double log_message_every_sec;

        std::string cloud_bridge_address, sio_path, uploader_address;
        int file_upload_port, sio_port;
        bool sio_ssl_verify, sio_verbose;
        double sio_connection_retry_sec;
        
        std::string conn_led_topic, data_led_topic, conn_led_gpio_chip;
        int conn_led_pin, data_led_pin;

        bool docker_control_enabled;

        std::vector<std::string> input_drivers;
        std::vector<CustomWidgetDef> custom_input_drivers, custom_service_widgets;
        std::vector<ServiceWidgetConfig> service_widgets;

        Json::Value input_defaults, service_defaults;
};