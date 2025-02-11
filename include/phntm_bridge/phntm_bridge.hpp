#pragma once

#include <cstdio>
#include <vector>
#include <fmt/core.h>
#include <json/json.h>

#include "rclcpp/rclcpp.hpp"

#include "const.hpp"
#include "lib.hpp"

// #include <future>
// #include "std_msgs/msg/string.hpp"
// #include <memory>
#include "std_srvs/srv/trigger.hpp"

class PhntmBridge : public rclcpp::Node
{
  public:
    PhntmBridge(std::string node_name);
    void loadConfig();
    void makeServices();
    void readGitRepoHead(std::string repo_path);
    // std::future<int> async_function(int x);
    
    struct CustomWidgetDef {
      std::string class_name;
      std::string url;
    };

    struct ServiceWidgetConfig {
      std::string service;
      std::string class_name;
      Json::Value data;
    };

  private:
    bool shutting_down = false;

    std::string id_robot, auth_key;
    std::vector<std::string> extra_packages;

    bool use_cloud_ice_config;
    std::vector<std::string> ice_servers;
    std::vector<std::string> ice_servers_custom;
    std::string ice_username, ice_secret;

    std::vector<std::string> blacklist_topics, blacklist_services, blacklist_msg_types;

    double log_message_every_sec;

    std::string cloud_bridge_address, sio_path, uploader_address;
    int file_upload_port, sio_port;
    bool sio_ssl_verify;
    double sio_connection_retry_sec;
    
    std::string conn_led_topic, data_led_topic, conn_led_gpio_chip;
    int conn_led_pin, data_led_pin;

    bool docker_control_enabled;

    std::vector<std::string> input_drivers;
    std::vector<CustomWidgetDef> custom_input_drivers, custom_service_widgets;
    std::vector<ServiceWidgetConfig> service_widgets;

    Json::Value input_defaults, service_defaults;

    // service refs
    void requestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv_clear_file_cache;

    std::string git_head_sha, latest_git_tag;

    // void timer_callback();
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // size_t count_;
};