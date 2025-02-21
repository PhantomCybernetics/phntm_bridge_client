#pragma once

#include <cstdio>
#include <vector>
#include <fmt/core.h>
#include <json/json.h>

#include "rclcpp/rclcpp.hpp"

#include "const.hpp"
#include "lib.hpp"
#include "config.hpp"

// #include <future>
// #include "std_msgs/msg/string.hpp"
// #include <memory>
#include "std_srvs/srv/trigger.hpp"
#include "sio_socket.h"

class BridgeSocket;

class PhntmBridge : public rclcpp::Node
{
  public:
    PhntmBridge(std::string node_name, std::shared_ptr<BridgeConfig> config);
    void loadConfig(std::shared_ptr<BridgeConfig> config);
    void makeServices();
    void callService(std::string service_name, std::string service_type, std::shared_ptr<BridgeSocket> sio, sio::event const& ev);
    void readGitRepoHead(std::string repo_path);

    std::shared_ptr<BridgeConfig> config = std::make_shared<BridgeConfig>();
    // std::future<int> async_function(int x);
    
  private:
    bool shutting_down = false;

    // service refs
    void requestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv_clear_file_cache;

    // void timer_callback();
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // size_t count_;
};