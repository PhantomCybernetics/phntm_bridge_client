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
    ~PhntmBridge();
    void loadConfig(std::shared_ptr<BridgeConfig> config);
    void setupLocalServices();
    void callGenericService(std::string service_name, std::string service_type, std::shared_ptr<BridgeSocket> sio, sio::event const& ev);
    void readGitRepoHead(std::string repo_path);

    std::shared_ptr<BridgeConfig> config = std::make_shared<BridgeConfig>();
    
  private:
    bool shutting_down = false;

    // local service refs
    void srvRequestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv_clear_file_cache;

    // generic service handling
    std::map<std::string, void *> srv_library_handle_cache; // package_name / package_name+"_introspection" => handle to open library
    std::map<std::string, rcl_client_t *> srv_client_cache; // service_name => client instance
    struct SrvTypeCache {
      const rosidl_message_type_support_t* request;
      const rosidl_message_type_support_t* response;
    };
    std::map<std::string, SrvTypeCache> srv_types_cache; // service_name => reg/res type_support
    void returnServiceError(std::string message, std::shared_ptr<BridgeSocket> sio, sio::event const& ev);
    void clearServicesCache();
};