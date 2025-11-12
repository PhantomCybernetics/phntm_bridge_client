#pragma once

#include <cstdio>
#include <vector>
#include <fmt/core.h>
#include <json/json.h>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "const.hpp"
#include "lib.hpp"
#include "config.hpp"

// #include <future>
// #include "std_msgs/msg/string.hpp"
// #include <memory>
#include "std_srvs/srv/trigger.hpp"
#include "sio_socket.h"

namespace phntm {

  class BridgeSocket;

  class PhntmBridge : public rclcpp::Node
  {
    public:
      PhntmBridge(std::string node_name, rclcpp::NodeOptions node_options, std::shared_ptr<BridgeConfig> config);
      ~PhntmBridge();
      void loadConfig(std::shared_ptr<BridgeConfig> config);
      rclcpp::QoS loadTopicQoSConfig(std::string topic, size_t default_depth=1, std::string default_reliability="BEST_EFFORT", std::string default_durability="VOLATILE", float default_lifespan_sec=-1.0);
      BridgeConfig::MediaTopicConfig loadMediaTopicConfig(std::string topic, std::string msg_type);

      std::vector<std::string> getAllConfigPrefixes(); // finds topic/service prefixes in the config
      sio::message::ptr loadPrefixedUIConfig(std::string prefix); // prefix is topic or service
      void setupLocalServices();
      void callGenericService(std::string service_name, std::string service_type, double timeout_sec, sio::event const& ev);
      void readGitRepoHead(std::string repo_path);

      std::shared_ptr<BridgeConfig> config = std::make_shared<BridgeConfig>();
      rclcpp::CallbackGroup::SharedPtr introspection_reentrant_group;
      rclcpp::CallbackGroup::SharedPtr media_reentrant_group;

    private:
      bool shutting_down = false;

      // local service refs
      void srvRequestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
      std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv_clear_file_cache;

      // generic service handling
      std::mutex srv_init_mutex; // make sure we don't init the same service types & client in paralel
      std::map<std::string, void *> srv_library_handle_cache; // package_name / package_name+"_introspection" => handle to open library
      struct SrvTypeCache {
        const rosidl_message_type_support_t* request;
        const rosidl_message_type_support_t* response;
      };
      std::map<std::string, SrvTypeCache> srv_types_cache; // service_name => reg/res type_support
      struct SrvClientCache {
        rcl_client_t * client;
        std::mutex * mutex; // makes sure we don't the service before another call finishes
      };
      std::map<std::string, SrvClientCache> srv_client_cache; // service_name => client instance & mutex
      void returnServiceError(std::string message, sio::event const& ev);
      void clearServicesCache();
  };

}