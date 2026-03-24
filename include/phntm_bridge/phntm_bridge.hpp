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

#include <rcl_action/action_client.h>

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
      void getExtraCustomParams(sio::object_message::ptr out_msg);
      void setupLocalServices();

      void callGenericService(std::string service_name, std::string service_type, double timeout_sec, sio::event const& ev);
      void callGenericAction(std::string action_name, std::string service_type, double timeout_sec, sio::event const& ev, const std::string id_peer);
      void cancelGenericActionGoal(std::string action_name, sio::event const& ev, const std::string id_peer);
      void readGitRepoHead(std::string repo_path);

      std::shared_ptr<BridgeConfig> config = std::make_shared<BridgeConfig>();
      rclcpp::CallbackGroup::SharedPtr introspection_reentrant_group;
      rclcpp::CallbackGroup::SharedPtr media_reentrant_group;

    private:
      bool shutting_down = false;

      void* getIntrospectionHandle(std::string package_name, sio::event const &ev);
      const rosidl_message_type_support_t* getSymbolTypeSupport(std::string symbol, void *introspection_handle, sio::event const &ev);
      static std::string SocketToROSMessage(const sio::message::ptr &in_data, void *request_msg, const rosidl_typesupport_introspection_cpp::MessageMembers *request_members, bool verbose, int indent = 0);
      static std::string ROSToSocketMessage(const void *response_msg, const rosidl_typesupport_introspection_cpp::MessageMembers *response_members, sio::message::ptr &out_data, bool verbose, int indent = 0);
      static std::string SetROSMessageFieldValue(void *field, const rosidl_typesupport_introspection_cpp::MessageMember *member, sio::message::ptr value, size_t index = 0, bool verbose = false, int indent = 0);
      static std::string SetSocketMessageFieldValue(const void *field_ptr, const rosidl_typesupport_introspection_cpp::MessageMember *member, sio::message::ptr out_target, std::string key, bool verbose, int indent);
      static std::string SetOneSocketMessageValue(const void *field_ptr, const rosidl_typesupport_introspection_cpp::MessageMember *member, sio::message::ptr out_target, std::string key, int index, bool verbose, int indent);

      // local service refs
      void srvRequestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
      std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> srv_clear_file_cache;

      // generic service handling
      std::mutex srv_init_mutex; // make sure we don't init the same service types & client in paralel
      std::map<std::string, void *> srv_library_handle_cache; // package_name / package_name+"_introspection" => handle to open library
      // struct SrvTypeCache {
      //   const rosidl_message_type_support_t* request;
      //   const rosidl_message_type_support_t* response;
      // };
      // std::map<std::string, SrvTypeCache> srv_types_cache; // service_name => reg/res type_support

      struct ServiceClientCache {
        rcl_client_t * rcl_client;
        std::string service_name;
        std::mutex mutex; // makes sure we don't the service before another call finishes

        const rosidl_typesupport_introspection_cpp::MessageMembers *request_members;
        const rosidl_typesupport_introspection_cpp::MessageMembers *response_members;
      };
      std::map<std::string, ServiceClientCache *> srv_client_cache; // service_name => client instance & mutex

      std::map<std::string, const rosidl_message_type_support_t*> type_support_cache; // action_name => reg/res type_support

      struct ActionGoal {
        uint8_t uuid[16];
        sio::event * reply_sio_ev;
        std::string id_peer;
        bool processing;
      };
      struct ActionClientCache {
        rcl_action_client_t * rcl_client;
        std::string action_name;
        std::mutex mutex; // makes sure we don't the service before another call finishes

        std::map<int64_t, ActionGoal *> setgoal_sequence;
        std::map<int64_t, ActionGoal *> result_sequence;
        
        bool verbose = false;
        bool mapping_verbose = false;
        
        rcl_guard_condition_t guard_condition;
        const rosidl_typesupport_introspection_cpp::MessageMembers *goal_members;
        const rosidl_typesupport_introspection_cpp::MessageMembers *goal_request_members;
        const rosidl_typesupport_introspection_cpp::MessageMembers *goal_response_members;
        const rosidl_typesupport_introspection_cpp::MessageMembers *result_request_members;
        const rosidl_typesupport_introspection_cpp::MessageMembers *result_response_members;
        //const rosidl_typesupport_introspection_cpp::MessageMembers *feedback_members;
      };
      std::map<std::string, ActionClientCache *> action_client_cache; // action_name => client instance & mutex
    
      static void OnAction_SetGoal_Response(const void * client_ptr, size_t number_of_events);
      static void OnAction_Result_Response(const void * client_ptr, size_t number_of_events);
      static void OnAction_Cancel(const void * client_ptr, size_t number_of_events);
      // static void _OnActionFeedback(const void * goal_data, size_t number_of_events);
      // static void _OnActionStatus(const void * goal_data, size_t number_of_events);

      void returnServiceError(std::string message, sio::event const& ev);
      void clearServicesCache();
  };

}