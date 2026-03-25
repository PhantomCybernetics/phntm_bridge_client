#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/sio.hpp"

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "sio_message.h"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <dlfcn.h>
#include <fmt/core.h>
#include <iterator>
#include <mutex>

#include "action_msgs/msg/goal_status.hpp"

#include <rcl/types.h>
#include <rcl_action/action_client.h>
#include <rcl_action/action_server.h>
#include <rcl_action/types.h>
#include <rclcpp/context.hpp>

#include <rmw/types.h>
#include <string>

#include <uuid/uuid.h>

#include <rcl_action/wait.h>

namespace phntm {

  std::string GetGoalStatusHR(int8_t status);

  // calling generic ROS services from the socket.io
  // running on an async thread
  void PhntmBridge::callGenericAction(std::string action_name, std::string action_type, double timeout_sec, sio::event const &ev, const std::string id_peer) {
    if (this->config->service_calls_verbose)
      log("Handling action call for: "+ action_name + " {" + action_type + "}, timeout=" + std::to_string(timeout_sec) + "s, msg_id=" + std::to_string(ev.get_msgId()));

    auto parts = split(action_type, '/');
    std::string package_name = parts[0];
    std::string act_name = parts[2];
    rcl_ret_t ret;
    int64_t timeout_ns = timeout_sec * 1000000000;

    if (this->config->service_calls_verbose)
      log("package_name: " + package_name + " action_name: " + act_name);

    ActionClientCache * client;
    
    auto context = this->get_node_base_interface()->get_context().get()->get_rcl_context();

    { // srv init mutex
      std::lock_guard<std::mutex> lock(this->srv_init_mutex); // make sure we don't init the same service types & client in paralel

      auto it = this->action_client_cache.find(action_name);
      if (it != this->action_client_cache.end()) { 
        client = it->second; // action client found in cache
      } else { // set up a new action client

        client = new ActionClientCache();
        this->action_client_cache.emplace(action_name, client);
        client->action_name = action_name;

        // load action lib
        void *action_lib_handle;
        if (this->srv_library_handle_cache.find(package_name) != this->srv_library_handle_cache.end()) {
          action_lib_handle = this->srv_library_handle_cache.at(package_name);
        } else {
          std::string lib_name = "lib" + package_name + "__rosidl_typesupport_cpp.so";
          if (this->config->service_calls_verbose)
            log("Loading lib: " + lib_name);
          action_lib_handle = dlopen(lib_name.c_str(), RTLD_LAZY);
          if (!action_lib_handle)
            return returnServiceError(fmt::format("Library load error: {}", dlerror()), ev);
          this->srv_library_handle_cache.emplace( package_name, action_lib_handle); // keep the handle open as long as we need the client
          if (this->config->service_calls_verbose)
            log("Lib " + lib_name + " loaded");
        }

        // get type support handle
        std::string action_symbol_name = "rosidl_typesupport_cpp__get_action_type_support_handle__" + package_name + "__action__" + act_name;
        if (this->config->service_calls_verbose)
          log("Getting get_action_ts " + action_symbol_name);
        auto get_action_ts = reinterpret_cast<const rosidl_action_type_support_t *(*)()>(dlsym(action_lib_handle, action_symbol_name.c_str()));
        if (!get_action_ts)
          return this->returnServiceError(fmt::format("Symbol load error: {}", dlerror()), ev);

        // get action type support
        const rosidl_action_type_support_t *action_type_support = get_action_ts();
        if (!action_type_support)
          return this->returnServiceError("Failed to get type support", ev);

        if (this->config->service_calls_verbose)
          log("Initializing " + action_name + " client...");

        client->rcl_client = new rcl_action_client_t();
        rcl_action_client_options_t options = rcl_action_client_get_default_options();
        //options.status_topic_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE; // https://github.com/ros2/rcl/issues/1155
        ret = rcl_action_client_init(client->rcl_client, this->get_node_base_interface()->get_rcl_node_handle(), action_type_support, action_name.c_str(), &options);
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Client init failed: {}", rcl_get_error_string().str), ev);
        
        // type support
        const rosidl_message_type_support_t *goal_request_ts, *goal_response_ts, *goal_ts, *result_request_ts, *result_response_ts; //, *feedback_ts, *goal_status_ts;
        void *introspection_handle = this->getIntrospectionHandle(package_name, ev);
        if (!introspection_handle) return;
        goal_request_ts = this->getSymbolTypeSupport("rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__action__" + act_name + "_SendGoal_Request", introspection_handle, ev);
        if (!goal_request_ts) return;
        goal_response_ts = this->getSymbolTypeSupport("rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__action__" + act_name + "_SendGoal_Response", introspection_handle, ev);
        if (!goal_response_ts) return;
        goal_ts = this->getSymbolTypeSupport("rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__action__" + act_name + "_Goal", introspection_handle, ev);
        if (!goal_ts) return;
        result_request_ts = this->getSymbolTypeSupport("rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__action__" + act_name + "_GetResult_Request", introspection_handle, ev);
        if (!result_request_ts) return;
        result_response_ts = this->getSymbolTypeSupport("rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__action__" + act_name + "_GetResult_Response", introspection_handle, ev);
        if (!result_response_ts) return;
        //feedback_ts = this->getSymbolTypeSupport("rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__action__" + act_name + "_FeedbackMessage", introspection_handle, ev);
        //if (!feedback_ts) return;
        // goal_status_ts = this->getSymbolTypeSupport("rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__action__" + act_name + "_GoalStatusArray", introspection_handle, ev);
        // if (!goal_status_ts) return;

        client->verbose = this->config->service_calls_verbose;
        client->mapping_verbose = this->config->service_call_mapping_verbose;

        client->guard_condition = rcl_get_zero_initialized_guard_condition();

        ret = rcl_guard_condition_init(&client->guard_condition, context.get(), rcl_guard_condition_get_default_options());
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Client error initializeind guard_condition: {}", rcl_get_error_string().str), ev);

        client->goal_request_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(goal_request_ts->data);
        client->goal_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(goal_ts->data);
        client->goal_response_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(goal_response_ts->data);
        client->result_request_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(result_request_ts->data);
        client->result_response_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(result_response_ts->data);
        //client->feedback_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(feedback_ts->data);

        ret = rcl_action_client_set_goal_client_callback(client->rcl_client, PhntmBridge::OnAction_SetGoal_Response, client);
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Client error setting goal_client_callback: {}", rcl_get_error_string().str), ev);
        ret = rcl_action_client_set_result_client_callback(client->rcl_client, PhntmBridge::OnAction_Result_Response, client);
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Client error setting result_client_callback: {}", rcl_get_error_string().str), ev);
        ret = rcl_action_client_set_cancel_client_callback(client->rcl_client, PhntmBridge::OnAction_Cancel, client);
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Client error setting cancel_client_callback: {}", rcl_get_error_string().str), ev);
        // ret = rcl_action_client_set_feedback_subscription_callback(client_goal_data->client, PhntmBridge::_OnActionFeedback, client_goal_data);
        // if (ret != RCL_RET_OK)
        //   return this->returnServiceError(fmt::format("Client error setting subscription_callbac: {}", rcl_get_error_string().str), ev);
        // ret = rcl_action_client_set_status_subscription_callback(client_goal_data->client, PhntmBridge::_OnActionStatus, client_goal_data);
        // if (ret != RCL_RET_OK)
        //   return this->returnServiceError(fmt::format("Client error setting status_subscription_callback: {}", rcl_get_error_string().str), ev);
 
        if (this->config->service_calls_verbose)
          log("Client callbacks init ok");
      } // client init done
    } // srv_init_mutex scope

    ActionGoal goal;
    goal.reply_sio_ev = const_cast<sio::event *>(&ev);
    goal.id_peer = id_peer;
    uuid_generate_random(goal.uuid);
    goal.processing = true;

    if (this->config->service_calls_verbose)
      log("Created goal with uuid " + toString(goal.uuid));

    // allocate memory for the request
    log("Allocating " + std::to_string(client->goal_request_members->size_of_) + " B");
    void *goal_request_msg = malloc(client->goal_request_members->size_of_);
    log("malloc ok");
    client->goal_request_members->init_function(goal_request_msg, rosidl_runtime_cpp::MessageInitialization::ALL);
    if (this->config->service_calls_verbose)
      log(YELLOW + "Goal request initial alloc ok (" + std::to_string(client->goal_request_members->size_of_) + "B) for " + action_name + CLR);
    
    auto request_data = ev.get_message()->get_map()["msg"];
    if (request_data == nullptr) { // mepty or trigger may send this => init as null message type
      if (this->config->service_calls_verbose)
        log("Action call message was null");
      request_data = sio::null_message::create();
    }

    // populate goal
    bool uuid_set = false;
    for (size_t i = 0; i < client->goal_request_members->member_count_; i++) {
    
      auto key = std::string(client->goal_request_members->members_[i].name_);
      auto value_ptr = static_cast<uint8_t *> (goal_request_msg) + client->goal_request_members->members_[i].offset_;
      auto type = client->goal_request_members->members_[i].type_id_;

      if (key == "goal") {

        auto req_err = PhntmBridge::SocketToROSMessage(request_data, value_ptr, client->goal_members, this->config->service_call_mapping_verbose);
        if (!req_err.empty())
          return this->returnServiceError(fmt::format("Error mapping request data: {}", req_err.c_str()), ev);

      } if (key == "goal_id" && type == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) { // goal_id is obj
        const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(client->goal_request_members->members_[i].members_->data);

        for (size_t j = 0; j < nested_members->member_count_; j++) {
          auto nested_key = std::string(nested_members->members_[j].name_);
          auto nested_value_ptr = static_cast<uint8_t *> (goal_request_msg) + nested_members->members_[j].offset_;
          auto nested_type = nested_members->members_[j].type_id_;
          auto is_array = nested_members->members_[j].is_array_;
          if (nested_key == "uuid" && (nested_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE || nested_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8)  && is_array) {
            std::copy(std::begin(goal.uuid), std::end(goal.uuid), nested_value_ptr);
            uuid_set = true;
            if (this->config->service_calls_verbose)
              log("  > Goal UUID set to " + toString(goal.uuid));
          }
        }
      }
    }
    if (!uuid_set) {
      log("Failed to set goal_request uuid for " + action_name + ", unexpected goal_request format!", true);
      return;
    }
    
    {  // call scope
      auto is_available = false;
      while (!is_available) {
        ret = rcl_action_server_is_available(this->get_node_base_interface()->get_rcl_node_handle(), client->rcl_client, &is_available); 
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Error waiting for the server: {}", rcl_get_error_string().str), ev);
      }
    
      log(BLUE + "Calling action: " + action_name + CLR + " {" + action_type + "} msg_id=" + std::to_string(ev.get_msgId()));

      // send the request
      int64_t sequence_number;
      ret = rcl_action_send_goal_request(client->rcl_client, goal_request_msg, &sequence_number);
      if (ret != RCL_RET_OK)
        return this->returnServiceError(fmt::format("Failed to send request: {}", rcl_get_error_string().str), ev);

      if (this->config->service_calls_verbose)
        log("Action goal " + action_name + " sent ok, sequence_number=" + std::to_string(sequence_number));

      client->setgoal_sequence.emplace(sequence_number, &goal);
      
      rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
      ret = rcl_wait_set_init(&wait_set,
                            2, // number of subscriptions
                            1, // number of guard conditions
                            0, // number of timers
                            3, // number of clients
                            0, // number of services
                            0, // number of events
                            context.get(),
                            rcl_get_default_allocator()); // Allocator
      if (ret != RCL_RET_OK)
        return this->returnServiceError(fmt::format("Failed to initialize wait set: {}", rcl_get_error_string().str), ev);

      if (this->config->service_calls_verbose)
        log(GRAY+ "Starting wait_set loop for " + action_name + "..." + CLR);

      while (goal.processing) {

        ret = rcl_wait_set_clear(&wait_set);
        if (ret != RCL_RET_OK)
          log("Error cleaning wait_set", true);
  
        ret = rcl_action_wait_set_add_action_client(&wait_set, client->rcl_client, NULL, NULL);
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Failed to add client to wait set: {}", rcl_get_error_string().str), ev);

        size_t guard_index;
        ret = rcl_wait_set_add_guard_condition(&wait_set, &client->guard_condition, &guard_index);
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Failed to add guard condition to wait set: {}", rcl_get_error_string().str), ev);

        ret = rcl_wait(&wait_set, timeout_ns);
        if (ret == RCL_RET_TIMEOUT) {
          log(RED + "Action call timed out after "+std::to_string(timeout_sec)+"s" + CLR);
          break;
        }
        else if (ret != RCL_RET_OK) {
          log(fmt::format("Error during wait: {}", rcl_get_error_string().str), true);
          break;
        }

        bool feedback_ready = false, status_ready = false, goal_response_ready = false, cancel_response_ready = false, result_response_ready = false;
        ret = rcl_action_client_wait_set_get_entities_ready(&wait_set, client->rcl_client,
          &feedback_ready,
          &status_ready,
          &goal_response_ready,
          &cancel_response_ready,
          &result_response_ready);
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Failed checking wait set: {}", rcl_get_error_string().str), ev);
        
        size_t num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services;
        ret = rcl_action_client_wait_set_get_num_entities(client->rcl_client, &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients, &num_services);
        if (ret != RCL_RET_OK)
          return this->returnServiceError(fmt::format("Failed checking wait set entity numbers: {}", rcl_get_error_string().str), ev);
      
      } // action wait_set loop done

      if (this->config->service_calls_verbose) {
        log(GRAY + "Action " + action_name+ " goal " + toString(goal.uuid) + " done." + CLR);
      }

      ret = rcl_wait_set_clear(&wait_set);
      if (ret != RCL_RET_OK)
        log("Error cleaning wait_set");

      ret = rcl_wait_set_fini(&wait_set);
      if (ret != RCL_RET_OK)
        return this->returnServiceError(fmt::format("Error clearing wait set: {}", rcl_get_error_string().str), ev);
    
    } // call scope end

    free(goal_request_msg);  // And goal_response_msg if alloc'd
  }

  void PhntmBridge::cancelGenericActionGoal(std::string action_name, sio::event const& ev, const std::string id_peer) {
    if (this->config->service_calls_verbose)
      log(BLUE + "Processing action "+ action_name +" goal cancel"+ CLR);

    ActionClientCache * client;
    auto it = this->action_client_cache.find(action_name);
    if (it == this->action_client_cache.end())
      return this->returnServiceError(fmt::format("Client not found for action : {}", action_name), ev);
    client = it->second;

    rcl_ret_t ret;
    auto is_available = false;
    while (!is_available) {
      ret = rcl_action_server_is_available(this->get_node_base_interface()->get_rcl_node_handle(), client->rcl_client, &is_available); 
      if (ret != RCL_RET_OK)
        return this->returnServiceError(fmt::format("Error waiting for the server: {}", rcl_get_error_string().str), ev);
    }

    if (this->config->service_calls_verbose)
      log("Action server available for cancel");

    auto src_msg = ev.get_message()->get_map()["msg"];
    if (src_msg->get_map().find("goal_uuid") == src_msg->get_map().end())
      return this->returnServiceError(fmt::format("Action goal_uuid not provided in cancel message"), ev);
    auto goal_uuid = src_msg->get_map()["goal_uuid"];
    if ((!goal_uuid->flag_integer && !goal_uuid->flag_double) || !goal_uuid->flag_array || goal_uuid->get_vector().size() != 16)
      return this->returnServiceError(fmt::format("Action goal_uuid invalid cancel message"), ev);

    action_msgs__srv__CancelGoal_Request cancel_request_msg = rcl_action_get_zero_initialized_cancel_request();    
    for (size_t i = 0; i < 16; i++) {
      if (goal_uuid->flag_integer) {
        auto one = goal_uuid->get_vector()[i]->get_int();
        cancel_request_msg.goal_info.goal_id.uuid[i] = static_cast<uint8_t>(one & 0xFF);
      } else {
        auto one = goal_uuid->get_vector()[i]->get_double();
        cancel_request_msg.goal_info.goal_id.uuid[i] = static_cast<uint8_t>(std::round(one));
      }
    }
    std::string uuid_str = toString( cancel_request_msg.goal_info.goal_id.uuid);

    int64_t sequence_number;
    ret = rcl_action_send_cancel_request(client->rcl_client, &cancel_request_msg, &sequence_number);
    if (ret != RCL_RET_OK)
      return this->returnServiceError(fmt::format("Error cancelling action goal {} for {}: {}", uuid_str, action_name, rcl_get_error_string().str), ev);

    if (this->config->service_calls_verbose)
      log("Cancel call finished ok for " + action_name + " goal " + uuid_str);

    auto ack_msg = sio::object_message::create(); // always obj
    ack_msg->get_map().emplace("goal_uuid", sio::string_message::create(uuid_str));
    ack_msg->get_map().emplace("success", sio::bool_message::create(true));
    BridgeSocket::ack(ev.get_msgId(), { ack_msg });
  }


  void PhntmBridge::OnAction_SetGoal_Response(const void * client_ptr, size_t number_of_events) {
    
    // READ SET GOAL RESPONSE

    PhntmBridge::ActionClientCache * client = static_cast<PhntmBridge::ActionClientCache *>(const_cast<void *>(client_ptr));
    std::lock_guard<std::mutex> lock(client->mutex);

    if (client->verbose)
      log(MAGENTA + " >> SET GOAL RESPONSE [" + std::to_string(number_of_events) + "]" + CLR);

    // alloc response msg
    void *goal_response_msg;
    goal_response_msg = malloc(client->goal_response_members->size_of_);
    client->goal_response_members->init_function(goal_response_msg, rosidl_runtime_cpp::MessageInitialization::ALL);
          
    if (client->verbose)
      log("Response init ok for " + client->action_name);

    rmw_request_id_t response_header;
    rcl_ret_t ret = rcl_action_take_goal_response(client->rcl_client, &response_header, goal_response_msg);
    if (ret != RCL_RET_OK) {
      free(goal_response_msg);
      log(fmt::format("Failed to take response: {}", rcl_get_error_string().str), true);
      return;
    }

    if (client->verbose)
      log("Has goal response from " + client->action_name +" sequence_number=" + std::to_string(response_header.sequence_number));

    auto it = client->setgoal_sequence.find(response_header.sequence_number);
    if (it == client->setgoal_sequence.end()) {
      log(RED + "Sequence_number=" + std::to_string(response_header.sequence_number)+" not found in setgoal_sequence; ignoring");
      return;
    }
    auto goal = it->second;
    client->setgoal_sequence.erase(it);

    // SEND SIO REPLY

    // map response to socket message
    auto ack_msg = sio::object_message::create(); // always obj
    auto res_err = ROSToSocketMessage(goal_response_msg, client->goal_response_members, ack_msg, client->mapping_verbose);
    if (!res_err.empty()) {
      free(goal_response_msg);
      //return this->returnServiceError(), ev);
      log(fmt::format("Error mapping result data: {}", res_err.c_str()), true);
      return;
    }
    
    if (client->verbose) {
      if (ack_msg->get_map().find("accepted") == ack_msg->get_map().end() || !ack_msg->get_map().at("accepted")->get_bool()) {
        log("Goal " + RED + "REFUSED" + CLR + " for " + client->action_name);
      } else {
        log("Goal " + GREEN + "ACCEPTED" + CLR + " for " + client->action_name);
      }
    }
    ack_msg->get_map().emplace("goal_uuid", sio::string_message::create(toString(goal->uuid)));

    // send the reply
    if (client->verbose)
      log(BLUE + "Sending " + client->action_name + " goal reply, goal_uuid="+toString(goal->uuid)+" ..." + CLR);

    BridgeSocket::ack(goal->reply_sio_ev->get_msgId(), {ack_msg});
    goal->reply_sio_ev = NULL; // only once
    free(goal_response_msg);

    // SEND RESULT REQUEST

    void *result_request_msg;
    result_request_msg = malloc(client->result_request_members->size_of_);

    bool uuid_set = false;
    for (size_t i = 0; i < client->result_request_members->member_count_; i++) {
    
      auto key = std::string(client->result_request_members->members_[i].name_);
      //auto value_ptr = static_cast<uint8_t *> (result_request_msg) + goal_data->result_request_members->members_[i].offset_;
      auto type = client->result_request_members->members_[i].type_id_;

      log("  result_request_msg." + key + " type="+std::to_string(type));

      //log("result_request_msg." + key + " type="+std::to_string(type));
      if (key == "goal_id" && type == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(client->result_request_members->members_[i].members_->data);
        for (size_t j = 0; j < nested_members->member_count_; j++) {

          auto nested_key = std::string(nested_members->members_[j].name_);
          auto nested_value_ptr = static_cast<uint8_t *> (result_request_msg) + nested_members->members_[j].offset_;
          auto nested_type = nested_members->members_[j].type_id_;
          auto is_array = nested_members->members_[j].is_array_;
          
          log("  result_request_msg." + key + "." + nested_key + " "+(is_array?"array ":"")+"type="+std::to_string(nested_type));
          if (nested_key == "uuid" && (nested_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE || nested_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8)  && is_array) {
            std::copy(std::begin(goal->uuid), std::end(goal->uuid), nested_value_ptr);
            uuid_set = true;
            if (client->verbose)
              log("  > Goal UUID set to " + toString(goal->uuid));
          }
        }
      }
    }
    if (!uuid_set) {
      log("Failed to set result_request uuid for " + client->action_name + ", unexpected result_request format!", true);
      return;
    }

    int64_t sequence_number;
    ret = rcl_action_send_result_request(client->rcl_client, result_request_msg, &sequence_number);
    if (ret != RCL_RET_OK) {
      log(fmt::format("Error sending action result request: {}", rcl_get_error_string().str), true);
      return;
    }

    log("Action result request for " + client->action_name + " sent ok, sequence_number=" + std::to_string(sequence_number));
    client->result_sequence.emplace(sequence_number, goal);

    // TRIGGER GUARD CONDITION

    ret = rcl_trigger_guard_condition(&client->guard_condition);
    if (ret != RCL_RET_OK) {
      log(fmt::format("Failed to trigger guard condition: {}", rcl_get_error_string().str));
    }
  }

  void PhntmBridge::OnAction_Result_Response(const void * client_ptr, size_t number_of_events) {

    PhntmBridge::ActionClientCache * client = static_cast<PhntmBridge::ActionClientCache *>(const_cast<void *>(client_ptr));
    std::lock_guard<std::mutex> lock(client->mutex);

    if (client->verbose)
      log(MAGENTA + " >> RESULT RESPONSE [" + std::to_string(number_of_events) + "]" + CLR);

    // alloc response msg
    void *result_response_msg;
    result_response_msg = malloc(client->result_response_members->size_of_);
    client->result_response_members->init_function(result_response_msg, rosidl_runtime_cpp::MessageInitialization::ALL);
    
    // if (this->config->service_calls_verbose)
    //   log("Feedback init ok for " + action_name);
    rmw_request_id_t response_header;
    rcl_ret_t ret = rcl_action_take_result_response(client->rcl_client, &response_header, result_response_msg);
    if (ret != RCL_RET_OK) {
      log(fmt::format("Failed to take feedback: {}", rcl_get_error_string().str));
      free(result_response_msg);
      return;
    }

    if (client->verbose)
      log("Has result response from " + client->action_name +" sequence_number=" + std::to_string(response_header.sequence_number));

    auto it = client->result_sequence.find(response_header.sequence_number);
    if (it == client->result_sequence.end()) {
      log(RED+"Sequence_number=" + std::to_string(response_header.sequence_number)+" not found in result_sequence; ignoring");
      return;
    }
    auto goal = it->second;
    client->result_sequence.erase(it);

    // READ STATUS

    int8_t status = -1;
    bool status_taken = false;
    for (size_t i = 0; i < client->result_response_members->member_count_; i++) {
    
      auto key = std::string(client->result_response_members->members_[i].name_);
     
      auto type = client->result_response_members->members_[i].type_id_;
      auto is_array = client->result_response_members->members_[i].is_array_;

      if (client->verbose)
        log("  result_response_msg." + key + " ["+std::to_string(type)+"] " + (is_array?" is_array":""));

      if (key == "status" && type == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8) {
          status_taken = true;
          auto value_ptr = static_cast<int8_t *> (result_response_msg) + client->result_response_members->members_[i].offset_;
          status = *value_ptr;
      }
    }
    if (!status_taken) {
        log("Faild to take status from result_response_msg, unknown message format", true);
    }

    if (client->verbose)
      log("Goal " + YELLOW + toString(goal->uuid) + CLR + " of " + client->action_name + " done with status=" + GetGoalStatusHR(status));
    
    // REPORT ACTION RESULT
    //goal_data->result_reply_received = true;
    auto result_msg = sio::object_message::create(); // always obj
    result_msg->get_map().emplace("goal_uuid", sio::string_message::create(toString(goal->uuid)));
    result_msg->get_map().emplace("status", sio::int_message::create(status));
    result_msg->get_map().emplace("id_peer", sio::string_message::create(goal->id_peer));

    if (client->verbose)
      log(BLUE + "Sending " + client->action_name + " result to peer "+ goal->id_peer + ", goal_uuid="+toString(goal->uuid)+" ..." + CLR);

    BridgeSocket::emit("action:result", result_msg, nullptr); // no callback

    goal->processing = false; // kills the loop

    free(result_response_msg);
  }

  // handled by result response
  void PhntmBridge::OnAction_Cancel(const void * client_ptr, size_t number_of_events) {
    PhntmBridge::ActionClientCache * client = static_cast<PhntmBridge::ActionClientCache *>(const_cast<void *>(client_ptr));
    if (client->verbose)
      log(GRAY + " >> CANCEL [" + std::to_string(number_of_events) + "]" + CLR); 
  }

  // unused, faulty and causing issues
  // void PhntmBridge::_OnActionStatus(const void * user_data, size_t number_of_events) {
    
  //   PhntmBridge::ActionClientCache * goal_data = static_cast<PhntmBridge::ActionClientCache *>(const_cast<void *>(user_data));
  //   std::lock_guard<std::mutex> status_lock(goal_data->mutex);

  //   if (goal_data->verbose)
  //    log(MAGENTA + " >> STATUS [" + std::to_string(number_of_events) + "]" + CLR);

  //   action_msgs__msg__GoalStatusArray  status_array; // = action_msgs__msg__GoalStatusArray__create();
  //   //status_array = static_cast<action_msgs__msg__GoalStatusArray *>(malloc(sizeof(action_msgs__msg__GoalStatusArray)));
  //   action_msgs__msg__GoalStatusArray__init(&status_array);
  //   //action_msgs__msg__GoalStatusArray__init(&status_array);
  //   //action_msgs__msg__GoalStatus__Sequence__init(&status_array->status_list, number_of_events);
  //   // //action_msgs__msg__GoalStatusArray__create();
  //   // rcl_ret_t ret = rcl_action_goal_status_array_init(&status_array, number_of_events, rcl_get_default_allocator());
  //   // if (ret != RCL_RET_OK) {
  //   //   log(fmt::format("Failed to initialize status array: {}", rcl_get_error_string().str));
  //   //   return;
  //   // }

  //   //log(fmt::format(YELLOW + "GoalStatus before size={} capacity={}" + CLR, status_array->status_list.size, status_array->status_list.capacity));
  //   rcl_ret_t ret = rcl_action_take_status(goal_data->client, &status_array);
  //   if (ret != RCL_RET_OK) {
  //     log(fmt::format("Failed to take status: {}", rcl_get_error_string().str));
  //     return;
  //   }
  //   log(fmt::format(YELLOW + "GoalStatus after size={} capacity={}" + CLR, status_array.status_list.size, status_array.status_list.capacity));

  //   //action_msgs__msg__GoalStatusArray * status_array = action_msgs__msg__GoalStatusArray__create();
  //   //action_msgs__msg__GoalStatusArray__Sequence__init(&status_array->status_list.data, number_of_events);
  //   //status_array->status_list

  //   // log(fmt::format(YELLOW + "GoalStatus after size {} capacity {}" + CLR, status_list->size, status_list->capacity));

  //   for (size_t j = 0; j < number_of_events; j++) {
  //       //aciton_msgs/StatusArray

  //     auto status = status_array.status_list.data[j];
      
  //     std::string status_hr = GetStatusHR(status.status);

  //     if (uuidsEqual(status.goal_info.goal_id.uuid, goal_data->goal_uuid)) {

  //       // log("Action " + action_name+ " status list capacity=" + std::to_string(status_array.msg.status_list.capacity)
  //       //         +"; size=" + std::to_string(status_array.msg.status_list.size)
  //       //         +"; status=" + std::to_string(status->status)
  //       //         +"; uuid=" + toString(status->goal_info.goal_id.uuid)
  //       //       );

  //       switch (status.status) {
  //         case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
  //         case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
  //         case action_msgs::msg::GoalStatus::STATUS_CANCELING:
  //           break;
  //         case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
  //         case action_msgs::msg::GoalStatus::STATUS_CANCELED:
  //         case action_msgs::msg::GoalStatus::STATUS_ABORTED:
  //         default:
  //           goal_data->status_complete = true;
  //           break;
  //       }
  
  //       if (goal_data->verbose)
  //         log( + "Action " + goal_data->action_name+ " status is: " + status_hr);

  //     } else {
  //       log(RED + "Ignoring status "+ status_hr + " for goal " + toString(status.goal_info.goal_id.uuid) + " (expecting "+ toString(goal_data->goal_uuid) +")" + CLR);
  //     }

  //   }

  //   // log("Clearing status array...");
  //   // action_msgs__msg__GoalStatus__Sequence__destroy(&status_array.status_list);
  //   // log("... status list clear...");
  //   // action_msgs__msg__GoalStatusArray__destroy(status_array);
  //   // log("... status array clear.");
  
  //   //free(status_array);
    
  //   //action_msgs__msg__GoalStatusArray__fini(&status_array);
  //   //log("action_msgs__msg__GoalStatusArray__Sequence__fini");
  //   //action_msgs__msg__GoalStatusArray__Sequence__fini(status_list);
  //   //log("action_msgs__msg__GoalStatusArray__Sequence__destroy");
  //   //action_msgs__msg__GoalStatusArray__Sequence__destroy(status_list);
  //   //action_msgs__msg__GoalStatusArray__fini(goal_status_array);
  //   // action_msgs__msg__GoalStatusArray__destroy(goal_status_array);

  //   ret = rcl_trigger_guard_condition(&goal_data->guard_condition);
  //   if (ret != RCL_RET_OK) {
  //     log(fmt::format("Failed to trigger guard condition: {}", rcl_get_error_string().str));
  //   }
  // }

  // unused 
  // void PhntmBridge::_OnActionFeedback(const void * user_data, size_t number_of_events) {
  //   log(MAGENTA + " >> FEEDBACK [" + std::to_string(number_of_events) + "]" + CLR);
  //   PhntmBridge::ActionClientCache * goal_data = static_cast<PhntmBridge::ActionClientCache *>(const_cast<void *>(user_data));

  //   if (goal_data->verbose)
  //     log("Action " + goal_data->action_name+ " got feedback (draining)");
          
  //   // alloc response msg
  //   void *feedback_msg;
  //   feedback_msg = malloc(goal_data->feedback_members->size_of_);
  //   goal_data->feedback_members->init_function(feedback_msg, rosidl_runtime_cpp::MessageInitialization::ALL);
    
  //   rcl_ret_t ret = rcl_action_take_feedback(goal_data->client, feedback_msg);
  //   free(feedback_msg);
  //   if (ret != RCL_RET_OK) {
  //     log(fmt::format("Failed to take feedback: {}", rcl_get_error_string().str));
  //     return;
  //   }
  // }

  std::string GetGoalStatusHR(int8_t status) {
    std::string status_hr;
    switch (status) {
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED: status_hr = GREEN + "ACCEPTED" + CLR; break;
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING: status_hr = MAGENTA + "EXECUTING" + CLR; break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELING: status_hr = CYAN + "CANCELING" + CLR; break;
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED: status_hr = GREEN + "SUCCEEDED" + CLR; break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELED: status_hr = CYAN + "CANCELED" + CLR; break;
        case action_msgs::msg::GoalStatus::STATUS_ABORTED: status_hr = GREEN + "ABORTED" + CLR; break;
        default: status_hr = RED + "UNKNOWN" + CLR; break;
      }
    return status_hr;
  }
}