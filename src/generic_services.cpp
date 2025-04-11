#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/sio.hpp"

#include "rcl/client.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"
#include "sio_message.h"
#include <cstdint>
#include <dlfcn.h>
#include <fmt/core.h>
#include <iostream>
#include <mutex>
#include <rcl/context.h>
#include <rclcpp/context.hpp>
#include <string>


std::string MapSocketMessageToRequest(const sio::message::ptr &in_data, void *request_msg, const rosidl_typesupport_introspection_cpp::MessageMembers *request_members, bool verbose, int indent = 0);
std::string MapResponseToSocketMessage(const void *response_msg, const rosidl_typesupport_introspection_cpp::MessageMembers *response_members, sio::message::ptr &out_data, bool verbose, int indent = 0);
std::string SetRequestFieldValue(void *field, const rosidl_typesupport_introspection_cpp::MessageMember *member, sio::message::ptr value, size_t index = 0, bool verbose = false, int indent = 0);
std::string SetResponseFieldValue(const void *field_ptr, const rosidl_typesupport_introspection_cpp::MessageMember *member, sio::message::ptr out_target, std::string key, bool verbose, int indent);
std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> converter;

// calling generic ROS services from the socket.io
// running on an async thread
void PhntmBridge::callGenericService(std::string service_name, std::string service_type, sio::event const &ev) {

  if (this->config->service_calls_verbose)
    std::cout << "Handling service call for: " << service_name << " {" << service_type << "} msg_id=" << ev.get_msgId() << std::endl;

  auto parts = split(service_type, '/');
  std::string package_name = parts[0];
  std::string srv_name = parts[2];
  rcl_ret_t ret;

  if (this->config->service_calls_verbose)
    std::cout << "package_name: " << package_name << " srv_name: " << srv_name << std::endl;

  rcl_client_t *client;
  std::mutex *client_mutex;
  const rosidl_message_type_support_t *request_introspection_ts, *response_introspection_ts;

  { 
    std::lock_guard<std::mutex> lock(this->srv_init_mutex); // make sure we don't init the same service types & client in paralel

    if (this->srv_client_cache.find(service_name) != this->srv_client_cache.end()) {

      // client instance found in cache
      client = srv_client_cache.at(service_name).client;
      client_mutex = srv_client_cache.at(service_name).mutex;

    } else { // set up a new client

      // load service lib
      void *service_lib_handle;
      if (this->srv_library_handle_cache.find(package_name) != this->srv_library_handle_cache.end()) {
        service_lib_handle = this->srv_library_handle_cache.at(package_name);
      } else {
        std::string lib_name = "lib" + package_name + "__rosidl_typesupport_cpp.so";
        if (this->config->service_calls_verbose)
          std::cout << "Loading lib: " << lib_name << std::endl;
        service_lib_handle = dlopen(lib_name.c_str(), RTLD_LAZY);
        if (!service_lib_handle)
          return returnServiceError(fmt::format("Library load error: {}", dlerror()), ev);
        this->srv_library_handle_cache.emplace( package_name, service_lib_handle); // keep the handle open as long as we need the client
        if (this->config->service_calls_verbose)
          std::cout << "Lib " << lib_name << " loaded" << std::endl;
      }

      // get type support handle
      std::string service_symbol_name = "rosidl_typesupport_cpp__get_service_type_support_handle__" + package_name + "__srv__" + srv_name;
      if (this->config->service_calls_verbose)
        std::cout << "Getting get_service_ts " << service_symbol_name << std::endl;
      auto get_service_ts = reinterpret_cast<const rosidl_service_type_support_t *(*)()>(dlsym(service_lib_handle, service_symbol_name.c_str()));
      if (!get_service_ts)
        return this->returnServiceError(fmt::format("Symbol load error: {}", dlerror()), ev);

      // get service type support
      const rosidl_service_type_support_t *service_type_support = get_service_ts();
      if (!service_type_support)
        return this->returnServiceError("Failed to get type support", ev);

      if (this->config->service_calls_verbose)
        std::cout << "Initializing " << service_name << " client..." << std::endl;
      // initialize rcl client
      client = new rcl_client_t();
      client_mutex = new std::mutex();
      rcl_client_options_t options = rcl_client_get_default_options();
      ret = rcl_client_init(client, this->get_node_base_interface()->get_rcl_node_handle(), service_type_support, service_name.c_str(), &options);

      if (ret != RCL_RET_OK)
        return this->returnServiceError(fmt::format("Client init failed: {}", rcl_get_error_string().str), ev);

      if (this->config->service_calls_verbose)
        std::cout << "Client init ok" << std::endl;
      SrvClientCache cache;
      cache.client = client;
      cache.mutex = client_mutex;
      this->srv_client_cache.emplace(service_name, cache); // cache the client instance for later
    }

    if (this->srv_types_cache.find(service_name) != this->srv_types_cache.end()) {

      // type supports found in cache
      request_introspection_ts = this->srv_types_cache.at(service_name).request;
      response_introspection_ts = this->srv_types_cache.at(service_name).response;

    } else {

      // load service introspection library
      auto id_introslection_lib = package_name + "_introspection";
      void *introspection_handle;
      if (this->srv_library_handle_cache.find(id_introslection_lib) != this->srv_library_handle_cache.end()) {
        // load from cache
        introspection_handle = this->srv_library_handle_cache.at(id_introslection_lib);
      } else {
        std::string introspection_lib_name = "lib" + package_name + "__rosidl_typesupport_introspection_cpp.so";
        introspection_handle = dlopen(introspection_lib_name.c_str(), RTLD_LAZY);
        if (!introspection_handle)
          return this->returnServiceError(fmt::format("Introspection library load error: {}", dlerror()), ev);

        if (this->config->service_calls_verbose)
          std::cout << "Introspection lib loaded ok " << introspection_lib_name << std::endl;
        this->srv_library_handle_cache.emplace(id_introslection_lib, introspection_handle); // keep the handle open as long as we need the client
      }

      // get request type support
      std::string request_introspection_symbol = "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__srv__" + srv_name + "_Request";
      auto get_request_introspection_ts = reinterpret_cast<const rosidl_message_type_support_t *(*)()>(dlsym(introspection_handle, request_introspection_symbol.c_str()));
      if (!get_request_introspection_ts)
        return this->returnServiceError(fmt::format("Request introspection symbol load error: {}", dlerror()), ev);
      request_introspection_ts = get_request_introspection_ts();
      if (this->config->service_calls_verbose)
        std::cout << "Got request type support " << request_introspection_ts->typesupport_identifier << std::endl;

      std::string response_introspection_symbol = "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__srv__" + srv_name + "_Response";
      auto get_response_introspection_ts = reinterpret_cast<const rosidl_message_type_support_t *(*)()>(dlsym(introspection_handle, response_introspection_symbol.c_str()));
      if (!get_response_introspection_ts)
        return this->returnServiceError(fmt::format("Response introspection symbol load error:: {}", dlerror()), ev);

      response_introspection_ts = get_response_introspection_ts();
      if (this->config->service_calls_verbose)
        std::cout << "Got response type support " << response_introspection_ts->typesupport_identifier << std::endl;

      SrvTypeCache cache;
      cache.request = request_introspection_ts;
      cache.response = response_introspection_ts;
      this->srv_types_cache.emplace(service_name, cache); // cache type supports for later
    }
  }

  const rosidl_typesupport_introspection_cpp::MessageMembers *request_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(request_introspection_ts->data);
  
  // allocate memory for the request
  void *request_msg = malloc(request_members->size_of_);
  request_members->init_function(request_msg, rosidl_runtime_cpp::MessageInitialization::ALL);
  if (this->config->service_calls_verbose)
    std::cout << YELLOW << "Request initial alloc ok (" << request_members->size_of_ << "B) for " << service_name << CLR << std::endl;
  auto request_data = ev.get_message()->get_map()["msg"];
  // std::cout << "Setting request data: " << BridgeSocket::PrintMessage(request_data) << std::endl;
  auto req_err = MapSocketMessageToRequest(request_data, request_msg, request_members, this->config->service_calls_verbose);
  if (!req_err.empty())
    return this->returnServiceError(fmt::format("Error mapping request data: {}", req_err.c_str()), ev);

  if (this->config->service_calls_verbose)
    std::cout << "Request data set ok for " << service_name << std::endl;

  const rosidl_typesupport_introspection_cpp::MessageMembers *response_members;
  void *response_msg;
  {
    if (this->config->service_calls_verbose)
      std::cout << "Waiting for previous " << service_name << " calls to finish..." << std::endl;
    // prevent another call to the service until previous call finishes
    std::lock_guard<std::mutex> lock(*client_mutex);

    std::cout << MAGENTA << "Calling service: " << service_name << CLR << " {" << service_type << "} msg_id=" << ev.get_msgId() << std::endl;

    // send the request
    int64_t sequence_number;
    ret = rcl_send_request(client, request_msg, &sequence_number);
    if (ret != RCL_RET_OK)
      return this->returnServiceError(fmt::format("Failed to send request: {}", rcl_get_error_string().str), ev);

    if (this->config->service_calls_verbose)
      std::cout << "Request " << service_name << " sent ok" << std::endl;

    // wait for the response
    auto context = this->get_node_base_interface()->get_context().get()->get_rcl_context();
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    ret = rcl_wait_set_init(&wait_set,
                            0, // number of subscriptions
                            1, // number of guard conditions
                            0, // number of timers
                            1, // number of clients
                            0, // number of services
                            0, // number of events
                            context.get(),
                            rcl_get_default_allocator()); // Allocator
    if (ret != RCL_RET_OK)
      return this->returnServiceError(fmt::format("Failed to initialize wait set: {}", rcl_get_error_string().str), ev);

    if (this->config->service_calls_verbose)
      std::cout << "Adding " << service_name << " client to wait set" << std::endl;

    ret = rcl_wait_set_add_client(&wait_set, client, NULL);
    if (ret != RCL_RET_OK)
      return this->returnServiceError(fmt::format("Failed to add client to wait set: {}", rcl_get_error_string().str), ev);

    if (this->config->service_calls_verbose)
      std::cout << "Waiting for " << service_name << "..." << std::endl;

    // wait for a response (timeout in ns)
    ret = rcl_wait(&wait_set, this->config->service_timeout_ns);
    if (ret == RCL_RET_TIMEOUT)
      return this->returnServiceError("Service call timed out.", ev);
    else if (ret != RCL_RET_OK)
      return this->returnServiceError(fmt::format("Error during wait: {}", rcl_get_error_string().str), ev);
    
    if (this->config->service_calls_verbose)
      std::cout << "Getting " << service_name << " response..." << std::endl;

    // alloc response msg
    response_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(response_introspection_ts->data);
    response_msg = malloc(response_members->size_of_);
    response_members->init_function(response_msg, rosidl_runtime_cpp::MessageInitialization::ALL);
    
    if (this->config->service_calls_verbose)
      std::cout << "Response init ok for " << service_name << std::endl;

    rmw_request_id_t request_header;
    ret = rcl_take_response(client, &request_header, response_msg);
    if (ret != RCL_RET_OK)
      return this->returnServiceError(fmt::format("Failed to take response: {}", rcl_get_error_string().str), ev);
    
    if (this->config->service_calls_verbose)
      std::cout << "Has response from " << service_name << std::endl;
  }
  
  // map response to socket message
  auto ack = sio::object_message::create(); // always obj
  auto res_err = MapResponseToSocketMessage(response_msg, response_members, ack, this->config->service_calls_verbose);
  if (!res_err.empty())
    return this->returnServiceError(fmt::format("Error mapping result data: {}", res_err.c_str()), ev);
  
  // send the reply
  if (this->config->service_calls_verbose)
    std::cout << "Sending " << service_name << " reply..." << std::endl;
  BridgeSocket::ack(ev.get_msgId(), {ack});
}


std::string SetRequestFieldValue(void *field, const rosidl_typesupport_introspection_cpp::MessageMember *member, sio::message::ptr value, size_t index, bool verbose, int indent) {
  
  std::string s_indent;
  s_indent.append(indent*4, ' ');
  if (verbose)
    std::cout << s_indent << "Assigning type " << std::to_string(member->type_id_) << (member->is_array_ ? " Array" : "") << std::endl;

  switch (member->type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
      if (verbose)
        std::cout <<  s_indent << GREEN << ">> BOOL: ";
      bool *v = new bool();
      *v = value->get_flag() == sio::message::flag::flag_boolean ? value->get_bool() : false; // tolerates null
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<bool *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> BYTE: ";
      uint8_t *v = new uint8_t();
      *v = static_cast<uint8_t>(value->get_flag() == sio::message::flag::flag_integer ? value->get_int() : 0);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<uint8_t *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_CHAR: ";
      int8_t *v = new int8_t();
      *v = static_cast<int8_t>(value->get_flag() == sio::message::flag::flag_integer ? value->get_int() : 0);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<int8_t *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_FLOAT32: ";
      float *v = new float();
      *v = static_cast<float>(value->get_flag() == sio::message::flag::flag_double ? value->get_double() : 0.0f);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<float *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_FLOAT64: ";
      double *v = new double();
      *v = static_cast<double>(value->get_flag() == sio::message::flag::flag_double ? value->get_double() : 0.0f);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<double *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_INT16: ";
      int16_t *v = new int16_t();
      *v = static_cast<int16_t>(value->get_flag() == sio::message::flag::flag_integer ? value->get_int() : 0);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<int16_t *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_UINT16: ";
      uint16_t *v = new uint16_t();
      *v = static_cast<uint16_t>(value->get_flag() == sio::message::flag::flag_integer ? value->get_int() : 0);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<uint16_t *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_INT32: ";
      int32_t *v = new int32_t();
      *v = static_cast<int32_t>(value->get_flag() == sio::message::flag::flag_integer ? value->get_int() : 0);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<int32_t *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_UINT32: ";
      uint32_t *v = new uint32_t();
      *v = static_cast<uint32_t>(value->get_flag() == sio::message::flag::flag_integer ? value->get_int() : 0);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<uint32_t *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> INT64: ";
      int64_t *v = new int64_t();
      *v = value->get_int();
      if (verbose)
        std::cout << *v << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<int64_t *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_UINT64: ";
      uint64_t *v = new uint64_t();
      *v = static_cast<uint64_t>(value->get_flag() == sio::message::flag::flag_integer ? value->get_int() : 0);
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        *static_cast<uint64_t *>(field) = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_STRING: ";
      std::string *v = new std::string();
      *v = value->get_string();
      auto str_len = (*v).length();
      if (verbose)
        std::cout << "'" << *v << "' (" << std::to_string(str_len) << ")" << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        std::string* str_field = static_cast<std::string*>(field);
        str_field->reserve(str_len);
        *str_field = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING: {
      if (verbose)
        std::cout << s_indent << GREEN << ">> ROS_TYPE_WSTRING: ";
      std::u16string *v = new std::u16string ();
      *v = converter.from_bytes(value->get_string());
      auto str_len = (*v).length();
      if (verbose)
        std::cout << "'" << converter.to_bytes(*v) << "' (" << std::to_string(str_len) << ")" << CLR << std::endl;
      if (member->is_array_) {
        member->assign_function(field, index, v);
      } else {
        std::u16string * str_field = static_cast<std::u16string*>(field);
        str_field->reserve(str_len);
        *str_field = *v;
      }
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
        if (value->get_flag() != sio::message::flag::flag_object)
          return "Attribute is not an object";

        const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member->members_->data);
        if (verbose)
          std::cout << s_indent << YELLOW << "{ >> ROS_TYPE_MESSAGE " << nested_members->message_name_ << ": " << CLR << std::endl;
      
        if (member->is_array_) {
          void *nested_field = malloc(nested_members->size_of_);
          nested_members->init_function(nested_field, rosidl_runtime_cpp::MessageInitialization::ALL);
          MapSocketMessageToRequest( value, nested_field, nested_members, verbose, indent+1);
          member->assign_function(field, index, nested_field);
        } else {
          // field = nested_field;
          MapSocketMessageToRequest( value, field, nested_members, verbose, indent+1);
        }
        if (verbose)
          std::cout << s_indent << "}" << std::endl;
        break;
    }
    default:
      return "Unsupported request member type: " + std::to_string(member->type_id_);
  }

  return "";
}

std::string MapSocketMessageToRequest(const sio::message::ptr &in_data, void *request_msg, const rosidl_typesupport_introspection_cpp::MessageMembers *request_members, bool verbose, int indent) {

  if (in_data->get_flag() == sio::message::flag::flag_null) {
    return ""; // skip nulls
  }

  if (in_data->get_flag() != sio::message::flag::flag_object) {
    return "Input data is not an object";
  }

  const auto &data_map = in_data->get_map();

  std::string s_indent;
  s_indent.append(indent*4, ' ');
  
  for (const auto &pair : data_map) {
    const std::string &key = pair.first;
    const sio::message::ptr &value = pair.second;

    // find the corresponding member in the ros message
    const rosidl_typesupport_introspection_cpp::MessageMember *member = nullptr;
    for (size_t i = 0; i < request_members->member_count_; i++) {
      if (std::string(request_members->members_[i].name_) == key) {
        member = &request_members->members_[i];
        break;
      }
    }

    if (!member)
      return "Key '" + key + "' not found in type definition";

    void *field = static_cast<char *>(request_msg) + member->offset_;

    if (verbose) {
      std::cout << s_indent << CYAN << "'" << key << "' type=" << std::to_string(member->type_id_) << ": " << std::endl;
      std::cout << BridgeSocket::printMessage(value, true, 1, s_indent) << CLR << std::endl;
    }

    if (member->is_array_) {

      if (value->get_flag() != sio::message::flag::flag_array)
        return "Request attribute '" + key + "' not an array in provided data";

      const auto &value_array = value->get_vector();
      auto array_size = member->is_upper_bound_ ? std::max(value_array.size(), member->array_size_) : value_array.size();

      if (verbose) {
        std::cout << s_indent << YELLOW << ">> ARRAY (" << array_size << ") of type " << std::to_string(member->type_id_) << CLR << std::endl;
        std::cout << s_indent << "[" << std::endl;
      }
      
      // if (array_size)
      member->resize_function(field, array_size);
      for (size_t i = 0; i < array_size; i++) {
        if (verbose) 
          std::cout << s_indent << "Assigning " << i << std::endl;
        auto err = SetRequestFieldValue(field, member, value_array[i], i, verbose,  indent+1);
        if (!err.empty())
          return err;
      }
      if (verbose) 
        std::cout << s_indent << "]" << std::endl;

    } else {

      auto err = SetRequestFieldValue(field, member, value, 0, verbose, indent);
      if (!err.empty())
          return err;

    }    
  }

  return ""; // ok
}

std::string SetOneResponseValue(const void *field_ptr, const rosidl_typesupport_introspection_cpp::MessageMember *member, sio::message::ptr out_target, std::string key, int index, bool verbose, int indent) {

  std::string s_indent;
  s_indent.append(indent*4, ' ');

  sio::message::ptr res;

  switch (member->type_id_) {

    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_BOOL: ";
      bool *v = new bool();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const bool *>(field_ptr);
      }
      if (verbose)
        std::cout << *v << CLR << std::endl;
      res = sio::bool_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_UINT8: ";
      uint8_t *v = new uint8_t();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const uint8_t *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::int_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_INT8: ";
      int8_t *v = new int8_t();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const int8_t *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::int_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_FLOAT32: ";
      float *v = new float();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const float *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::double_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_FLOAT64: ";
      double *v = new double();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const double *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::double_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_INT16: ";
      int16_t *v = new int16_t();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const int16_t *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::int_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_UINT16: ";
      uint16_t *v = new uint16_t();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const uint16_t *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::int_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_INT32: ";
      int32_t *v = new int32_t();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const int32_t *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::int_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_UINT32: ";
      uint32_t *v = new uint32_t();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const uint32_t *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::int_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_INT64: ";
      int64_t *v = new int64_t();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const int64_t *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::int_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_UINT64: ";
      uint64_t *v = new uint64_t();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const uint64_t *>(field_ptr);
      }
      if (verbose)
        std::cout << std::to_string(*v) << CLR << std::endl;
      res = sio::int_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_STRING: ";
      std::string *v = new std::string();
      if (index > -1) {
        member->fetch_function(field_ptr, index, v);
      } else {
        *v = *static_cast<const std::string *>(field_ptr);
      }
      if (verbose)
        std::cout << *v << CLR << std::endl;
      res = sio::string_message::create(*v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING: {
      std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> converter;
      if (verbose)
        std::cout << s_indent << MAGENTA << ">> ROS_TYPE_WSTRING: ";
      std::u16string *w = new std::u16string ();
      if (index > -1) {
        member->fetch_function(field_ptr, index, w);
      } else {
        *w = *static_cast<const std::u16string *>(field_ptr);
      }
      std::string v = converter.to_bytes(*w);
      if (verbose)
        std::cout << v << CLR << std::endl;
      res = sio::string_message::create(v);
      break;
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
      if (verbose)
        std::cout << s_indent << YELLOW << ">> " << (member->is_array_?"ARRAY OF ":"") << "ROS_TYPE_MESSAGE " << CLR << std::endl;

      const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member->members_->data);
      const void * obj_base_ptr = index > -1 ? member->get_const_function(field_ptr, index) : field_ptr;
      res = sio::object_message::create();

      for (size_t i = 0; i < nested_members->member_count_; i++) {
        auto nested_member = &nested_members->members_[i];
        auto nested_key = nested_member->name_;
        const void *nested_field = static_cast<const char *>(obj_base_ptr) + nested_member->offset_;

        if (nested_member->is_array_ || nested_member->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {

          auto err = SetResponseFieldValue(nested_field, nested_member, res, nested_key, verbose, indent+1);
          if (!err.empty())
            return err;

        } else {

          auto err = SetOneResponseValue(nested_field, nested_member, res, nested_key, -1, verbose, indent+1);
          if (!err.empty())
            return err;

        }
      }

      break;
    }
    default:
      return "Unsupported response member type " + std::to_string(member->type_id_);
  }

  if (index > -1) { // add to an array
    out_target->get_vector().push_back(res);
  } else {
    out_target->get_map().emplace(key, res);
  }

  return ""; //ok
}

std::string SetResponseFieldValue(const void *field_ptr, const rosidl_typesupport_introspection_cpp::MessageMember *member, sio::message::ptr out_target, std::string key, bool verbose, int indent) {

  std::string s_indent;
  s_indent.append(indent*4, ' ');

  if (member->is_array_) { // member is an array
    auto array_size = member->size_function(field_ptr);
    auto out_array = sio::array_message::create();
    if (verbose)
      std::cout  << s_indent << YELLOW << "'" << key << "' is an array(" << std::to_string(array_size) << ") of type " << std::to_string(member->type_id_) << CLR << std::endl;
      
    for (size_t j = 0; j < array_size; j++) {
      if (verbose)
        std::cout << s_indent << "setting [" << j << "]:" << std::endl;
      auto err = SetOneResponseValue(field_ptr, member, out_array, key, j, verbose, indent+1);
      if (!err.empty())
          return err;
    }

    out_target->get_map().emplace(key, out_array);
    
  } else if (member->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) { // member is an obj
    
    const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member->members_->data);
    auto out_obj = sio::object_message::create();

    for (size_t i = 0; i < nested_members->member_count_; i++) {
      auto nested_member = &nested_members->members_[i];
      auto nested_key = nested_member->name_;
      const void *nested_field_ptr = static_cast<const char *>(field_ptr) + nested_member->offset_;  

      if (verbose)
      std::cout << s_indent << "setting [" << nested_key << "]:" << std::endl;
      if (nested_member->is_array_) {
        auto err = SetResponseFieldValue(nested_field_ptr, nested_member, out_obj, nested_key, verbose, indent+1);
        if (!err.empty())
          return err;
      } else {
        auto err = SetOneResponseValue(nested_field_ptr, nested_member, out_obj, nested_key, -1, verbose, indent+1);
        if (!err.empty())
          return err;
      }
    }

    out_target->get_map().emplace(key, out_obj);

  } else { // primitive type

    auto err = SetOneResponseValue(field_ptr, member, out_target, key, -1, verbose, indent+1);
    if (!err.empty())
        return err;

  }

  return ""; // ok
}


std::string MapResponseToSocketMessage(const void *response_msg, const rosidl_typesupport_introspection_cpp::MessageMembers *response_members, sio::message::ptr &out_data, bool verbose, int indent) {

  std::string s_indent;
  s_indent.append(indent*4, ' ');

  for (size_t i = 0; i < response_members->member_count_; i++) { // always obj
    auto member = &response_members->members_[i];
    auto key = member->name_;
    const void *field_ptr = static_cast<const char *>(response_msg) + member->offset_;

    if (verbose)
      std::cout << s_indent << "Assigning '" << key << "' of type " << std::to_string(member->type_id_) << std::endl;

    auto err = SetResponseFieldValue(field_ptr, member, out_data, key, verbose, indent);
    if (!err.empty())
        return err;
  }

  return ""; // ok
}

// report error during service call via socket.io + rosout
void PhntmBridge::returnServiceError(std::string message, sio::event const &ev) {
  RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  rcl_reset_error();
  auto err_ack = sio::object_message::create();
  err_ack->get_map().emplace("err", sio::int_message::create(2));
  err_ack->get_map().emplace("msg", sio::string_message::create(message));
  BridgeSocket::ack(ev.get_msgId(), {err_ack});
}

// free library handles and service related cache
void PhntmBridge::clearServicesCache() {
  for (auto cache : this->srv_library_handle_cache) {
    dlclose(cache.second); // free package library handle
  }
  this->srv_library_handle_cache.clear();
  this->srv_client_cache.clear();
  this->srv_types_cache.clear();
}
