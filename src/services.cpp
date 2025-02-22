#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/sio.hpp"

#include "rcl/client.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"
#include <dlfcn.h>
#include <fmt/core.h>
#include <iostream>
#include <rcl/context.h>
#include <rclcpp/context.hpp>
#include <string>

std::string MapSocketMessageToRequest(const sio::message::ptr &in_data, void *request_msg, const rosidl_typesupport_introspection_cpp::MessageMembers *request_members);

// calling generic ROS services from the socket.io
// running on an async thread
void PhntmBridge::callGenericService(std::string service_name, std::string service_type, std::shared_ptr<BridgeSocket> sio, sio::event const &ev) {

  std::cout << MAGENTA << "Calling service: " << service_name << CLR << " {" << service_type << "}" << std::endl;

  auto parts = split(service_type, '/');
  std::string package_name = parts[0];
  std::string srv_name = parts[2];
  rcl_ret_t ret;

  std::cout << "package_name: " << package_name << " srv_name: " << srv_name << std::endl;

  rcl_client_t *client;
  if (this->srv_client_cache.find(service_name) != this->srv_client_cache.end()) {

    // client instance found in cache
    client = srv_client_cache.at(service_name);

  } else { // set up a new client

    // load service lib
    void *service_lib_handle;
    if (this->srv_library_handle_cache.find(package_name) != this->srv_library_handle_cache.end()) {
      service_lib_handle = this->srv_library_handle_cache.at(package_name);
    } else {
      std::string lib_name = "lib" + package_name + "__rosidl_typesupport_cpp.so";
      std::cout << "Loading lib: " << lib_name << std::endl;
      service_lib_handle = dlopen(lib_name.c_str(), RTLD_LAZY);
      if (!service_lib_handle)
        return returnServiceError(fmt::format("Library load error: {}", dlerror()), sio, ev);
      this->srv_library_handle_cache.emplace( package_name, service_lib_handle); // keep the handle open as long as we need the client
      std::cout << "Lib loaded" << std::endl;
    }

    // get type support handle
    std::string service_symbol_name = "rosidl_typesupport_cpp__get_service_type_support_handle__" + package_name + "__srv__" + srv_name;
    std::cout << "Getting get_service_ts " << service_symbol_name << std::endl;
    auto get_service_ts = reinterpret_cast<const rosidl_service_type_support_t *(*)()>(dlsym(service_lib_handle, service_symbol_name.c_str()));
    if (!get_service_ts)
      return this->returnServiceError(fmt::format("Symbol load error: {}", dlerror()), sio, ev);

    // get service type support
    const rosidl_service_type_support_t *service_type_support = get_service_ts();
    if (!service_type_support)
      return this->returnServiceError("Failed to get type support", sio, ev);

    std::cout << "Initializing client..." << std::endl;
    // initialize rcl client
    client = new rcl_client_t();
    rcl_client_options_t options = rcl_client_get_default_options();
    ret = rcl_client_init(client, this->get_node_base_interface()->get_rcl_node_handle(), service_type_support, service_name.c_str(), &options);

    if (ret != RCL_RET_OK)
      return this->returnServiceError(fmt::format("Client init failed: {}", rcl_get_error_string().str), sio, ev);

    std::cout << "Client init ok" << std::endl;
    this->srv_client_cache.emplace(service_name, client); // cache the client instance for later
  }

  const rosidl_message_type_support_t *request_introspection_ts, *response_introspection_ts;

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
        return this->returnServiceError(fmt::format("Introspection library load error: {}", dlerror()), sio, ev);

      std::cout << "Introspection lib loaded ok " << introspection_lib_name << std::endl;
      this->srv_library_handle_cache.emplace(id_introslection_lib, introspection_handle); // keep the handle open as long as we need the client
    }

    // get request type support
    std::string request_introspection_symbol = "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__srv__" + srv_name + "_Request";
    auto get_request_introspection_ts = reinterpret_cast<const rosidl_message_type_support_t *(*)()>(dlsym(introspection_handle, request_introspection_symbol.c_str()));
    if (!get_request_introspection_ts)
      return this->returnServiceError(fmt::format("Request introspection symbol load error: {}", dlerror()), sio, ev);
    request_introspection_ts = get_request_introspection_ts();
    std::cout << "Got request type support " << request_introspection_ts->typesupport_identifier << std::endl;

    std::string response_introspection_symbol = "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__srv__" + srv_name + "_Response";
    auto get_response_introspection_ts = reinterpret_cast<const rosidl_message_type_support_t *(*)()>(dlsym(introspection_handle, response_introspection_symbol.c_str()));
    if (!get_response_introspection_ts)
      return this->returnServiceError(fmt::format("Response introspection symbol load error:: {}", dlerror()), sio, ev);

    response_introspection_ts = get_response_introspection_ts();

    std::cout << "Got response type support " << response_introspection_ts->typesupport_identifier << std::endl;

    SrvTypeCache cache;
    cache.request = request_introspection_ts;
    cache.response = response_introspection_ts;
    this->srv_types_cache.emplace(service_name, cache); // cache type supports for later
  }

  const rosidl_typesupport_introspection_cpp::MessageMembers *request_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(request_introspection_ts->data);

  std::cout << "Request has " << request_members->member_count_ << " members" << std::endl;
  std::cout << "Request size " << request_members->size_of_ << "" << std::endl;
  std::cout << "Request message_name " << request_members->message_name_ << "" << std::endl;

  // Allocate memory for the message
  void *request_msg = malloc(request_members->size_of_);
  request_members->init_function(request_msg, rosidl_runtime_cpp::MessageInitialization::ALL);

  std::cout << "Request alloc ok" << std::endl;
  auto request_data = ev.get_message()->get_map()["msg"];
  std::cout << "Setting request data: " << BridgeSocket::PrintMessage(request_data) << std::endl;
  if (request_data->get_flag() != sio::message::flag::flag_null) {
    auto err = MapSocketMessageToRequest(request_data, request_msg, request_members);
    if (!err.empty())
      return this->returnServiceError(fmt::format("Error mapping request data: {}", err.c_str()), sio, ev);
  }

  // response msg
  const rosidl_typesupport_introspection_cpp::MessageMembers *response_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(response_introspection_ts->data);

  void *response_msg = malloc(response_members->size_of_);
  response_members->init_function(response_msg, rosidl_runtime_cpp::MessageInitialization::ALL);

  std::cout << "Response init ok" << std::endl;

  // send the request
  int64_t sequence_number;
  ret = rcl_send_request(client, request_msg, &sequence_number);
  if (ret != RCL_RET_OK)
    return this->returnServiceError(fmt::format("Failed to send request: {}", rcl_get_error_string().str), sio, ev);

  std::cout << "Request sent ok" << std::endl;

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
    return this->returnServiceError(fmt::format("Failed to initialize wait set: {}", rcl_get_error_string().str), sio, ev);

  std::cout << "Adding client to wait set" << std::endl;

  ret = rcl_wait_set_add_client(&wait_set, client, NULL);
  if (ret != RCL_RET_OK)
    return this->returnServiceError(fmt::format("Failed to add client to wait set: {}", rcl_get_error_string().str), sio, ev);

  std::cout << "Waiting..." << std::endl;

  // Wait for a response (timeout in nanoseconds)
  ret = rcl_wait(&wait_set, this->config->service_timeout_ns);
  if (ret == RCL_RET_TIMEOUT)
    return this->returnServiceError("Service call timed out.", sio, ev);
  else if (ret != RCL_RET_OK)
    return this->returnServiceError(fmt::format("Error during wait: {}", rcl_get_error_string().str), sio, ev);

  std::cout << "Getting response..." << std::endl;

  rmw_request_id_t request_header;
  ret = rcl_take_response(client, &request_header, response_msg);
  if (ret != RCL_RET_OK)
    return this->returnServiceError(fmt::format("Failed to take response: {}", rcl_get_error_string().str), sio, ev);

  std::cout << "Has response!" << std::endl;

  auto ack = sio::object_message::create();
  sio->ack(ev.get_msgId(), {ack});
}

// assign request here
std::string MapSocketMessageToRequest(const sio::message::ptr &in_data, void *request_msg, const rosidl_typesupport_introspection_cpp::MessageMembers *request_members) {

  if (in_data->get_flag() != sio::message::flag::flag_object) {
    return "Input data is not an object";
  }

  const auto &data_map = in_data->get_map();

  for (const auto &pair : data_map) {
    const std::string &key = pair.first;
    const sio::message::ptr &value = pair.second;

    // find the corresponding member in the ros message
    const rosidl_typesupport_introspection_cpp::MessageMember *member = nullptr;
    for (size_t i = 0; i < request_members->member_count_; ++i) {
      if (std::string(request_members->members_[i].name_) == key) {
        member = &request_members->members_[i];
        break;
      }
    }

    if (!member)
      return "Key '" + key + "' not found in type definition";

    void *field = static_cast<char *>(request_msg) + member->offset_;

    std::cout << YELLOW << "  " << key << " type=" << std::to_string(member->type_id_) << " << " << BridgeSocket::PrintMessage(value) << CLR << std::endl;

    switch (member->type_id_) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
        *static_cast<bool *>(field) = value->get_bool();
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        *static_cast<uint8_t *>(field) = static_cast<uint8_t>(value->get_int());
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        *static_cast<int8_t *>(field) = static_cast<int8_t>(value->get_int());
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
        *static_cast<float *>(field) = static_cast<float>(value->get_double());
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
        *static_cast<double *>(field) = value->get_double();
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        *static_cast<int16_t *>(field) = static_cast<int16_t>(value->get_int());
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        *static_cast<uint16_t *>(field) = static_cast<uint16_t>(value->get_int());
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        *static_cast<int32_t *>(field) = value->get_int();
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        *static_cast<uint32_t *>(field) = static_cast<uint32_t>(value->get_int());
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        *static_cast<int64_t *>(field) = value->get_int();
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        *static_cast<uint64_t *>(field) = static_cast<uint64_t>(value->get_int());
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        std::cout << RED << "ROS_TYPE_STRING " << value->get_string() << CLR << std::endl;
        *static_cast<std::string *>(field) = value->get_string();
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
        // recursive call for nested messages
        if (value->get_flag() == sio::message::flag::flag_object) {
          MapSocketMessageToRequest( value, field, static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member->members_->data));
        }
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        // handle wide strings if needed
        std::cout << RED << "ROS_TYPE_WSTRING " << value->get_string() << CLR << std::endl;
        break;
      default:
        // handle arrays, bounded vectors, etc.
        if (member->is_array_ && value->get_flag() == sio::message::flag::flag_array) {
          const auto &array = value->get_vector();
          if (member->array_size_ > 0 && !member->is_upper_bound_) {
            // fixed-size array
            for (size_t i = 0; i < std::min(array.size(), member->array_size_); ++i) {
              // you'll need to implement array element assignment based on the
              // element type
            }
          } else {
            // unbounded array or bounded vector
            // you'll need to implement dynamic array handling
          }
        }
        break;
    }
  }

  return "";
}

// report error during service call via socket.io + rosout
void PhntmBridge::returnServiceError(std::string message, std::shared_ptr<BridgeSocket> sio, sio::event const &ev) {
  RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  rcl_reset_error();
  auto err_ack = sio::object_message::create();
  err_ack->get_map().emplace("err", sio::int_message::create(2));
  err_ack->get_map().emplace("msg", sio::string_message::create(message));
  sio->ack(ev.get_msgId(), {err_ack});
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

// set up local service of this node here
void PhntmBridge::setupLocalServices() {
  this->srv_clear_file_cache = this->create_service<std_srvs::srv::Trigger>(fmt::format("/{}/clear_cloud_file_cache", this->get_name()),
                                                                            std::bind(&PhntmBridge::srvRequestClearFileCache, this, std::placeholders::_1, std::placeholders::_2));
}

// request the files cache on the cloud bridge be cleared
void PhntmBridge::srvRequestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::cout << MAGENTA << "Requesting clear server file cache" << CLR << std::endl;

  (void)request;
  (void)response;

  // clear_data = {
  //     'idRobot': id_robot,
  //     'authKey': auth_key
  // }
  // clear_response = requests.post(f'{upload_host}/clear_cache',
  // json=clear_data)

  // if clear_response.status_code == 200:
  //     logger.debug("Cache cleared")
  //     response.success = True
  //     response.message = clear_response.text
  // else:
  //     logger.error(f"Error clearing cache: {clear_response.text}")
  //     response.success = False
  //     response.message = clear_response.text
}
