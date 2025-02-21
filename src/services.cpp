#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/sio.hpp"

#include <dlfcn.h>
#include <iostream>
#include <rcl/context.h>
#include <rclcpp/context.hpp>
#include "rcl/client.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"


// #include <rcpputils/shared_library.hpp>
// #include <rosidl_typesupport_cpp/service_type_support.hpp>
// #include <rosidl_typesupport_cpp/message_type_support.hpp>

// set up service of this node here
void PhntmBridge::makeServices() {
    this->srv_clear_file_cache = this->create_service<std_srvs::srv::Trigger>(fmt::format("/{}/clear_cloud_file_cache", this->get_name()),
                                                                              std::bind(&PhntmBridge::requestClearFileCache, this, std::placeholders::_1, std::placeholders::_2));
}

// call service from the socker.io
// running on an async thread
void PhntmBridge::callService(std::string service_name, std::string service_type, std::shared_ptr<BridgeSocket> sio, sio::event const& ev) {

    std::cout << MAGENTA << "Calling service: " << service_name << CLR << " {" << service_type << "}" << std::endl;

    auto parts = split(service_type, '/');
    std::string package_name = parts[0];
    std::string srv_name = parts[2];

    std::cout << "package_name: " << package_name << " srv_name: " << srv_name << std::endl;

    std::string lib_name = "lib" + package_name + "__rosidl_typesupport_cpp.so";
    void* handle = dlopen(lib_name.c_str(), RTLD_LAZY);
    if (!handle) {
      RCLCPP_ERROR(get_logger(), "Library load error: %s", dlerror());
      return;
    }

    std::cout << "lib_name: " << lib_name << std::endl;

    // Get type support handle
    std::string service_symbol_name = "rosidl_typesupport_cpp__get_service_type_support_handle__" + package_name + "__srv__" + srv_name;
    auto get_service_ts = reinterpret_cast<const rosidl_service_type_support_t*(*)()>(dlsym(handle, service_symbol_name.c_str()));
    if (!get_service_ts) {
      RCLCPP_ERROR(get_logger(), "Symbol load error: %s", dlerror());
      dlclose(handle);
      return;
    }

    const rosidl_service_type_support_t* service_type_support = get_service_ts();
    if (!service_type_support) {
      RCLCPP_ERROR(get_logger(), "Failed to get type support");
      dlclose(handle);
      return;
    }

    // Initialize rcl client
    rcl_client_t client = rcl_get_zero_initialized_client();
    rcl_client_options_t options = rcl_client_get_default_options();
    rcl_ret_t ret = rcl_client_init(
        &client,
        this->get_node_base_interface()->get_rcl_node_handle(),
        service_type_support,
        service_name.c_str(),
        &options);

    if (ret != RCL_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Client init failed: %s", rcl_get_error_string().str);
      rcl_reset_error();
      dlclose(handle);
      return;
    }

    std::cout << "Client init ok" << std::endl;

    std::string introspection_lib_name = "lib" + package_name + "__rosidl_typesupport_introspection_cpp.so";
    void* introspection_handle = dlopen(introspection_lib_name.c_str(), RTLD_LAZY);
    if (!introspection_handle) {
      RCLCPP_ERROR(get_logger(), "Introspection library load error: %s", dlerror());
      return;
    }

    std::cout << "introspection lib loaded ok " << introspection_lib_name << std::endl;

    std::string request_introspection_symbol = "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__srv__" + srv_name + "_Request";
    auto get_request_introspection_ts = reinterpret_cast<const rosidl_message_type_support_t*(*)()>(dlsym(introspection_handle, request_introspection_symbol.c_str()));
    if (!get_request_introspection_ts) {
      RCLCPP_ERROR(get_logger(), "Request introspection symbol load error: %s", dlerror());
      dlclose(introspection_handle);
      dlclose(handle);
      return;
    }
    const rosidl_message_type_support_t* request_introspection_ts = get_request_introspection_ts();

    std::cout << "Got request type support " << request_introspection_ts->typesupport_identifier << std::endl;

    std::string response_introspection_symbol = "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + package_name + "__srv__" + srv_name + "_Response";
    auto get_response_introspection_ts = reinterpret_cast<const rosidl_message_type_support_t*(*)()>(dlsym(introspection_handle, response_introspection_symbol.c_str()));
    if (!get_response_introspection_ts) {
      RCLCPP_ERROR(get_logger(), "Response introspection symbol load error: %s", dlerror());
      dlclose(introspection_handle);
      dlclose(handle);
      return;
    }
    const rosidl_message_type_support_t* response_introspection_ts = get_response_introspection_ts();

    std::cout << "Got response type support " << response_introspection_ts->typesupport_identifier << std::endl;

    const rosidl_typesupport_introspection_cpp::MessageMembers* request_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(request_introspection_ts->data);

    std::cout << "Request has " << request_members->member_count_ << " members" << std::endl;
    std::cout << "Request size " << request_members->size_of_ << "" << std::endl;
    std::cout << "Request message_name " << request_members->message_name_ << "" << std::endl;

    // Allocate memory for the message
    void* request_msg = malloc(request_members->size_of_);
    request_members->init_function(request_msg, rosidl_runtime_cpp::MessageInitialization::ALL);

    std::cout << "Request init ok" << std::endl;

    // assign request here
    for (size_t i = 0; i < request_members->member_count_; ++i) {
      const auto& member = request_members->members_[i];
      std::cout << "Member name: " << member.name_ << ", type: " << std::to_string(member.type_id_) << std::endl;
      // rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT
    }

    // response msg
    const rosidl_typesupport_introspection_cpp::MessageMembers* response_members = 
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(response_introspection_ts->data);

    void* response_msg = malloc(response_members->size_of_);
    response_members->init_function(response_msg, rosidl_runtime_cpp::MessageInitialization::ALL);

    std::cout << "Response init ok" << std::endl;

    // send the request
    int64_t sequence_number;
    ret = rcl_send_request(&client, request_msg, &sequence_number);
    if (ret != RCL_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Failed to send request: %s", rcl_get_error_string().str);
        rcl_reset_error();
        dlclose(handle);
        dlclose(introspection_handle);
        return;
    }  

    std::cout << "Request sent ok" << std::endl;

    // wait for the response
    auto context = this->get_node_base_interface()->get_context().get()->get_rcl_context();
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    ret = rcl_wait_set_init(
        &wait_set,
        0,  // number of subscriptions
        1,  // number of guard conditions
        0,  // number of timers
        1,  // number of clients
        0,  // number of services
        0,
        context.get(),
        rcl_get_default_allocator()
    ); // Allocator
    if (ret != RCL_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize wait set: %s", rcl_get_error_string().str);
        rcl_reset_error();
        dlclose(handle);
        dlclose(introspection_handle);
        return;
    }

    std::cout << "Adding client to wait set" << std::endl;

    ret = rcl_wait_set_add_client(&wait_set, &client, NULL);
    if (ret != RCL_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Failed to add client to wait set: %s", rcl_get_error_string().str);
        rcl_reset_error();
        dlclose(handle);
        dlclose(introspection_handle);
        return;
    }

    std::cout << "Waiting..." << std::endl;

    // Wait for a response (timeout in nanoseconds)
    const int64_t timeout_ns = RCL_S_TO_NS(10); // wait 10 seconds
    ret = rcl_wait(&wait_set, timeout_ns);
    if (ret == RCL_RET_TIMEOUT) {
      RCLCPP_WARN(get_logger(), "Service call timed out.");
      dlclose(handle);
      dlclose(introspection_handle);
      return;
    } else if (ret != RCL_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error during wait: %s", rcl_get_error_string().str);
      dlclose(handle);
      dlclose(introspection_handle);
      rcl_reset_error();
      return;
    }

    std::cout << "Getting response..." << std::endl;

    rmw_request_id_t request_header;
    ret = rcl_take_response(&client, &request_header, response_msg);
    if (ret != RCL_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Failed to take response: %s", rcl_get_error_string().str);
        rcl_reset_error();
        dlclose(handle);
        dlclose(introspection_handle);
        return;
    }
    
    std::cout << "Has response!" << std::endl;

    dlclose(handle);
    dlclose(introspection_handle);

    auto ack = sio::object_message::create();
    sio->ack(ev.get_msgId(), { ack });
}

void PhntmBridge::requestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::cout << MAGENTA << "Requesting clear server file cache" << CLR << std::endl;
    
    (void) request;
    (void) response;

    // clear_data = {
    //     'idRobot': id_robot,
    //     'authKey': auth_key
    // }
    // clear_response = requests.post(f'{upload_host}/clear_cache', json=clear_data)

    // if clear_response.status_code == 200:
    //     logger.debug("Cache cleared")
    //     response.success = True
    //     response.message = clear_response.text
    // else:
    //     logger.error(f"Error clearing cache: {clear_response.text}")
    //     response.success = False
    //     response.message = clear_response.text
}
    