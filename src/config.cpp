#include "phntm_bridge/const.hpp"
#include "phntm_bridge/config.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include <fstream>


// 'ClassName https://public.url/file.js'
std::optional<CustomWidgetDef> parseCustomPluginDef(std::string one) {
    CustomWidgetDef def;
    size_t pos = one.find(' '); //split by 1st space
    def.class_name = trim(one.substr(0, pos));
    def.url = trim(one.substr(pos + 1));

    if (def.class_name.empty() || def.url.empty())
        return std::nullopt;

    return def;
}

// '/id_service ClassName { "var1" 1, "var2": 2, ... }'
std::optional<ServiceWidgetConfig> parseServiceWidgetConfig(std::string one) {
    ServiceWidgetConfig conf;

    size_t json_start = one.find('{');
    std::string assignment = json_start == std::string::npos ? trim(one) : trim(one.substr(0, json_start));

    size_t pos = assignment.find(' '); //split by 1st space
    conf.service = trim(assignment.substr(0, pos));
    conf.class_name = trim(assignment.substr(pos));

    if (json_start != std::string::npos) {
        auto json_str = trim(one.substr(json_start));
        Json::Reader reader;
        if (!reader.parse(json_str, conf.data)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error parsing json: %s", json_str.c_str());
            return std::nullopt; //error parsing
        }
    }
    
    if (conf.service.empty() || conf.class_name.empty())
        return std::nullopt;

    return conf;
}

void PhntmBridge::loadConfig(std::shared_ptr<BridgeConfig> config) {

    // Robot ID
    rcl_interfaces::msg::ParameterDescriptor id_robot_descriptor;
    id_robot_descriptor.description = "Robot ID on Phntm Cloud Brudge";
    id_robot_descriptor.read_only = true;
    this->declare_parameter("id_robot", "", id_robot_descriptor);
    
    // Robot auth key
    rcl_interfaces::msg::ParameterDescriptor key_descriptor;
    key_descriptor.description = "Robot auth key on Phntm Cloud Brudge";
    this->declare_parameter("key", "", key_descriptor);

    config->id_robot = this->get_parameter("id_robot").as_string();
    config->auth_key = this->get_parameter("key").as_string();
  
    if (config->id_robot.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Param id_robot not provided!");
        exit(1);
    }

    if (config->auth_key.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Param auth_key not provided!");
        exit(1);
    }
    
    // Robot name
    rcl_interfaces::msg::ParameterDescriptor name_descriptor;
    name_descriptor.description = "Robot name";
    this->declare_parameter("name", "Unnamed Robot", name_descriptor);
    config->robot_name = this->get_parameter("name").as_string();

    // Maintainer's email
    rcl_interfaces::msg::ParameterDescriptor email_descriptor;
    email_descriptor.description = "Maintainer e-mail address";
    this->declare_parameter("maintainer_email", "", email_descriptor);
    config->maintainer_email = this->get_parameter("maintainer_email").as_string();

    // will check these packages on 1st (container) start
    rcl_interfaces::msg::ParameterDescriptor extra_pkg_descriptor;
    extra_pkg_descriptor.description = "ROS packages to check for on first Bridge run";
    extra_pkg_descriptor.additional_constraints = "Folder path or ROS package name";
    this->declare_parameter("extra_packages", std::vector<std::string>(), extra_pkg_descriptor);
    config->extra_packages = this->get_parameter("extra_packages").as_string_array();
    
    // webrtc config
    this->declare_parameter("use_cloud_ice_config", true);
    config->use_cloud_ice_config = this->get_parameter("use_cloud_ice_config").as_bool();
    this->declare_parameter("ice_servers", std::vector<std::string>());
    this->declare_parameter("ice_username", ""); // id_robot if empty
    this->declare_parameter("ice_secret", "");

    config->ice_servers_custom = this->get_parameter("ice_servers").as_string_array(); // these get added to cloud config, or used when use_cloud_ice_config=false
    if (config->ice_servers_custom.size()) {
        std::cout << "Custom ICE servers: " << std::endl;
        for (size_t i = 0; i < config->ice_servers_custom.size(); ++i) {
            std::cout << "\t" << config->ice_servers_custom[i] << std::endl;
        }
    }
    std::copy(config->ice_servers_custom.begin(), config->ice_servers_custom.end(), std::back_inserter(config->ice_servers)); // server config gets added here
    
    // ice credentials
    config->ice_username = this->get_parameter("ice_username").as_string();
    if (config->ice_username.empty())
        config->ice_username = config->id_robot;
    config->ice_secret = this->get_parameter("ice_secret").as_string();

    // prevent reading sensitive stuffs
    this->set_parameter(rclcpp::Parameter("key", "*************"));
    this->set_parameter(rclcpp::Parameter("ice_username", "*************"));
    this->set_parameter(rclcpp::Parameter("ice_secret", "*************"));
    
    // Cloud Bridge host
    this->declare_parameter("cloud_bridge_address", "https://us-ca.bridge.phntm.io");
    
    /// Cloud Bridge files uploader port
    this->declare_parameter("file_upload_port", 1336);
    
    // socket.io config
    this->declare_parameter("sio_port", 1337);
    this->declare_parameter("sio_path", "/robot/socket.io");
    this->declare_parameter("sio_connection_retry_sec", 2.0);
    this->declare_parameter("sio_ssl_verify", true);
    this->declare_parameter("sio_verbose", false);

    // services collapsed in the ui menu (still operational, parameneter services by default)
    rcl_interfaces::msg::ParameterDescriptor collapse_services_descriptor;
    collapse_services_descriptor.description = "The UI will collapse these services";
    collapse_services_descriptor.additional_constraints="Service id or type";
    std::vector<std::string> default_collapsed_services {"rcl_interfaces/srv/DescribeParameters", "rcl_interfaces/srv/GetParameterTypes", "rcl_interfaces/srv/GetParameters", "rcl_interfaces/srv/ListParameters", "rcl_interfaces/srv/SetParameters", "rcl_interfaces/srv/SetParametersAtomically" };
    this->declare_parameter("collapse_services", default_collapsed_services, collapse_services_descriptor);
    config->collapse_services = this->get_parameter("collapse_services").as_string_array();

    rcl_interfaces::msg::ParameterDescriptor collape_unhandled_services_descriptor;
    collape_unhandled_services_descriptor.description = "The UI will collapse services with unsupported message types";
    this->declare_parameter("collapse_unhandled_services", true, collape_unhandled_services_descriptor);
    config->collapse_unhandled_services = this->get_parameter("collapse_unhandled_services").as_bool();

    // blacklist topics from discovery (msg type or full topic id)
    this->declare_parameter("blacklist_topics", std::vector<std::string>());
    config->blacklist_topics = this->get_parameter("blacklist_topics").as_string_array();
    if (config->blacklist_topics.size()) {
        std::cout << "Blacklisted topics: " << std::endl;
        for (size_t i = 0; i < config->blacklist_topics.size(); ++i) {
            std::cout << "\t" << config->blacklist_topics[i] << std::endl;
        }
    }
    
    // blacklist services from discovery (msg type or full topic id)
    this->declare_parameter("blacklist_services", std::vector<std::string>());
    config->blacklist_services = this->get_parameter("blacklist_services").as_string_array();
    if (config->blacklist_services.size()) {
        std::cout << "Blacklisted services: " << std::endl;
        for (size_t i = 0; i < config->blacklist_services.size(); ++i) {
            std::cout << "\t" << config->blacklist_services[i] << std::endl;
        }
    }

    // blacklist msg types (topics/services are discovered but not deserialized or serialized)
    // pointcloud and costmap are here until fully suported (until then break browsers with too much unoptimized data)
    std::vector<std::string> default_blacklisted_services { "sensor_msgs/PointCloud", "sensor_msgs/msg/PointCloud2", "cost_map_msgs/CostMap", "nav_msgs/msg/OccupancyGrid" };
    this->declare_parameter("blacklist_msg_types", default_blacklisted_services);
    config->blacklist_msg_types = this->get_parameter("blacklist_msg_types").as_string_array();
    if (config->blacklist_services.size()) {
        std::cout << "Blacklisted message types: " << std::endl;
        for (size_t i = 0; i < config->blacklist_msg_types.size(); ++i) {
            std::cout << "\t" << config->blacklist_msg_types[i] << std::endl;
        }
    }
    
    // logging
    this->declare_parameter("log_sdp", false);
    this->declare_parameter("log_heartbeat", false);
    this->declare_parameter("log_message_every_sec", 10.0f);
    config->log_message_every_sec = this->get_parameter("log_message_every_sec").as_double();

    // bloud bridge stuffs
    config->cloud_bridge_address = this->get_parameter("cloud_bridge_address").as_string();
    config->file_upload_port = this->get_parameter("file_upload_port").as_int();
    config->sio_port = this->get_parameter("sio_port").as_int();
    config->sio_path = this->get_parameter("sio_path").as_string();
    config->sio_ssl_verify = this->get_parameter("sio_ssl_verify").as_bool();
    config->sio_verbose = this->get_parameter("sio_verbose").as_bool();
    config->sio_connection_retry_sec = this->get_parameter("sio_connection_retry_sec").as_double();
    if (config->cloud_bridge_address.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Param cloud_bridge_address not provided!");
        exit(1);
    }
    config->uploader_address = fmt::format("{}:{}", config->cloud_bridge_address, config->file_upload_port);

    // conn LED control via topic (blinks when connecting; on when connected; off = bridge not running)
    this->declare_parameter("conn_led_topic", "");
    config->conn_led_topic = this->get_parameter("conn_led_topic").as_string();
    // data LED control via topic (flashes when any data is sent via webrtc; off when not connected)
    this->declare_parameter("data_led_topic", "");
    config->data_led_topic = this->get_parameter("data_led_topic").as_string();
    
    // conn/data LED control via GPIO
    this->declare_parameter("conn_led_gpio_chip", "/dev/gpiochip0"); // PI5 default, use gpiodetect to list available
    config->conn_led_gpio_chip = this->get_parameter("conn_led_gpio_chip").as_string();
    this->declare_parameter("conn_led_pin", -1); // set GPIO number
    config->conn_led_pin = this->get_parameter("conn_led_pin").as_int();
    this->declare_parameter("data_led_pin", -1); // set GPIO number
    config->data_led_pin = this->get_parameter("data_led_pin").as_int();

    // introspection
    this->declare_parameter("discovery_period_sec", 2.0f);
    config->discovery_period_sec = this->get_parameter("discovery_period_sec").as_double();
    this->declare_parameter("stop_discovery_after_sec", -1.0f); // <0 = run indefinitely (don't show control)
    config->stop_discovery_after_sec = this->get_parameter("stop_discovery_after_sec").as_double();
    this->declare_parameter("introspection_verbose", false);
    config->introspection_verbose = this->get_parameter("introspection_verbose").as_bool();

    // wifi monitoring + scan
    this->declare_parameter("ui_wifi_monitor_topic", "/iw_status"); // Agent writes here
    config->ui_wifi_monitor_topic = this->get_parameter("ui_wifi_monitor_topic").as_string();
    this->declare_parameter("ui_enable_wifi_scan", true); // enables scan without roaming
    config->ui_enable_wifi_scan = this->get_parameter("ui_enable_wifi_scan").as_bool();
    this->declare_parameter("ui_enable_wifi_roam", false); // enables roaming (potentially dangerous)
    config->ui_enable_wifi_roam = this->get_parameter("ui_enable_wifi_roam").as_bool();
    
    this->declare_parameter("ui_battery_topic", "/battery"); // use this in the ui 
    config->ui_battery_topic = this->get_parameter("ui_battery_topic").as_string();
    this->declare_parameter("ui_docker_control", true);
    config->docker_control_enabled = this->get_parameter("ui_docker_control").as_bool();
    this->declare_parameter("docker_monitor_topic", "/docker_info");
    config->docker_monitor_topic = this->get_parameter("docker_monitor_topic").as_string();

    // input configs that get passed to ui
    std::vector<std::string> default_input_drivers { "Joy" };
    this->declare_parameter("input_drivers", default_input_drivers); // empty array to disable input entirely, services are still set up
    config->input_drivers = this->get_parameter("input_drivers").as_string_array();

    // custom input drivers to be injected into the web UI
    // array of 'ClassName https://public.url/file.js'
    this->declare_parameter("custom_input_drivers", std::vector<std::string>());
    auto custom_input_drivers = this->get_parameter("custom_input_drivers").as_string_array();
    for (auto one: custom_input_drivers) {
        if (one.empty()) continue;
        auto def = parseCustomPluginDef(one);
        if (!def) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid custom input driver definition: %s", one.c_str());
            exit(1);
        }
        config->custom_input_drivers.push_back(def.value());
        std::cout << "Adding custom input driver: " << def.value().class_name << " from " << def.value().url << std::endl;
    }
    
    // custom service menu widgets to be injected into the web UI
    // array of 'ClassName https://public.url/file.js'
    this->declare_parameter("custom_service_widgets", std::vector<std::string>());
    auto custom_service_widgets = this->get_parameter("custom_service_widgets").as_string_array();
    for (auto one : custom_service_widgets) {
        if (one.empty()) continue;
        auto def = parseCustomPluginDef(one);
        if (!def) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid custom service widget definition: %s", one.c_str());
            exit(1);
        }
        config->custom_service_widgets.push_back(def.value());
        std::cout << "Adding custom service widget: " << def.value().class_name << " from " << def.value().url << std::endl;
    }
    
    // service widget mapping with custom payload
    // array of '/id_service ClassName { "var1" 1, "var2": 2, ... }' (JSON payload is optional)
    this->declare_parameter("service_widgets", std::vector<std::string>());
    auto service_widgets_map = this->get_parameter("service_widgets").as_string_array();
    for (auto one : service_widgets_map) {
        if (one.empty()) continue;
        auto conf = parseServiceWidgetConfig(one);
        if (!conf) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid service widget config: %s", one.c_str());
            exit(1);
        }
        config->service_widgets.push_back(conf.value());
        std::cout << "Adding service widget mapping: " << conf.value().service << " is " << conf.value().class_name;
        if (!conf.value().data.empty())
            std::cout << "; data=" << conf.value().data;
        std::cout << std::endl;
    }

    // default input config for the web UI
    this->declare_parameter("input_defaults", "");
    auto input_defaults_file = this->get_parameter("input_defaults").as_string();
    if (!input_defaults_file.empty()) {
        Json::Reader reader;
        std::ifstream file(input_defaults_file.c_str());
        if (!reader.parse(file, config->input_defaults)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed loading input defaults from: %s", input_defaults_file.c_str());
            exit(1);
        }
    }
    
    // default services config for the web UI
    this->declare_parameter("service_defaults", "");
    auto service_defaults_file = this->get_parameter("service_defaults").as_string();
    if (!service_defaults_file.empty()) {
        Json::Reader reader;
        std::ifstream file(service_defaults_file.c_str());
        if (!reader.parse(file, config->service_defaults)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed loading service defaults from: %s", service_defaults_file.c_str());
            exit(1);
        }
    }

    this->declare_parameter("service_timeout_sec", 20.0f); // 20 sec
    config->service_timeout_ns = this->get_parameter("service_timeout_sec").as_double() * 1000000000;

    this->declare_parameter("service_message_mapping_verbose", false);
    config->service_message_mapping_verbose = this->get_parameter("service_message_mapping_verbose").as_bool();    
}