#include "phntm_bridge/const.hpp"
#include "phntm_bridge/config.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include "sio_message.h"
#include <fstream>
#include <chrono>
#include <rclcpp/parameter_value.hpp>
#include <string>
#include <vector>

namespace phntm {

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
        try {
            this->declare_parameter("id_robot", "", id_robot_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

        // Robot auth key
        rcl_interfaces::msg::ParameterDescriptor key_descriptor;
        key_descriptor.description = "Robot auth key on Phntm Cloud Brudge";
        try {
            this->declare_parameter("key", "", key_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

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
        try {
            this->declare_parameter("name", "Unnamed Robot", name_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->robot_name = this->get_parameter("name").as_string();

        // Maintainer's email
        rcl_interfaces::msg::ParameterDescriptor email_descriptor;
        email_descriptor.description = "Maintainer e-mail address";
        try {
            this->declare_parameter("maintainer_email", "", email_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->maintainer_email = this->get_parameter("maintainer_email").as_string();

        // description shown in the UI

        rcl_interfaces::msg::ParameterDescriptor about_dialog_header_descriptor;
        about_dialog_header_descriptor.description = "Robot description header shown in the pop-up dialog";
        about_dialog_header_descriptor.additional_constraints = "Some HTML is ok";
        try {
            this->declare_parameter("about_dialog_header", "", about_dialog_header_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->about_dialog_header = this->get_parameter("about_dialog_header").as_string();

        rcl_interfaces::msg::ParameterDescriptor about_dialog_descriptor;
        about_dialog_descriptor.description = "Robot description shown in the pop-up dialog";
        about_dialog_descriptor.additional_constraints = "Some HTML is ok";
        try {
            this->declare_parameter("about_dialog", "", about_dialog_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->about_dialog = this->get_parameter("about_dialog").as_string();

        // will check these packages on 1st (container) start
        rcl_interfaces::msg::ParameterDescriptor extra_pkg_descriptor;
        extra_pkg_descriptor.description = "ROS packages to check for on first Bridge run";
        extra_pkg_descriptor.additional_constraints = "Folder path or ROS package name";
        try {
            this->declare_parameter("extra_packages", std::vector<std::string>(), extra_pkg_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->extra_packages = this->get_parameter("extra_packages").as_string_array();
        
        // webrtc config
        try {
            this->declare_parameter("use_cloud_ice_config", true);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->use_cloud_ice_config = this->get_parameter("use_cloud_ice_config").as_bool();
        try {
            this->declare_parameter("ice_servers", std::vector<std::string>());
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("ice_username", ""); // id_robot if empty
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("ice_secret", "");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

        config->ice_servers_custom = this->get_parameter("ice_servers").as_string_array(); // these get added to cloud config, or used when use_cloud_ice_config=false
        if (config->ice_servers_custom.size()) {
            log("Custom ICE servers: ");
            for (size_t i = 0; i < config->ice_servers_custom.size(); ++i) {
                log("\t" + config->ice_servers_custom[i]);
            }
        }
        std::copy(config->ice_servers_custom.begin(), config->ice_servers_custom.end(), std::back_inserter(config->ice_servers)); // server config gets added here
        
        // ice credentials
        config->ice_username = this->get_parameter("ice_username").as_string();
        if (config->ice_username.empty())
            config->ice_username = config->id_robot;
        config->ice_secret = this->get_parameter("ice_secret").as_string();

        try {
            this->declare_parameter("enable_ice_udp_mux", true); // multiple WebRTC peer connections can be multiplexed over a single UDP port
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("enable_ice_tcp", false); // in addition to the default UDP candidates, the library will also attempt to establish peer-to-peer connections over TCP if UDP is unavailable or blocked by network restrictions.
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("disable_fingerprint_verification", false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->enable_ice_udp_mux = this->get_parameter("enable_ice_udp_mux").as_bool();
        config->enable_ice_tcp = this->get_parameter("enable_ice_tcp").as_bool();
        config->disable_fingerprint_verification = this->get_parameter("disable_fingerprint_verification").as_bool();

        // prevent reading sensitive stuffs
        this->set_parameter(rclcpp::Parameter("key", "*************"));
        this->set_parameter(rclcpp::Parameter("ice_username", "*************"));
        this->set_parameter(rclcpp::Parameter("ice_secret", "*************"));
        
        // Cloud Bridge host
        try {
            this->declare_parameter("bridge_server_address", "https://us-ca.bridge.phntm.io");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        
        /// Cloud Bridge files uploader port
        try {
            this->declare_parameter("file_upload_port", 1336);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        
        // socket.io config
        try {
            this->declare_parameter("sio_port", 1337);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("sio_path", "/robot/socket.io/"); // needs to end with /
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("sio_connection_retry_sec", 2.0);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("sio_ssl_verify", true);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("sio_debug", false); // prints internal sio debug
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("sio_verbose", false); // prints received and sent messages
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

        // services collapsed in the ui menu (still operational, parameneter services by default)
        std::vector<std::string> collapse_services_default = { "rcl_interfaces/srv/DescribeParameters", // we have UI for better access to parameteres
                                                               "rcl_interfaces/srv/GetParameterTypes",
                                                               "rcl_interfaces/srv/GetParameters",
                                                               "rcl_interfaces/srv/ListParameters",
                                                               "rcl_interfaces/srv/SetParameters",
                                                               "rcl_interfaces/srv/SetParametersAtomically",
                                                               "type_description_interfaces/srv/GetTypeDescription",
                                                               "rcl_interfaces/srv/GetLoggerLevels",
                                                               "rcl_interfaces/srv/SetLoggerLevels"
                                                             };
        try {
            this->declare_parameter("collapse_services_default", collapse_services_default);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->collapse_services = this->get_parameter("collapse_services_default").as_string_array(); // system defaults
        rcl_interfaces::msg::ParameterDescriptor collapse_services_descriptor;
        collapse_services_descriptor.description = "The UI will collapse these services";
        collapse_services_descriptor.additional_constraints="Service id or type";
        try {
            this->declare_parameter("collapse_services", std::vector<std::string>(), collapse_services_descriptor); // custom - will be added to defauls
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto collapse_services_val = this->get_parameter("collapse_services").as_string_array();
        for (auto srv : collapse_services_val) {
            if (std::find(config->collapse_services.begin(), config->collapse_services.end(), srv) == config->collapse_services.end()) {
                config->collapse_services.push_back(srv);
            }
        }

        // UI will collapse services we can't handle
        rcl_interfaces::msg::ParameterDescriptor collape_unhandled_services_descriptor;
        collape_unhandled_services_descriptor.description = "The UI will collapse services with unsupported message types";
        try {
            this->declare_parameter("collapse_unhandled_services", true, collape_unhandled_services_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->collapse_unhandled_services = this->get_parameter("collapse_unhandled_services").as_bool();

        // blacklist topics from discovery (msg type or full topic id)
        std::vector<std::string> blacklist_topics_default { }; // no system defaults here
        try {
            this->declare_parameter("blacklist_topics_default", blacklist_topics_default); 
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->blacklist_topics = this->get_parameter("blacklist_topics_default").as_string_array(); // system default
        rcl_interfaces::msg::ParameterDescriptor blacklist_topics_descriptor;
        blacklist_topics_descriptor.description = "Blacklist topics from discovery";
        blacklist_topics_descriptor.additional_constraints="Topic id or type";
        try {
            this->declare_parameter("blacklist_topics", std::vector<std::string>(), blacklist_topics_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto blacklist_topics_val = this->get_parameter("blacklist_topics").as_string_array();
        for (auto t : blacklist_topics_val) {
            if (std::find(config->blacklist_topics.begin(), config->blacklist_topics.end(), t) == config->blacklist_topics.end()) {
                config->blacklist_topics.push_back(t);
            }
        }
        if (config->blacklist_topics.size()) {
            log("Blacklisted topics:");
            for (size_t i = 0; i < config->blacklist_topics.size(); ++i) {
                log("\t" + config->blacklist_topics[i]);
            }
        }
        
        // blacklist services from discovery (msg type or full service id)
        std::vector<std::string> blacklist_services_default { "phntm_interfaces/srv/FileRequest" }; // FileRequest srv only intended for internal use between Bridge and Agents
        try {
            this->declare_parameter("blacklist_services_default", blacklist_services_default);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->blacklist_services = this->get_parameter("blacklist_services_default").as_string_array(); // system defaults
        rcl_interfaces::msg::ParameterDescriptor blacklist_services_descriptor;
        blacklist_services_descriptor.description = "Blacklist services from discovery";
        blacklist_services_descriptor.additional_constraints="Service id or type";
        try {
            this->declare_parameter("blacklist_services", std::vector<std::string>(), blacklist_services_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto blacklist_services_val = this->get_parameter("blacklist_services").as_string_array();
        for (auto srv : blacklist_services_val) {
            if (std::find(config->blacklist_services.begin(), config->blacklist_services.end(), srv) == config->blacklist_services.end()) {
                config->blacklist_services.push_back(srv);
            }
        }
        if (config->blacklist_services.size()) {
            log("Blacklisted services:");
            for (size_t i = 0; i < config->blacklist_services.size(); ++i) {
                log("\t" + config->blacklist_services[i]);
            }
        }

        // blacklist msg types (topics/services are discovered but not deserialized or serialized)
        std::vector<std::string> blacklist_msg_types_default { "sensor_msgs/PointCloud", // pointclouds need compression before streaming, not supported yet
                                                                "sensor_msgs/msg/PointCloud2",
                                                                "cost_map_msgs/CostMap", // cost map needs compression before streaming, not supported yet
                                                                "nav_msgs/msg/OccupancyGrid",
                                                                "phntm_interfaces/msg/FileChunk" // only intended for internal use between Bridge and Agents
                                                              };
        try {
            this->declare_parameter("blacklist_msg_types_default", blacklist_msg_types_default);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->blacklist_msg_types = this->get_parameter("blacklist_msg_types_default").as_string_array(); // system defaults;
        rcl_interfaces::msg::ParameterDescriptor blacklist_msg_types_descriptor;
        blacklist_msg_types_descriptor.description = "Blacklist message types discovery";
        blacklist_msg_types_descriptor.additional_constraints="Topic or service type";
        try {
            this->declare_parameter("blacklist_msg_types", std::vector<std::string>(), blacklist_msg_types_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto blacklist_msg_types_val = this->get_parameter("blacklist_msg_types").as_string_array();
        for (auto msg_type : blacklist_msg_types_val) {
            if (std::find(config->blacklist_msg_types.begin(), config->blacklist_msg_types.end(), msg_type) == config->blacklist_services.end()) {
                config->blacklist_msg_types.push_back(msg_type);
            }
        }
        if (config->blacklist_msg_types.size()) {
            log("Blacklisted message types:");
            for (size_t i = 0; i < config->blacklist_msg_types.size(); ++i) {
                log("\t" + config->blacklist_msg_types[i]);
            }
        }

        // params editing
        try {
            this->declare_parameter("enable_node_parameters_read", true);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("enable_node_parameters_write", false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("blacklist_parameter_services", std::vector<std::string>());
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->enable_node_parameters_read = this->get_parameter("enable_node_parameters_read").as_bool();
        config->enable_node_parameters_write = this->get_parameter("enable_node_parameters_write").as_bool();
        config->blacklist_parameter_services = this->get_parameter("blacklist_parameter_services").as_string_array();
        
        // logging
        try {
            this->declare_parameter("log_sdp", false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->log_sdp = this->get_parameter("log_sdp").as_bool();
        try {
            this->declare_parameter("log_heartbeat", false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->log_heartbeat = this->get_parameter("log_heartbeat").as_bool();
        try {
            this->declare_parameter("log_message_every_sec", 10.0f);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->log_message_every_sec = this->get_parameter("log_message_every_sec").as_double();

        // bloud bridge stuffs
        config->bridge_server_address = this->get_parameter("bridge_server_address").as_string();
        config->file_upload_port = this->get_parameter("file_upload_port").as_int();
        config->sio_port = this->get_parameter("sio_port").as_int();
        config->sio_path = this->get_parameter("sio_path").as_string();
        config->sio_ssl_verify = this->get_parameter("sio_ssl_verify").as_bool();
        config->sio_debug = this->get_parameter("sio_debug").as_bool();
        config->sio_verbose = this->get_parameter("sio_verbose").as_bool();
        config->sio_connection_retry_sec = this->get_parameter("sio_connection_retry_sec").as_double();
        if (config->bridge_server_address.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Param bridge_server_address not provided!");
            exit(1);
        }
        config->uploader_address = fmt::format("{}:{}", config->bridge_server_address, config->file_upload_port);

        // conn LED control via topic (blinks when connecting; on when connected; off = bridge not running)
        try {
            this->declare_parameter("conn_led_topic", "");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->conn_led_topic = this->get_parameter("conn_led_topic").as_string();
        // data LED control via topic (flashes when any data is sent via webrtc; off when not connected)
        try {
            this->declare_parameter("data_led_topic", "");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->data_led_topic = this->get_parameter("data_led_topic").as_string();
        
        // conn/data LED control via GPIO
        try {
            this->declare_parameter("conn_led_gpio_chip", "/dev/gpiochip0"); // PI5 default, use gpiodetect to list available
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->conn_led_gpio_chip = this->get_parameter("conn_led_gpio_chip").as_string();
        try {
            this->declare_parameter("conn_led_pin", -1); // set GPIO number
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->conn_led_pin = this->get_parameter("conn_led_pin").as_int();
        try {
            this->declare_parameter("data_led_pin", -1); // set GPIO number
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->data_led_pin = this->get_parameter("data_led_pin").as_int();

        // introspection
        try {
            this->declare_parameter("discovery_period_sec", 2.0f);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->discovery_period_sec = this->get_parameter("discovery_period_sec").as_double();
        try {
            this->declare_parameter("stop_discovery_after_sec", -1.0f); // <0 = run indefinitely (don't show control)
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->stop_discovery_after_sec = this->get_parameter("stop_discovery_after_sec").as_double();
        try {
            this->declare_parameter("introspection_verbose", false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->introspection_verbose = this->get_parameter("introspection_verbose").as_bool();

        // wifi monitoring + scan
        try {
            this->declare_parameter("wifi_monitor_topic", "/iw_status"); // Agent writes here
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->wifi_monitor_topic = this->get_parameter("wifi_monitor_topic").as_string();
        try {
            this->declare_parameter("enable_wifi_scan", true); // enables scan without roaming
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->enable_wifi_scan = this->get_parameter("enable_wifi_scan").as_bool();
        try {
            this->declare_parameter("enable_wifi_roam", false); // enables roaming (potentially dangerous)
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->enable_wifi_roam = this->get_parameter("enable_wifi_roam").as_bool();
        
        // battery
        try {
            this->declare_parameter("battery_topic", "/battery"); // use this in the ui 
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->battery_topic = this->get_parameter("battery_topic").as_string();

        // docker
        try {
            this->declare_parameter("enable_docker_control", true);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->docker_control_enabled = this->get_parameter("enable_docker_control").as_bool();
        try {
            this->declare_parameter("docker_monitor_topic", "/docker_info");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->docker_monitor_topic = this->get_parameter("docker_monitor_topic").as_string();

        // input configs that get passed to ui
        std::vector<std::string> default_input_drivers { "JoyInputDriver" };
        try {
            this->declare_parameter("input_drivers", default_input_drivers); // empty array to disable input entirely, services are still set up
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->input_drivers = this->get_parameter("input_drivers").as_string_array();

        // default input config for the web UI
        try {
            this->declare_parameter("input_defaults", "");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
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
        try {
            this->declare_parameter("service_defaults", "");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto service_defaults_file = this->get_parameter("service_defaults").as_string();
        if (!service_defaults_file.empty()) {
            Json::Reader reader;
            std::ifstream file(service_defaults_file.c_str());
            if (!reader.parse(file, config->service_defaults)) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed loading service defaults from: %s", service_defaults_file.c_str());
                exit(1);
            }
        }

        try {
            this->declare_parameter("default_service_timeout_sec", 20.0f); // 20 sec
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->default_service_timeout_sec = this->get_parameter("default_service_timeout_sec").as_double();

        try {
            this->declare_parameter("service_calls_verbose", false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->service_calls_verbose = this->get_parameter("service_calls_verbose").as_bool();    

        try {
            this->declare_parameter("webrtc_debug", true);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->webrtc_debug = this->get_parameter("webrtc_debug").as_bool(); 

        try {
            this->declare_parameter("webrtc_verbose", false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->webrtc_verbose = this->get_parameter("webrtc_verbose").as_bool();

        try {
            this->declare_parameter("file_chunks_topic", "/file_chunks");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->file_chunks_topic = this->get_parameter("file_chunks_topic").as_string();

        try {
            this->declare_parameter("low_fps_default", 25); // overwrite per topic
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

        // defaults overriden by topic
        try {
            this->declare_parameter("encoder_default_hw_device", "sw"); // sw, cuda, vaapi
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("encoder_default_thread_count", 2);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("encoder_default_gop_size", 30); // kf every 
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("encoder_default_bit_rate", 5000000); // 610 KB/s
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

        try {
            this->declare_parameter("video_topics_default_depth", 10); // don't miss keyframes
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("video_topics_default_reliability", "RELIABLE"); // don't miss keyframes
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("video_topics_default_durability", "VOLATILE");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("video_topics_default_lifespan_sec", -1.0);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

        try {
            this->declare_parameter("image_topics_default_depth", 1);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("image_topics_default_reliability", "BEST_EFFORT");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("image_topics_default_durability", "VOLATILE");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter("image_topics_default_lifespan_sec", -1.0);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

        // ui peer limit
        rcl_interfaces::msg::ParameterDescriptor peer_limit_descriptor;
        peer_limit_descriptor.description = "UI connected peers limit (0 = no limit)";
        try {
            this->declare_parameter("peer_limit", 10, peer_limit_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->peer_limit = this->get_parameter("peer_limit").as_int();
        log("UI connected peers limit: " + std::to_string(config->peer_limit));

        // ui disconnect timeout
        rcl_interfaces::msg::ParameterDescriptor ui_background_disconnect_descriptor;
        ui_background_disconnect_descriptor.description = "UI will disconnect after this many seconds in the background (0 = immediately)";
        try {
            this->declare_parameter("ui_background_disconnect_sec", 120.0, ui_background_disconnect_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        config->ui_background_disconnect_sec = this->get_parameter("ui_background_disconnect_sec").as_double();
        log("UI bg disconnect is: " + std::to_string(config->ui_background_disconnect_sec) + " sec");

        // custom JS includes
        rcl_interfaces::msg::ParameterDescriptor custom_includes_js_descriptor;
        custom_includes_js_descriptor.description = "Custom JS includes";
        try {
            this->declare_parameter("ui_custom_includes_js", std::vector<std::string>(), custom_includes_js_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto custom_includes_js_val = this->get_parameter("ui_custom_includes_js").as_string_array();
        for (auto inc : custom_includes_js_val) {
            if (std::find(config->ui_custom_includes_js.begin(), config->ui_custom_includes_js.end(), inc) == config->ui_custom_includes_js.end()) {
                config->ui_custom_includes_js.push_back(inc);
            }
        }
        if (config->ui_custom_includes_js.size()) {
            log("Custom UI JS includes:");
            for (size_t i = 0; i < config->ui_custom_includes_js.size(); ++i) {
                log("\t" + config->ui_custom_includes_js[i]);
            }
        }

        // custom CSS includes
        rcl_interfaces::msg::ParameterDescriptor custom_includes_css_descriptor;
        custom_includes_css_descriptor.description = "Custom CSS includes";
        try {
            this->declare_parameter("ui_custom_includes_css", std::vector<std::string>(), custom_includes_css_descriptor);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto custom_includes_css_val = this->get_parameter("ui_custom_includes_css").as_string_array();
        for (auto inc : custom_includes_css_val) {
            if (std::find(config->ui_custom_includes_css.begin(), config->ui_custom_includes_css.end(), inc) == config->ui_custom_includes_css.end()) {
                config->ui_custom_includes_css.push_back(inc);
            }
        }
        if (config->ui_custom_includes_css.size()) {
            log("Custom UI CSS includes:");
            for (size_t i = 0; i < config->ui_custom_includes_css.size(); ++i) {
                log("\t" + config->ui_custom_includes_css[i]);
            }
        }
    }

    rclcpp::QoS PhntmBridge::loadTopicQoSConfig(std::string topic, size_t default_depth, std::string default_reliability, std::string default_durability, float default_lifespan_sec) {
    
        try {
            this->declare_parameter(topic + ".history_depth", static_cast<int>(default_depth)); // 0 = system default, 1 = reliable, 2 = best effort (default)
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto depth = this->get_parameter(topic + ".history_depth").as_int();
        rclcpp::QoS qos(depth); // keep last 1

        try {
            this->declare_parameter(topic + ".reliability", default_reliability); // 0 = system default, 1 = reliable, 2 = best effort (default)
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        try {
            this->declare_parameter(topic + ".durability", default_durability); // 0 = system default, 1 = transient local, 2 = volatile (default)
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

        auto reliability_str = trim(strToLower(this->get_parameter(topic + ".reliability").as_string()));
        if (reliability_str == "reliable") {
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        } else if (reliability_str == "best_effort" || reliability_str == "besteffort") {
            qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        } else if (reliability_str == "system_default" || reliability_str == "systemdefault") {
            qos.reliability(rclcpp::ReliabilityPolicy::SystemDefault);
        } else {
            log("Invalid reliability specified for " + topic + ": '" + reliability_str + "'; using system_default", true);
            qos.reliability(rclcpp::ReliabilityPolicy::SystemDefault);
        }

        auto durability_str = trim(strToLower(this->get_parameter(topic + ".durability").as_string()));
        if (durability_str == "transient_local" || durability_str == "transientlocal") {
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        } else if (durability_str == "volatile") {
            qos.durability(rclcpp::DurabilityPolicy::Volatile);
        } else if (durability_str == "system_default" || durability_str == "systemdefault") {
            qos.durability(rclcpp::DurabilityPolicy::SystemDefault);
        } else {
            log("Invalid durability specified for " + topic + ": '" + durability_str + "'; using system_default", true);
            qos.durability(rclcpp::DurabilityPolicy::SystemDefault);
        }
        
        try {
            this->declare_parameter(topic + ".lifespan_sec", default_lifespan_sec); // num sec as double, -1.0 infinity (default)
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        auto lifespan_sec = this->get_parameter(topic + ".lifespan_sec").as_double();
        if (lifespan_sec < 0.0f) {
            qos.lifespan(rclcpp::Duration::max());
        } else {
            auto lifespan_total_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(lifespan_sec));
            auto lifespan_sec_part = std::chrono::duration_cast<std::chrono::seconds>(lifespan_total_ns);
            auto lifespan_ns_part = lifespan_total_ns - lifespan_sec_part;
            qos.lifespan(lifespan_sec < 0.0 ? rclcpp::Duration::max() : rclcpp::Duration(lifespan_sec_part.count(), lifespan_ns_part.count()));
        }

        return qos;
    }

    BridgeConfig::MediaTopicConfig PhntmBridge::loadMediaTopicConfig(std::string topic, std::string msg_type) {
        auto res = BridgeConfig::MediaTopicConfig();

        if (isImageOrVideoType(msg_type)) { 
            try {
                this->declare_parameter(topic + ".debug_num_frames", 1); // will debug this many frames (inspects NAL units)
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".debug_verbose", false); // will debug this many frames (instects NAL units)
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".create_node", true); // if false, subscriber is created on the main node
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".pts_source", "LOCAL"); // LOCAL, PACKET or MESSAGE
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            
            res.debug_num_frames = this->get_parameter(topic + ".debug_num_frames").as_int();
            res.debug_verbose = this->get_parameter(topic + ".debug_verbose").as_bool();
            auto pts_source_str = trim(strToLower(this->get_parameter(topic + ".pts_source").as_string()));
            if (pts_source_str == "packet") {
                res.pts_source = PTS_SOURCE_PACKET_PTS;
            } else if (pts_source_str == "message") {
                res.pts_source = PTS_SOURCE_MESSAGE_HEADER;
            } else if (pts_source_str == "local") {
                res.pts_source = PTS_SOURCE_LOCAL_TIME;
            } else {
                log("Invalid value for "+topic+".pts_source: " + pts_source_str+"; using local time");
                res.pts_source = PTS_SOURCE_LOCAL_TIME;
            }
            res.create_node = this->get_parameter(topic + ".create_node").as_bool();
        }

        // Depth visualization extras
        if (isImageType(msg_type)) {
            try {
                this->declare_parameter(topic + ".colormap", 0); // will debug this many frames (instects NAL units)
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".max_sensor_value", 255.0f); // will debug this many frames (instects NAL units)
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
             try {
                this->declare_parameter(topic + ".encoder_hw_device", this->get_parameter("encoder_default_hw_device").as_string()); // none, cuda, vaapi
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
             try {
                this->declare_parameter(topic + ".encoder_thread_count", this->get_parameter("encoder_default_thread_count").as_int()); //2
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
             try {
                this->declare_parameter(topic + ".encoder_gop_size", this->get_parameter("encoder_default_gop_size").as_int()); //60,  kf every 
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
             try {
                this->declare_parameter(topic + ".encoder_bit_rate", this->get_parameter("encoder_default_bit_rate").as_int()); // 5 * 8 * 1024
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

            res.colormap = this->get_parameter(topic + ".colormap").as_int();
            res.max_sensor_value = this->get_parameter(topic + ".max_sensor_value").as_double();
            res.encoder_hw_device = this->get_parameter(topic + ".encoder_hw_device").as_string();
            res.encoder_thread_count = this->get_parameter(topic + ".encoder_thread_count").as_int();
            res.encoder_gop_size = this->get_parameter(topic + ".encoder_gop_size").as_int();
            res.encoder_bit_rate = this->get_parameter(topic + ".encoder_bit_rate").as_int();
        }
        
        return res;
    }

    bool isQoSParam(std::string param) {
        return std::find(QOS_TOPIC_CONFIG_PARAMS.begin(), QOS_TOPIC_CONFIG_PARAMS.end(), param) != QOS_TOPIC_CONFIG_PARAMS.end();
    }

    bool isBlacklistedTopicUIParam(std::string param) {
        return std::find(UI_BLACKLIST_TOPIC_CONFIG_PARAMS.begin(), UI_BLACKLIST_TOPIC_CONFIG_PARAMS.end(), param) != UI_BLACKLIST_TOPIC_CONFIG_PARAMS.end();
    }

     bool isBlacklistedGlobalUIParam(std::string param) {
        return std::find(UI_BLACKLIST_GLOBAL_CONFIG_PARAMS.begin(), UI_BLACKLIST_GLOBAL_CONFIG_PARAMS.end(), param) != UI_BLACKLIST_GLOBAL_CONFIG_PARAMS.end();
    }

    void addParamToMessage(std::string key, rclcpp::Parameter param, sio::object_message::ptr out_msg) {
        switch (param.get_type()) {
            case rclcpp::ParameterType::PARAMETER_BOOL:
                out_msg->get_map().emplace(key, sio::bool_message::create(param.as_bool()));
                break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                out_msg->get_map().emplace(key, sio::int_message::create(param.as_int()));
                break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
                out_msg->get_map().emplace(key, sio::double_message::create(param.as_double()));
                break;
            case rclcpp::ParameterType::PARAMETER_STRING:
                out_msg->get_map().emplace(key, sio::string_message::create(param.as_string()));
                break;
            case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
                auto vals = param.as_bool_array();
                auto arr = sio::array_message::create();
                for (auto v : vals) {
                    arr->get_vector().push_back(sio::bool_message::create(v));
                }
                out_msg->get_map().emplace(key, arr);
                } break;
            case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
                auto vals = param.as_integer_array();
                auto arr = sio::array_message::create();
                for (auto v : vals) {
                    arr->get_vector().push_back(sio::int_message::create(v));
                }
                out_msg->get_map().emplace(key, arr);
                } break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
                auto vals = param.as_double_array();
                auto arr = sio::array_message::create();
                for (auto v : vals) {
                    arr->get_vector().push_back(sio::double_message::create(v));
                }
                out_msg->get_map().emplace(key, arr);
                } break;
            case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
                auto vals = param.as_string_array();
                auto arr = sio::array_message::create();
                for (auto v : vals) {
                    arr->get_vector().push_back(sio::string_message::create(v));
                }
                out_msg->get_map().emplace(key, arr);
                } break;
            default:
                std::cout << "Conf " << key << " type invalid; ignoring" << std::endl;
                break;
        }
    }

    std::vector<std::string> PhntmBridge::getAllConfigPrefixes() {
        std::vector<std::string> res;

        std::map< std::string, rclcpp::Parameter> all_params;
        this->get_parameters("", all_params);

        for (const auto& p : all_params) {
            auto key = p.first;
            if (key.find('/') != 0)
                continue;
            
            auto dot_pos = key.find('.');
            if (dot_pos == std::string::npos)
                continue;
            
            std::string prefix = key.substr(0, dot_pos);
            std::string param = key.substr(dot_pos + 1);
            
            if (std::find(res.begin(), res.end(), prefix) == res.end()) {
                res.push_back(prefix);
                //std::cout << CYAN << "Found prefixed conf '" << prefix << "' param " << param << CLR << std::endl;
            }
        }
        return res;
    }

    sio::message::ptr PhntmBridge::loadPrefixedUIConfig(std::string prefix) {
        auto res = sio::object_message::create();

        std::map< std::string, rclcpp::Parameter> all_params;
        this->get_parameters(prefix, all_params);
        for (const auto& p : all_params) {
            if (res->get_map().find(p.first) != res->get_map().end() // only add once
                || isQoSParam(p.first) || isBlacklistedTopicUIParam(p.first)
            ) continue;

            auto key = p.first;
            //std::cout << CYAN << "Passing custom UI conf '" << key << "' for " << prefix << ": " << key << "=" << param << CLR << std::endl;
            
            addParamToMessage(key, p.second, res);
        }

        return res;
     }

    void PhntmBridge::getExtraCustomParams(sio::object_message::ptr out_msg) {
        
        std::map< std::string, rclcpp::Parameter> all_params;
        this->get_parameters("", all_params);

        for (const auto& p : all_params) {
            auto key = p.first;
            if (key.find('/') == 0)
                continue; // skip prefixed

             if (key.find('_') == 0)
                continue; // skip underscored

            if(isBlacklistedGlobalUIParam(key))
                continue;
            
            if (out_msg->get_map().find(key) != out_msg->get_map().end())
                continue; // already in json
            
            std::cout << CYAN << "Adding custom UI conf param: " << key << std::endl;

            addParamToMessage(key, p.second, out_msg);
        }

    }

    

}