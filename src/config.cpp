#include "phntm_bridge/const.hpp"
#include "phntm_bridge/config.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include "sio_message.h"
#include <fstream>
#include <chrono>

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

        // description shown in the UI

        rcl_interfaces::msg::ParameterDescriptor description_header_descriptor;
        description_header_descriptor.description = "Robot description ehader shown in the UI";
        description_header_descriptor.additional_constraints = "Some HTML is ok";
        this->declare_parameter("description_header", "", description_header_descriptor);
        config->description_header = this->get_parameter("description_header").as_string();

        rcl_interfaces::msg::ParameterDescriptor description_descriptor;
        description_descriptor.description = "Robot description shown in the UI";
        description_descriptor.additional_constraints = "Some HTML is ok";
        this->declare_parameter("description", "", description_descriptor);
        config->description = this->get_parameter("description").as_string();

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

        this->declare_parameter("enable_ice_udp_mux", true); // multiple WebRTC peer connections can be multiplexed over a single UDP port
        this->declare_parameter("enable_ice_tcp", false); // in addition to the default UDP candidates, the library will also attempt to establish peer-to-peer connections over TCP if UDP is unavailable or blocked by network restrictions.
        this->declare_parameter("disable_fingerprint_verification", false);
        config->enable_ice_udp_mux = this->get_parameter("enable_ice_udp_mux").as_bool();
        config->enable_ice_tcp = this->get_parameter("enable_ice_tcp").as_bool();
        config->disable_fingerprint_verification = this->get_parameter("disable_fingerprint_verification").as_bool();

        // prevent reading sensitive stuffs
        this->set_parameter(rclcpp::Parameter("key", "*************"));
        this->set_parameter(rclcpp::Parameter("ice_username", "*************"));
        this->set_parameter(rclcpp::Parameter("ice_secret", "*************"));
        
        // Cloud Bridge host
        this->declare_parameter("bridge_server_address", "https://us-ca.bridge.phntm.io");
        
        /// Cloud Bridge files uploader port
        this->declare_parameter("file_upload_port", 1336);
        
        // socket.io config
        this->declare_parameter("sio_port", 1337);
        this->declare_parameter("sio_path", "/robot/socket.io/"); // needs to end with /
        this->declare_parameter("sio_connection_retry_sec", 2.0);
        this->declare_parameter("sio_ssl_verify", true);
        this->declare_parameter("sio_debug", false); // prints internal sio debug
        this->declare_parameter("sio_verbose", false); // prints received and sent messages

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
        this->declare_parameter("collapse_services_default", collapse_services_default);
        config->collapse_services = this->get_parameter("collapse_services_default").as_string_array(); // system defaults
        rcl_interfaces::msg::ParameterDescriptor collapse_services_descriptor;
        collapse_services_descriptor.description = "The UI will collapse these services";
        collapse_services_descriptor.additional_constraints="Service id or type";
        this->declare_parameter("collapse_services", std::vector<std::string>(), collapse_services_descriptor); // custom - will be added to defauls
        auto collapse_services_val = this->get_parameter("collapse_services").as_string_array();
        for (auto srv : collapse_services_val) {
            if (std::find(config->collapse_services.begin(), config->collapse_services.end(), srv) == config->collapse_services.end()) {
                config->collapse_services.push_back(srv);
            }
        }

        // UI will collapse services we can't handle
        rcl_interfaces::msg::ParameterDescriptor collape_unhandled_services_descriptor;
        collape_unhandled_services_descriptor.description = "The UI will collapse services with unsupported message types";
        this->declare_parameter("collapse_unhandled_services", true, collape_unhandled_services_descriptor);
        config->collapse_unhandled_services = this->get_parameter("collapse_unhandled_services").as_bool();

        // blacklist topics from discovery (msg type or full topic id)
        std::vector<std::string> blacklist_topics_default { }; // no system defaults here
        this->declare_parameter("blacklist_topics_default", blacklist_topics_default); 
        config->blacklist_topics = this->get_parameter("blacklist_topics_default").as_string_array(); // system default
        rcl_interfaces::msg::ParameterDescriptor blacklist_topics_descriptor;
        blacklist_topics_descriptor.description = "Blacklist topics from discovery";
        blacklist_topics_descriptor.additional_constraints="Topic id or type";
        this->declare_parameter("blacklist_topics", std::vector<std::string>(), blacklist_topics_descriptor);
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
        this->declare_parameter("blacklist_services_default", blacklist_services_default);
        config->blacklist_services = this->get_parameter("blacklist_services_default").as_string_array(); // system defaults
        rcl_interfaces::msg::ParameterDescriptor blacklist_services_descriptor;
        blacklist_services_descriptor.description = "Blacklist services from discovery";
        blacklist_services_descriptor.additional_constraints="Service id or type";
        this->declare_parameter("blacklist_services", std::vector<std::string>(), blacklist_services_descriptor);
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
        this->declare_parameter("blacklist_msg_types_default", blacklist_msg_types_default);
        config->blacklist_msg_types = this->get_parameter("blacklist_msg_types_default").as_string_array(); // system defaults;
        rcl_interfaces::msg::ParameterDescriptor blacklist_msg_types_descriptor;
        blacklist_msg_types_descriptor.description = "Blacklist message types discovery";
        blacklist_msg_types_descriptor.additional_constraints="Topic or service type";
        this->declare_parameter("blacklist_msg_types", std::vector<std::string>(), blacklist_msg_types_descriptor);
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
        
        // logging
        this->declare_parameter("log_sdp", false);
        config->log_sdp = this->get_parameter("log_sdp").as_bool();
        this->declare_parameter("log_heartbeat", false);
        config->log_heartbeat = this->get_parameter("log_heartbeat").as_bool();
        this->declare_parameter("log_message_every_sec", 10.0f);
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
            log("Adding custom input driver: " + def.value().class_name + " from " + def.value().url);
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
            log("Adding custom service widget: " + def.value().class_name + " from " + def.value().url);
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
            auto d = "Adding service widget mapping: " + conf.value().service + " is " + conf.value().class_name;
            if (!conf.value().data.empty()) {
                d += "; data=" + std::string(conf.value().data.toStyledString());
                log(d, false, false);
            }
            else {
                log(d);
            }
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

        this->declare_parameter("service_calls_verbose", false);
        config->service_calls_verbose = this->get_parameter("service_calls_verbose").as_bool();    

        this->declare_parameter("webrtc_debug", true);
        config->webrtc_debug = this->get_parameter("webrtc_debug").as_bool(); 

        this->declare_parameter("webrtc_verbose", false);
        config->webrtc_verbose = this->get_parameter("webrtc_verbose").as_bool();

        this->declare_parameter("file_chunks_topic", "/file_chunks");
        config->file_chunks_topic = this->get_parameter("file_chunks_topic").as_string();

        this->declare_parameter("low_fps_default", 25); // overwrite per topic

        // defaults overriden by topic
        this->declare_parameter("encoder_default_hw_device", "sw"); // sw, cuda, vaapi
        this->declare_parameter("encoder_default_thread_count", 2);
        this->declare_parameter("encoder_default_gop_size", 30); // kf every 
        this->declare_parameter("encoder_default_bit_rate", 5000000); // 610 KB/s

        this->declare_parameter("video_topics_default_depth", 10); // don't miss keyframes
        this->declare_parameter("video_topics_default_reliability", "RELIABLE"); // don't miss keyframes
        this->declare_parameter("video_topics_default_durability", "VOLATILE");
        this->declare_parameter("video_topics_default_lifespan_sec", -1.0);

        this->declare_parameter("image_topics_default_depth", 1);
        this->declare_parameter("image_topics_default_reliability", "BEST_EFFORT");
        this->declare_parameter("image_topics_default_durability", "VOLATILE");
        this->declare_parameter("image_topics_default_lifespan_sec", -1.0);
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

    sio::message::ptr PhntmBridge::loadTopicMsgTypeExtraConfig(std::string topic, std::string msg_type) {
        auto res = sio::object_message::create();

        try {
            this->declare_parameter(topic + ".low_fps", this->get_parameter("low_fps_default").as_int()); // if 0, no fps warning is shown
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
        res->get_map().emplace("low_fps", sio::int_message::create(this->get_parameter(topic + ".low_fps").as_int()));

        // h.264 encoded frames
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
            
            res->get_map().emplace("debug_num_frames", sio::int_message::create(this->get_parameter(topic + ".debug_num_frames").as_int()));
            res->get_map().emplace("debug_verbose", sio::bool_message::create(this->get_parameter(topic + ".debug_verbose").as_bool()));
            auto pts_source_str = trim(strToLower(this->get_parameter(topic + ".pts_source").as_string()));
            uint pts_source;
            if (pts_source_str == "packet") {
                pts_source = PTS_SOURCE_PACKET_PTS;
            } else if (pts_source_str == "message") {
                pts_source = PTS_SOURCE_MESSAGE_HEADER;
            } else if (pts_source_str == "local") {
                pts_source = PTS_SOURCE_LOCAL_TIME;
            } else {
                log("Invalid value for "+topic+".pts_source: " + pts_source_str+"; using local time");
                pts_source = PTS_SOURCE_LOCAL_TIME;
            }
            res->get_map().emplace("pts_source", sio::int_message::create(pts_source));
            res->get_map().emplace("create_node", sio::bool_message::create(this->get_parameter(topic + ".create_node").as_bool()));
        }

        // depth visualization extras
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

            res->get_map().emplace("colormap", sio::int_message::create(this->get_parameter(topic + ".colormap").as_int()));
            res->get_map().emplace("max_sensor_value", sio::double_message::create(this->get_parameter(topic + ".max_sensor_value").as_double()));
            res->get_map().emplace("encoder_hw_device", sio::string_message::create(this->get_parameter(topic + ".encoder_hw_device").as_string()));
            res->get_map().emplace("encoder_thread_count", sio::int_message::create(this->get_parameter(topic + ".encoder_thread_count").as_int()));
            res->get_map().emplace("encoder_gop_size", sio::int_message::create(this->get_parameter(topic + ".encoder_gop_size").as_int()));
            res->get_map().emplace("encoder_bit_rate", sio::int_message::create(this->get_parameter(topic + ".encoder_bit_rate").as_int()));
        }

        // NN Detections
        if (msg_type == "vision_msgs/msg/Detection2DArray" || msg_type == "vision_msgs/msg/Detection3DArray") { 
            try {
                this->declare_parameter(topic + ".input_width", 416);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".input_height", 416);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".label_map", std::vector<std::string>());  // array of nn class labels
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
                
            res->get_map().emplace("input_width", sio::int_message::create(this->get_parameter(topic + ".input_width").as_int()));
            res->get_map().emplace("input_height", sio::int_message::create(this->get_parameter(topic + ".input_height").as_int()));
            auto labels = sio::array_message::create();
            auto labels_arr = this->get_parameter(topic + ".label_map").as_string_array();
            for (auto l : labels_arr)
                labels->get_vector().push_back(sio::string_message::create(l));
            res->get_map().emplace("label_map", labels);
        }

        if (msg_type == "vision_msgs/msg/Detection3DArray") { 
            try {
                this->declare_parameter(topic + ".model_map", std::vector<std::string>());  // array of nn class models
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".use_model_materials", true);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

            auto models = sio::array_message::create();
            auto models_arr = this->get_parameter(topic + ".model_map").as_string_array();
            for (auto l : models_arr)
                models->get_vector().push_back(sio::string_message::create(l));
            res->get_map().emplace("model_map", models);
            res->get_map().emplace("use_model_materials", sio::bool_message::create(this->get_parameter(topic + ".use_model_materials").as_bool()));
        }

        // Camera Info
        else if (msg_type == "sensor_msgs/msg/CameraInfo") {
            try {
                this->declare_parameter(topic + ".frustum_color", "cyan");
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".frustum_near", 0.01f);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".frustum_far", 1.0f);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".force_frame_id", "");
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

            res->get_map().emplace("frustum_color", sio::string_message::create(this->get_parameter(topic + ".frustum_color").as_string()));
            res->get_map().emplace("frustum_near", sio::double_message::create(this->get_parameter(topic + ".frustum_near").as_double()));
            res->get_map().emplace("frustum_far", sio::double_message::create(this->get_parameter(topic + ".frustum_far").as_double()));
            auto force_frame_id = this->get_parameter(topic + ".force_frame_id").as_string();
            if (!force_frame_id.empty())
                res->get_map().emplace("force_frame_id", sio::string_message::create(force_frame_id));
        }
                
        // Battery
        else if (msg_type == "sensor_msgs/msg/BatteryState") {
            try {
                this->declare_parameter(topic + ".min_voltage", 0.0f);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }
            try {
                this->declare_parameter(topic + ".max_voltage", 12.0f);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) { }

            res->get_map().emplace("min_voltage", sio::double_message::create(this->get_parameter(topic + ".min_voltage").as_double()));
            res->get_map().emplace("max_voltage", sio::double_message::create(this->get_parameter(topic + ".max_voltage").as_double()));
        }

        return res;
    }

}