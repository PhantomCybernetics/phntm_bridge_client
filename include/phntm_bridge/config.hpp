#pragma once

#include <cstdio>
#include <vector>
#include <string>
#include <json/json.h>

#include "lib.hpp"

namespace phntm {

    class BridgeConfig {
        public:
            std::string id_robot, auth_key, robot_name, maintainer_email;
            std::string about_dialog, about_dialog_header; // shown in the ui pop-up dialog
            std::string ros_distro, git_head_sha, latest_git_tag, rmw_implementation;
            
            std::vector<std::string> extra_packages;

            bool use_cloud_ice_config;
            std::vector<std::string> ice_servers, ice_servers_custom;
            std::string ice_username, ice_secret;

            std::vector<std::string> blacklist_topics, blacklist_services, blacklist_msg_types, blacklist_parameter_services;
            bool enable_node_parameters_read;
            bool enable_node_parameters_write;

            double log_message_every_sec;
            double discovery_period_sec, stop_discovery_after_sec;
            bool introspection_verbose;

            std::string bridge_server_address, sio_path, uploader_address;
            int file_upload_port, sio_port;
            bool sio_ssl_verify, sio_debug, sio_verbose;
            double sio_connection_retry_sec;
            
            std::string conn_led_topic, data_led_topic, conn_led_gpio_chip;
            int conn_led_pin, data_led_pin;

            std::vector<std::string> collapse_services;
            std::string battery_topic, wifi_monitor_topic, docker_monitor_topic;
            bool docker_control_enabled, enable_wifi_scan, enable_wifi_roam, collapse_unhandled_services;
            float default_service_timeout_sec;

            std::vector<std::string> input_drivers;
            bool service_calls_verbose;

            Json::Value input_defaults, service_defaults;

            bool webrtc_debug, webrtc_verbose;
            bool enable_ice_udp_mux, enable_ice_tcp, disable_fingerprint_verification;
            bool log_heartbeat, log_sdp;

            std::string file_chunks_topic;

            std::vector<std::string> ui_custom_includes_js, ui_custom_includes_css;
            uint peer_limit;
            float ui_background_disconnect_sec;

            struct MediaTopicConfig {
                size_t debug_num_frames;
                bool debug_verbose;
                bool create_node;
                uint pts_source;
                uint colormap;
                double max_sensor_value;
                std::string encoder_hw_device;
                int encoder_thread_count;
                uint encoder_gop_size;
                uint encoder_bit_rate;
                bool print_time_deltas;
            };
    };

}