#pragma once

#include <string>
#include <map>
#include <vector>

namespace phntm {
    
    const std::string GRAY = "\033[1;30m";
    const std::string RED = "\033[31m";
    const std::string GREEN = "\033[92m";
    const std::string LIME = "\033[32m";
    const std::string BLUE = "\033[94m";
    const std::string YELLOW = "\033[33m";
    const std::string MAGENTA = "\033[35m";
    const std::string CYAN = "\033[96m";
    const std::string WHITE = "\033[37m2";
    const std::string CLR = "\033[0m";

    const std::string IMAGE_MSG_TYPE = "sensor_msgs/msg/Image";
    const std::string COMPRESSED_IMAGE_MSG_TYPE = "sensor_msgs/msg/CompressedImage";
    const std::string VIDEO_STREAM_MSG_TYPE = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";

    const std::string HEARTBEAT_CHANNEL_ID = "_heartbeat";
    
    const uint PTS_SOURCE_LOCAL_TIME     = 0; // default
    const uint PTS_SOURCE_PACKET_PTS     = 1;
    const uint PTS_SOURCE_MESSAGE_HEADER = 2;

    const std::vector<std::string> QOS_TOPIC_CONFIG_PARAMS = {
        "durability",
        "reliability",
        "history_depth",
        "lifespan_sec"
    };

    const std::vector<std::string> UI_BLACKLIST_TOPIC_CONFIG_PARAMS = {
        "debug_num_frames",
        "debug_verbose",
        "create_node",
        "pts_source",
        "colormap",
        "max_sensor_value",
        "encoder_hw_device",
        "encoder_thread_count",
        "encoder_gop_size","",
        "encoder_bit_rate",
    };

    const std::vector<std::string> UI_BLACKLIST_GLOBAL_CONFIG_PARAMS = {
        "agent_update_period_sec",
        "blacklist_msg_types",
        "blacklist_msg_types_default",
        "blacklist_services",
        "blacklist_services_default",
        "blacklist_topics",
        "blacklist_topics_default",
        "bridge_server_address",
        "collapse_services_default",
        "conn_led_gpio_chip",
        "conn_led_pin",
        "conn_led_topic",
        "data_led_pin",
        "data_led_topic",
        "disable_fingerprint_verification",
        "discovery_period_sec",
        "disk_volume_paths",
        "enable_ice_tcp",
        "enable_ice_udp_mux",
        "encoder_default_bit_rate",
        "encoder_default_gop_size",
        "encoder_default_hw_device",
        "encoder_default_thread_count",
        "extra_packages",
        "file_chunks_topic",
        "file_upload_port",
        "host_name",
        "ice_secret",
        "ice_servers",
        "ice_username",
        "id_robot",
        "image_topics_default_depth",
        "image_topics_default_durability",
        "image_topics_default_lifespan_sec",
        "image_topics_default_reliability",
        "input_defaults",
        "input_drivers",
        "introspection_verbose",
        "key",
        "log_heartbeat",
        "log_message_every_sec",
        "log_sdp",
        "low_fps_default",
        "maintainer_email",
        "name",
        "peer_limit",
        "qos_overrides./parameter_events.publisher.depth",
        "qos_overrides./parameter_events.publisher.durability",
        "qos_overrides./parameter_events.publisher.history",
        "qos_overrides./parameter_events.publisher.reliability",
        "service_calls_verbose",
        "service_defaults",
        "sio_connection_retry_sec",
        "sio_debug",
        "sio_path",
        "sio_port",
        "sio_ssl_verify",
        "sio_verbose",
        "start_type_description_service",
        "stop_discovery_after_sec",
        "system_info_topic",
        "ui_custom_includes_css",
        "ui_custom_includes_js",
        "use_cloud_ice_config",
        "use_sim_time",
        "video_topics_default_depth",
        "video_topics_default_durability",
        "video_topics_default_lifespan_sec",
        "video_topics_default_reliability",
        "webrtc_debug",
        "webrtc_verbose",
        "wifi_interface"
    };
}
