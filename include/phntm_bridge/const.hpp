#pragma once

#include <string>
#include <map>

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
    
}
