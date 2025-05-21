#pragma once
#include <cstdint>
#include <map>
#include <rclcpp/subscription.hpp>
#include <string>
#include <memory>
#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "rtc/frameinfo.hpp"
#include "rtc/peerconnection.hpp"
#include "rtc/rtc.hpp"
#include "phntm_bridge/h264_prepacketizer.hpp"

#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include <condition_variable>

namespace phntm {

    class PhntmBridge;
    class WRTCPeer;

    class TopicReaderH264 {

        public:
            static std::shared_ptr<TopicReaderH264> getForTopic(std::string topic, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos, bool use_pts = true, bool debug_verbose = false, int debug_num_frames = 0);
            static std::shared_ptr<TopicReaderH264> getForTopic(std::string topic); // does not create a new one
            static void destroy(std::string topic);

            struct MediaTrackInfo {
                std::shared_ptr<rtc::Track> track;
                std::string id_track;
                uint ssrc;
                bool in_use = true; // false when paused
                bool init_complete = false; // waits for 1st stable signaling state after track open
                std::shared_ptr<rtc::RtpPacketizationConfig> rtpConfig;
                std::shared_ptr<H264PrePacketizer> packetizer;
            };
            bool addOutput(std::shared_ptr<MediaTrackInfo> track_info, std::shared_ptr<rtc::PeerConnection> pc);
            bool removeOutput(std::shared_ptr<MediaTrackInfo> track_info);
            // static void onDCOpen(std::shared_ptr<rtc::DataChannel> dc, std::shared_ptr<rtc::PeerConnection> pc);
            static void onPCSignalingStateChange(std::shared_ptr<WRTCPeer> peer);
        
            static std::string openMediaTrackForTopic(std::string topic, std::shared_ptr<WRTCPeer> peer);
            static void closeMediaTrackForTopic(std::string topic, std::shared_ptr<WRTCPeer> peer);

            TopicReaderH264(std::string topic, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos, bool use_pts, bool debug_verbose, int debug_num_frames);
            ~TopicReaderH264();

            struct OutputMsg {
                std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg;
                uint ts;
                bool is_keyframe;
            };

            struct Output {
                std::shared_ptr<MediaTrackInfo> track_info;
                std::shared_ptr<rtc::PeerConnection> pc;
                
                uint16_t num_sent = 0;
                bool active = true;
                // bool initial_ts_set = false;
                uint32_t ts_base = 0;

                std::optional<std::vector<uint8_t>> last_nal_unit_7 = std::nullopt;
                std::optional<std::vector<uint8_t>> last_nal_unit_8 = std::nullopt;
                std::optional<std::vector<uint8_t>> last_nal_unit_5 = std::nullopt;

                bool logged_closed = false;
                bool logged_init_incomplete = false;
                bool logged_error = false;
                bool logged_exception = false;

                std::thread thread;
                std::queue<OutputMsg> queue;
                std::mutex queue_mutex;
                std::condition_variable queue_cv;
            };

        private:
            static std::map<std::string, std::shared_ptr<TopicReaderH264>> readers;

            std::vector<std::shared_ptr<Output>> outputs; // target data channels & pcs
            std::mutex outputs_mutex;
            static std::mutex readers_mutex;
            std::mutex subscriber_mutex;

            std::string topic;
            std::shared_ptr<PhntmBridge> bridge_node;
            rclcpp::QoS qos;

            void start();
            void stop();
            void onFrame(const std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> data);
            std::shared_ptr<rclcpp::Subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>> sub;
            rclcpp::CallbackGroup::SharedPtr callback_group;
            std::thread processor_thread;
            std::queue<std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>> frame_queue;
            std::mutex frame_queue_mutex;
            std::condition_variable frame_queue_cv;
            void processFramesWorker();
            void processFrame(std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg);
            bool worker_running;
            void outputFramesWorker(std::shared_ptr<Output> output);

            std::vector<std::byte> latest_payload;
            size_t latest_payload_size = 0;
            // rtc::FrameInfo latest_frame_info = rtc::FrameInfo(0);

            bool use_pts;
            bool debug_verbose = false;
            int debug_num_frames = 0; // set number of frames to be analyzed (nal units debug)
            bool logged_receiving = false;
    };

}