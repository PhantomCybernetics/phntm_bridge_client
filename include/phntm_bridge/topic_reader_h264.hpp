#pragma once
#include <cstdint>
#include <map>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/compressed_image__struct.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <memory>
#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "rtc/frameinfo.hpp"
#include "rtc/peerconnection.hpp"
#include "rtc/rtc.hpp"

#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sio_message.h"
#include <condition_variable>

#include "phntm_bridge/ffmpeg_encoder.hpp"

namespace phntm {

    class PhntmBridge;
    class WRTCPeer;

    class TopicReaderH264 {

        public:
            static std::shared_ptr<TopicReaderH264> getForTopic(std::string topic, std::string msg_type, rclcpp::QoS qos, std::shared_ptr<rclcpp::Node> node, rclcpp::CallbackGroup::SharedPtr callback_group, sio::message::ptr topic_conf);
            static std::shared_ptr<TopicReaderH264> getForTopic(std::string topic); // does not create a new one
            static void destroy(std::string topic);

            struct MediaTrackInfo {
                std::shared_ptr<rtc::Track> track;
                std::string msid;
                uint ssrc;
                bool in_use = true; // false when paused
                bool init_complete = false; // waits for 1st stable signaling state after track open
                std::shared_ptr<rtc::RtpPacketizationConfig> rtpConfig;
                std::shared_ptr<rtc::H264RtpPacketizer> packetizer;
            };
            bool addOutput(std::shared_ptr<MediaTrackInfo> track_info, std::shared_ptr<WRTCPeer> peer);
            bool removeOutput(std::shared_ptr<MediaTrackInfo> track_info);
            uint media_reader_node_no = 0;

            // static void onDCOpen(std::shared_ptr<rtc::DataChannel> dc, std::shared_ptr<rtc::PeerConnection> pc);
            static void onPCSignalingStateChange(std::shared_ptr<WRTCPeer> peer);
        
            static std::string openMediaTrackForTopic(std::string topic, std::shared_ptr<WRTCPeer> peer);
            static void closeMediaTrackForTopic(std::string topic, std::shared_ptr<WRTCPeer> peer);

            TopicReaderH264(std::string topic, std::string msg_type, rclcpp::QoS qos, std::shared_ptr<rclcpp::Node> node, rclcpp::CallbackGroup::SharedPtr callback_group, sio::message::ptr topic_conf);
            ~TopicReaderH264();

            struct OutputMsg {
                std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg;
                uint64_t ts;
                bool is_keyframe = false;
            };

            struct SendMsg {
                std::byte* data;
                std::size_t size;
                rtc::FrameInfo info;

                SendMsg(std::byte* d, std::size_t s, rtc::FrameInfo i)
                    : data(d), size(s), info(i) {};
            };

            struct Output {
                std::shared_ptr<MediaTrackInfo> track_info;
                std::shared_ptr<WRTCPeer> peer;
                
                uint16_t num_sent = 0;
                bool active = true;
                // bool initial_ts_set = false;
                uint64_t ts_first = 0; // 1st pts from ros
                uint64_t ts_offset = 0; // trp time on 1st frame (1/90000)
                uint64_t last_raw_ts = 0; // last received pts form ros
                bool start_ts_set = false;

                std::optional<std::vector<uint8_t>> last_nal_unit_7 = std::nullopt;
                std::optional<std::vector<uint8_t>> last_nal_unit_8 = std::nullopt;
                std::optional<std::vector<uint8_t>> last_nal_unit_5 = std::nullopt;

                bool logged_closed = false;
                bool logged_init_incomplete = false;
                bool logged_error = false;
                bool logged_exception = false;

                std::thread thread;
                std::queue<OutputMsg> in_queue;
                std::mutex in_queue_mutex;
                std::condition_variable in_queue_cv;
            };
        
        private:
            static std::map<std::string, std::shared_ptr<TopicReaderH264>> readers;
            static std::mutex readers_mutex;
            std::string topic;
            std::string msg_type;
            rclcpp::QoS qos;
            std::shared_ptr<rclcpp::Node> node; // node to run subs on, new if null
            rclcpp::CallbackGroup::SharedPtr callback_group;

            std::vector<std::shared_ptr<Output>> outputs; // target data channels & pcs
            std::mutex outputs_mutex;
            std::mutex start_stop_mutex;
            std::mutex subscriber_mutex;
            
            bool subscriber_running = false;
            void spinSubscriber();
            std::shared_ptr<rclcpp::Executor> executor;
            std::thread subscriber_thread;

            void start();
            void stop();

            void onEncodedFrame(const std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> data);
            void onImageFrame(const std::shared_ptr<sensor_msgs::msg::Image> data);
            void onCompressedFrame(const std::shared_ptr<sensor_msgs::msg::CompressedImage> data);
            
            // only one of these used at the time
            std::shared_ptr<rclcpp::Subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>> sub_enc;
            std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> sub_img;
            std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>> sub_cmp;
            
            bool create_node;
            
            //std::queue<std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>> frame_queue;
            //std::mutex frame_queue_mutex;
            //std::condition_variable frame_queue_cv;
            // void processFramesWorker();
            // void processFrame(std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg);
            
            void outputFramesWorker(std::shared_ptr<Output> output);

            std::vector<std::byte> latest_payload;
            //size_t latest_payload_size = 0;
            // rtc::FrameInfo latest_frame_info = rtc::FrameInfo(0);

            bool use_pts;
            bool debug_verbose = false;
            int debug_num_frames = 0; // set number of frames to be analyzed (nal units debug)
            bool logged_receiving = false;
            bool logged_error = false;
            
            int colormap; // used to colorize mono images
            double max_sensor_value; // used to normalize raw sensor data

            bool encoder_error = false;
            std::string encoder_hw_device;
            int encoder_thread_count;
            int encoder_gop_size;
            int encoder_bit_rate;

            std::shared_ptr<FFmpegEncoder> encoder;
            //void onH264Encoded();
    };

}