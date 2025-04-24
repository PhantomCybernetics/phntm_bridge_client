#pragma once
#include <cstdint>
#include <map>
#include <rclcpp/generic_publisher.hpp>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "rtc/peerconnection.hpp"
#include "rtc/rtc.hpp"

class PhntmBridge;
class WRTCPeer;

class TopicWriterData : public std::enable_shared_from_this<TopicWriterData> {

    public:
        static std::shared_ptr<TopicWriterData> getForTopic(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos);
        static std::shared_ptr<TopicWriterData> getForTopic(std::string topic); // does not create a new one

        bool addInput(std::shared_ptr<rtc::DataChannel> dc);
        bool removeInput(std::shared_ptr<rtc::DataChannel> dc);
        // static void onDCOpen(std::shared_ptr<rtc::DataChannel> dc, std::shared_ptr<rtc::PeerConnection> pc);
        // static void onPCSignalingStateChange(std::shared_ptr<rtc::PeerConnection> pc);
    
        TopicWriterData(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos);
        ~TopicWriterData();

        struct Input {
            std::shared_ptr<rtc::DataChannel> dc;
            // std::shared_ptr<rtc::PeerConnection> pc;
            uint16_t num_received = 0;
            // bool init_complete = false;
            bool active = true;
            
            //bool logged_closed = false;
            //bool logged_init_incomplete = false;
            // bool logged_error = false;
            // bool logged_exception = false;
            bool logged_receiving = false;
        };

        static bool onData(std::shared_ptr<WRTCPeer> peer, std::shared_ptr<rtc::DataChannel> dc, std::string topic, std::variant<rtc::binary, rtc::string> message);

    private:
        static std::map<std::string, std::shared_ptr<TopicWriterData>> writers;
        static std::map<rtc::DataChannel *, std::shared_ptr<TopicWriterData>> dc_to_writer_map; // for fast search on data
        static std::shared_ptr<TopicWriterData> getForDC(std::shared_ptr<rtc::DataChannel> dc); // does not create a new one

        std::vector<std::shared_ptr<Input>> inputs; // target data channels & pcs
        std::mutex inputs_mutex;
        
        std::string topic, msg_type;
        std::shared_ptr<PhntmBridge> bridge_node;
        rclcpp::QoS qos;

        void start();
        void stop();

        //void sendLatestData(std::shared_ptr<Output> output);
        std::shared_ptr<rclcpp::GenericPublisher> pub;

        //std::vector<std::byte> latest_payload;
        //size_t latest_payload_size = 0;
};