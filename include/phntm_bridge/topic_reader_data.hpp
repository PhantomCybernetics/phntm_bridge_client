#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "rtc/peerconnection.hpp"
#include "rtc/rtc.hpp"

namespace phntm {

    class PhntmBridge;

    class TopicReaderData {

        public:
            static std::shared_ptr<TopicReaderData> getForTopic(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos);
            static std::shared_ptr<TopicReaderData> getForTopic(std::string topic); // does not create a new one
            static void destroy(std::string topic);

            bool addOutput(std::shared_ptr<rtc::DataChannel> dc, std::shared_ptr<rtc::PeerConnection> pc);
            bool removeOutput(std::shared_ptr<rtc::DataChannel> dc);
            // static void onDCOpen(std::shared_ptr<rtc::DataChannel> dc, std::shared_ptr<rtc::PeerConnection> pc);
            static void onPCSignalingStateChange(std::shared_ptr<rtc::PeerConnection> pc);
        
            TopicReaderData(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos);
            ~TopicReaderData();

            struct Output {
                std::shared_ptr<rtc::DataChannel> dc;
                std::shared_ptr<rtc::PeerConnection> pc;
                
                uint16_t num_sent = 0;
                bool init_complete = false; // waits for 1st stable signaling state after dc open
                bool active = true;

                bool logged_closed = false;
                bool logged_init_incomplete = false;
                bool logged_error = false;
                bool logged_exception = false;
            };

        private:
            static std::map<std::string, std::shared_ptr<TopicReaderData>> readers;
            // static std::shared_ptr<TopicReaderData> getForDC(std::shared_ptr<rtc::DataChannel> dc); // does not create a new one

            std::vector<std::shared_ptr<Output>> outputs; // target data channels & pcs
            std::mutex outputs_mutex;
            static std::mutex readers_mutex;
            bool is_reliable; // sends latest message on new dc add
            std::string topic, msg_type;
            std::shared_ptr<PhntmBridge> bridge_node;
            rclcpp::QoS qos;

            void start();
            void stop();
            void onData(std::shared_ptr<rclcpp::SerializedMessage> data);
            void sendLatestData(std::shared_ptr<Output> output);
            std::shared_ptr<rclcpp::GenericSubscription> sub;

            std::vector<std::byte> latest_payload;
            size_t latest_payload_size = 0;

            bool logged_receiving = false;
    };

}