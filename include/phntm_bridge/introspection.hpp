#pragma once

#include "const.hpp"
#include "config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "phntm_interfaces/msg/docker_status.hpp"
#include "phntm_interfaces/srv/file_request.hpp"
#include <rclcpp/qos.hpp>
#include <rclcpp/timer.hpp>

namespace phntm {

    class BridgeSocket;

    class Introspection {

        public:
            static void init(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<BridgeConfig> config);
            static void start();
            static void stop();
            static void report();
            static bool isRunning() { return Introspection::instance != nullptr && Introspection::instance->running; };
            static std::string getService(std::string service); //srv type empty
            static std::string getTopic(std::string topic); //msg type or empty 
            static std::map<std::string, rclcpp::Client<phntm_interfaces::srv::FileRequest>::SharedPtr> getFileExtractors();
            
        private:
            Introspection(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<BridgeConfig> config);
            ~Introspection();
            static Introspection * instance;

            bool running;

            std::shared_ptr<rclcpp::Node> node;
            std::shared_ptr<BridgeConfig> config;

            std::map<std::string, phntm_interfaces::msg::DockerStatus>discovered_docker_containers;

            void reportDocker();
            void reportIDLs();
            void reportNodes();
            void reportRunningState();

            void onDockerMonitorMessage(phntm_interfaces::msg::DockerStatus const & msg);
            std::shared_ptr<rclcpp::Subscription<phntm_interfaces::msg::DockerStatus>> docker_sub;

            void runIntrospection();
            rclcpp::TimerBase::SharedPtr timer;
            bool introspection_in_progress;
            rclcpp::Time start_time;

            struct NodePubTopic  {
                std::string msg_type;
                std::vector<rclcpp::QoS> qos; // there can be multiple pubs to the same topic on one node
            };

            struct SubQoS {
                rclcpp::QoS qos;
                std::string error; // errors from QoS compatibility check
                std::string warning;
            };

            struct NodeSubTopic  {
                std::string msg_type;
                std::vector<SubQoS> qos; // there can be multiple subs to the same topic on one node
            };

            struct DiscoveredNode {
                std::string ns; //namespace
                std::map<std::string, NodePubTopic> publishers; // topic => NodePubTopic
                std::map<std::string, NodeSubTopic> subscribers; // topic => NodeSubTopic
                std::map<std::string, std::string> services; // service => msg_type

                std::map<std::string, NodePubTopic> tmp_publishers; // topic => NodePubTopic; 1st pass
                std::map<std::string, NodeSubTopic> tmp_subscribers; // topic => NodeSubTopic; 1st pass
                bool needs_subscribers_qos_check;
            };

            std::map<std::string, DiscoveredNode> discovered_nodes; // node id => node
            std::map<std::string, std::string> discovered_topics; // topic => msg_type
            std::map<std::string, rclcpp::Client<phntm_interfaces::srv::FileRequest>::SharedPtr> discovered_file_extractors; // node id => service client

            bool collectIDLs(std::string msg_type);
            std::map<std::string, std::string> discovered_idls; // type => def
            
            void checkSubscriberQos(std::string id_subscriber, DiscoveredNode *subscriber_node);
    };

}