#pragma once

#include "const.hpp"
#include "config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "phntm_interfaces/msg/docker_status.hpp"
#include <rclcpp/qos.hpp>
#include <rclcpp/timer.hpp>

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
            rclcpp::QoS qos;
        };

        struct NodeSubTopic  {
            std::string msg_type;
            rclcpp::QoS qos;
            std::string qos_error;
            std::string qos_warning;
        };

        struct DiscoveredNode {
            std::string ns; //namespace
            std::map<std::string, NodePubTopic> publishers; // topic => NodeTopic
            std::map<std::string, NodeSubTopic> subscribers; // topic => NodeTopic
            std::map<std::string, std::string> services; // service => msg_type
        };

        std::map<std::string, DiscoveredNode> discovered_nodes;
        std::map<std::string, std::string> discovered_topics; // topic => msg_type

        bool collectIDLs(std::string msg_type);
        std::map<std::string, std::string> discovered_idls; // type => def
        
        void checkSubscriberQos();
};