#pragma once

#include "const.hpp"
#include "config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "phntm_interfaces/msg/docker_status.hpp"

class BridgeSocket;

class Introspection {

    public:
        Introspection(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<BridgeSocket> sio, std::shared_ptr<BridgeConfig> config);
        void start();
        void stop();
        void report();
        ~Introspection();

    private:
        bool running;
        std::shared_ptr<BridgeSocket> sio;
        std::shared_ptr<rclcpp::Node> node;
        std::shared_ptr<BridgeConfig> config;

        void reportDocker();
        void reportServices();
        void reportTopics();
        void reportIdsl();
        void reportNodes();
        void reportRunningState();

        void onDockerMonitorMessage(phntm_interfaces::msg::DockerStatus const & msg);
        std::shared_ptr<rclcpp::Subscription<phntm_interfaces::msg::DockerStatus>> docker_sub;
};