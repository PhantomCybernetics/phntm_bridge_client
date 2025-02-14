#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/sio.hpp"
#include "sio_message.h"
#include <ostream>
#include <iostream>
#include <phntm_interfaces/msg/detail/docker_status__struct.hpp>
#include <rclcpp/duration.hpp>

Introspection::Introspection(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<BridgeSocket> sio, std::shared_ptr<BridgeConfig> config) {
    this->node = node;
    this->sio = sio;
    this->config = config;
    this->running = false;

    // auto my_callback_group = rclcpp::create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    if (!this->config->docker_monitor_topic.empty()) {
        auto qos = rclcpp::QoS(1);
        qos.best_effort();
        qos.durability_volatile();
        qos.lifespan(rclcpp::Duration::max());
        // rclcpp::SubscriptionOptions options;
        // options.callback_group = my_callback_group;
        std::cout << "Subscribing to " << this->config->docker_monitor_topic << std::endl;
        this->docker_sub = this->node->create_subscription<phntm_interfaces::msg::DockerStatus>(this->config->docker_monitor_topic, qos,
            std::bind(&Introspection::onDockerMonitorMessage, this, std::placeholders::_1));
    }
}

void Introspection::start() {
    if (this->running)
        return;

    this->running = true;
    std::cout << CYAN << "Introspection starting..." << CLR << std::endl;
    this->reportRunningState();
}

void Introspection::stop() {
    if (!this->running)
        return;

    this->running = false;
    std::cout << CYAN << "Introspection stopped." << CLR << std::endl;
    this->reportRunningState();
}

void Introspection::onDockerMonitorMessage(phntm_interfaces::msg::DockerStatus const & msg) {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "onDockerMonitorMessage");
    std::cout << "onDockerMonitorMessage " << msg.header.frame_id << " " << msg.header.stamp.sec << ":" << msg.header.stamp.nanosec << std::endl;
}

void Introspection::report() {
    this->reportIdsl();
    this->reportNodes();
    this->reportTopics();
    this->reportServices();
    this->reportDocker();
    this->reportRunningState();
}

void Introspection::reportIdsl() {
    // TODO
}

void Introspection::reportNodes() {
    // TODO
}

void Introspection::reportTopics() {
    // TODO
}

void Introspection::reportServices() {
    // TODO
}

void Introspection::reportDocker() {
    // TODO
}

void Introspection::reportRunningState() {
    auto msg = sio::bool_message::create(this->running);
    this->sio->emit("introspection", { msg }, nullptr);
}

Introspection::~Introspection() {
    
}
