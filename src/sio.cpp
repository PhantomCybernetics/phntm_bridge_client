
#include <fmt/core.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <new>

#include "../sioclient/sio_message.h"

#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/const.hpp"
#include "sio_socket.h"

BridgeSocket::BridgeSocket(std::shared_ptr<BridgeConfig> config) {
    this->config = config;

    uint reconnect_ms = this->config->sio_connection_retry_sec * 1000;
    this->client.set_reconnect_delay(reconnect_ms);
    this->client.set_open_listener(std::bind(&BridgeSocket::onConnected, this));
    this->client.set_disconnect_listener(std::bind(&BridgeSocket::onDisconnected, this));
    this->client.set_close_listener(std::bind(&BridgeSocket::onClosed, this, std::placeholders::_1));
    this->client.set_fail_listener(std::bind(&BridgeSocket::onFailed, this));
    this->client.set_socket_close_listener(std::bind(&BridgeSocket::onSocketClose, this));
    if (this->config->sio_verbose)
        this->client.set_logs_verbose();
}

bool BridgeSocket::connect() {
    // self.get_logger().info(f'Socket.io connecting to ')

    std::string socket_url = fmt::format("{}:{}{}", this->config->cloud_bridge_address, this->config->sio_port, this->config->sio_path);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket.io connecting to %s", socket_url.c_str());

    std::cout << YELLOW << "Connecting id_robot: " << this->config->id_robot << CLR << std::endl;

    auto auth_data = sio::object_message::create();
    auth_data->get_map()["id_robot"] = sio::string_message::create(this->config->id_robot);
    auth_data->get_map()["key"] = sio::string_message::create(this->config->auth_key);
    auth_data->get_map()["name"] = sio::string_message::create(this->config->robot_name);
    auth_data->get_map()["maintainer_email"] = sio::string_message::create(this->config->maintainer_email);
    auth_data->get_map()["ros_distro"] = sio::string_message::create(this->config->ros_distro);
    auth_data->get_map()["git_sha"] = sio::string_message::create(this->config->git_head_sha);
    auth_data->get_map()["git_tag"] = sio::string_message::create(this->config->latest_git_tag);

    this->client.connect(socket_url, auth_data);
    // this->socket = this->client.socket();
    this->client.socket()->on_error(std::bind(&BridgeSocket::onSocketError, this, std::placeholders::_1));
    // this->client.on_disconnect();
    // this->client.socket()->
    this->client.socket()->on_any(std::bind(&BridgeSocket::onSocketMessage, this, std::placeholders::_1));
    return true;
}

void BridgeSocket::onConnected() {
    std::cout << GREEN << "CONNECTED" << CLR << std::endl;
}

void BridgeSocket::onDisconnected() {
    std::cout << RED << "DISCONNECTED" << CLR << std::endl;
}

void BridgeSocket::onSocketError(sio::message::ptr const& message) {
    switch (message->get_flag()) {
        case sio::message::flag::flag_string:
            std::cout << RED << "SOCKET ERROR: " << message->get_string() << CLR << std::endl;
            break;
        case sio::message::flag::flag_null:
            std::cout << RED << "SOCKET ERROR: NULL" << CLR << std::endl;
            break;
        case sio::message::flag::flag_boolean:
            std::cout << RED << "SOCKET ERROR: " << message->get_bool() << CLR << std::endl;
            break;
        case sio::message::flag::flag_integer:
            std::cout << RED << "SOCKET ERROR: " << message->get_int() << CLR << std::endl;
            break;
        case sio::message::flag::flag_double:
            std::cout << RED << "SOCKET ERROR: " << message->get_double() << CLR << std::endl;
            break;
        case sio::message::flag::flag_object:
            {
                std::cout << RED << "SOCKET ERROR: " << CLR << std::endl;
                auto map = message->get_map();
                for (auto one : map) {
                    std::cout << "\t" << RED << one.first << ": " << one.second->get_string() << CLR << std::endl;
                }
                break;
            }
        default:
            std::cout << RED << "SOCKET ERROR: (type=" << message->get_flag() << ")" << CLR << std::endl;
            break;
    }
}

void BridgeSocket::onSocketMessage(sio::event const& message) {
    std::cout << MAGENTA << "MSG: " << message.get_name() << CLR << std::endl;
}

void BridgeSocket::onClosed(sio::client::close_reason const& reason) {
    std::cout << RED << "CLOSED (reason " << reason << ")" << CLR << std::endl;
}

void BridgeSocket::onFailed() {
    std::cout << RED << "FAILED" << CLR << std::endl;
}

void BridgeSocket::onSocketClose() {
    std::cout << RED << "SOCKET CLOSED" << CLR << std::endl;
}

void BridgeSocket::disconnect() {
    this->client.close();
}

BridgeSocket::~BridgeSocket() {
    
}