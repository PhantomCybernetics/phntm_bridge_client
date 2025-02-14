
#include <fmt/core.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>

#include "../sioclient/sio_message.h"

#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/const.hpp"
#include "sio_socket.h"

std::string printMessage(const sio::message::ptr& message, bool pretty = true, int indent = 1) {
    std::string out;
    const int sp = 2;

    switch (message->get_flag()) {
        case sio::message::flag::flag_string:
            out += '"' + message->get_string() + '"';
            break;
        case sio::message::flag::flag_null:
            out += "null";
            break;
        case sio::message::flag::flag_binary:
            out += 'b';
            out += message->get_binary()->c_str();
            break;
        case sio::message::flag::flag_boolean:
            out += message->get_bool() ? "true" : "false";
            break;
        case sio::message::flag::flag_integer:
            out += message->get_int();
            break;
        case sio::message::flag::flag_double:
            out += message->get_double();
            break;
        case sio::message::flag::flag_object:
            {
                out += "{";
                if (pretty)
                        out += '\n';
                auto map = message->get_map();
                for (auto p : map) {
                    if (pretty)
                        out.append(sp*indent, ' ');
                    out += p.first + ": " + printMessage(p.second, pretty, indent+1) + ", ";
                    if (pretty)
                        out += '\n';
                }
                if (pretty)
                    out.append(sp*(indent-1), ' ');
                out += "}";
                break;
            }
        case sio::message::flag::flag_array:
            {
                out += "[";
                if (pretty)
                        out += '\n';
                auto map = message->get_vector();
                for (auto one : map) {
                    if (pretty)
                        out.append(sp*indent, ' ');
                    out += printMessage(one, pretty, indent+1) + ", ";
                    if (pretty)
                        out += '\n';
                }
                if (pretty)
                    out.append(sp*(indent-1), ' ');
                out += "]";
                break;
            }
        default:
            break;
    }
    return out;
}

BridgeSocket::BridgeSocket(std::shared_ptr<BridgeConfig> config) {
    this->config = config;
    this->connected = false;

    uint reconnect_ms = this->config->sio_connection_retry_sec * 1000;
    this->client.set_reconnect_delay(reconnect_ms);
    this->client.set_open_listener(std::bind(&BridgeSocket::onConnected, this));
    this->client.set_disconnect_listener(std::bind(&BridgeSocket::onDisconnected, this));
    this->client.set_close_listener(std::bind(&BridgeSocket::onClosed, this, std::placeholders::_1));
    this->client.set_fail_listener(std::bind(&BridgeSocket::onFailed, this));

    this->client.set_socket_open_listener(std::bind(&BridgeSocket::onSocketOpen, this));
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

    this->handled_events.emplace("ice-servers", std::bind(&BridgeSocket::onIceServers, this, std::placeholders::_1));
    this->handled_events.emplace("peer", std::bind(&BridgeSocket::onPeerConnected, this, std::placeholders::_1));
    this->handled_events.emplace("peer:disconnected", std::bind(&BridgeSocket::onPeerDisconnected, this, std::placeholders::_1));
    this->handled_events.emplace("introspection", std::bind(&BridgeSocket::onIntrospection, this, std::placeholders::_1));

    this->client.connect(socket_url, auth_data);
    for (auto ev : this->handled_events) {
        this->client.socket()->on(ev.first, ev.second);
    }

    this->client.socket()->on_any(std::bind(&BridgeSocket::onOtherSocketMessage, this, std::placeholders::_1));
    this->client.socket()->on_error(std::bind(&BridgeSocket::onSocketError, this, std::placeholders::_1));

    return true;
}

void BridgeSocket::emit(std::string const& name, sio::message::list const& msglist, std::function<void (sio::message::list const&)> const& ack) {
    if (!this->connected) {
        std::cout << "Socket.io not connected, ignoring \"" + name << "\"" << std::endl;
        return;
    }
    this->client.socket()->emit(name, msglist, ack);
}

// socket connected (before handshale)
void BridgeSocket::onConnected() {
    std::cout << CYAN << "Socket.io connection established" << CLR << std::endl;
}

// auth done, socket ready
void BridgeSocket::onSocketOpen() {
    std::cout << GREEN << "Socket.io auth successful for #" << this->config->id_robot << CLR << std::endl;
    this->connected = true;

    this->introspection->report();
}

void BridgeSocket::onDisconnected() {
    std::cout << RED << "DISCONNECTED" << CLR << std::endl;
    this->connected = false;
}

void BridgeSocket::onIceServers(sio::event const& ev) {
    std::cout << CYAN << "ice-servers: " << CLR << std::endl;
    std::cout << printMessage(ev.get_message()) << std::endl;
}

void BridgeSocket::onPeerConnected(sio::event const& ev) {
    std::cout << GREEN << "peer connected: " << CLR << std::endl;
    std::cout << printMessage(ev.get_message()) << std::endl;
}

void BridgeSocket::onPeerDisconnected(sio::event const& ev) {
    std::cout << BLUE << "peer disconnected: " << CLR << std::endl;
    std::cout << printMessage(ev.get_message()) << std::endl;
}

void BridgeSocket::onIntrospection(sio::event const& ev) {
    if (ev.get_message()->get_map()["state"]->get_bool()) {
        this->introspection->start();
    } else {
        this->introspection->stop();
    }
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
                auto map = message->get_map();
                for (auto one : map) {
                    std::cout << RED << "SOCKET ERROR: "  << (one.first == "message" ? "" : one.first + ": ") << one.second->get_string() << CLR << std::endl;
                }
                break;
            }
        default:
            std::cout << RED << "SOCKET ERROR: (type=" << message->get_flag() << ")" << CLR << std::endl;
            break;
    }
}

void BridgeSocket::onOtherSocketMessage(sio::event const& ev) {
    if (this->handled_events.find(ev.get_name().c_str()) != this->handled_events.end())
        return;
    
    std::cout << MAGENTA << "UNHANDLED SOCKER MSG " << ev.get_name() << ": " << CLR << std::endl;
    std::cout << printMessage(ev.get_message()) << std::endl;
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