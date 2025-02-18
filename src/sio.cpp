
#include <fmt/core.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>

#include "../sioclient/sio_message.h"
#include "sio_socket.h"

#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "phntm_bridge/status_leds.hpp"

BridgeSocket::BridgeSocket(std::shared_ptr<BridgeConfig> config) {
    this->config = config;
    this->connected = false;
    this->shutting_down = false;
    ConnLED::FastPulse();

    uint reconnect_ms = this->config->sio_connection_retry_sec * 1000;
    std::cout << "SIO reconenct ms: " << reconnect_ms << std::endl;
    this->client.set_reconnect_delay(reconnect_ms);
    this->client.set_reconnect_delay_max(reconnect_ms);
    this->client.set_reconnect_attempts(10000000);
    this->client.set_open_listener(std::bind(&BridgeSocket::onConnected, this));
    this->client.set_disconnect_listener(std::bind(&BridgeSocket::onDisconnected, this));
    this->client.set_close_listener(std::bind(&BridgeSocket::onClosed, this, std::placeholders::_1));
    this->client.set_fail_listener(std::bind(&BridgeSocket::onFailed, this));
    this->client.set_reconnecting_listener(std::bind(&BridgeSocket::onReconnecting, this));
    this->client.set_reconnect_listener(std::bind(&BridgeSocket::onReconnect, this, std::placeholders::_1, std::placeholders::_2));

    this->client.set_socket_open_listener(std::bind(&BridgeSocket::onSocketOpen, this));
    this->client.set_socket_close_listener(std::bind(&BridgeSocket::onSocketClose, this));

    if (this->config->sio_verbose)
        this->client.set_logs_verbose();
}

bool BridgeSocket::connect() {
    // self.get_logger().info(f'Socket.io connecting to ')

    this->socket_url = fmt::format("{}:{}{}", this->config->cloud_bridge_address, this->config->sio_port, this->config->sio_path);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket.io connecting to %s", socket_url.c_str());

    std::cout << YELLOW << "Connecting id_robot: " << this->config->id_robot << CLR << std::endl;

    this->auth_data = sio::object_message::create();
    this->auth_data->get_map()["id_robot"] = sio::string_message::create(this->config->id_robot);
    this->auth_data->get_map()["key"] = sio::string_message::create(this->config->auth_key);
    this->auth_data->get_map()["name"] = sio::string_message::create(this->config->robot_name);
    this->auth_data->get_map()["maintainer_email"] = sio::string_message::create(this->config->maintainer_email);
    this->auth_data->get_map()["ros_distro"] = sio::string_message::create(this->config->ros_distro);
    this->auth_data->get_map()["git_sha"] = sio::string_message::create(this->config->git_head_sha);
    this->auth_data->get_map()["git_tag"] = sio::string_message::create(this->config->latest_git_tag);

    this->handled_events.emplace("ice-servers", std::bind(&BridgeSocket::onIceServers, this, std::placeholders::_1));
    this->handled_events.emplace("peer", std::bind(&BridgeSocket::onPeerConnected, this, std::placeholders::_1));
    this->handled_events.emplace("peer:disconnected", std::bind(&BridgeSocket::onPeerDisconnected, this, std::placeholders::_1));
    this->handled_events.emplace("introspection", std::bind(&BridgeSocket::onIntrospection, this, std::placeholders::_1));

    this->handled_events.emplace("subscribe", std::bind(&BridgeSocket::onSubscribeRead, this, std::placeholders::_1));
    this->handled_events.emplace("subscribe:write", std::bind(&BridgeSocket::onSubscribeWrite, this, std::placeholders::_1));
    this->handled_events.emplace("service", std::bind(&BridgeSocket::onServiceCall, this, std::placeholders::_1));

    this->client.connect(this->socket_url, this->auth_data);
    
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
    std::cout << "Socket.io client connection established" << std::endl;

    for (auto ev : this->handled_events) {
        this->client.socket()->on(ev.first, ev.second);
    }

    this->client.socket()->on_any(std::bind(&BridgeSocket::onOtherSocketMessage, this, std::placeholders::_1));
    this->client.socket()->on_error(std::bind(&BridgeSocket::onSocketError, this, std::placeholders::_1));
}

// auth done, socket ready
void BridgeSocket::onSocketOpen() {
    std::cout << GREEN << "Socket.io auth successful for #" << this->config->id_robot << CLR << std::endl;
    this->connected = true;
    ConnLED::On();
    this->introspection->report();
}

void BridgeSocket::onDisconnected() {
    std::cout << RED << "Socket.io client disconnected" << CLR << std::endl;
    this->connected = false;
    ConnLED::FastPulse();
}

void BridgeSocket::onIceServers(sio::event const& ev) {
    if (!this->config->use_cloud_ice_config) {
        std::cout << "Got ICE server config (ignoring)" << std::endl;
        return;
    }

    std::cout << "Got cloud ICE server config" << std::endl;
    // std::cout << PrintMessage(ev.get_message()) << std::endl;

    this->config->ice_servers.clear();
    // first add custom
    std::copy(this->config->ice_servers_custom.begin(), this->config->ice_servers_custom.end(), std::back_inserter(this->config->ice_servers));
    // add cloud config
    for (auto & one : ev.get_message()->get_map().at("servers")->get_vector()) {
        config->ice_servers.push_back(one->get_string());
    }
    // set received secret
    this->config->ice_secret = ev.get_message()->get_map().at("secret")->get_string();

    std::cout << CYAN << "Complete ICE config: " << CLR << std::endl;
    std::cout << "  username: " << this->config->ice_username << std::endl;
    std::cout << "  secret: " << this->config->ice_secret << std::endl;
    std::cout << "  servers: " << std::endl;
    for (auto & one : this->config->ice_servers) {
        std::cout << "    " << one << std::endl;
    }
}

void BridgeSocket::onPeerConnected(sio::event &ev) {
    std::cout << "Peer connecttion request: " << std::endl;
    std::cout << PrintMessage(ev.get_message()) << std::endl;
    auto id_peer = WRTCPeer::GetId(ev.get_message());
    auto ack = sio::object_message::create();
    if (id_peer.empty()) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("No valid peer id provided"));
    } else {
        WRTCPeer::OnPeerConnected(id_peer, ev.get_message(), this->config, ack);
    }
    ev.put_ack_message({ ack });
}

void BridgeSocket::onPeerDisconnected(sio::event const& ev) {
    std::cout << "Peer disconnect request: " << std::endl;
    std::cout << PrintMessage(ev.get_message()) << std::endl;
    auto id_peer = WRTCPeer::GetId(ev.get_message());
    if (!id_peer.empty()) {
        WRTCPeer::OnPeerDisconnected(id_peer);
    }
}

void BridgeSocket::onIntrospection(sio::event & ev) {
    auto id_peer = WRTCPeer::GetId(ev.get_message());
    auto ack = sio::object_message::create();
    if (id_peer.empty()) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("No valid peer id provided"));
    } else if (!WRTCPeer::IsConnected(id_peer)) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("Peer not connected"));
    } else {
        if (ev.get_message()->get_map()["state"]->get_bool()) {
            this->introspection->start();
        } else {
            this->introspection->stop();
        }
        ack->get_map().emplace("success", sio::int_message::create(1));
        ack->get_map().emplace("introspection", sio::bool_message::create(this->introspection->isRunning()));
    }
    ev.put_ack_message({ ack });
}

void BridgeSocket::onSubscribeRead(sio::event & ev) {

    std::cout << "Subscribe read request: " << std::endl;
    std::cout << PrintMessage(ev.get_message()) << std::endl;

    auto id_peer = WRTCPeer::GetId(ev.get_message());
    auto ack = sio::object_message::create();
    if (id_peer.empty()) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("No valid peer id provided"));
    } else if (!WRTCPeer::IsConnected(id_peer)) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("Peer not connected"));
    } else {
        
        // TODO

    }
    ev.put_ack_message({ ack });
}

void BridgeSocket::onSubscribeWrite(sio::event & ev) {

    std::cout << "Subscribe write request: " << std::endl;
    std::cout << PrintMessage(ev.get_message()) << std::endl;

    auto id_peer = WRTCPeer::GetId(ev.get_message());
    auto ack = sio::object_message::create();
    if (id_peer.empty()) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("No valid peer id provided"));
    } else if (!WRTCPeer::IsConnected(id_peer)) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("Peer not connected"));
    } else {
        
        // TODO

    }
    ev.put_ack_message({ ack });
}

void BridgeSocket::onServiceCall(sio::event & ev) {

    std::cout << "Service call: " << std::endl;
    std::cout << PrintMessage(ev.get_message()) << std::endl;

    auto id_peer = WRTCPeer::GetId(ev.get_message());
    auto ack = sio::object_message::create();
    if (id_peer.empty()) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("No valid peer id provided"));
    } else if (!WRTCPeer::IsConnected(id_peer)) {
        ack->get_map().emplace("err", sio::int_message::create(2));
        ack->get_map().emplace("msg", sio::string_message::create("Peer not connected"));
    } else {
        
        // TODO

    }
    ev.put_ack_message({ ack });
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
    
    std::cout << RED << "UNHANDLED SOCKER MSG '" << ev.get_name() << "': " << CLR << std::endl;
    std::cout << PrintMessage(ev.get_message()) << std::endl;
}

void BridgeSocket::onClosed(sio::client::close_reason const& reason) {
    std::string reason_hr;
    switch (reason) {
        case sio::client::close_reason_drop: reason_hr = "drop"; break;
        case sio::client::close_reason_normal: reason_hr = "normal"; break;
        default: reason_hr = std::to_string(reason); break;
    }
    
    std::cout << RED << "Socket.io client closed (reason " << reason_hr << ")" << CLR << std::endl;
    this->client.socket()->off_all();
    this->client.socket()->off_error();
    
    if (!this->shutting_down) {
        std::cout << "  Attempting to reconnect..." << std::endl;
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            this->client.connect(this->socket_url, this->auth_data);
        }).detach();
    }
}

void BridgeSocket::onFailed() {
    std::cout << RED << "Socket.io client failed" << CLR << std::endl;
}

void BridgeSocket::onReconnecting() {
    std::cout << "Socket.io client attempting to reconnect..." << std::endl;
}

void BridgeSocket::onReconnect(uint attemptCount, uint delay) {
    std::cout << "Socket.io client reconnected after " << attemptCount << " attempts. Next attempt in " << delay << "ms" << std::endl;
}

void BridgeSocket::onSocketClose() {
    std::cout << RED << "Socket.io socket closed" << CLR << std::endl;
}

void BridgeSocket::shutdown() {
    this->shutting_down = true;
    this->client.set_reconnect_attempts(0);
    this->client.clear_con_listeners();
    this->client.sync_close();
}

BridgeSocket::~BridgeSocket() {
    ConnLED::Off();
}

std::string BridgeSocket::PrintMessage(const sio::message::ptr & message, bool pretty, int indent) {
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
            out += std::to_string(message->get_int());
            break;
        case sio::message::flag::flag_double:
            out += std::to_string(message->get_double());
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
                    out += p.first + ": " + PrintMessage(p.second, pretty, indent+1) + ", ";
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
                    out += PrintMessage(one, pretty, indent+1) + ", ";
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

sio::message::ptr BridgeSocket::JsonToSioMessage(Json::Value val) {

    if (val.isBool()) {
        return sio::bool_message::create(val.asBool());
    } else if (val.isDouble()) {
        return sio::double_message::create(val.asDouble());
    } else if (val.isInt()) {
        return sio::int_message::create(val.asInt());
    } else if (val.isInt64()) {
        return sio::int_message::create(val.asInt64());
    } else if (val.isNumeric()) {
        return sio::double_message::create(val.asDouble());
    } else if (val.isNull()) {
        return sio::null_message::create();
    } else if (val.isString()) {
        return sio::string_message::create(val.asString());
    } else if (val.isArray()) {
        auto res = sio::array_message::create();
        for (auto & one : val) {
            res->get_vector().push_back(JsonToSioMessage(one));
        }
        return res;
    } else if (val.isObject()) {
        auto res = sio::object_message::create();
        for (auto it = val.begin(); it != val.end(); ++it) {
            const std::string& key = it.key().asString();
            res->get_map().emplace(key, JsonToSioMessage(*it));
        }
        return res;
    } else {
        std::cerr << RED << "Invalid Json value type: " << std::endl;
        std::cerr << val.toStyledString() << std::endl;
        return sio::null_message::create();
    }
}