
#include <fmt/core.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>

#include "sio_message.h"

#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "phntm_bridge/status_leds.hpp"

BridgeSocket* BridgeSocket::instance = nullptr;

BridgeSocket::BridgeSocket(std::shared_ptr<PhntmBridge> node, std::shared_ptr<BridgeConfig> config) {
    this->config = config;
    this->node = node;
    this->connected = false;
    this->shutting_down = false;

    ConnLED::fastPulse();

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

    if (this->config->sio_debug)
        this->client.set_logs_verbose();
}

void BridgeSocket::init(std::shared_ptr<PhntmBridge> node, std::shared_ptr<BridgeConfig> config) {
    if (BridgeSocket::instance != nullptr) {
        return;
    }
    BridgeSocket::instance = new BridgeSocket(node, config);
}

bool BridgeSocket::connect() {
    // self.get_logger().info(f'Socket.io connecting to ')
    auto instance = BridgeSocket::instance;
    if (instance == nullptr) {
        std::cerr << "BridgeSocket not initialized" << std::endl;
        return false;
    }

    instance->socket_url = fmt::format("{}:{}{}", instance->config->cloud_bridge_address, instance->config->sio_port, instance->config->sio_path);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Socket.io connecting to %s", instance->socket_url.c_str());

    std::cout << YELLOW << "Connecting id_robot: " << instance->config->id_robot << CLR << std::endl;

    instance->auth_data = sio::object_message::create();
    instance->auth_data->get_map()["id_robot"] = sio::string_message::create(instance->config->id_robot);
    instance->auth_data->get_map()["key"] = sio::string_message::create(instance->config->auth_key);
    instance->auth_data->get_map()["name"] = sio::string_message::create(instance->config->robot_name);
    instance->auth_data->get_map()["maintainer_email"] = sio::string_message::create(instance->config->maintainer_email);
    instance->auth_data->get_map()["ros_distro"] = sio::string_message::create(instance->config->ros_distro);
    instance->auth_data->get_map()["git_sha"] = sio::string_message::create(instance->config->git_head_sha);
    instance->auth_data->get_map()["git_tag"] = sio::string_message::create(instance->config->latest_git_tag);

    instance->handled_events.emplace("ice-servers", std::bind(&BridgeSocket::onIceServers, instance, std::placeholders::_1));
    instance->handled_events.emplace("peer", std::bind(&BridgeSocket::onPeerConnected, instance, std::placeholders::_1));
    instance->handled_events.emplace("peer:disconnected", std::bind(&BridgeSocket::onPeerDisconnected, instance, std::placeholders::_1));
    instance->handled_events.emplace("introspection", std::bind(&BridgeSocket::onIntrospection, instance, std::placeholders::_1));

    instance->handled_events.emplace("subscribe", std::bind(&BridgeSocket::onSubscribeRead, instance, std::placeholders::_1));
    instance->handled_events.emplace("unsubscribe", std::bind(&BridgeSocket::onUnsubscribeRead, instance, std::placeholders::_1));
    instance->handled_events.emplace("subscribe:write", std::bind(&BridgeSocket::onSubscribeWrite, instance, std::placeholders::_1));
    instance->handled_events.emplace("unsubscribe:write", std::bind(&BridgeSocket::onUnsubscribeWrite, instance, std::placeholders::_1));
    instance->handled_events.emplace("service", std::bind(&BridgeSocket::onServiceCall, instance, std::placeholders::_1));
    instance->handled_events.emplace("sdp:answer", std::bind(&BridgeSocket::onSDPAnswer, instance, std::placeholders::_1));

    instance->client.connect(instance->socket_url, instance->auth_data);
    
    return true;
}

void BridgeSocket::emit(std::string const& name, sio::message::list const& msglist, std::function<void (sio::message::list const&)> const& ack) {
    auto instance = BridgeSocket::instance;
    if (instance == nullptr || !instance->connected) {
        std::cout << "Socket.io not connected, ignoring \"" + name << "\"" << std::endl;
        return;
    }
    if (instance->config->sio_verbose) {
        std::cout << "SIO emitting '" << name << "':" << std::endl
                  << printMessage(msglist.at(0)) << std::endl;
    }
    instance->client.socket()->emit(name, msglist, ack);
}

void BridgeSocket::ack(int msg_id, sio::message::list const& msglist) {
    auto instance = BridgeSocket::instance;
    if (instance == nullptr || !instance->connected) {
        std::cout << "Socket.io not connected, ignoring ack(" << msg_id << ")" << std::endl;
        return;
    }
    if (instance->config->sio_verbose) {
        std::cout << "SIO sending ack for msg_id=" << std::to_string(msg_id) << ":" << std::endl
                  << printMessage(msglist.at(0)) << std::endl;
    }
    instance->client.socket()->ack(msg_id, msglist);
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
    ConnLED::on();
    Introspection::report();
}

void BridgeSocket::onDisconnected() {
    std::cout << RED << "Socket.io client disconnected" << CLR << std::endl;
    this->connected = false;
    ConnLED::fastPulse();
    WRTCPeer::OnAllPeersDisconnected();
}

void BridgeSocket::onIceServers(sio::event const& ev) {
    if (!this->config->use_cloud_ice_config) {
        std::cout << "Got ICE server config (ignoring)" << std::endl;
        return;
    }

    if (this->config->sio_verbose) {
        std::cout << "Got cloud ICE server config" << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    }

    this->config->ice_servers.clear();
    // first add custom
    std::copy(this->config->ice_servers_custom.begin(), this->config->ice_servers_custom.end(), std::back_inserter(this->config->ice_servers));
    // add cloud config
    for (auto & one : ev.get_message()->get_map().at("servers")->get_vector()) {
        config->ice_servers.push_back(one->get_string());
    }
    // set received secret
    this->config->ice_secret = ev.get_message()->get_map().at("secret")->get_string();

    if (this->config->webrtc_debug) {
        std::cout << CYAN << "Complete ICE config: " << CLR << std::endl;
        std::cout << "  username: " << this->config->ice_username << std::endl;
        std::cout << "  secret: " << this->config->ice_secret << std::endl;
        std::cout << "  servers: " << std::endl;
        for (auto & one : this->config->ice_servers) {
            std::cout << "    " << one << std::endl;
        }
    }
}


// peer connected
void BridgeSocket::onPeerConnected(sio::event &ev) {
    if (this->config->sio_verbose) {
        std::cout << "Peer connection request: " << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    }
    
    auto id_peer = WRTCPeer::GetId(ev.get_message());
    
    if (id_peer.empty()) {
        auto err_ack = sio::object_message::create();
        err_ack->get_map().emplace("err", sio::int_message::create(2));
        err_ack->get_map().emplace("msg", sio::string_message::create("No valid peer id provided"));
        this->client.socket()->ack(ev.get_msgId(), { err_ack });
        return;
    } 

    WRTCPeer::OnPeerConnected(id_peer, ev, this->config);        
}

// peer disconnected
void BridgeSocket::onPeerDisconnected(sio::event & ev) {
    if (this->config->sio_verbose) {
        std::cout << "Peer disconnect request: " << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    }
    auto peer = WRTCPeer::GetConnectedPeer(ev);
    if (peer == nullptr) return;
    peer->onDisconnected();
}

// introspection control
void BridgeSocket::onIntrospection(sio::event & ev) {
    auto peer = WRTCPeer::GetConnectedPeer(ev);
    if (peer == nullptr) return;
        
    if (ev.get_message()->get_map()["state"]->get_bool()) {
        Introspection::start();
    } else {
        Introspection::stop();
    }

    auto ack = sio::object_message::create();
    ack->get_map().emplace("success", sio::int_message::create(1));
    ack->get_map().emplace("introspection", sio::bool_message::create(Introspection::isRunning()));
    this->client.socket()->ack(ev.get_msgId(), { ack });
}

// peer subscribing to stuffs
void BridgeSocket::onSubscribeRead(sio::event & ev) {
    if (this->config->sio_verbose) {
        std::cout << "Subscribe read request: " << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    }
    
    auto peer = WRTCPeer::GetConnectedPeer(ev);
    if (peer == nullptr) return;
    
    auto msg = ev.get_message();
    if (msg->get_map().find("sources") != msg->get_map().end()) {
        auto arr = msg->get_map().at("sources");
        if (arr->get_flag() == sio::message::flag_array) {
            for (auto t : arr->get_vector()){
                peer->addReqReadSubscription(t->get_string());
            }
        } 
    }

    if (ev.need_ack())
        peer->processSubscriptions(ev.get_msgId(), nullptr); // will emit ack
    else
        peer->processSubscriptions(-1, nullptr); // will emit new msg
}

// peer unsubscribing from stuffs
void BridgeSocket::onUnsubscribeRead(sio::event & ev) {

    if (this->config->sio_verbose) {
        std::cout << "Unsubscribe read request: " << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    }

    auto peer = WRTCPeer::GetConnectedPeer(ev);
    if (peer == nullptr) return;
    
    auto msg = ev.get_message();
    if (msg->get_map().find("sources") != msg->get_map().end()) {
        auto arr = msg->get_map().at("sources");
        if (arr->get_flag() == sio::message::flag_array) {
            for (auto t : arr->get_vector()){
                peer->removeReqReadSubscription(t->get_string());
            }
        } 
    }

    if (ev.need_ack())
        peer->processSubscriptions(ev.get_msgId(), nullptr); // will emit ack
    else
        peer->processSubscriptions(-1, nullptr); // will emit new msg
}

// peer requesting write subscriptions
void BridgeSocket::onSubscribeWrite(sio::event & ev) {

    if (this->config->sio_verbose) {
        std::cout << "Subscribe write request: " << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    }

    auto peer = WRTCPeer::GetConnectedPeer(ev);
    if (peer == nullptr) return;
        
    auto msg = ev.get_message();
    if (msg->get_map().find("sources") != msg->get_map().end()) {
        auto arr = msg->get_map().at("sources");
        if (arr->get_flag() == sio::message::flag_array) {
            for (auto one : arr->get_vector()){
                if (one->get_flag() == sio::message::flag_array) {
                    peer->addReqWriteSubscription(one->get_vector()[0]->get_string(), one->get_vector()[1]->get_string());
                }
            }
        } 
    }

    if (ev.need_ack())
        peer->processSubscriptions(ev.get_msgId(), nullptr); // will emit ack
    else
        peer->processSubscriptions(-1, nullptr); // will emit new msg
}

// peer closing write subscriptions
void BridgeSocket::onUnsubscribeWrite(sio::event & ev) {

    if (this->config->sio_verbose) {
        std::cout << "Unsubscribe write request: " << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    }

    auto peer = WRTCPeer::GetConnectedPeer(ev);
    if (peer == nullptr) return;
        
    auto msg = ev.get_message();
    if (msg->get_map().find("sources") != msg->get_map().end()) {
        auto arr = msg->get_map().at("sources");
        if (arr->get_flag() == sio::message::flag_array) {
            for (auto t : arr->get_vector()){
                peer->removeReqWriteSubscription(t->get_string());
            }
        }
    }

    if (ev.need_ack())
        peer->processSubscriptions(ev.get_msgId(), nullptr); // will emit ack
    else
        peer->processSubscriptions(-1, nullptr); // will emit new msg
}

// peer calling a service
void BridgeSocket::onSDPAnswer(sio::event & ev) {

    // if (this->config->sio_verbose) {
        std::cout << RED << "!!! (unhandled) Received SDP Answer: !!!" << CLR << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    // }

    auto ack = sio::object_message::create();
    ack->get_map().emplace("success", sio::int_message::create(0)); // TODO 1 !!
    BridgeSocket::ack(ev.get_msgId(), { ack });
}

// report error call via socket.io + rosout
void BridgeSocket::returnError(std::string message, sio::event const &ev) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", message.c_str());
    auto err_ack = sio::object_message::create();
    err_ack->get_map().emplace("err", sio::int_message::create(2));
    err_ack->get_map().emplace("msg", sio::string_message::create(message));
    BridgeSocket::ack(ev.get_msgId(), {err_ack});
}

// peer calling a service
void BridgeSocket::onServiceCall(sio::event & ev) {

    if (this->config->sio_verbose) {
        std::cout << "Service call request: " << std::endl;
        std::cout << printMessage(ev.get_message()) << std::endl;
    }
    
    auto peer = WRTCPeer::GetConnectedPeer(ev);
    if (peer == nullptr) return;

    auto service_name = ev.get_message()->get_map().at("service")->get_string();
    if (service_name.empty()) {
        return this->returnError("Service id not provided", ev);
    }
    auto service_type = Introspection::getService(service_name);
    if (service_type.empty()) {
        return this->returnError("Service '" + service_name + "' not discovered", ev);
    }

    // async thread
    std::thread newThread([this, ev, service_name, service_type]() {
        DataLED::once();
        this->node->callGenericService(service_name, service_type, ev);
    });
    newThread.detach();
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

// report unhandled messages
void BridgeSocket::onOtherSocketMessage(sio::event const& ev) {
    if (this->handled_events.find(ev.get_name().c_str()) != this->handled_events.end())
        return;
    
    std::cout << RED << "UNHANDLED SOCKER MSG '" << ev.get_name() << "': " << CLR << std::endl;
    std::cout << printMessage(ev.get_message()) << std::endl;
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
    auto instance = BridgeSocket::instance;
    if (instance == nullptr)
        return;
    instance->shutting_down = true;
    WRTCPeer::OnAllPeersDisconnected();
    instance->client.set_reconnect_attempts(0);
    instance->client.clear_con_listeners();
    instance->client.sync_close();
}

BridgeSocket::~BridgeSocket() {
    ConnLED::off();
}

std::string BridgeSocket::printMessage(const sio::message::ptr & message, bool pretty, int indent, std::string indent_prefix) {
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
                    if (pretty) {
                        out.append(sp*indent, ' ');
                    }
                    out += p.first + ": " + printMessage(p.second, pretty, indent+1, "") + ", ";
                    if (pretty)
                        out += '\n';
                }
                if (pretty) {
                    out.append(sp*(indent-1), ' ');
                }
                    
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
                    if (pretty) {
                        out.append(sp*indent, ' ');
                    }
                    out += printMessage(one, pretty, indent+1, "") + ", ";
                    if (pretty)
                        out += '\n';
                }
                if (pretty) {
                    out.append(sp*(indent-1), ' ');
                }
                out += "]";
                break;
            }
        default:
            break;
    }

    if (indent == 1) {
        auto lines = split(out, '\n');
        std::string out_shifted;
        for (size_t i = 0; i < lines.size(); i++) {
            out_shifted += indent_prefix + lines[i];
            if (i < lines.size()-1)
                out_shifted += "\n";
        }
        return out_shifted;
    }

    return out;
}

sio::message::ptr BridgeSocket::jsonToSioMessage(Json::Value val) {

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
            res->get_vector().push_back(jsonToSioMessage(one));
        }
        return res;
    } else if (val.isObject()) {
        auto res = sio::object_message::create();
        for (auto it = val.begin(); it != val.end(); ++it) {
            const std::string& key = it.key().asString();
            res->get_map().emplace(key, jsonToSioMessage(*it));
        }
        return res;
    } else {
        std::cerr << RED << "Invalid Json value type: " << std::endl;
        std::cerr << val.toStyledString() << std::endl;
        return sio::null_message::create();
    }
}