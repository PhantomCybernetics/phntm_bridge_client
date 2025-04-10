#include "phntm_bridge/const.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "phntm_bridge/sio.hpp"

#include "rtc/peerconnection.hpp"
#include "sio_message.h"
#include "sio_socket.h"
#include <algorithm>
#include <memory>
#include <ostream>
#include <iostream>
#include <string>
#include <uuid/uuid.h>

std::map<std::string, std::shared_ptr<WRTCPeer>> WRTCPeer::connected_peers;

std::string WRTCPeer::GetId(sio::object_message::ptr data) {
    std::string id_peer;
    if (data->get_map().find("id_app") != data->get_map().end()) 
        id_peer = data->get_map().at("id_app")->get_string();
    if (data->get_map().find("id_instance") != data->get_map().end()) 
        id_peer = data->get_map().at("id_instance")->get_string();
    return id_peer;
}

bool WRTCPeer::IsConnected(std::string id_peer) {
    return connected_peers.find(id_peer) != connected_peers.end();
}

// vaidates peer id, checks if connected, calls error ack if requested
std::shared_ptr<WRTCPeer> WRTCPeer::GetConnectedPeer(sio::event & ev, std::shared_ptr<BridgeSocket> sio) {
    auto id_peer = WRTCPeer::GetId(ev.get_message());
    if (id_peer.empty()) {
        if (ev.need_ack()) {
            auto err_ack = sio::object_message::create();
            err_ack->get_map().emplace("err", sio::int_message::create(2));
            err_ack->get_map().emplace("msg", sio::string_message::create("No valid peer id provided"));
            sio->ack(ev.get_msgId(), { err_ack });
        }
        return nullptr;
    } else if (!WRTCPeer::IsConnected(id_peer)) {
        if (ev.need_ack()) {
            auto err_ack = sio::object_message::create();
            err_ack->get_map().emplace("err", sio::int_message::create(2));
            err_ack->get_map().emplace("msg", sio::string_message::create("Peer not connected"));
            sio->ack(ev.get_msgId(), { err_ack });
        }
        return nullptr;
    } else {
        return connected_peers.at(id_peer); // ok
    }
}

void WRTCPeer::OnPeerConnected(std::string id_peer, std::shared_ptr<BridgeSocket> sio, sio::event &ev, std::shared_ptr<BridgeConfig> config) {
    std::cout << GREEN << "Peer " << id_peer << " connected..." << CLR << std::endl;

    auto data = ev.get_message();

    uuid_t session;
    uuid_generate(session);
    char session_str[37];
    uuid_unparse(session, session_str);
    auto id_session= replace(session_str, "-", "");
    std::cout << "Generated new session id " << id_session << std::endl;

    auto peer = std::make_shared<WRTCPeer>(id_peer,
        data->get_map()["id_app"]->get_string(),
        data->get_map()["id_instance"]->get_string(),
        id_session,
        sio,
        config);
    connected_peers.emplace(id_peer, peer);

    auto msg = ev.get_message();
    peer->req_read_subs.clear();
    if (msg->get_map().find("read") != msg->get_map().end())  {
        auto arr = msg->get_map().at("read");
        if (arr->get_flag() != sio::message::flag_array) {
            std::cerr << RED << "Write subs provided is not an array" << CLR << std::endl;
        } else {
            for (auto t : arr->get_vector()){
                peer->req_read_subs.push_back(t->get_string());
            }
        }
    }
    peer->req_write_subs.clear();
    if (msg->get_map().find("write") != msg->get_map().end())  {
        auto arr = msg->get_map().at("write");
        if (arr->get_flag() != sio::message::flag_array) {
            std::cerr << RED << "Write subs provided is not an array" << CLR << std::endl;
        } else {
            for (auto tt : arr->get_vector()){
                if (tt->get_flag() != sio::message::flag_array) {
                    std::cerr << RED << "Write sub provided is not an array of [topic, msg_type]" << CLR << std::endl;
                } else if (tt->get_vector().size() != 2) {
                    std::cerr << RED << "Write sub provided is not 2 items long" << CLR << std::endl;
                } else {
                    std::vector<std::string> one;
                    one.push_back(tt->get_vector()[0]->get_string()); //topic
                    one.push_back(tt->get_vector()[1]->get_string()); //type
                    peer->req_write_subs.push_back(one);
                }
            }
        }   
    }

    auto ack = sio::object_message::create();
    AddUIConfigToMessage(ack, config);
    peer->processSubscription(ev, ack);
}

void WRTCPeer::onDisconnected() {
    std::cout << BLUE << this->toString() << "Disconnected." << CLR << std::endl;
    this->req_read_subs.clear(); // empty all subs
    this->req_write_subs.clear();
    this->is_connected = false;
    sio::message::list list_ignored;
    sio::event ev_ignored("", "", 0, list_ignored, false);
    this->processSubscription(ev_ignored, nullptr); // clean up
    connected_peers.erase(this->id);
}

void WRTCPeer::OnAllPeersDisconnected() {
    std::vector<std::string> user_ids;
    for (const auto& pair : connected_peers) {
        user_ids.push_back(pair.first); // Access the key
    }
    for (auto id_peer : user_ids){
        connected_peers.at(id_peer)->onDisconnected(); // removes from connected_peers
    }
}

std::string WRTCPeer::toString() {
    return "[RTC Peer #" + this->id + "] ";
}

WRTCPeer::WRTCPeer(std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeSocket> sio, std::shared_ptr<BridgeConfig> config) {
    this->id = id_peer;
    this->id_app = id_app;
    this->id_instance = id_instance;
    this->session = session;
    this->sio = sio;
    this->config = config;
    this->is_connected = true; // false after disconnect is received

    rtc::Configuration rtc_config;
    for (auto & one : this->config->ice_servers) {
        // std::cout << "    " << one << std::endl;
        if (one.compare(0, 5, "turn:") == 0) {
            auto turn_url = "turn:" + this->config->ice_username + ":" + this->config->ice_secret + "@" +  one.substr(5);
            if (this->config->webrtc_debug)
                std::cout << YELLOW << this->toString() << "Config adding: " << turn_url << CLR << std::endl;
            rtc_config.iceServers.emplace_back(turn_url);
        }
    }
    this->pc = new rtc::PeerConnection(rtc_config);
    this->pc->onStateChange(std::bind(&WRTCPeer::onRTCStateChange, this, std::placeholders::_1));
    this->pc->onGatheringStateChange(std::bind(&WRTCPeer::onRTCGatheringStateChange, this, std::placeholders::_1));

    // this->shared_ptr = std::shared_ptr<WRTCPeer>(this);
}

void WRTCPeer::onRTCStateChange(rtc::PeerConnection::State state) {
    if (this->config->webrtc_debug)
        std::cout << YELLOW << this->toString() << "State: " << state << CLR << std::endl;
}

void WRTCPeer::onRTCGatheringStateChange(rtc::PeerConnection::GatheringState state) {
    if (this->config->webrtc_debug)
        std::cout << YELLOW << this->toString() << "Gathering state: " << state << CLR << std::endl;
}

void WRTCPeer::processSubscription(sio::event &ev, sio::object_message::ptr ack) {

    std::shared_ptr<WRTCPeer> that = nullptr;
    for (auto pair : connected_peers) {
        if (pair.second->id == this->id) {
            that = pair.second;
            break;
        }
    }
    if (that == nullptr) {
        std::cerr << this->toString() << " Ptr not found in connected_peers, not processing subscriptions" << std::endl;
        return;
    }
    std::thread newThread([that, ev, ack]() {

        std::lock_guard<std::mutex> lock(that->processing_subscriptions_mutex); // wait until previus pressing completes
    
        bool disconnected = !that->is_connected || that->pc->state() == rtc::PeerConnection::State::Failed || !that->sio->connected || that->sio->shutting_down;

        if (that->config->webrtc_debug) {
            std::vector<std::string> req_writes_joined;
            for (auto one : that->req_write_subs) {
                req_writes_joined.push_back(one[0]+" {"+ one[1]+ "}");
            }
            std::cout << that->toString() << "Processing " << (disconnected ? BLUE + "disconnected " + CLR : "") << "subs, "
            << "read: " << BLUE << "[" << join( that->req_read_subs, ", ") << "] " << CLR
            << "write: " << RED << "[" << join(req_writes_joined, ", ") << "] " << CLR
            << "signalingState=" << YELLOW << that->pc->signalingState() << " " << CLR
            << "iceGatheringState=" << YELLOW << that->pc->gatheringState() << CLR
            << std::endl;
        }

        if (disconnected)
            return; // done here, not producing any reply

        sio::object_message::ptr reply;
        bool emit_new_reply = false;
        if (ack == nullptr) {
            reply = sio::object_message::create();
            emit_new_reply = true;
        } else {
            reply = ack;
        }
        
        reply->get_map().emplace("session", sio::string_message::create(that->session));
        reply->get_map().emplace("read_video_streams", sio::array_message::create());
        reply->get_map().emplace("read_data_channels", sio::array_message::create());
        reply->get_map().emplace("write_data_channels", sio::array_message::create());

        if (emit_new_reply) { // return as new sio message

            // self.get_logger().info(f'Sending update to {peer}')
            // if peer.id_app:
            //     update_data['id_app'] = peer.id_app
            // if peer.id_instance:
            //     update_data['id_instance'] = peer.id_instance
            // # print(update_data)
            // await self.sio.emit(event='peer:update',
            //                     data=update_data,
            //                     callback=peer.on_answer_reply)

        } else { // return as ack
            that->sio->ack(ev.get_msgId(), { reply });
        }
    });
    newThread.detach();
}

void WRTCPeer::AddUIConfigToMessage(sio::object_message::ptr msg, std::shared_ptr<BridgeConfig> config) {
     // enabled input drivers
     auto input_drivers = sio::array_message::create();
     for (auto & one : config->input_drivers) {
         input_drivers->get_vector().push_back(sio::string_message::create(one));
     }
     msg->get_map().emplace("input_drivers", input_drivers);

     // default input mappings json
     msg->get_map().emplace("input_defaults", BridgeSocket::JsonToSioMessage(config->input_defaults));
     
     //  custom input driver includes
     auto custom_input_drivers = sio::array_message::create();
     for (auto & one : config->custom_input_drivers) {
         auto one_msg = sio::object_message::create();
         one_msg->get_map().emplace("class", sio::string_message::create(one.class_name));
         one_msg->get_map().emplace("url", sio::string_message::create(one.url));
         custom_input_drivers->get_vector().push_back(one_msg);
     }
     msg->get_map().emplace("custom_input_drivers", custom_input_drivers);

     // pass service buttons json
     msg->get_map().emplace("service_defaults", BridgeSocket::JsonToSioMessage(config->service_defaults));

     // custom service widget includes
     auto custom_service_widgets = sio::array_message::create();
     for (auto & one : config->custom_service_widgets) {
         auto one_msg = sio::object_message::create();
         one_msg->get_map().emplace("class", sio::string_message::create(one.class_name));
         one_msg->get_map().emplace("url", sio::string_message::create(one.url));
         custom_service_widgets->get_vector().push_back(one_msg);
     }
     msg->get_map().emplace("custom_service_widgets", custom_service_widgets);

     // custom service widget-class mapping + custom data payload
     auto service_widgets = sio::array_message::create();
     for (auto & one : config->service_widgets) {
         auto one_msg = sio::object_message::create();
         one_msg->get_map().emplace("class", sio::string_message::create(one.class_name));
         one_msg->get_map().emplace("srv", sio::string_message::create(one.service));
         if (one.data) {
             one_msg->get_map().emplace("data", BridgeSocket::JsonToSioMessage(one.data));
         }
         service_widgets->get_vector().push_back(one_msg);
     }
     msg->get_map().emplace("service_widgets", service_widgets);

     // ui config vars
     auto ui_config = sio::object_message::create();
     ui_config->get_map().emplace("introspection_control", sio::bool_message::create(config->stop_discovery_after_sec > 0.0f)); // don't show introspection icon if continuous
     ui_config->get_map().emplace("battery_topic", sio::string_message::create(config->ui_battery_topic));
     ui_config->get_map().emplace("docker_control", sio::bool_message::create(config->docker_control_enabled));
     ui_config->get_map().emplace("docker_monitor_topic", sio::string_message::create(config->docker_monitor_topic));
     ui_config->get_map().emplace("wifi_monitor_topic", sio::string_message::create(config->ui_wifi_monitor_topic));
     ui_config->get_map().emplace("enable_wifi_scan", sio::bool_message::create(config->ui_enable_wifi_scan));
     ui_config->get_map().emplace("enable_wifi_roam", sio::bool_message::create(config->ui_enable_wifi_roam));
     auto collapse_services = sio::array_message::create();
     for (auto one : config->collapse_services) {
         collapse_services->get_vector().push_back(sio::string_message::create(one));
     }
     ui_config->get_map().emplace("collapse_services", collapse_services);
     ui_config->get_map().emplace("collapse_unhandled_services", sio::bool_message::create(config->collapse_unhandled_services));
     msg->get_map().emplace("ui", ui_config);

     // ice config
     auto ice_servers = sio::array_message::create();
     for (auto one : config->ice_servers) {
         ice_servers->get_vector().push_back(sio::string_message::create(one));
     }
     msg->get_map().emplace("ice_servers", ice_servers);
     msg->get_map().emplace("ice_username", sio::string_message::create(config->ice_username));
     msg->get_map().emplace("ice_secret", sio::string_message::create(config->ice_secret));
}


WRTCPeer::~WRTCPeer() {
    
}