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

std::string WRTCPeer::getId(sio::object_message::ptr data) {
    std::string id_peer;
    if (data->get_map().find("id_app") != data->get_map().end()) 
        id_peer = data->get_map().at("id_app")->get_string();
    if (data->get_map().find("id_instance") != data->get_map().end()) 
        id_peer = data->get_map().at("id_instance")->get_string();
    return id_peer;
}

bool WRTCPeer::isConnected(std::string id_peer) {
    return connected_peers.find(id_peer) != connected_peers.end();
}

// vaidates peer id, checks if connected, calls error ack if requested
std::shared_ptr<WRTCPeer> WRTCPeer::getConnectedPeer(sio::event & ev) {
    auto id_peer = WRTCPeer::getId(ev.get_message());
    if (id_peer.empty()) {
        if (ev.need_ack()) {
            auto err_ack = sio::object_message::create();
            err_ack->get_map().emplace("err", sio::int_message::create(2));
            err_ack->get_map().emplace("msg", sio::string_message::create("No valid peer id provided"));
            BridgeSocket::ack(ev.get_msgId(), { err_ack });
        }
        return nullptr;
    } else if (!WRTCPeer::isConnected(id_peer)) {
        if (ev.need_ack()) {
            auto err_ack = sio::object_message::create();
            err_ack->get_map().emplace("err", sio::int_message::create(2));
            err_ack->get_map().emplace("msg", sio::string_message::create("Peer not connected"));
            BridgeSocket::ack(ev.get_msgId(), { err_ack });
        }
        return nullptr;
    } else {
        return connected_peers.at(id_peer); // ok
    }
}

void WRTCPeer::onPeerConnected(std::string id_peer, sio::event &ev, std::shared_ptr<BridgeConfig> config) {
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
                peer->addReqReadSubscription(t->get_string());
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
                    peer->addReqWriteSubscription(tt->get_vector()[0]->get_string(), tt->get_vector()[1]->get_string());
                }
            }
        }   
    }

    auto ack = sio::object_message::create();
    WRTCPeer::addUIConfigToMessage(ack, config); // adds ui config to reply that is emitted later
    peer->processSubscriptions(ev.get_msgId(), ack);
}

bool WRTCPeer::addReqReadSubscription(std::string topic) {
    if (std::find(this->req_read_subs.begin(), this->req_read_subs.end(), topic) != this->req_read_subs.end())
        return false;
    if (this->config->webrtc_debug)
        std::cout << this->toString() << "Adding read subscription to: " << GREEN << topic << CLR << std::endl;
    this->req_read_subs.push_back(topic);
    return true;
}

bool WRTCPeer::removeReqReadSubscription(std::string topic) {
    auto pos = std::find(this->req_read_subs.begin(), this->req_read_subs.end(), topic);
    if (pos == this->req_read_subs.end())
        return false;
    if (this->config->webrtc_debug)
        std::cout << this->toString() << "Removing read subscription to: " << topic << std::endl;
    this->req_read_subs.erase(pos);
    return true;
}

bool WRTCPeer::addReqWriteSubscription(std::string topic, std::string msg_type) {
    for (size_t i = 0; i <  this->req_write_subs.size(); ) {
        if (this->req_write_subs[i][0] == topic) {
            if (this->req_write_subs[i][1] == msg_type)
                return false; // don't add duplicates
            this->req_write_subs.erase(this->req_write_subs.begin() + i); // remove and add w new type
        } else {
            i++;
        }
    }
    if (this->config->webrtc_debug)
        std::cout << this->toString() << "Adding write subscription to: " << MAGENTA << topic << " {" << msg_type <<"}" << CLR << std::endl;
    std::vector<std::string> one;
    one.push_back(topic);
    one.push_back(msg_type);
    this->req_write_subs.push_back(one);
    return true;
}

bool WRTCPeer::removeReqWriteSubscription(std::string topic) {
    for (size_t i = 0; i <  this->req_write_subs.size(); ) {
        if (this->req_write_subs[i][0] == topic) {
            if (this->config->webrtc_debug)
                std::cout << this->toString() << "Removing write subscription to: " << topic << std::endl;
            this->req_write_subs.erase(this->req_write_subs.begin() + i); // remove and add w new type
            return true;
        } else {
            i++;
        }
    }
    return false;
}

void WRTCPeer::onDisconnected() {
    std::cout << BLUE << this->toString() << "Disconnected." << CLR << std::endl;
    this->req_read_subs.clear(); // empty all subs
    this->req_write_subs.clear();
    this->is_connected = false;
    this->processSubscriptions(-1, nullptr); // clean up
    connected_peers.erase(this->id);
}

void WRTCPeer::onAllPeersDisconnected() {
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

WRTCPeer::WRTCPeer(std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeConfig> config) {
    this->id = id_peer;
    this->id_app = id_app;
    this->id_instance = id_instance;
    this->session = session;
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

void WRTCPeer::processAllPeerSubscriptions() {
    for (auto it = WRTCPeer::connected_peers.begin(); it != WRTCPeer::connected_peers.end(); ++it) {  
        it->second->processSubscriptions(-1, nullptr);
    }
}

void WRTCPeer::processSubscriptions(int ack_msg_id, sio::object_message::ptr ack) {

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
    std::thread newThread([that, ack_msg_id, ack]() {

        std::lock_guard<std::mutex> lock(that->processing_subscriptions_mutex); // wait until previus pressing completes
    
        bool disconnected = !that->is_connected || that->pc->state() == rtc::PeerConnection::State::Failed || !BridgeSocket::isConnected() || BridgeSocket::isShuttingDown();

        if (that->config->webrtc_debug) {
            std::vector<std::string> req_writes_joined;
            for (auto one : that->req_write_subs) {
                req_writes_joined.push_back(one[0]+" {"+ one[1]+ "}");
            }
            std::cout << that->toString() << "Processing " << (disconnected ? BLUE + "disconnected " + CLR : "") << "subs, "
            << "read: " << GREEN << "[" << join( that->req_read_subs, ", ") << "] " << CLR
            << "write: " << MAGENTA << "[" << join(req_writes_joined, ", ") << "] " << CLR
            << "signalingState=" << YELLOW << that->pc->signalingState() << " " << CLR
            << "iceGatheringState=" << YELLOW << that->pc->gatheringState() << CLR
            << std::endl;
        }

        // open read data and media channels
        for (auto sub : that->req_read_subs) {

        }

        // open write data channels
        for (auto sub : that->req_write_subs) {
            
        }

        // unsubscribe from data channels
        // for topic in list(peer.outbound_data_channels.keys()):
        //     if not topic in peer.read_subs:
        //         self.get_logger().info(f'{peer} unsubscribing from {topic}')
        //         await self.unsubscribe_data_topic(topic, peer=peer, msg_callback=None)
        //         res['read_data_channels'].append([ topic ]) # no id => unsubscribed

        // unsubscribe from video streams
        // for sub in list(peer.video_tracks.keys()):
        //     if not sub in peer.read_subs:
        //         self.get_logger().info(f'{peer} unsubscribing from image topic {sub}')
        //         await self.unsubscribe_image_topic(sub, peer)
        //         res['read_video_streams'].append([ sub ]) # no id => unsubscribed

        // close write channels
        // for topic in list(peer.inbound_data_channels.keys()):
        //     topic_active = False
        //     for sub in peer.write_subs:
        //         if sub[0] == topic:
        //             topic_active = True
        //             break
        //     if not topic_active:
        //         self.get_logger().info(f'{peer} stopped writing into {topic}')
        //         await self.close_write_channel(topic, peer)
        //         res['write_data_channels'].append([ topic ]) # no id => unsubscribed

        if (disconnected)
            return; // done here, not producing any reply

        sio::object_message::ptr reply;
        if (ack == nullptr) {
            reply = sio::object_message::create();
        } else {
            reply = ack;
        }
        
        reply->get_map().emplace("session", sio::string_message::create(that->session));
        reply->get_map().emplace("read_video_streams", sio::array_message::create());
        reply->get_map().emplace("read_data_channels", sio::array_message::create());
        reply->get_map().emplace("write_data_channels", sio::array_message::create());

        if (ack_msg_id < 0) { // return as new sio message

            if (!that->id_app.empty())
                reply->get_map().emplace("id_app", sio::string_message::create(that->id_app));
            if (!that->id_instance.empty())
                reply->get_map().emplace("id_instance", sio::string_message::create(that->id_instance));
                        
            BridgeSocket::emit("peer:update", { reply }, std::bind(&WRTCPeer::onAnswerReply, that, std::placeholders::_1)); //no ack

        } else { // return as ack
            BridgeSocket::ack(ack_msg_id, { reply });
        }
    });
    newThread.detach();
}

void WRTCPeer::onAnswerReply(sio::message::list const& reply) {
    if (this->config->sio_verbose) {
        std::cout << "SIO got reply for peer:update message: " << std::endl
                  << BridgeSocket::printMessage(reply.at(0)) << std::endl;
    }
    auto msg = reply.at(0);

    if (msg->get_map().find("err") != msg->get_map().end()) {
        std::cerr << RED << "Client returned error for peer:update: " << msg->get_map().at("err")->get_string() << CLR << std::endl;
        return;
    }

    // if self.pc.signalingState != 'have-local-offer':
    //     self.logger.error(f'Not setting SDP answer from {self}, signalingState={self.pc.signalingState}')
    //     self.processing_subscriptions = False
    //     return
    
    // self.logger.info(c(f'Got peer answer:', 'cyan'))
    // print(reply_data)
    // answer = RTCSessionDescription(sdp=reply_data['sdp'], type='answer')
    // await self.pc.setRemoteDescription(answer)
    // self.processing_subscriptions = False
}

void WRTCPeer::addUIConfigToMessage(sio::object_message::ptr msg, std::shared_ptr<BridgeConfig> config) {
     // enabled input drivers
     auto input_drivers = sio::array_message::create();
     for (auto & one : config->input_drivers) {
         input_drivers->get_vector().push_back(sio::string_message::create(one));
     }
     msg->get_map().emplace("input_drivers", input_drivers);

     // default input mappings json
     msg->get_map().emplace("input_defaults", BridgeSocket::jsonToSioMessage(config->input_defaults));
     
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
     msg->get_map().emplace("service_defaults", BridgeSocket::jsonToSioMessage(config->service_defaults));

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
             one_msg->get_map().emplace("data", BridgeSocket::jsonToSioMessage(one.data));
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