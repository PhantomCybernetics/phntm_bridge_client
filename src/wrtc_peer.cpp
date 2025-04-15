#include "phntm_bridge/const.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "phntm_bridge/sio.hpp"

#include "rtc/configuration.hpp"
#include "rtc/global.hpp"
#include "rtc/peerconnection.hpp"
#include "sio_message.h"
#include "sio_socket.h"
#include <algorithm>
#include <memory>
#include <ostream>
#include <iostream>
#include <rclcpp/qos.hpp>
#include <string>
#include <uuid/uuid.h>

std::map<std::string, std::shared_ptr<WRTCPeer>> WRTCPeer::connected_peers;

void WRTCPeer::initLogging(std::shared_ptr<BridgeConfig> config) {
    if (config->webrtc_debug && config->webrtc_verbose)
        rtc::InitLogger(rtc::LogLevel::Verbose);
    else if (config->webrtc_debug)
        rtc::InitLogger(rtc::LogLevel::Debug);
    else
        rtc::InitLogger(rtc::LogLevel::Error);
}

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
std::shared_ptr<WRTCPeer> WRTCPeer::getConnectedPeer(sio::event const & ev) {
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

void WRTCPeer::onPeerConnected(std::shared_ptr<PhntmBridge> node, std::string id_peer, sio::event &ev, std::shared_ptr<BridgeConfig> config) {
    log(GREEN + "Peer " + id_peer + " connected..." + CLR);

    auto data = ev.get_message();

    uuid_t session;
    uuid_generate(session);
    char session_str[37];
    uuid_unparse(session, session_str);
    auto id_session= replace(session_str, "-", "");
    log("Generated new session id " + id_session);

    auto peer = std::make_shared<WRTCPeer>(
        node,
        id_peer,
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
            log("Write subs provided is not an array", true);
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
            log("Write subs provided is not an array", true);
        } else {
            for (auto tt : arr->get_vector()){
                if (tt->get_flag() != sio::message::flag_array) {
                    log("Write sub provided is not an array of [topic, msg_type]", true);
                } else if (tt->get_vector().size() != 2) {
                    log("Write sub provided is not 2 items long", true);
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
       log(this->toString() + "Adding read subscription to: " + GREEN + topic + CLR);
    this->req_read_subs.push_back(topic);
    return true;
}

bool WRTCPeer::removeReqReadSubscription(std::string topic) {
    auto pos = std::find(this->req_read_subs.begin(), this->req_read_subs.end(), topic);
    if (pos == this->req_read_subs.end())
        return false;
    if (this->config->webrtc_debug)
        log(this->toString() + "Removing read subscription to: " + topic);
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
        log(this->toString() + "Adding write subscription to: " + MAGENTA + topic + " {" + msg_type + "}" + CLR);
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
                log(this->toString() + "Removing write subscription to: " + topic);
            this->req_write_subs.erase(this->req_write_subs.begin() + i); // remove and add w new type
            return true;
        } else {
            i++;
        }
    }
    return false;
}

void WRTCPeer::onDisconnected() {
    log(BLUE + this->toString() + "Disconnected." + CLR);
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

WRTCPeer::WRTCPeer(std::shared_ptr<PhntmBridge> node, std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeConfig> config) {
    this->id = id_peer;
    this->id_app = id_app;
    this->id_instance = id_instance;
    this->session = session;
    this->config = config;
    this->node = node;
    this->is_connected = true; // false after disconnect is received
    this->next_channel_id = 0;

    rtc::Configuration rtc_config;
    rtc_config.disableAutoNegotiation = true;
    rtc_config.disableAutoGathering = false;
    rtc_config.certificateType = rtc::CertificateType::Rsa;
    rtc_config.enableIceUdpMux = true;
    rtc_config.disableFingerprintVerification = false;
    for (auto & one : this->config->ice_servers) {
        if (one.compare(0, 5, "turn:") == 0) {
            auto turn_url = "turns://" + this->config->ice_username + ":" + this->config->ice_secret + "@" +  one.substr(5);
            rtc::IceServer ice_server(turn_url);
            // ice_server.relayType = rtc::IceServer::RelayType::TurnTls;
            if (this->config->webrtc_debug) {
                log(YELLOW + this->toString() + "Config adding: " + turn_url + CLR);
                log(" => user: " + ice_server.username + " @ " + ice_server.password);
                log(" => type: " + std::string(ice_server.type == rtc::IceServer::Type::Turn ? "TURN" : (ice_server.type == rtc::IceServer::Type::Stun ? "STUN" : "?!")));
                log(" => relay type: " + std::string(ice_server.relayType == rtc::IceServer::RelayType::TurnUdp ? "TurnUdp" : (ice_server.relayType == rtc::IceServer::RelayType::TurnTcp ? "TurnTcp" : (ice_server.relayType == rtc::IceServer::RelayType::TurnTls ? "TurnTls" : "?!"))));
                log(" => hostname: " + ice_server.hostname + ":" + std::to_string(ice_server.port));
            }
                
            rtc_config.iceServers.emplace_back(ice_server);
        }
    }
    
    this->pc = new rtc::PeerConnection(rtc_config);
    this->pc->onStateChange(std::bind(&WRTCPeer::onRTCStateChange, this, std::placeholders::_1));
    this->pc->onGatheringStateChange(std::bind(&WRTCPeer::onRTCGatheringStateChange, this, std::placeholders::_1));
    this->pc->onSignalingStateChange(std::bind(&WRTCPeer::onRTCSignalingStateChange, this, std::placeholders::_1));
    this->pc->onIceStateChange(std::bind(&WRTCPeer::onIceStateChange, this, std::placeholders::_1));
    this->pc->onLocalCandidate(std::bind(&WRTCPeer::onLocalCandidate, this, std::placeholders::_1));
    // this->shared_ptr = std::shared_ptr<WRTCPeer>(this);
}

void WRTCPeer::onLocalCandidate(rtc::Candidate candidate) {
    if (this->config->webrtc_debug)
        log(YELLOW + this->toString() + "Local candidate: " + std::string(candidate) + CLR);
}

void WRTCPeer::onRTCStateChange(rtc::PeerConnection::State state) {
    if (this->config->webrtc_debug)
        log(YELLOW + this->toString() + "State: " + toString(state) + CLR);
}

void WRTCPeer::onRTCGatheringStateChange(rtc::PeerConnection::GatheringState state) {
    if (this->config->webrtc_debug)
        log(YELLOW + this->toString() + "Gathering state: " + toString(state) + CLR);
}

void WRTCPeer::onRTCSignalingStateChange(rtc::PeerConnection::SignalingState state) {
    if (this->config->webrtc_debug)
        log(YELLOW + this->toString() + "Signaling state: " + toString(state) + CLR);
}

void WRTCPeer::onIceStateChange(rtc::PeerConnection::IceState state) {
    if (this->config->webrtc_debug)
        log(YELLOW + this->toString() + "ICE state: " + toString(state) + CLR);
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
        log(this->toString() + " Ptr not found in connected_peers, not processing subscriptions", true);
        return;
    }
    std::thread newThread([that, ack_msg_id, ack]() {
        log(BLUE + that->toString() + "Processing requested [msg #" + std::to_string(ack_msg_id) + "]" + CLR);

        {
            std::lock_guard<std::mutex> lock(that->processing_subscriptions_mutex); // wait until previus pressing completes
            log(BLUE + that->toString() + "Processing peer subs begins [msg #" + std::to_string(ack_msg_id) + "]..." + CLR);

            bool disconnected = !that->is_connected || that->pc->state() == rtc::PeerConnection::State::Failed || !BridgeSocket::isConnected() || BridgeSocket::isShuttingDown();
            that->negotiation_needed = false; // reset

            if (that->config->webrtc_debug) {
                std::vector<std::string> req_writes_joined;
                for (auto one : that->req_write_subs) {
                    req_writes_joined.push_back(one[0]+" {"+ one[1]+ "}");
                }
                log(that->toString() + "Processing " + (disconnected ? BLUE + "disconnected " + CLR : "") + "subs:\n"
                    + "    read: " + GREEN + "[" + join( that->req_read_subs, ", ") + "] " + CLR + "\n"
                    + "    write: " + MAGENTA + "[" + join(req_writes_joined, ", ") + "] " + CLR + "\n"
                    + "    state=" + YELLOW + toString(that->pc->state()) + CLR + " signalingState=" + YELLOW + toString(that->pc->signalingState()) + CLR + " iceGatheringState=" + YELLOW + toString(that->pc->gatheringState()) + CLR
                );
            }

            sio::object_message::ptr reply = ack == nullptr ? sio::object_message::create() : ack;
            reply->get_map().emplace("session", sio::string_message::create(that->session));
            reply->get_map().emplace("read_video_streams", sio::array_message::create());
            reply->get_map().emplace("read_data_channels", sio::array_message::create());
            reply->get_map().emplace("write_data_channels", sio::array_message::create());

            // open read data and media channels
            for (auto topic : that->req_read_subs) {
                auto msg_type = Introspection::getTopic(topic);
                if (msg_type.empty()) {
                    log(GRAY + that->toString() + " Topic '"+topic+"' not yet discovered" + CLR);
                    continue; // topic not yet discovered
                }
                if (!isImageOrVideoType(msg_type)) {
                    auto channel_config = that->subscribeDataTopic(topic, msg_type);
                    reply->get_map().at("read_data_channels")->get_vector().push_back(channel_config);
                } else {
                    auto channel_config = that->subscribeImageOrVideoTopic(topic, msg_type);
                    reply->get_map().at("read_video_streams")->get_vector().push_back(channel_config);
                }
            }

            // open write data channels
            for (auto sub : that->req_write_subs) {
                auto topic = sub[0];
                auto msg_type = sub[1];

                auto channel_config = that->subscribeWriteDataTopic(topic, msg_type);
                reply->get_map().at("write_data_channels")->get_vector().push_back(channel_config);
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

            that->awaiting_peer_reply = false;
            if (that->negotiation_needed) { // that->pc->negotiationNeeded()
                if (that->config->webrtc_debug)
                    log(YELLOW + that->toString() + "RTC negotiation needed" + CLR);

                //auto offer = that->pc->createOffer();
                std::string sdpBuffer;
                that->pc->onLocalDescription([&](auto sdp) {
                     sdpBuffer = std::string(sdp);
                });
                that->pc->setLocalDescription(rtc::Description::Type::Offer);
                while(sdpBuffer.empty()) { // wait for the offer to be (re-generated)
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                while(that->pc->gatheringState() != rtc::PeerConnection::GatheringState::Complete) { // wait for the offer
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                auto local_desc = that->pc->localDescription();
                auto sdp = local_desc->generateSdp();
                if (that->config->webrtc_debug) {
                    log(GRAY + that->toString() + "Generated local SDP offer:\n" + sdp + CLR);
                }
                reply->get_map().emplace("offer", sio::string_message::create(sdp));

                that->awaiting_peer_reply = true;
            }

            if (disconnected)
                return; // done here, not producing any reply

            if (ack_msg_id < 0) { // return as new sio message
                if (!that->id_app.empty())
                    reply->get_map().emplace("id_app", sio::string_message::create(that->id_app));
                if (!that->id_instance.empty())
                    reply->get_map().emplace("id_instance", sio::string_message::create(that->id_instance));
                BridgeSocket::emit("peer:update", { reply }, std::bind(&WRTCPeer::onSIOOfferReply, that, std::placeholders::_1)); //no ack

            } else { // return as ack
                BridgeSocket::ack(ack_msg_id, { reply });
            }

            while(that->awaiting_peer_reply) { // block the mutex until reply is processed
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            log(BLUE + that->toString() + "Processing subs complete [msg #" + std::to_string(ack_msg_id) + "]" + CLR);
        }
    });
    newThread.detach();
}

void WRTCPeer::onSIOOfferReply(sio::message::list const& reply) {
    if (this->config->sio_verbose) {
        log("SIO got reply for peer:update message:\n"
          + BridgeSocket::printMessage(reply.at(0)));
    }
    auto msg = reply.at(0);

    if (msg->get_map().find("err") != msg->get_map().end()) {
        log("Client returned error for peer:update: " + msg->get_map().at("err")->get_string(), true);
        this->awaiting_peer_reply = false;
        return;
    }

    this->onSDPAnswer(msg);
}

void WRTCPeer::onSDPAnswer(sio::message::ptr const& msg) {
    if (this->pc->signalingState() != rtc::PeerConnection::SignalingState::HaveLocalOffer) {
        log(this->toString() + "Not setting SDP answer; signalingState=" + toString(this->pc->signalingState()));
        this->awaiting_peer_reply = false;
        return;
    }
    
    auto sdp = msg->get_map().at("sdp")->get_string();
    if (this->config->webrtc_debug) {
        log(CYAN + this->toString() + "Got peer answer:\n" + sdp + CLR);
    }

    rtc::Description answer(sdp, rtc::Description::Type::Answer);
    this->pc->setRemoteDescription(answer);
    this->awaiting_peer_reply = false; // unlocks mutex
}


uint16_t WRTCPeer::openDataChannelForTopic(std::string topic, std::string msg_type, bool is_reliable, bool write) {
    rtc::Reliability reliability;
    reliability.unordered = !is_reliable;
    if (!is_reliable)
        reliability.maxRetransmits = 0;

    rtc::DataChannelInit dc_init;
    dc_init.reliability = reliability;
    dc_init.negotiated = true;
    dc_init.id = ++this->next_channel_id;
    dc_init.protocol = msg_type;

    log(GRAY + this->toString() + "Opening new DC for " + topic + " {" + msg_type + "} #" + std::to_string(this->next_channel_id) + CLR);

    auto dc = this->pc->createDataChannel(topic, dc_init);

    dc->onOpen([this, topic, msg_type]() {
        log(GREEN + this->toString() + "DC is open for " + topic + " {" + msg_type + "}" + CLR);
    });

    dc->onClosed([this, topic]() {
        log(BLUE + this->toString() + "DC is closed for " + topic + CLR);
    });

    if (!write) {
        this->outbound_data_channels.emplace(topic, dc);
        return this->outbound_data_channels.at(topic)->id().value();
    } else {

        dc->onMessage([this, topic, msg_type](std::variant<rtc::binary, rtc::string> message) {
            // if (std::holds_alternative<rtc::string>(message)) {
            //     std::cout << "Received: " << get<rtc::string>(message) << std::endl;
            // }
            log(GREEN + this->toString() + "DC for " + topic + " got message" + CLR);
        });

        this->inbound_data_channels.emplace(topic, dc);
        return this->inbound_data_channels.at(topic)->id().value();
    }
    
}

sio::array_message::ptr WRTCPeer::subscribeDataTopic(std::string topic, std::string msg_type) {
    
    auto qos = this->node->loadTopicQoSConfig(topic);
    auto topic_conf = this->node->loadTopicMsgTypeExtraConfig(topic, msg_type);
    bool is_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;

    uint16_t id_dc;
    bool send_latest = false;
    if (this->outbound_data_channels.find(topic) != this->outbound_data_channels.end()) {
        id_dc = this->outbound_data_channels.at(topic)->id().value();
    } else {
        id_dc = this->openDataChannelForTopic(topic, msg_type, is_reliable);
        send_latest = is_reliable;
        this->negotiation_needed = true;
    }

    if (send_latest) {
        // ???
    }

    // dc config for the peer
    auto channel_config = sio::array_message::create();
    channel_config->get_vector().push_back(sio::string_message::create(topic));
    channel_config->get_vector().push_back(sio::int_message::create(id_dc));
    channel_config->get_vector().push_back(sio::bool_message::create(is_reliable));
    channel_config->get_vector().push_back( topic_conf);
    return channel_config;
}

sio::array_message::ptr WRTCPeer::subscribeWriteDataTopic(std::string topic, std::string msg_type) {
    
    auto qos = this->node->loadTopicQoSConfig(topic);
    auto topic_conf = this->node->loadTopicMsgTypeExtraConfig(topic, msg_type);
    bool is_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;

    uint16_t id_dc;
    if (this->inbound_data_channels.find(topic) != this->inbound_data_channels.end()) {
        id_dc = this->inbound_data_channels.at(topic)->id().value();
    } else {
        id_dc = this->openDataChannelForTopic(topic, msg_type, is_reliable, true);
        this->negotiation_needed = true;

        
    }

    // dc config for the peer
    auto channel_config = sio::array_message::create();
    channel_config->get_vector().push_back(sio::string_message::create(topic));
    channel_config->get_vector().push_back(sio::int_message::create(id_dc));
    channel_config->get_vector().push_back(sio::string_message::create(msg_type));
    // channel_config->get_vector().push_back( topic_conf);
    return channel_config;
}

sio::array_message::ptr WRTCPeer::subscribeImageOrVideoTopic(std::string topic, std::string msg_type) {
    
    auto qos = this->node->loadTopicQoSConfig(topic);

    std::string id_track = ""; // TODO

    // media stream config for the peer
    auto channel_config = sio::array_message::create();
    channel_config->get_vector().push_back(sio::string_message::create(topic));
    channel_config->get_vector().push_back(sio::string_message::create(id_track));
    return channel_config;
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

std::string WRTCPeer::toString(rtc::PeerConnection::State state) {
    switch (state) {
        case rtc::PeerConnection::State::Connected:  return "Connected";
        case rtc::PeerConnection::State::Connecting: return "Connecting";
        case rtc::PeerConnection::State::Closed:          return "Closed";
        case rtc::PeerConnection::State::Disconnected: return "Disconnected";
        case rtc::PeerConnection::State::Failed: return "Failed";
        case rtc::PeerConnection::State::New: return "New";
        default: return "Unknown";
    }
}

std::string WRTCPeer::toString(rtc::PeerConnection::SignalingState state) {
    switch (state) {
        case rtc::PeerConnection::SignalingState::Stable: return "Stable";
        case rtc::PeerConnection::SignalingState::HaveLocalOffer: return "HaveLocalOffer";
        case rtc::PeerConnection::SignalingState::HaveRemoteOffer: return "HaveRemoteOffer";
        case rtc::PeerConnection::SignalingState::HaveLocalPranswer: return "HaveLocalPranswer";
        case rtc::PeerConnection::SignalingState::HaveRemotePranswer: return "HaveRemotePranswer";
        default: return "Unknown";
    }
}

std::string WRTCPeer::toString(rtc::PeerConnection::GatheringState state) {
    switch (state) {
        case rtc::PeerConnection::GatheringState::New: return "New";
        case rtc::PeerConnection::GatheringState::InProgress: return "InProgress";
        case rtc::PeerConnection::GatheringState::Complete: return "Complete";
        default: return "Unknown";
    }
}

std::string WRTCPeer::toString(rtc::PeerConnection::IceState state) {
    switch (state) {
        case rtc::PeerConnection::IceState::New: return "New";
        case rtc::PeerConnection::IceState::Checking: return "Checking";
        case rtc::PeerConnection::IceState::Connected: return "Connected";
        case rtc::PeerConnection::IceState::Completed: return "Completed";
        case rtc::PeerConnection::IceState::Failed: return "Failed";
        case rtc::PeerConnection::IceState::Disconnected: return "Disconnected";
        case rtc::PeerConnection::IceState::Closed: return "Closed";
        default: return "Unknown";
    }
}