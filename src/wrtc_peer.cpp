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
#include <cstdint>
#include <memory>
#include <ostream>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <string>

namespace phntm {

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

    bool WRTCPeer::anyPeersConnected() {
        return connected_peers.size() > 0;
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
        // RCLCPP_INFO(node->get_logger(), "Peer %s connected...", id_peer.c_str());

        auto data = ev.get_message();

        // uuid_t session;
        // uuid_generate(session);
        // char session_str[37]; // session id unique to peer connection, change will force the peer to restart its pc
        // uuid_unparse(session, session_str);
        auto id_session = generateId(24);//= replace(session_str, "-", "");
        log("Generated new session id " + id_session);

        auto peer = std::make_shared<WRTCPeer>(
            node,
            id_peer,
            data->get_map()["id_app"]->get_string(),
            data->get_map()["id_instance"]->get_string(),
            id_session,
            config);
        connected_peers.emplace(id_peer, peer);
        RCLCPP_INFO(node->get_logger(), "%s Connected", peer->toString().c_str());

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

    void WRTCPeer::onWRTCInfo(sio::object_message::ptr msg) {
        if (msg->get_map().find("state") != msg->get_map().end()) {
            auto peer_state = msg->get_map().at("state")->get_string();
            if (peer_state == "n/a" || peer_state == "failed") {
                log(RED+this->toString() + "Got peer WRTC fail, state: " + peer_state + "; restarting" + CLR);
                // this->removePeerConnection(); //force reconnect
                this->peer_needs_restart = true;
                this->processSubscriptions();
            } else {
                log(this->toString() + "Got peer WRTC state: " + peer_state);
            }
            
        } else {
            log(this->toString() + "Got unknown WRTC state from the peer", true);
        }
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
        RCLCPP_INFO(node->get_logger(), "%s Peer disconnected", this->toString().c_str());
        // log(BLUE + this->toString() + "Disconnected" + CLR);
        this->req_read_subs.clear(); // empty all subs
        this->req_write_subs.clear();
        this->is_connected = false;
        this->processSubscriptionsSync(); // clean up in this thread
        connected_peers.erase(this->id);
    }

    void WRTCPeer::onAllPeersDisconnected() {
        std::vector<std::string> user_ids;
        for (const auto& pair : connected_peers) {
            user_ids.push_back(pair.first);
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
        this->peer_needs_restart = false;

        this->createPeerConnection();
        this->connected_rtp_time_base = getCurrentRtpTimestamp();
    }

    void WRTCPeer::createPeerConnection() {

        if (this->config->webrtc_debug)
            log(GRAY + this->toString() + "Creating a new PC" + CLR);

        rtc::Configuration rtc_config;
        rtc_config.disableAutoNegotiation = true;
        rtc_config.disableAutoGathering = false;
        rtc_config.certificateType = rtc::CertificateType::Default;
        rtc_config.enableIceUdpMux = config->enable_ice_udp_mux;
        rtc_config.iceTransportPolicy = rtc::TransportPolicy::All;
        rtc_config.disableFingerprintVerification = config->disable_fingerprint_verification;
        rtc_config.enableIceTcp = config->enable_ice_tcp;
        rtc_config.forceMediaTransport = true;
        
        for (auto & one : this->config->ice_servers) {
            if (one.compare(0, 5, "turn:") == 0) {
                auto turn_url = "turn://" + this->config->ice_username + ":" + this->config->ice_secret + "@" +  one.substr(5);
                auto turn_url_log = replace(turn_url, this->config->ice_secret, "*******");
                rtc::IceServer ice_server(turn_url);
                // ice_server.relayType = rtc::IceServer::RelayType::TurnTls;
                // if (this->config->webrtc_debug) {
                    log(YELLOW + this->toString() + "Config adding ICE: " + turn_url_log + CLR);
                    // log(" => user: " + ice_server.username + (!ice_server.password.empty() ? " @ ******* " : ""));
                    // log(" => type: " + std::string(ice_server.type == rtc::IceServer::Type::Turn ? "TURN" : (ice_server.type == rtc::IceServer::Type::Stun ? "STUN" : "?!")));
                    // log(" => relay type: " + std::string(ice_server.relayType == rtc::IceServer::RelayType::TurnUdp ? "TurnUdp" : (ice_server.relayType == rtc::IceServer::RelayType::TurnTcp ? "TurnTcp" : (ice_server.relayType == rtc::IceServer::RelayType::TurnTls ? "TurnTls" : "?!"))));
                    // log(" => hostname: " + ice_server.hostname + ":" + std::to_string(ice_server.port));
                // }
                    
                rtc_config.iceServers.emplace_back(ice_server);
            }
        }
        
        this->pc = std::make_shared<rtc::PeerConnection>(rtc_config);
        this->pc->onStateChange([&](rtc::PeerConnection::State state){
            if (this->config->webrtc_debug)
                log(YELLOW + this->toString() + "PC State: " + toString(state) + CLR);
        });
        this->pc->onGatheringStateChange([&](rtc::PeerConnection::GatheringState state){
            if (this->config->webrtc_debug)
                log(YELLOW + this->toString() + "Gathering state: " + toString(state) + CLR);
        });
        this->pc->onSignalingStateChange([&](rtc::PeerConnection::SignalingState state){
            if (this->config->webrtc_debug)
                log(YELLOW + this->toString() + "Signaling state: " + toString(state) + CLR);
            
            TopicReaderData::onPCSignalingStateChange(this->pc);
            TopicReaderH264::onPCSignalingStateChange(shared_from_this());
        });
        this->pc->onLocalCandidate([&](rtc::Candidate candidate){
            if (this->config->webrtc_debug)
                log(YELLOW + this->toString() + "Got local candidate: " + std::string(candidate) + CLR);
        });
        this->pc->onIceStateChange([&](rtc::PeerConnection::IceState state){
            if (this->config->webrtc_debug)
                log(YELLOW + this->toString() + "ICE state: " + toString(state) + CLR);

            if (state == rtc::PeerConnection::IceState::Failed && this->is_connected) {
                if (this->config->webrtc_debug)
                    log(GRAY + this->toString() + "Reconnecting bcs of ICE failure... " + CLR);
                this->processSubscriptions(); // new negotiation
            }
        });
        // this->pc->onTrack([&](std::shared_ptr<rtc::Track> track){
        //     // if (this->config->webrtc_debug)
        //     // log(YELLOW + this->toString() + "Track added! Fixing ssrcs:" + CLR);
        //     // auto description = track->description();
        //     // for (auto ssrc : description.getSSRCs()) {
        //     //     log(std::to_string(ssrc));
        //     // }
        // //     description.addSSRC(mySSRC, "video");
        // //     track->setDescription(std::move(description));
        // });
    }

    void WRTCPeer::removePeerConnection() {
        if (this->pc == nullptr)
            return;

        if (this->config->webrtc_debug)
            log(GRAY + this->toString() + "Removing current PC" + CLR);

        this->pc->resetCallbacks();
        this->pc->close();
        this->pc.reset();
        this->pc = nullptr;
    }


    void WRTCPeer::processAllPeerSubscriptions() {
        for (auto it = WRTCPeer::connected_peers.begin(); it != WRTCPeer::connected_peers.end(); ++it) {  
            it->second->processSubscriptions();
        }
    }

    void WRTCPeer::processSubscriptionsSync(int ack_msg_id, sio::object_message::ptr ack) {

        std::lock_guard<std::mutex> lock(this->processing_subscriptions_mutex); // wait until previus pressing completes

        log(BLUE + this->toString() + "Processing peer subs begins" + (ack_msg_id > -1 ? " [msg #" + std::to_string(ack_msg_id) + "]" : "") + " >>" + CLR);

        bool is_reconnect = this->is_connected && BridgeSocket::isConnected() && (this->peer_needs_restart || this->pc == nullptr || this->pc->state() == rtc::PeerConnection::State::Failed || this->pc->iceState() == rtc::PeerConnection::IceState::Closed);
        this->negotiation_needed = is_reconnect; // reset

        if (this->config->webrtc_debug) {
            std::vector<std::string> req_writes_joined;
            for (auto one : this->req_write_subs) {
                req_writes_joined.push_back(one[0]+" {"+ one[1]+ "}");
            }
            log(this->toString() + (this->is_connected ? "Processing " : "Cleaning up ") + (is_reconnect ? MAGENTA + "re-connect " + CLR : (!this->is_connected ? BLUE + "disconnected " + CLR : "")) + "subs:\n"
                + "    read: " + GREEN + "[" + join( this->req_read_subs, ", ") + "] " + CLR + "\n"
                + "    write: " + MAGENTA + "[" + join(req_writes_joined, ", ") + "] " + CLR + "\n"
                + "    " + (this->pc != nullptr ? ("pc state=" + YELLOW + toString(this->pc->state()) + CLR + " signalingState=" + YELLOW + toString(this->pc->signalingState()) + CLR + " iceGatheringState=" + YELLOW + toString(this->pc->gatheringState()) + CLR) : "pc = " + RED + "null" + CLR)
            );
        }

        sio::object_message::ptr reply = ack == nullptr ? sio::object_message::create() : ack;
        reply->get_map().emplace("session", sio::string_message::create(this->session));
        reply->get_map().emplace("read_video_streams", sio::array_message::create());
        reply->get_map().emplace("read_data_channels", sio::array_message::create());
        reply->get_map().emplace("write_data_channels", sio::array_message::create());
        
        if (is_reconnect) {
            this->peer_needs_restart = false;
            this->removePeerConnection();
            this->outbound_data_channels.clear();
            this->inbound_data_channels.clear();
            this->createPeerConnection();
        }

        // open read data and media channels
        for (auto topic : this->req_read_subs) {
            auto msg_type = Introspection::getTopic(topic);
            if (msg_type.empty()) {
                log(GRAY + this->toString() + "Topic '"+topic+"' not yet discovered" + CLR);
                continue; // topic not yet discovered
            }
            if (!isImageOrVideoType(msg_type)) {
                auto channel_config = this->subscribeReadDataTopic(topic, msg_type);
                reply->get_map().at("read_data_channels")->get_vector().push_back(channel_config);
            } else {
                auto channel_config = this->subscribeMediaTopic(topic, msg_type);
                reply->get_map().at("read_video_streams")->get_vector().push_back(channel_config);
            }
        }

        // open write data channels
        for (auto sub : this->req_write_subs) {
            auto topic = sub[0];
            auto msg_type = sub[1];

            auto channel_config = this->subscribeWriteDataTopic(topic, msg_type);
            reply->get_map().at("write_data_channels")->get_vector().push_back(channel_config); 
        }

        // close unused read channels
        std::vector<std::string> r_dcs_to_close;
        for (const auto& pair : this->outbound_data_channels)
            if (std::find(this->req_read_subs.begin(), this->req_read_subs.end(), pair.first) == this->req_read_subs.end())
                r_dcs_to_close.push_back(pair.first);
        for (auto topic : r_dcs_to_close) {
            auto removed_channel_config = this->unsubscribeReadDataTopic(topic);
            if (removed_channel_config->get_vector().size())
                reply->get_map().at("read_data_channels")->get_vector().push_back(removed_channel_config); // no id => unsubscribed                 
        }

        // unsubscribe from unused media streams
        std::vector<std::string> r_media_tracks_to_close;
        for (const auto& pair : this->outbound_media_tracks)
            if (pair.second->in_use && std::find(this->req_read_subs.begin(), this->req_read_subs.end(), pair.first) == this->req_read_subs.end())
                r_media_tracks_to_close.push_back(pair.first);
        for (auto topic : r_media_tracks_to_close) {
            auto removed_channel_config = this->unsubscribeMediaTopic(topic, is_reconnect || !this->is_connected);
            if (removed_channel_config->get_vector().size())
                reply->get_map().at("read_video_streams")->get_vector().push_back(removed_channel_config); // no id => unsubscribed               
        }
        
        // close unused write channels
        std::vector<std::string> w_dcs_to_close;
        for (const auto& pair : this->inbound_data_channels) {
            bool active = false;
            for (size_t i = 0; i < this->req_write_subs.size(); i++) {
                if (this->req_write_subs[i][0] == pair.first) {
                    active = true;
                    break;
                }
            }
            if (!active)
                w_dcs_to_close.push_back(pair.first);
        }
        for (auto topic : w_dcs_to_close) {
            auto removed_channel_config = this->unsubscribeWriteDataTopic(topic);
            if (removed_channel_config->get_vector().size())
                reply->get_map().at("write_data_channels")->get_vector().push_back(removed_channel_config); // no id => unsubscribed   
        }

        this->awaiting_peer_reply = false;
        if (this->is_connected && this->negotiation_needed) { // that->pc->negotiationNeeded()
            if (this->config->webrtc_debug)
                log(YELLOW + this->toString() + "RTC negotiation needed, generating local offer..." + CLR);

            //auto offer = that->pc->createOffer();
            std::string sdpBuffer;
            this->pc->onLocalDescription([&](auto sdp) {
                sdpBuffer = std::string(sdp);
            });
            this->pc->setLocalDescription(rtc::Description::Type::Offer);
            while(sdpBuffer.empty() && this->is_connected) { // wait for the offer to be (re-generated)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            while(this->pc->gatheringState() != rtc::PeerConnection::GatheringState::Complete && this->is_connected) { // wait for the offer
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            auto local_desc = this->pc->localDescription();
            auto sdp = local_desc->generateSdp(); // this includes late discovered candidates, sdpBuffer doesn't
            if (this->config->webrtc_debug) {
                log(GRAY + this->toString() + "Generated local SDP offer" + (this->config->log_sdp ? ":\n" + sdp : "") + CLR);
            }
            reply->get_map().emplace("offer", sio::string_message::create(sdp));

            this->awaiting_peer_reply = true;
        }

        if (!this->is_connected && !is_reconnect) { //cleanup and end here

            log(GRAY + this->toString() + "Disconnected, closing PC" + CLR);
            this->removePeerConnection();

        } else if (this->is_connected) { // produce update and wait for peer reply

            if (ack_msg_id < 0) { // emit as new sio peer:update message 
                if (!this->id_app.empty())
                    reply->get_map().emplace("id_app", sio::string_message::create(this->id_app));
                if (!this->id_instance.empty())
                    reply->get_map().emplace("id_instance", sio::string_message::create(this->id_instance));
                BridgeSocket::emit("peer:update", { reply }, std::bind(&WRTCPeer::onSIOOfferReply, this, std::placeholders::_1)); //no ack

            } else { // return as ack
                BridgeSocket::ack(ack_msg_id, { reply });
            }

            while(this->awaiting_peer_reply && this->is_connected) { // block the mutex until reply is received and processed
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

        }
        
        log(BLUE + this->toString() + "<< Processing subs finished" + (ack_msg_id > -1 ? " [msg #" + std::to_string(ack_msg_id) + "]" : "") + "." + CLR);
    }

    void WRTCPeer::processSubscriptions(int ack_msg_id, sio::object_message::ptr ack) {
        std::thread newThread([that = shared_from_this(), ack_msg_id, ack]() {
            log(GRAY + that->toString() + "Requested peer subs processing" + (ack_msg_id > -1 ? " [msg #" + std::to_string(ack_msg_id) + "]" : "") + CLR);

            try {
                that->processSubscriptionsSync(ack_msg_id, ack);

            } catch (const std::exception& ex) {
                log(that->toString() + "Exception in subs processor: " + std::string(ex.what()), true);
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
            this->awaiting_peer_reply = false; // unlocks mutex
            return;
        }
        
        auto sdp = msg->get_map().at("sdp")->get_string();
        if (this->config->webrtc_debug) {
            log(CYAN + this->toString() + "Got peer answer" + (this->config->log_sdp ? ":\n" + sdp : "") + CLR);
        }

        rtc::Description answer(sdp, rtc::Description::Type::Answer);
        this->pc->setRemoteDescription(answer);
        this->awaiting_peer_reply = false; // unlocks mutex
    }


    uint16_t WRTCPeer::openDataChannelForTopic(std::string topic, std::string msg_type, bool is_reliable, bool write) {
        rtc::Reliability reliability;
        reliability.unordered = !is_reliable;
        reliability.maxRetransmits = is_reliable ? 10 : 0;

        rtc::DataChannelInit dc_init;
        dc_init.reliability = reliability;
        dc_init.negotiated = true; // dcs negotiated by the bridge, not webrtc
        dc_init.id = this->nextChannelId();
        dc_init.protocol = msg_type;

        log(GRAY + this->toString() + "Opening new " +(write?"write":"read") + (is_reliable ? " RELIABLE":"") + " DC for " + topic + " {" + msg_type + "} #" + std::to_string(this->next_channel_id) + CLR);

        auto dc = this->pc->createDataChannel(topic, dc_init);

        dc->onOpen([this, topic, msg_type]() {
            log(GREEN + this->toString() + "DC is open for " + topic + " {" + msg_type + "}" + CLR);
            // if (!write) {
            //     TopicReaderData::onDCOpen(dc, this->pc); // send latest data when available
            // }
        });

        dc->onClosed([this, topic]() {
            log(BLUE + this->toString() + "DC is closed for " + topic + CLR);
        });

        if (!write) { // read channels

            this->outbound_data_channels.emplace(topic, dc);
            return this->outbound_data_channels.at(topic)->id().value();

        } else { // write channels

            bool is_heartbeat = topic == HEARTBEAT_CHANNEL_ID;

            dc->onMessage([this, topic, msg_type, dc, is_heartbeat](std::variant<rtc::binary, rtc::string> message) {
            
                if (is_heartbeat) {
                    if (std::holds_alternative<rtc::binary>(message)) { // always bin
                        if (this->config->log_heartbeat) {
                            log(GRAY + this->toString() + "got heartbeat PING" + CLR);
                        }
                        rtc::binary& bin = std::get<rtc::binary>(message);
                        dc->send(bin);
                    }
                    return;
                }

                if (!TopicWriterData::onData(shared_from_this(), dc, topic, message)) {
                    log(RED + this->toString() + "DC for " + topic + " got unhandled message" + CLR);            
                }
            });

            this->inbound_data_channels.emplace(topic, dc);
            return this->inbound_data_channels.at(topic)->id().value();
        }
    }

    // /// Send previous key frame so browser can show something to user
    // /// @param stream Stream
    // /// @param video Video track data
    // void sendInitialNalus(std::shared_ptr<rtc::Track> track, std::shared_ptr<rtc::ClientTrackData> video) {
    //     auto h264 = track->
        
    //     video.get()
    //     auto initialNalus = h264->initialNALUS();

    //     // send previous NALU key frame so users don't have to wait to see stream works
    //     if (!initialNalus.empty()) {
    //         const double frameDuration_s = double(h264->getSampleDuration_us()) / (1000 * 1000);
    //         const uint32_t frameTimestampDuration = video->sender->rtpConfig->secondsToTimestamp(frameDuration_s);
    //         video->sender->rtpConfig->timestamp = video->sender->rtpConfig->startTimestamp - frameTimestampDuration * 2;
    //         video->track->send(initialNalus);
    //         video->sender->rtpConfig->timestamp += frameTimestampDuration;
    //         // Send initial NAL units again to start stream in firefox browser
    //         video->track->send(initialNalus);
    //     }
    // }

    void WRTCPeer::closeDataChannelForTopic(std::string topic, bool write) {

        log(GRAY + this->toString() + "Closing " +(write?"write":"read")+ " DC for " + topic + CLR);

        if (!write) {
            if (this->outbound_data_channels.at(topic)->isOpen()) {
                this->outbound_data_channels.at(topic)->close();
            }
            this->outbound_data_channels.erase(topic);
        } else {
            if (this->inbound_data_channels.at(topic)->isOpen()) {
                this->inbound_data_channels.at(topic)->close();
            }
            this->inbound_data_channels.erase(topic);
        }
    }

    

    sio::array_message::ptr WRTCPeer::subscribeReadDataTopic(std::string topic, std::string msg_type) {
        
        auto qos = this->node->loadTopicQoSConfig(topic); // get topic qos config from yaml
        auto topic_conf = this->node->loadTopicMsgTypeExtraConfig(topic, msg_type); // get topic extras from yaml
        bool is_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;

        uint16_t id_dc;
        std::shared_ptr<rtc::DataChannel> dc;
        if (this->outbound_data_channels.find(topic) != this->outbound_data_channels.end()) {
            dc = this->outbound_data_channels.at(topic);
            id_dc = dc->id().value();
        } else {
            id_dc = this->openDataChannelForTopic(topic, msg_type, is_reliable);
            dc = this->outbound_data_channels.at(topic);
            this->negotiation_needed = true;
        }
        auto topic_reader = TopicReaderData::getForTopic(topic, msg_type, this->node, qos);
        topic_reader->addOutput(dc, this->pc); // only adds once

        // dc config for the peer
        auto channel_config = sio::array_message::create();
        channel_config->get_vector().push_back(sio::string_message::create(topic));
        channel_config->get_vector().push_back(sio::int_message::create(id_dc));
        channel_config->get_vector().push_back(sio::string_message::create(msg_type));
        channel_config->get_vector().push_back(sio::bool_message::create(is_reliable));
        channel_config->get_vector().push_back( topic_conf);
        return channel_config;
    }

    sio::array_message::ptr WRTCPeer::subscribeWriteDataTopic(std::string topic, std::string msg_type) {
        
        auto qos = this->node->loadTopicQoSConfig(topic);
        auto topic_conf = this->node->loadTopicMsgTypeExtraConfig(topic, msg_type);
        bool is_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;

        uint16_t id_dc;
        std::shared_ptr<rtc::DataChannel> dc;
        if (this->inbound_data_channels.find(topic) != this->inbound_data_channels.end()) {
            dc = this->inbound_data_channels.at(topic);
            id_dc = dc->id().value();
        } else {
            id_dc = this->openDataChannelForTopic(topic, msg_type, is_reliable, true);
            dc = this->inbound_data_channels.at(topic);
            this->negotiation_needed = true;        
        }
        if (topic != HEARTBEAT_CHANNEL_ID) {
            auto topic_writer = TopicWriterData::getForTopic(topic, msg_type, this->node, qos);
            topic_writer->addInput(dc); // only adds once
        }
        
        // dc config for the peer
        auto channel_config = sio::array_message::create();
        channel_config->get_vector().push_back(sio::string_message::create(topic));
        channel_config->get_vector().push_back(sio::int_message::create(id_dc));
        channel_config->get_vector().push_back(sio::string_message::create(msg_type));
        // channel_config->get_vector().push_back( topic_conf);
        return channel_config;
    }

    sio::array_message::ptr WRTCPeer::unsubscribeReadDataTopic(std::string topic) {
        auto removed_channel_config = sio::array_message::create();
        if (this->outbound_data_channels.find(topic) != this->outbound_data_channels.end()) {
            if (topic != HEARTBEAT_CHANNEL_ID) {
                auto dc = this->outbound_data_channels.at(topic);
                auto tr = TopicReaderData::getForTopic(topic);
                if (tr != nullptr) {
                    if (tr->removeOutput(dc)) { // stops if no other peer subs left, returns true of empty
                        TopicReaderData::destroy(topic);
                    }
                } else {
                    log(RED + "TopicReader not found for " + topic +"!") ;
                }
            }
            this->closeDataChannelForTopic(topic, false); // removes from outbound_data_channels
            removed_channel_config->get_vector().push_back(sio::string_message::create(topic));
        }
        return removed_channel_config;
    }

    sio::array_message::ptr WRTCPeer::unsubscribeWriteDataTopic(std::string topic) {
        auto removed_channel_config = sio::array_message::create();
        if (this->inbound_data_channels.find(topic) != this->inbound_data_channels.end()) {
            if (topic != HEARTBEAT_CHANNEL_ID) {
                auto dc = this->inbound_data_channels.at(topic);
                auto tw = TopicWriterData::getForTopic(topic);
                if (tw != nullptr) {
                    tw->removeInput(dc); // stops if no other peer subs left
                } else {
                    log(RED + "TopicWriter not found for " + topic +"!" + CLR) ;
                }
            }
            this->closeDataChannelForTopic(topic, true); // removes from inbound_data_channels
            removed_channel_config->get_vector().push_back(sio::string_message::create(topic));
        }
        return removed_channel_config;
    }

    sio::array_message::ptr WRTCPeer::subscribeMediaTopic(std::string topic, std::string msg_type) {
        
        auto qos = this->node->loadTopicQoSConfig(topic);
        auto topic_conf = this->node->loadTopicMsgTypeExtraConfig(topic, msg_type); // get topic extras from yaml

        std::string stream_id;
        std::shared_ptr<TopicReaderH264::MediaTrackInfo> track_info;
        if (this->outbound_media_tracks.find(topic) != this->outbound_media_tracks.end()) {
            log(GRAY + this->toString() + "Re-using media track for " + topic + CLR);
            track_info = this->outbound_media_tracks.at(topic);
            track_info->in_use = true;
            stream_id = track_info->msid;
        } else {
            stream_id = TopicReaderH264::openMediaTrackForTopic(topic, shared_from_this());
            track_info = this->outbound_media_tracks.at(topic);
            this->negotiation_needed = true;
        }

        if (isEncodedVideoType(msg_type)) {
            auto topic_reader = TopicReaderH264::getForTopic(topic, qos,
                                        topic_conf->get_map().at("create_node")->get_bool() ? nullptr : this->node,
                                        nullptr,
                                        topic_conf->get_map().at("use_pts")->get_bool(),
                                        topic_conf->get_map().at("debug_verbose")->get_bool(),
                                        topic_conf->get_map().at("debug_num_frames")->get_int()
                                        );
            topic_reader->addOutput(track_info, shared_from_this()); // only adds once
        } else {
            //TODO Image topics
        }

        // media stream config for the peer
        auto channel_config = sio::array_message::create();
        channel_config->get_vector().push_back(sio::string_message::create(topic));
        auto stream_id_msg = sio::string_message::create(stream_id);
        channel_config->get_vector().push_back(stream_id_msg);
        return channel_config;
    }

    sio::array_message::ptr WRTCPeer::unsubscribeMediaTopic(std::string topic, bool close_channel) {
        auto removed_channel_config = sio::array_message::create();
        if (this->outbound_media_tracks.find(topic) != this->outbound_media_tracks.end()) {

            auto track_info = this->outbound_media_tracks.at(topic);

            if (!track_info->in_use && !close_channel)
                return removed_channel_config; // no change

            track_info->in_use = false;

            try {
                if (auto tr = TopicReaderH264::getForTopic(topic)) {
                    if (tr->removeOutput(track_info)) { // stops if no other peer subs left, returns true of empty
                        TopicReaderH264::destroy(topic);
                    }
                } else if (false) {
                    //TODO Image topics
                } else {
                    log(RED + "Topic reader not found for " + topic +"!" + CLR) ;
                }
            } catch (const std::exception & ex) {
                log("Exception destroying media topic reader: " + std::string(ex.what()), true);
            }
            if (close_channel) 
                TopicReaderH264::closeMediaTrackForTopic(topic, shared_from_this()); // when disconected, removes from outbound_media_tracks
            else
                log(GRAY + this->toString() + "Keeping open media track for " + topic + CLR); 
            removed_channel_config->get_vector().push_back(sio::string_message::create(topic)); // no id_track
        }
        return removed_channel_config;
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
            case rtc::PeerConnection::State::Connected: return "Connected";
            case rtc::PeerConnection::State::Connecting: return "Connecting";
            case rtc::PeerConnection::State::Closed: return "Closed";
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

}