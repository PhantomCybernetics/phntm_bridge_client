#pragma once
#include <cstdio>
#include <vector>
#include <string>

#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/config.hpp"

#include "rtc/datachannel.hpp"
#include "rtc/rtc.hpp"

class WRTCPeer {

    public: 
        WRTCPeer(std::shared_ptr<PhntmBridge> node, std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeConfig> config);
        ~WRTCPeer();
        static std::string getId(sio::object_message::ptr data);
        static bool isConnected(std::string id_peer);
        static std::shared_ptr<WRTCPeer> getConnectedPeer(sio::event const & ev);
        static void onPeerConnected(std::shared_ptr<PhntmBridge> node, std::string id_peer, sio::event &ev, std::shared_ptr<BridgeConfig> config);
        static void onAllPeersDisconnected();

        void onDisconnected();
        std::string toString();

        // std::shared_ptr<WRTCPeer> shared_ptr;
        void processSubscriptions(int ack_msg_id, sio::object_message::ptr ack);
        static void processAllPeerSubscriptions();
        // void processWriteSubscriptions(int ack_msg_id);
        bool addReqReadSubscription(std::string topic);
        bool removeReqReadSubscription(std::string topic);
        bool addReqWriteSubscription(std::string topic, std::string msg_type);
        bool removeReqWriteSubscription(std::string topic);

        void onSIOOfferReply(sio::message::list const& reply);
        void onSDPAnswer(sio::message::ptr const& msg);

        static void initLogging(std::shared_ptr<BridgeConfig> config);

        static std::string toString(rtc::PeerConnection::SignalingState state);
        static std::string toString(rtc::PeerConnection::State state);
        static std::string toString(rtc::PeerConnection::GatheringState state);
        static std::string toString(rtc::PeerConnection::IceState state);
        
    private:
        static std::map<std::string, std::shared_ptr<WRTCPeer>> connected_peers;
        static void addUIConfigToMessage(sio::object_message::ptr msg, std::shared_ptr<BridgeConfig> config);
        
        std::mutex processing_subscriptions_mutex;
        bool awaiting_peer_reply;

        std::string id;
        std::string id_app;
        std::string id_instance;
        std::string session;
        bool is_connected;
        std::shared_ptr<PhntmBridge> node;
        
        std::shared_ptr<BridgeConfig> config;

        rtc::PeerConnection *pc;
        void onRTCStateChange(rtc::PeerConnection::State state);
        void onRTCGatheringStateChange(rtc::PeerConnection::GatheringState state);
        void onRTCSignalingStateChange(rtc::PeerConnection::SignalingState state);
        void onIceStateChange(rtc::PeerConnection::IceState state);
        void onLocalCandidate(rtc::Candidate candidate);

        std::vector<std::string> req_read_subs; // topic ids to subscribe
        std::vector<std::vector<std::string>> req_write_subs; // // [topic_id, msg_type]'s to write to

        sio::array_message::ptr subscribeDataTopic(std::string topic, std::string msg_type);
        sio::array_message::ptr subscribeImageOrVideoTopic(std::string topic, std::string msg_type);
        sio::array_message::ptr subscribeWriteDataTopic(std::string topic, std::string msg_type);

        std::map<std::string, std::shared_ptr<rtc::DataChannel>> outbound_data_channels; 
        uint16_t openDataChannelForTopic(std::string topic, std::string msg_type, bool is_reliable, bool write=false);

        std::map<std::string, std::shared_ptr<rtc::DataChannel>> inbound_data_channels; 

        uint16_t next_channel_id;
        bool negotiation_needed;
};
