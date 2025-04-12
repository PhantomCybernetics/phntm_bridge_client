#pragma once
#include <cstdio>
#include <vector>
#include <string>

#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/config.hpp"

#include "rtc/rtc.hpp"

class WRTCPeer {

    public: 
        WRTCPeer(std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeConfig> config);
        ~WRTCPeer();
        static std::string getId(sio::object_message::ptr data);
        static bool isConnected(std::string id_peer);
        static std::shared_ptr<WRTCPeer> getConnectedPeer(sio::event &ev);
        static void onPeerConnected(std::string id_peer, sio::event &ev, std::shared_ptr<BridgeConfig> config);
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

    private:
        static std::map<std::string, std::shared_ptr<WRTCPeer>> connected_peers;
        static void addUIConfigToMessage(sio::object_message::ptr msg, std::shared_ptr<BridgeConfig> config);
        void onAnswerReply(sio::message::list const& reply);
        std::mutex processing_subscriptions_mutex;

        std::string id;
        std::string id_app;
        std::string id_instance;
        std::string session;
        bool is_connected;
        
        std::shared_ptr<BridgeConfig> config;

        rtc::PeerConnection *pc;
        void onRTCStateChange(rtc::PeerConnection::State state);
        void onRTCGatheringStateChange(rtc::PeerConnection::GatheringState state);

        std::vector<std::string> req_read_subs; // topic ids to subscribe
        std::vector<std::vector<std::string>> req_write_subs; // // [topic_id, msg_type]'s to write to

        sio::array_message::ptr subscribeDataTopic(std::string topic);
        sio::array_message::ptr subscribeImageOrVideoTopic(std::string topic);
};
