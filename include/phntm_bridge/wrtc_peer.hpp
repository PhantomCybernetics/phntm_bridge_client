#pragma once
#include <cstdio>
#include <vector>
#include <string>

#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/config.hpp"

#include "rtc/rtc.hpp"

class WRTCPeer {

    public: 
        WRTCPeer(std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeSocket> sio, std::shared_ptr<BridgeConfig> config);
        ~WRTCPeer();
        static std::string GetId(sio::object_message::ptr data);
        static bool IsConnected(std::string id_peer);
        static std::shared_ptr<WRTCPeer> GetConnectedPeer(sio::event &ev, std::shared_ptr<BridgeSocket> sio);
        static void OnPeerConnected(std::string id_peer, std::shared_ptr<BridgeSocket> sio, sio::event &ev, std::shared_ptr<BridgeConfig> config);
        static void OnAllPeersDisconnected();

        void onDisconnected();
        std::string toString();

        // std::shared_ptr<WRTCPeer> shared_ptr;
        std::mutex processing_subscriptions_mutex;

    private:
        static std::map<std::string, std::shared_ptr<WRTCPeer>> connected_peers;
        static void AddUIConfigToMessage(sio::object_message::ptr msg, std::shared_ptr<BridgeConfig> config);

        std::string id;
        std::string id_app;
        std::string id_instance;
        std::string session;
        bool is_connected;
        
        std::shared_ptr<BridgeConfig> config;
        std::shared_ptr<BridgeSocket> sio;

        rtc::PeerConnection *pc;
        void onRTCStateChange(rtc::PeerConnection::State state);
        void onRTCGatheringStateChange(rtc::PeerConnection::GatheringState state);

        void processSubscription(sio::event &ev, sio::object_message::ptr ack);
        std::vector<std::string> req_read_subs; // topic ids to subscribe
        std::vector<std::vector<std::string>> req_write_subs; // // [topic_id, msg_type]'s to write to
};
