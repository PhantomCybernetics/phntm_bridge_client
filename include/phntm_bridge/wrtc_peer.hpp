#pragma once
#include <cstdio>
#include <vector>
#include <string>

#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/config.hpp"

class WRTCPeer {

    public: 
        WRTCPeer(std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeSocket> sio, std::shared_ptr<BridgeConfig> config);
        ~WRTCPeer();
        static std::string GetId(sio::object_message::ptr data);
        static bool IsConnected(std::string id_peer);
        static void OnPeerConnected(std::string id_peer, std::shared_ptr<BridgeSocket> sio, sio::event &ev, std::shared_ptr<BridgeConfig> config);
        static void OnPeerDisconnected(std::string id_peer);

    private:
        static std::map<std::string, WRTCPeer> connected_peers;
        static void AddUIConfigToMessage(sio::object_message::ptr msg, std::shared_ptr<BridgeConfig> config);

        std::string id;
        std::string id_app;
        std::string id_instance;
        std::string session;

        std::shared_ptr<BridgeConfig> config;
        std::shared_ptr<BridgeSocket> sio;

        bool processing_subscriptions;

        void processSubscription(sio::event &ev, sio::object_message::ptr ack);
};
