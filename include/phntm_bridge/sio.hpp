#pragma once

#include "sio_client.h"
#include "config.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include "sio_message.h"
#include "sio_socket.h"
#include <map>
#include <string>
#include <json/json.h>

class PhntmBridge;

class BridgeSocket
{
    public:
        static void init(std::shared_ptr<PhntmBridge> node, std::shared_ptr<BridgeConfig> config);
        static bool connect();
        static void shutdown();
        static void emit(std::string const& name, sio::message::list const& msglist, std::function<void (sio::message::list const&)> const& ack);
        static void ack(int msg_id, sio::message::list const& msglist);
        static bool isConnected() { return BridgeSocket::instance != nullptr && BridgeSocket::instance->connected; };
        static bool isShuttingDown() { return BridgeSocket::instance != nullptr && BridgeSocket::instance->shutting_down; };

        static std::string printMessage(const sio::message::ptr & message, bool pretty = true, int indent = 1, std::string indent_prefix = "");
        static sio::message::ptr jsonToSioMessage(Json::Value val);

    private:
        BridgeSocket(std::shared_ptr<PhntmBridge> node, std::shared_ptr<BridgeConfig> config);
        ~BridgeSocket();
        static BridgeSocket * instance;

        std::shared_ptr<PhntmBridge> node;

        bool connected, shutting_down;

        sio::client client;
        sio::socket::ptr socket;

        std::shared_ptr<BridgeConfig> config;
        
        std::string socket_url;
        sio::message::ptr auth_data;

        void onConnected();
        void onDisconnected();
        void onClosed(sio::client::close_reason const& reason);
        void onFailed();
        void onReconnecting();
        void onReconnect(uint attemptCount, uint delay);
        void onSocketOpen();
        void onSocketClose();
        void onSocketError(sio::message::ptr const& message);
        void returnError(std::string message, sio::event const &ev);
        void returnSuccess(sio::event const &ev, int success = 1);

        std::map<std::string, sio::socket::event_listener> handled_events;
        void onIceServers(sio::event const& ev);
        void onPeerConnected(sio::event &ev);
        void onPeerWRTCInfo(sio::event & ev);
        void onPeerDisconnected(sio::event & ev);
        void onIntrospection(sio::event & ev);

        void onSubscribeRead(sio::event & ev);
        void onUnsubscribeRead(sio::event & ev);
        void onSubscribeWrite(sio::event & ev);
        void onUnsubscribeWrite(sio::event & ev);
        void onServiceCall(sio::event & ev);

        void onSDPAnswer(sio::event & ev);

        void onOtherSocketMessage(sio::event const& ev);

        static std::string msgDebugHeader (sio::event const & ev);
};