#pragma once

#include "../../sioclient/sio_client.h"
#include "config.hpp"
#include "sio_message.h"
#include "sio_socket.h"
#include <map>
#include <string>

class Introspection;

class BridgeSocket
{
    public:
        BridgeSocket(std::shared_ptr<BridgeConfig> config);
        bool connect();
        void disconnect();
        void emit(std::string const& name, sio::message::list const& msglist, std::function<void (sio::message::list const&)> const& ack);
        ~BridgeSocket();
        
        void setIntrospection(std::shared_ptr<Introspection> introspection) { this->introspection = introspection; };

        static std::string PrintMessage(const sio::message::ptr & message, bool pretty = true, int indent = 1);

    private:
        std::shared_ptr<Introspection> introspection;

        sio::client client;
        sio::socket::ptr socket;
        std::shared_ptr<BridgeConfig> config;
        bool connected;

        void onConnected();
        void onDisconnected();
        void onClosed(sio::client::close_reason const& reason);
        void onFailed();
        void onSocketOpen();
        void onSocketClose();
        void onSocketError(sio::message::ptr const& message);

        std::map<std::string, sio::socket::event_listener> handled_events;
        void onIceServers(sio::event const& ev);
        void onPeerConnected(sio::event const& ev);
        void onPeerDisconnected(sio::event const& ev);
        void onIntrospection(sio::event const& ev);

        void onOtherSocketMessage(sio::event const& ev);
};