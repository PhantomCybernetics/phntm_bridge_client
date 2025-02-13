#pragma once

#include "../../sioclient/sio_client.h"
#include "config.hpp"
#include "sio_socket.h"

class BridgeSocket
{
    public:
        BridgeSocket(std::shared_ptr<BridgeConfig> config);
        bool connect();
        void onConnected();
        void onFailed();
        void onSocketError(sio::message::ptr const& message);
        void onSocketMessage(sio::message::ptr const& message);
        void onSocketClose();
        void disconnect();
        ~BridgeSocket();

    private:
        sio::client client;
        sio::socket::ptr socket;
        std::shared_ptr<BridgeConfig> config;
};