#pragma once
#include <cstdio>
#include <vector>
#include <string>

#include "sio_message.h"

class WRTCPeer {

    public: 
        WRTCPeer();
        ~WRTCPeer();
        static std::string GetId(sio::object_message::ptr data);
        static void OnPeerConnected(std::string id_peer, sio::object_message::ptr data, sio::object_message::ptr ack);

    private:
        static std::map<std::string, WRTCPeer> peers;
};
