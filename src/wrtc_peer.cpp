#include "phntm_bridge/wrtc_peer.hpp"

std::string WRTCPeer::GetId(sio::object_message::ptr data) {
    std::string id_peer;
    if (data->get_map().find("id_app") != data->get_map().end()) 
        id_peer = data->get_map().at("id_app")->get_string();
    if (data->get_map().find("id_instance") != data->get_map().end()) 
        id_peer = data->get_map().at("id_instance")->get_string();
    return id_peer;
}

void WRTCPeer::OnPeerConnected(std::string id_peer, sio::object_message::ptr data, sio::object_message::ptr ack) {

}

WRTCPeer::WRTCPeer() {
    
}