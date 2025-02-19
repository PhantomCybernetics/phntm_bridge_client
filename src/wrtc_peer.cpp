#include "phntm_bridge/const.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "phntm_bridge/sio.hpp"

#include "sio_message.h"
#include <algorithm>
#include <ostream>
#include <iostream>
#include <string>
#include <uuid/uuid.h>

std::map<std::string, WRTCPeer> WRTCPeer::connected_peers;

std::string WRTCPeer::GetId(sio::object_message::ptr data) {
    std::string id_peer;
    if (data->get_map().find("id_app") != data->get_map().end()) 
        id_peer = data->get_map().at("id_app")->get_string();
    if (data->get_map().find("id_instance") != data->get_map().end()) 
        id_peer = data->get_map().at("id_instance")->get_string();
    return id_peer;
}

bool WRTCPeer::IsConnected(std::string id_peer) {
    return connected_peers.find(id_peer) != connected_peers.end();
}

void WRTCPeer::OnPeerConnected(std::string id_peer, std::shared_ptr<BridgeSocket> sio, sio::event &ev, std::shared_ptr<BridgeConfig> config) {
    std::cout << GREEN << "Peer " << id_peer << " connected..." << CLR << std::endl;

    auto data = ev.get_message();

    uuid_t session;
    uuid_generate(session);
    char session_str[37];
    uuid_unparse(session, session_str);
    auto id_session= replace(session_str, "-", "");
    std::cout << "Generated new session id " << id_session << std::endl;

    auto peer = WRTCPeer(id_peer,
                               data->get_map()["id_app"]->get_string(),
                               data->get_map()["id_instance"]->get_string(),
                               id_session,
                               sio,
                               config
                            );
    
    connected_peers.emplace(id_peer, peer);

    auto ack = sio::object_message::create();
    AddUIConfigToMessage(ack, config);
    peer.processSubscription(ev, ack);
}

void WRTCPeer::OnPeerDisconnected(std::string id_peer) {
    if (!IsConnected(id_peer))
        return;
    std::cout << BLUE << "Peer " << id_peer << " disconnected." << CLR << std::endl;
    connected_peers.erase(id_peer);
}

WRTCPeer::WRTCPeer(std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeSocket> sio, std::shared_ptr<BridgeConfig> config) {
    this->id = id_peer;
    this->id_app = id_app;
    this->id_instance = id_instance;
    this->session = session;
    this->sio = sio;
    this->config = config;

    this->processing_subscriptions = false;
}

void WRTCPeer::processSubscription(sio::event &ev, sio::object_message::ptr reply) {

    if (this->processing_subscriptions) {
        std::cerr << RED << "Failed to process " << this->id << " subs, peer busy" << CLR << std::endl;
        return;
    }

    this->processing_subscriptions = true;

    bool emit_reply = (reply == nullptr);
    if (emit_reply) {
        reply = sio::object_message::create();
    }
    
    reply->get_map().emplace("session", sio::string_message::create(this->session));
    reply->get_map().emplace("read_video_streams", sio::array_message::create());
    reply->get_map().emplace("read_data_channels", sio::array_message::create());
    reply->get_map().emplace("write_data_channels", sio::array_message::create());

    if (emit_reply) {

        // self.get_logger().info(f'Sending update to {peer}')
        // if peer.id_app:
        //     update_data['id_app'] = peer.id_app
        // if peer.id_instance:
        //     update_data['id_instance'] = peer.id_instance
        // # print(update_data)
        // await self.sio.emit(event='peer:update',
        //                     data=update_data,
        //                     callback=peer.on_answer_reply)

    } else {
        this->sio->ack(ev.get_msgId(), { reply });
        this->processing_subscriptions = false; // all done here
    }
}

void WRTCPeer::AddUIConfigToMessage(sio::object_message::ptr msg, std::shared_ptr<BridgeConfig> config) {
     // enabled input drivers
     auto input_drivers = sio::array_message::create();
     for (auto & one : config->input_drivers) {
         input_drivers->get_vector().push_back(sio::string_message::create(one));
     }
     msg->get_map().emplace("input_drivers", input_drivers);

     // default input mappings json
     msg->get_map().emplace("input_defaults", BridgeSocket::JsonToSioMessage(config->input_defaults));
     
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
     msg->get_map().emplace("service_defaults", BridgeSocket::JsonToSioMessage(config->service_defaults));

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
             one_msg->get_map().emplace("data", BridgeSocket::JsonToSioMessage(one.data));
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