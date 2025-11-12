#pragma once
#include <cstdio>
#include <vector>
#include <string>

#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/config.hpp"

#include <rtc/datachannel.hpp>
#include <rtc/rtc.hpp>

#include "phntm_bridge/topic_reader_data.hpp"
#include "phntm_bridge/topic_writer_data.hpp"
#include "phntm_bridge/topic_reader_h264.hpp"

namespace phntm {

    class WRTCPeer : public std::enable_shared_from_this<WRTCPeer> {

        public: 
            WRTCPeer(std::shared_ptr<PhntmBridge> node, std::string id_peer, std::string id_app, std::string id_instance, std::string session, std::shared_ptr<BridgeConfig> config);
            ~WRTCPeer();
            static std::string getId(sio::object_message::ptr data);
            static bool isConnected(std::string id_peer);
            static std::shared_ptr<WRTCPeer> getConnectedPeer(sio::event const & ev);
            static void onPeerConnected(std::shared_ptr<PhntmBridge> node, std::string id_peer, sio::event &ev, std::shared_ptr<BridgeConfig> config);
            static void onAllPeersDisconnected();
            static bool anyPeersConnected();

            void createPeerConnection();
            void removePeerConnection();
            void onWRTCInfo(sio::object_message::ptr msg);
            void onDisconnected();
            std::string toString();

            // std::shared_ptr<WRTCPeer> shared_ptr;
            void processSubscriptions(int ack_msg_id = -1, sio::object_message::ptr ack = nullptr);
            void processSubscriptionsSync(int ack_msg_id = -1, sio::object_message::ptr ack = nullptr);
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

            std::shared_ptr<rtc::PeerConnection> getPC() { return  this->pc ;};
            uint nextChannelId() { return  ++this->next_channel_id; };

            std::map<std::string, std::shared_ptr<rtc::DataChannel>> outbound_data_channels; 
            std::map<std::string, std::shared_ptr<rtc::DataChannel>> inbound_data_channels; 
            std::map<std::string, std::shared_ptr<TopicReaderH264::MediaTrackInfo>> outbound_media_tracks;

            uint64_t connectedRTPTimeBase() { return this->connected_rtp_time_base; };
            std::mutex processing_subscriptions_mutex;

        private:
            static std::map<std::string, std::shared_ptr<WRTCPeer>> connected_peers;
            void addUIConfigToMessage(sio::object_message::ptr msg, std::shared_ptr<BridgeConfig> config);

            bool awaiting_peer_reply;

            std::string id;
            std::string id_app;
            std::string id_instance;
            std::string session;
            bool is_connected;
            uint64_t connected_rtp_time_base = 0;
            std::shared_ptr<PhntmBridge> node;

            std::shared_ptr<BridgeConfig> config;

            std::shared_ptr<rtc::PeerConnection> pc;

            std::vector<std::string> req_read_subs; // topic ids to subscribe
            std::vector<std::vector<std::string>> req_write_subs; // // [topic_id, msg_type]'s to write to

            sio::array_message::ptr subscribeReadDataTopic(std::string topic, std::string msg_type);
            sio::array_message::ptr subscribeMediaTopic(std::string topic, std::string msg_type);
            sio::array_message::ptr subscribeWriteDataTopic(std::string topic, std::string msg_type);

            sio::array_message::ptr unsubscribeReadDataTopic(std::string topic);
            sio::array_message::ptr unsubscribeWriteDataTopic(std::string topic);
            sio::array_message::ptr unsubscribeMediaTopic(std::string topic, bool close_channel);

            uint16_t openDataChannelForTopic(std::string topic, std::string msg_type, bool is_reliable, bool write=false);
            void closeDataChannelForTopic(std::string topic, bool write);
            // std::string openMediaTrackForTopic(std::string topic);
            // void closeMediaTrackForTopic(std::string topic);
            
            uint16_t next_channel_id;
            bool negotiation_needed;
            bool peer_needs_restart;
    };

}