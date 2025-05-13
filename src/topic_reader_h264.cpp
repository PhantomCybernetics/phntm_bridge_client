#include "phntm_bridge/const.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/status_leds.hpp"
#include "phntm_bridge/topic_reader_h264.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "rtc/peerconnection.hpp"
#include "rtc/track.hpp"
#include <ffmpeg_image_transport_msgs/msg/detail/ffmpeg_packet__struct.hpp>
#include <stdexcept>
#include <string>
#include <chrono>
#include <iostream>

namespace phntm {

    std::map<std::string, std::shared_ptr<TopicReaderH264>> TopicReaderH264::readers;
    std::mutex TopicReaderH264::readers_mutex;

    std::shared_ptr<TopicReaderH264> TopicReaderH264::getForTopic(std::string topic, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos, int debug_num_frames) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            return readers.at(topic);
        } else { // create subscriber
            auto tr = std::make_shared<TopicReaderH264>(topic, bridge_node, qos, debug_num_frames);
            readers.emplace(topic, tr);
            return tr;
        }
    }

    std::shared_ptr<TopicReaderH264> TopicReaderH264::getForTopic(std::string topic) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            return readers.at(topic);
        } else {
            return nullptr;
        }
    }

    void TopicReaderH264::destroy(std::string topic) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            log(GRAY + "Destroying reader for " + topic + CLR);
            readers.erase(topic);
        }
    }

    TopicReaderH264::TopicReaderH264(std::string topic, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos, int debug_num_frames)
        : topic(topic), bridge_node(bridge_node), qos(qos), debug_num_frames(debug_num_frames) {
    }

    TopicReaderH264::~TopicReaderH264() {
        this->stop();
    }

    bool TopicReaderH264::addOutput(std::shared_ptr<WRTCPeer::MediaTrackInfo> track_info, std::shared_ptr<rtc::PeerConnection> pc) {
        std::lock_guard<std::mutex> lock(this->outputs_mutex);
        auto pos = std::find_if(
            this->outputs.begin(),
            this->outputs.end(),
            [&](const std::shared_ptr<Output> output) {
                return output->track_info->track.get() == track_info->track.get();
            }
        );
        if (pos != this->outputs.end())
            return false;

        auto output = std::make_shared<Output>();
        output->track_info = track_info;
        output->pc = pc;
        output->ssrc = track_info->track->description().getSSRCs()[0];
        this->outputs.push_back(output);
        this->start();

        return true;
    }

    void TopicReaderH264::onPCSignalingStateChange(std::shared_ptr<rtc::PeerConnection> pc) {
        if (pc->signalingState() != rtc::PeerConnection::SignalingState::Stable)
            return;
            
        for (auto & tr : TopicReaderH264::readers) {
            for (auto & output : tr.second->outputs) {
                if (output->pc.get() == pc.get()) {
                    if (!output->init_complete) {
                        output->init_complete = true;
                        log(GRAY + "Init complete for track #" + std::to_string(output->ssrc) + CLR);
                    }
                    break;
                }
            }
        }
    }

    bool TopicReaderH264::removeOutput(std::shared_ptr<WRTCPeer::MediaTrackInfo> track_info) {
        std::lock_guard<std::mutex> lock(this->outputs_mutex);
        auto pos = std::find_if(
            this->outputs.begin(),
            this->outputs.end(),
            [&](const std::shared_ptr<Output> output) {
                return output->track_info->track.get() == track_info->track.get();
            }
        );
        if (pos != this->outputs.end()) {
            auto o = *pos;
            o->active = false;
            this->outputs.erase(pos);    
        } else {
            log("Track not found in " + this->topic + " reader");
        }
        if (this->outputs.size() == 0) {
            this->stop();
            return true; // good to destoy
        }
        return false;
    }

    std::vector<std::vector<uint8_t>> splitNalUnits(const uint8_t* data, size_t size) {
        std::vector<std::vector<uint8_t>> nalUnits;
        size_t start = 0;
        
        // Find start codes (0x00000001 or 0x000001)
        for(size_t i = 0; i < size; ++i) {
            if((i >= 3 && data[i] == 0x01 && data[i-1] == 0x00 && data[i-2] == 0x00 && data[i-3] == 0x00) ||
               (i >= 2 && data[i] == 0x01 && data[i-1] == 0x00 && data[i-2] == 0x00)) {
                if(start > 0) {
                    // Skip start code (4 or 3 bytes)
                    size_t nalStart = (data[i-3] == 0x00) ? i-3 : i-2;
                    nalUnits.emplace_back(data + start, data + nalStart);
                }
                start = i + 1;
            }
        }
        
        if(start < size) {
            nalUnits.emplace_back(data + start, data + size);
        }
        
        return nalUnits;
    }

    bool hasSpsPps(const std::vector<std::vector<uint8_t>>& nalUnits) {
        for(const auto& unit : nalUnits) {
            if(!unit.empty()) {
                uint8_t nalType = unit[0] & 0x1F;
                if(nalType == 7 || nalType == 8) return true; // 7=SPS, 8=PPS
            }
        }
        return false;
    }

    void logNalUnit(const uint8_t* data, size_t size, std::string topic) {
        if(size < 1) return;
        
        uint8_t nalType = data[0] & 0x1F;
        log("[" + topic + "] " +  "NAL Unit Type: " + std::to_string((int)nalType)
            + ", Size: " + std::to_string(size)
            + ", First bytes: "
            + toHex((int)data[0]) + " "
            + toHex((int)data[1]) + " "
            + toHex((int)data[2])
        );
    }

    uint32_t convertToRtpTimestamp(uint64_t sec, uint64_t nanosec) {
        // Convert to nanoseconds first to avoid floating-point precision loss
        constexpr uint64_t NS_PER_SEC = 1'000'000'000ULL;
        constexpr uint64_t CLOCK_RATE = 90'000ULL; // 90kHz
    
        uint64_t total_ns = static_cast<uint64_t>(sec) * NS_PER_SEC + nanosec;
        uint64_t rtpTimestamp = (total_ns * CLOCK_RATE) / NS_PER_SEC;
    
        // Wrap around if necessary (32-bit)
        return static_cast<uint32_t>(rtpTimestamp);
    }

    void TopicReaderH264::onFrame(std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg) {

        // this->latest_payload_size = msg->data.size();
        // this->latest_payload.resize(this->latest_payload_size);
        // std::memcpy(this->latest_payload.data(), msg->data.data(), this->latest_payload_size);

        // auto ts = convertToRtpTimestamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
        // auto ts = msg->pts;
        auto now = std::chrono::system_clock::now();
        auto duration_since_epoch = now.time_since_epoch();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
        auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch - sec);
        auto ts = convertToRtpTimestamp(sec.count(), nanosec.count());

        if (this->debug_num_frames > 0) {
            auto is_keyframe = msg->flags == 1;
            auto nal_units = splitNalUnits(msg->data.data(), msg->data.size());
            auto has_sps_pps = hasSpsPps(nal_units);
            log("[" + this->topic + "] " + std::string(is_keyframe ? CYAN + "Keyframe" + CLR : "Frame") + " has "+std::to_string(nal_units.size())+" nal units "+(has_sps_pps?" HAS SPS/PPS":"")+"; ts=" + std::to_string(ts));
            for (const auto & nal_unit: nal_units) {
                logNalUnit(nal_unit.data(), nal_unit.size(), this->topic);
            }
            --this->debug_num_frames;
        }

        if (!this->logged_receiving) {
            log(MAGENTA + "Receiving " + std::to_string(msg->width) + "x" + std::to_string(msg->height)+ " " + msg->encoding + " frames from " + this->topic + " " + std::to_string(this->latest_payload_size) + " B" + CLR);
            this->logged_receiving = true;
        }

        auto msg_sent = false;
        std::lock_guard<std::mutex> lock(this->outputs_mutex);
        for (auto output : this->outputs) {
            if (!output->track_info->track->isOpen()) {
                if (!output->logged_closed) {
                    output->logged_closed = true;
                    log(GRAY + "Track #" + std::to_string(output->ssrc) + " is closed for " + this->topic + CLR);
                }
                continue;
            }
            if (!output->init_complete) {
                if (!output->logged_init_incomplete) {
                    output->logged_init_incomplete = true;
                    log(GRAY + "Track #" + std::to_string(output->ssrc) + " not sending msgs yet for " + this->topic + " (init incomplete)" + CLR);
                }
                continue;
            }

            // uint32_t ts;
            // if (!output->ts_base) {
            //     if (!is_keyframe)
            //         continue;
            //     output->ts_base = ts;
            //     // ts = 0; 
            //     output->track_info->rtpConfig->timestamp = output->track_info->rtpConfig->startTimestamp = ts;
            //     log("Track #" + std::to_string(output->ssrc) + " initial ts set to " + std::to_string(ts));
            // }
            // } else {
            //     ts = ts_src - output->ts_base;
            // }


            try {
                
                output->track_info->track->sendFrame(reinterpret_cast<const std::byte*>(msg->data.data()), msg->data.size(), ts);

                output->num_sent++;
                msg_sent = true; // flash LED

                // reset
                output->logged_closed = false;
                output->logged_init_incomplete = false;
                output->logged_error = false;
                output->logged_exception = false;
                // }
            } catch(const std::runtime_error & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->bridge_node->get_logger(), "Error sending %s into track #%i: %s", this->topic.c_str(), output->ssrc, ex.what());
                }
            } catch(const std::invalid_argument & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->bridge_node->get_logger(), "Error sending %s into track #%i: %s", this->topic.c_str(), output->ssrc, ex.what());
                }
            }
        }

        if (msg_sent)
            DataLED::once();
    }

    void TopicReaderH264::start() {
        if (this->sub != nullptr)
            return;
        try {
            this->sub = this->bridge_node->create_subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(this->topic, this->qos, std::bind(&TopicReaderH264::onFrame, this, std::placeholders::_1));
            log(GREEN + "[" + this->topic + "] Created subscriber" + CLR);
        } catch(const std::runtime_error & ex) {
            this->sub = nullptr;
            log("Error creating subscriber for " + this->topic + " {"+ VIDEO_STREAM_MSG_TYPE +"}: " + ex.what(), true);
        }
    }

    void TopicReaderH264::stop() {
        if (this->sub == nullptr)
            return; // already stopped
        
        this->sub.reset(); // removes sub
        this->sub = nullptr;
        this->logged_receiving = false;
        log(BLUE + "[" + this->topic + "] Removed subscriber" + CLR);
    }

}