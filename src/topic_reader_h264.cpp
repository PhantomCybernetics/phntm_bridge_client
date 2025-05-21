#include "message.hpp"
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/status_leds.hpp"
#include "phntm_bridge/topic_reader_h264.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "rtc/common.hpp"
#include "rtc/peerconnection.hpp"
#include "rtc/track.hpp"
#include <cstdint>
#include <exception>
#include <ffmpeg_image_transport_msgs/msg/detail/ffmpeg_packet__struct.hpp>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <chrono>
#include <iostream>
#include <thread>
#include <arpa/inet.h>

namespace phntm {

    std::map<std::string, std::shared_ptr<TopicReaderH264>> TopicReaderH264::readers;
    std::mutex TopicReaderH264::readers_mutex;

    std::shared_ptr<TopicReaderH264> TopicReaderH264::getForTopic(std::string topic, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos, bool use_pts, bool debug_verbose, int debug_num_frames) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            return readers.at(topic);
        } else { // create subscriber
            auto tr = std::make_shared<TopicReaderH264>(topic, bridge_node, qos, use_pts, debug_verbose, debug_num_frames);
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

    TopicReaderH264::TopicReaderH264(std::string topic, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos, bool use_pts, bool debug_verbose, int debug_num_frames)
        : topic(topic), bridge_node(bridge_node), qos(qos), use_pts(use_pts), debug_verbose(debug_verbose), debug_num_frames(debug_num_frames) {
    }

    bool TopicReaderH264::addOutput(std::shared_ptr<MediaTrackInfo> track_info, std::shared_ptr<rtc::PeerConnection> pc) {
        // std::lock_guard<std::mutex> lock(this->outputs_mutex);

        this->start(); // worker running

        {
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
            output->thread = std::thread(&TopicReaderH264::outputFramesWorker, this, output);
            output->thread.detach();
            this->outputs.push_back(output);
        }

        return true;
    }

    void TopicReaderH264::onPCSignalingStateChange(std::shared_ptr<WRTCPeer> peer) {
        if (peer->getPC()->signalingState() != rtc::PeerConnection::SignalingState::Stable)
            return;
            
        for (auto & track_info : peer->outbound_media_tracks) {
            if (!track_info.second->init_complete) {
                track_info.second->init_complete = true;
                log(GRAY + "Init complete for track #" + std::to_string(track_info.second->ssrc) + CLR);
            }
        }
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


    // std::vector<rtc::NalUnit> splitFrame(const unsigned char* data, size_t dataSize, rtc::NalUnit::Separator separator) {
   
    // }

    // std::vector<rtc::binary> fragment(unsigned char* data, size_t dataSize, 
    //                                 size_t maxFragmentSize, 
    //                                 rtc::NalUnit::Separator separator) {
    //     // Directly process raw data without converting to vector<std::byte>
    //     return rtc::NalUnit::GenerateFragments(splitFrame(data, dataSize, separator), maxFragmentSize);
    // }

    uint32_t convertToRtpTimestamp(int32_t sec, uint32_t nanosec) {
        // Convert to nanoseconds first to avoid floating-point precision loss
        constexpr uint64_t NS_PER_SEC = 1'000'000'000ULL;
        constexpr uint64_t CLOCK_RATE = 90'000ULL; // 90kHz
    
        uint64_t total_ns = static_cast<uint64_t>(sec) * NS_PER_SEC + nanosec;
        uint64_t rtpTimestamp = (total_ns * CLOCK_RATE) / NS_PER_SEC;
    
        // Wrap around if necessary (32-bit)
        return static_cast<uint32_t>(rtpTimestamp);
    }

    void TopicReaderH264::onFrame(const std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg) {
        std::lock_guard<std::mutex> lock(this->subscriber_mutex); // this will prevent ros from calling all subs on the same thread
        if (!this->worker_running)
             return;
        //std::lock_guard<std::mutex> lock(this->frame_queue_mutex); 
        //this->latest_payload_size = msg->data.size();
        //this->latest_payload.resize(this->latest_payload_size);
        //std::memcpy(this->latest_payload.data(), msg->data.data(), this->latest_payload_size);
        //log("received sec=" + std::to_string(msg->header.stamp.sec) + " nanosec=" + std::to_string(msg->header.stamp.nanosec));
        // this->frame_queue.push(std::move(msg));
        // this->frame_queue_cv.notify_one(); // Notify one waiting thread

        this->processFrame(msg); // TODO not using the worker queue here
    }

    void TopicReaderH264::processFrame(std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg) {
        
        auto is_keyframe = msg->flags == 1;
        uint ts;
        if (this->use_pts) {
            ts = msg->pts; // this must be in 1/90000 increments
        } else {
            ts = convertToRtpTimestamp(msg->header.stamp.sec, msg->header.stamp.nanosec); // convert message timestamp to 1/90000 increments
        }

        if (!this->logged_receiving) {
            log(MAGENTA + "[" + getThreadId() + "] Receiving " + std::to_string(msg->width) + "x" + std::to_string(msg->height)+ " " + msg->encoding + " frames from " + this->topic + " " + std::to_string(this->latest_payload_size) + " B" + CLR);
            this->logged_receiving = true;
        }

        if (this->debug_num_frames > 0) {
            auto nal_units = splitNalUnits(msg->data.data(), msg->data.size());
            auto has_sps_pps = hasSpsPps(nal_units);
            log("[" + getThreadId() + "]["+ this->topic + "] " + std::string(is_keyframe ? CYAN + "Keyframe" + CLR : "Frame") + " has "+std::to_string(nal_units.size())+" nal units "+(has_sps_pps?" HAS SPS/PPS":"")+"; ts=" + std::to_string(ts));
            for (const auto & nal_unit: nal_units) {
                logNalUnit(nal_unit.data(), nal_unit.size(), this->topic);
            }
            --this->debug_num_frames;
        }

        // std::shared_ptr<rtc::FrameInfo> info = std::make_shared<rtc::FrameInfo>(ts);
        // auto bytes_data = reinterpret_cast<const std::byte*>(msg->data.data());
        // auto msg_binary = std::make_shared<rtc::binary>(bytes_data, bytes_data + msg->data.size());
        // // auto fragments = std::make_shared<std::vector<rtc::binary>>(this->outputs[0]->track_info->packetizer->fragment(msg_binary));

        // auto message = rtc::make_message(msg_binary, info);
        
        auto msg_sent = false;
        {
            std::lock_guard<std::mutex> lock(this->outputs_mutex);
            for (auto output : this->outputs) {

                if (output->num_sent == 0 && !is_keyframe) {
                    continue;
                }

                if (!output->track_info->init_complete) {
                    if (!output->logged_init_incomplete) {
                        output->logged_init_incomplete = true;
                        log(GRAY + "[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " not sending msgs yet for " + this->topic + " (init incomplete)" + CLR);
                    }
                    continue;
                }

                if (!output->track_info->track->isOpen()) {
                    if (!output->logged_closed) {
                        output->logged_closed = true;
                        log(GRAY + "[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " is closed for " + this->topic + CLR);
                    }
                    continue;
                }
                
                // lock.unlock();
                // if (output->ts_base == 0) {
                //     output->ts_base = ts;
                //     output->track_info->rtpConfig->timestamp = output->track_info->rtpConfig->startTimestamp = ts;
                //     log("Track #" + std::to_string(output->ssrc) + " initial ts set to " + std::to_string(ts)
                //         + " sec=" + std::to_string(msg->header.stamp.sec) + " nanosec=" + std::to_string(msg->header.stamp.nanosec));
                // }

                {
                    if (this->debug_verbose)
                        log(GRAY + "[" + getThreadId() + "] Pushing one of " + this->topic + " to output queue for track #" + std::to_string(output->track_info->ssrc) + CLR);
                    // std::lock_guard<std::mutex> ouput_lock(output->queue_mutex); 
                    OutputMsg frame { msg, ts, is_keyframe };
                    output->queue.push(frame);
                    output->queue_cv.notify_one(); // Notify one waiting thread
                    msg_sent = true; // flash LED
                }

                // lock.lock();

                // try {                
                //     // this->latest_payload_size = msg->data.size();
                //     // this->latest_payload.resize(this->latest_payload_size);
                //     // std::memcpy(this->latest_payload.data(), msg->data.data(), this->latest_payload_size);
                //     //log("received sec=" + std::to_string(msg->header.stamp.sec) + " nanosec=" + std::to_string(msg->header.stamp.nanosec));

                //     //output->track_info->track->sendFrame(reinterpret_cast<const std::byte*>(msg->data.data()), msg->data.size(), ts);
                //     // output->track_info->track->sendPrepacketizedFrame(fragments, info);
                //     // output->track_info->track->send(*message);

                //     output->num_sent++;
                //     msg_sent = true; // flash LED

                //     // reset
                //     output->logged_closed = false;
                //     output->logged_init_incomplete = false;
                //     output->logged_error = false;
                //     output->logged_exception = false;
                //     // }
                // } catch(const std::runtime_error & ex) {
                //     if (!output->logged_exception) {
                //         output->logged_exception = true;
                //         RCLCPP_ERROR(this->bridge_node->get_logger(), "Error sending %s into track #%i: %s", this->topic.c_str(), output->ssrc, ex.what());
                //     }
                // } catch(const std::invalid_argument & ex) {
                //     if (!output->logged_exception) {
                //         output->logged_exception = true;
                //         RCLCPP_ERROR(this->bridge_node->get_logger(), "Error sending %s into track #%i: %s", this->topic.c_str(), output->ssrc, ex.what());
                //     }
                // }
            }
        }

        if (msg_sent)
            DataLED::once();
    }

    void TopicReaderH264::processFramesWorker() {
        while (this->worker_running) {
            std::unique_lock<std::mutex> lock(this->frame_queue_mutex);
            this->frame_queue_cv.wait(lock, [this] { return !this->frame_queue.empty() || !this->worker_running; });

            while (!this->frame_queue.empty()) {
                auto msg = std::move(this->frame_queue.front());
                this->frame_queue.pop();
                lock.unlock(); // release lock during processing

                this->processFrame(msg);

                lock.lock();   // reacquire lock before next check
            }
        }
    }

    void TopicReaderH264::outputFramesWorker(std::shared_ptr<Output> output) {
        
        log(GRAY + "[" + getThreadId() + "] Output worker started for track " + output->track_info->id_track + CLR);
        while (this->worker_running && output->active) {

            std::unique_lock<std::mutex> lock(output->queue_mutex);
            // if (this->debug_verbose)
            //     log("Track #" + std::to_string(output->track_info->ssrc) + " waiting for queue signal"); 
            output->queue_cv.wait(lock, [this, output] { return !output->queue.empty() || !this->worker_running || !output->active; });

            OutputMsg frame;
            while (!output->queue.empty()) {
                frame = std::move(output->queue.front());
                output->queue.pop();

                if (frame.is_keyframe)
                    break; // don't skip keyframes

                // lock.unlock(); // release lock during processing

                // this->processFrame(msg);

                // lock.lock();   // reacquire lock before next check
            }
            lock.unlock();

            if (!output->active || !this->worker_running) {
                log("[" + getThreadId() + "] Output closed for " + this->topic + " track #" + std::to_string(output->track_info->ssrc));
                return;
            }

            if (output->ts_base == 0) {
                output->ts_base = frame.ts;
                output->track_info->rtpConfig->timestamp = output->track_info->rtpConfig->startTimestamp = frame.ts;
                log("[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " initial ts set to " + std::to_string(frame.ts)
                    + " sec=" + std::to_string(frame.msg->header.stamp.sec) + " nanosec=" + std::to_string(frame.msg->header.stamp.nanosec));
            }

            try {                
                // this->latest_payload_size = msg->data.size();
                // this->latest_payload.resize(this->latest_payload_size);
                // std::memcpy(this->latest_payload.data(), msg->data.data(), this->latest_payload_size);
                if (this->debug_verbose)
                    log("[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " sending " + (frame.is_keyframe ? CYAN + "KEYFRAME" + CLR : "frame") + " w ts=" + std::to_string(frame.ts)+" sec=" + std::to_string(frame.msg->header.stamp.sec) + " nanosec=" + std::to_string(frame.msg->header.stamp.nanosec));

                output->track_info->track->sendFrame(reinterpret_cast<const std::byte*>(frame.msg->data.data()), frame.msg->data.size(), frame.ts);
                // output->track_info->track->sendPrepacketizedFrame(fragments, info);
                // output->track_info->track->send(*message);

                output->num_sent++;
                //msg_sent = true; // flash LED

                // reset
                output->logged_closed = false;
                output->logged_init_incomplete = false;
                output->logged_error = false;
                output->logged_exception = false;

                output->num_sent++;

                // reset
                output->logged_closed = false;
                output->logged_init_incomplete = false;
                output->logged_error = false;
                output->logged_exception = false;
                
                // }
            } catch(const std::runtime_error & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->bridge_node->get_logger(), "Error sending %s into track #%i: %s", this->topic.c_str(), output->track_info->ssrc, ex.what());
                }
            } catch(const std::invalid_argument & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->bridge_node->get_logger(), "Error sending %s into track #%i: %s", this->topic.c_str(), output->track_info->ssrc, ex.what());
                }
            } catch(const std::exception & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->bridge_node->get_logger(), "Exception while sending %s into track #%i: %s", this->topic.c_str(), output->track_info->ssrc, ex.what());
                }
            }
        }
        log(BLUE + "[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " frames worker finished" /*worker_running=" + std::to_string(this->worker_running) + ", " */ + "output.active=" + std::to_string(output->active) + CLR); 
    }

    void TopicReaderH264::start() {
        if (this->sub != nullptr)
            return; //already running

        try {
            this->callback_group = this->bridge_node->create_callback_group(
                rclcpp::CallbackGroupType::Reentrant
            ); // one group per topic to allow multithreaed subscription
            auto options = rclcpp::SubscriptionOptions();
            // options.callback_group = this->bridge_node->media_reentrant_group;
            options.callback_group = this->callback_group; // each subscription in its own group
            this->sub = this->bridge_node->create_subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
                this->topic,
                this->qos,
                std::bind(&TopicReaderH264::onFrame, this, std::placeholders::_1),
                options
            );
            log(GREEN + "[" + this->topic + "] Created subscriber" + CLR);

            this->worker_running = true;
            this->processor_thread = std::thread(&TopicReaderH264::processFramesWorker, this);
            this->processor_thread.detach();

        } catch(const std::runtime_error & ex) {
            this->sub = nullptr;
            log("Error creating subscriber for " + this->topic + " {"+ VIDEO_STREAM_MSG_TYPE +"}: " + ex.what(), true);
        }
    }

    bool TopicReaderH264::removeOutput(std::shared_ptr<MediaTrackInfo> track_info) {
        std::lock_guard<std::mutex> lock(this->outputs_mutex);
        if (this->outputs.size() == 0) {
            return true;
        }
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
            o->queue_cv.notify_one();
            this->outputs.erase(pos);    
        } else {
            log("Track not found in " + this->topic + " reader");
        }
        if (this->outputs.size() == 0) {
            return true; // good to destoy
        }
        return false;
    }

    void TopicReaderH264::destroy(std::string topic) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            log(GRAY + "Destroying reader for " + topic + CLR);
            readers.erase(topic);
        }
    }

    void TopicReaderH264::stop() {
        this->worker_running = false; //kills worker thread
        {
            std::lock_guard<std::mutex> lock(this->outputs_mutex);
            for (auto & output : this->outputs) {
                output->active = false; //kill worker
                output->queue_cv.notify_one();
            }
            this->outputs.clear();
        }
        
        this->frame_queue_cv.notify_one();

        if (this->sub == nullptr) {
            return; // already stopped
        }
            
        if (!rclcpp::ok()) {
            return;
        }
        try {
            this->sub.reset(); // removes sub
            this->sub = nullptr;
            this->logged_receiving = false;
            log(BLUE + "[" + this->topic + "] Removed subscriber" + CLR);
        } catch (const std::exception & ex) {
            log("Exception closing media subscriber: " + std::string(ex.what()), true);
        }
    }

     TopicReaderH264::~TopicReaderH264() {
        this->stop();
    }


    std::string TopicReaderH264::openMediaTrackForTopic(std::string topic, std::shared_ptr<WRTCPeer> peer) {

        rtc::SSRC ssrc = peer->nextChannelId();
        auto payload_type = 96;
        auto cname = topic;
        auto msid = topic;
        auto id_track = topic;
        auto mid = generateId(16); ///"stream-" + std::to_string(ssrc); // must be unique per stream and < 16 chars in order to open a new stream
                                                        // when adding audio, it should use the same mid to be added to the same stream (?!)
        rtc::Description::Video media(mid, rtc::Description::Direction::SendOnly);
		media.addH264Codec(payload_type); // Must match the payload type of the external h264 RTP stream
		media.addSSRC(ssrc, cname, msid, id_track); //track id is used to identify streams by the client
		auto track = peer->getPC()->addTrack(media);

        // create RTP configuration
        auto rtpConfig = std::make_shared<rtc::RtpPacketizationConfig>(ssrc, cname, payload_type, rtc::H264RtpPacketizer::ClockRate);
        // create packetizer
        auto packetizer = std::make_shared<H264PrePacketizer>(rtc::NalUnit::Separator::StartSequence, rtpConfig);
        // add RTCP SR handler
        auto srReporter = std::make_shared<rtc::RtcpSrReporter>(rtpConfig);
        packetizer->addToChain(srReporter);
        // add RTCP NACK handler
        auto nackResponder = std::make_shared<rtc::RtcpNackResponder>();
        packetizer->addToChain(nackResponder);
        // set handler
        track->setMediaHandler(packetizer);

        track->onOpen([peer, topic](){
            log(GREEN + peer->toString() + "Media track open for " + topic + CLR);
        });

        track->onClosed([peer, topic](){
            log(BLUE + peer->toString() + "Media track closed for "+topic + CLR);
        });

        auto track_info = std::make_shared<MediaTrackInfo>(MediaTrackInfo {
            track,
            id_track,
            ssrc,
            true,  // in use
            false, // init incomplete
            rtpConfig,
            packetizer
        });
        peer->outbound_media_tracks.emplace(topic, track_info);

        return id_track;
    }

     void TopicReaderH264::closeMediaTrackForTopic(std::string topic, std::shared_ptr<WRTCPeer> peer) {
        log(GRAY + peer->toString() + "Closing media track for " + topic + CLR);
        if (peer->outbound_media_tracks.at(topic)->track->isOpen()) {
            peer->outbound_media_tracks.at(topic)->track->close();
        }
        peer->outbound_media_tracks.erase(topic);
    }

}