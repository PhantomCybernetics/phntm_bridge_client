//#include "h264rtppacketizer.hpp"
#include "phntm_bridge/ffmpeg_encoder.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/introspection.hpp"
#include "rtc/message.hpp"
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
#include <libavutil/pixfmt.h>
#include <memory>
#include <mutex>
#include <opencv2/core/hal/interface.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <chrono>
#include <iostream>
#include <thread>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>

namespace phntm {

    std::map<std::string, std::shared_ptr<TopicReaderH264>> TopicReaderH264::readers;
    std::mutex TopicReaderH264::readers_mutex;

    std::shared_ptr<TopicReaderH264> TopicReaderH264::getForTopic(std::string topic, std::string msg_type, rclcpp::QoS qos, std::shared_ptr<rclcpp::Node> node, rclcpp::CallbackGroup::SharedPtr callback_group, sio::message::ptr topic_conf) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            return readers.at(topic);
        } else { // create subscriber
            auto tr = std::make_shared<TopicReaderH264>(topic, msg_type, qos, node, callback_group, topic_conf);
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

    TopicReaderH264::TopicReaderH264(std::string topic, std::string msg_type, rclcpp::QoS qos, std::shared_ptr<rclcpp::Node> node, rclcpp::CallbackGroup::SharedPtr callback_group, sio::message::ptr topic_conf)
        : topic(topic), msg_type(msg_type), qos(qos), node(node), callback_group(callback_group) {
        
        this->pts_source = topic_conf->get_map().at("pts_source")->get_int();
        this->debug_verbose = topic_conf->get_map().at("debug_verbose")->get_bool();
        this->debug_num_frames = topic_conf->get_map().at("debug_num_frames")->get_int();

        if (isImageType(msg_type)) {
            this->colormap = topic_conf->get_map().at("colormap")->get_int();
            this->max_sensor_value = topic_conf->get_map().at("max_sensor_value")->get_double();
            this->encoder_hw_device = topic_conf->get_map().at("encoder_hw_device")->get_string();
            this->encoder_thread_count = topic_conf->get_map().at("encoder_thread_count")->get_int();
            this->encoder_gop_size = topic_conf->get_map().at("encoder_gop_size")->get_int();
            this->encoder_bit_rate = topic_conf->get_map().at("encoder_bit_rate")->get_int();
        }
        this->create_node = node == nullptr; //make a node if none provided
    }

    bool TopicReaderH264::addOutput(std::shared_ptr<MediaTrackInfo> track_info, std::shared_ptr<WRTCPeer> peer) {
        this->start(); // make sure sub exists and worker is running
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
            output->peer = peer;
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

    // on subscriber thread
    void TopicReaderH264::onEncodedFrame(const std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg) {
        // std::lock_guard<std::mutex> lock(this->subscriber_mutex); // this will prevent ros from calling all subs on the same thread
        if (!this->subscriber_running)
             return;

        auto is_keyframe = msg->flags == 1;
        uint ts;
        switch (this->pts_source) {
            case PTS_SOURCE_PACKET_PTS:
                ts = msg->pts; // this must be in 1/90000 increments
                break;
            case PTS_SOURCE_MESSAGE_HEADER:
                ts = convertToRtpTimestamp(msg->header.stamp.sec, msg->header.stamp.nanosec); // convert message timestamp to 1/90000 increments
                break;
            default: // set pts from local time
                std::chrono::steady_clock::duration timestamp = std::chrono::steady_clock::now().time_since_epoch();
                auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
                std::int32_t sec = static_cast<std::int32_t>(seconds.count());
                // Get remaining nanoseconds
                auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - seconds);
                std::uint32_t nsec = static_cast<std::uint32_t>(nanoseconds.count());
                ts = convertToRtpTimestamp(sec, nsec);
                break;  
        }

        if (!this->logged_receiving) {
            log(MAGENTA + "[" + getThreadId() + "] Receiving " + std::to_string(msg->width) + "x" + std::to_string(msg->height)+ " " + msg->encoding + " frames from " + this->topic + " " + std::to_string(msg->data.size()) + " B" + CLR);
            this->logged_receiving = true;
        }

        if (this->debug_num_frames > 0) {
            auto nal_units = splitNalUnits(msg->data.data(), msg->data.size());
            auto has_sps_pps = hasSpsPps(nal_units);
            log("[" + getThreadId() + "]["+ this->topic + "] " + std::string(is_keyframe ? CYAN + "Keyframe" + CLR : "Frame") + " has "+std::to_string(nal_units.size())+" nal units"+(has_sps_pps?(CYAN+" HAS SPS/PPS"+CLR):"")+"; ts=" + std::to_string(ts));
            for (const auto & nal_unit: nal_units) {
                logNalUnit(nal_unit.data(), nal_unit.size(), this->topic);
            }
        }

        // pre-packetize??
        // std::shared_ptr<rtc::FrameInfo> info = std::make_shared<rtc::FrameInfo>(ts);
        // auto bytes_data = reinterpret_cast<const std::byte*>(msg->data.data());
        // auto msg_binary = std::make_shared<rtc::binary>(bytes_data, bytes_data + msg->data.size());
        // // auto fragments = std::make_shared<std::vector<rtc::binary>>(this->outputs[0]->track_info->packetizer->fragment(msg_binary));

        // auto message = rtc::make_message(msg_binary, info);
        
        auto msg_sent = false;
        {
            std::unique_lock<std::mutex> outputs_lock(this->outputs_mutex);
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
                
                outputs_lock.unlock();

                // push to output queue without locking
                {
                    if (this->debug_verbose || this->debug_num_frames > 0)
                        log(GRAY + "[" + getThreadId() + "] Pushing one of " + this->topic + " to output queue for track #" + std::to_string(output->track_info->ssrc) + CLR);
                    // std::lock_guard<std::mutex> ouput_lock(output->queue_mutex); 
                    OutputMsg frame { msg, ts, is_keyframe };
                    output->in_queue.push(frame);
                    output->in_queue_cv.notify_one(); // notify one waiting thread
                    msg_sent = true; // flash LED
                }

                outputs_lock.lock();
            }
        }

        if (this->debug_num_frames > 0) {
            --this->debug_num_frames;
            if (this->debug_num_frames == 0) {
                log(GRAY + "Frame debugging finished for " + this->topic + "" + CLR);
            }
        }

        if (msg_sent)
            DataLED::once();
    }

    // on subscriber thread
    void TopicReaderH264::onImageFrame(const std::shared_ptr<sensor_msgs::msg::Image> im) {
        auto size = im->data.size();
        // log("Got Image " + std::to_string(size)+ "B for " + this->topic + "; encoding=" + im->encoding);

        if (!this->subscriber_running || size == 0 || this->encoder_error)
            return;

        auto debug_log = this->debug_num_frames > 0;

        if (debug_log) {
            log(this->topic + " Received Image frame w enc=" + im->encoding+"; sending to encoder");
        }

        // auto enc = strToLower();

        if (this->encoder.get() != nullptr && !this->encoder->checkCompatibility(im->width, im->height, im->encoding)) {
            RCLCPP_INFO(this->node->get_logger(), "Removing old encoder for %s {%s}, format change detected",
                        this->topic.c_str(), this->msg_type.c_str());
            this->encoder.reset(); // re-create encoder below
        }

        // make encoder
        if (this->encoder.get() == nullptr) {

            RCLCPP_INFO(this->node->get_logger(), "Making encoder %dx%d for %s {%s} with hw_device=%s",
                        im->width, im->height, this->topic.c_str(), this->msg_type.c_str(), encoder_hw_device.c_str());
            try {
                this->encoder = std::make_shared<FFmpegEncoder>(im->width, im->height, im->encoding,
                                                im->header.frame_id, this->topic, this->colormap, this->max_sensor_value,
                                                this->node,
                                                this->encoder_hw_device,
                                                this->encoder_thread_count,
                                                this->encoder_gop_size,
                                                this->encoder_bit_rate,
                                                std::bind(&TopicReaderH264::onEncodedFrame, this, std::placeholders::_1));
            } catch (const std::runtime_error & ex) {
                this->encoder.reset();
                this->encoder_error = true;
                this->debug_num_frames = 0;
                RCLCPP_ERROR(this->node->get_logger(), "%s", ex.what());
                return;
            }
        }

        if (this->encoder.get() == nullptr) {
            this->debug_num_frames = 0;
            RCLCPP_ERROR(this->node->get_logger(), "Encoder not ready for %s", this->topic.c_str());
            return;
        }

        try {
            this->encoder->encodeFrame(im); // sw-scales and sends to encoder on a separate thread
        } catch (const std::runtime_error & ex) {
            RCLCPP_ERROR(this->node->get_logger(), "Error encoding frame: %s", ex.what());
        }
    }

    // on subscriber thread
    void TopicReaderH264::onCompressedFrame(const std::shared_ptr<sensor_msgs::msg::CompressedImage> data) {
        log("Got CompressedImage for " + this->topic + "; format=" + data->format);
    }

    // void TopicReaderH264::onH264Encoded(std::shared_ptr<>) {
    //     log("Packet Encoded for " + this->topic + " " + std::to_string(size)+" B pts="+std::to_string(pts)+" flags="+std::to_string(flags));

        
    //     this->onEncodedFrame(frame);
    // }


    // on own thread
    void TopicReaderH264::outputFramesWorker(std::shared_ptr<Output> output) {
        
        log(GRAY + "[" + getThreadId() + "] Output worker started for track " + output->track_info->msid + CLR);
        while (this->subscriber_running && output->active) {

            std::unique_lock<std::mutex> queue_lock(output->in_queue_mutex);
            output->in_queue_cv.wait(queue_lock, [this, output] { return !output->in_queue.empty() || !this->subscriber_running || !output->active; });

            if (output->in_queue.empty() || !output->active || !this->subscriber_running) {
                log("[" + getThreadId() + "] Output closed for " + this->topic + " track #" + std::to_string(output->track_info->ssrc));
                break;
            }

            auto out_frame_msg = output->in_queue.front();
            output->in_queue.pop();
            queue_lock.unlock();

            auto debug_log = this->debug_num_frames > 0;

            if (out_frame_msg.ts <= output->last_raw_ts) {
                log(GRAY + "[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " Resetting TS" + CLR);
                output->ts_first = 0; // reset
            }
            output->last_raw_ts = out_frame_msg.ts;

            uint64_t pts_client;
            if (output->ts_first == 0) {
                output->ts_first = out_frame_msg.ts;
                output->ts_offset = getCurrentRtpTimestamp() - output->peer->connectedRTPTimeBase(); // synchrinize clocks
                // pts_client = output->ts_offset;
                // // pts_client = getCurrentRtpTimestamp();
                // pts_client -= ;
                // output->track_info->rtpConfig->timestamp = pts_client;
                if (!output->start_ts_set) {
                    //output->track_info->rtpConfig->timestamp = pts_client;
                    output->track_info->rtpConfig->startTimestamp = (uint32_t) output->peer->connectedRTPTimeBase();
                    output->start_ts_set = true;
                }
                if (this->debug_verbose || debug_log) {
                    log(GRAY + "[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " initial ts set to " + std::to_string(pts_client)
                        + " msg.sec=" + std::to_string(out_frame_msg.msg->header.stamp.sec) + " msg.nanosec=" + std::to_string(out_frame_msg.msg->header.stamp.nanosec) + CLR);
                }
            }

            pts_client = (out_frame_msg.ts - output->ts_first) + output->ts_offset;
            // pts_client = getCurrentRtpTimestamp();
            // pts_client -= output->peer->connectedRTPTimeBase();
                // pts_client = (uint) getCurrentRtpTimestamp();
            //}

            rtc::FrameInfo info { (uint32_t) pts_client };
            // info.isKeyframe = frame.is_keyframe;

            try {                
                // this->latest_payload_size = msg->data.size();
                // this->latest_payload.resize(this->latest_payload_size);
                // std::memcpy(this->latest_payload.data(), msg->data.data(), this->latest_payload_size);
                if (this->debug_verbose || debug_log)
                    log("[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " sending " + (out_frame_msg.is_keyframe ? CYAN + "KEYFRAME" + CLR : "frame") + " w pts_client=" + std::to_string(pts_client) + " orig " + std::to_string(out_frame_msg.ts)+" msg.sec=" + std::to_string(out_frame_msg.msg->header.stamp.sec) + " msg.nanosec=" + std::to_string(out_frame_msg.msg->header.stamp.nanosec));

                output->track_info->rtpConfig->timestamp = pts_client;
                // auto output_to_send = std::make_shared<SendMsg>(reinterpret_cast<std::byte*>(frame.msg->data.data()), frame.msg->data.size(), info);
                // {
                //     std::unique_lock<std::mutex> output_lock (output->output_to_send_mutex);
                //     output->output_to_send.push(output_to_send);
                // }
                // {
                //     std::unique_lock<std::mutex> peer_queue_lock(output->peer->h264_queue_mutex);
                //     output->peer->h264_queue.push(output);
                // }
                // output->peer->h264_queue_cv.notify_one(); // notify the waiting peer thread
                output->track_info->track->sendFrame(reinterpret_cast<const std::byte*>(out_frame_msg.msg->data.data()), out_frame_msg.msg->data.size(), info);
                // output->track_info->track->sendPrepacketizedFrame(fragments, info);
                // output->track_info->track->send(*message);

                output->num_sent++;
                //msg_sent = true; // flash LED

                // reset
                output->logged_closed = false;
                output->logged_init_incomplete = false;
                output->logged_error = false;
                output->logged_exception = false;
        
                // }
            } catch(const std::runtime_error & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->node->get_logger(), "Error sending %s into track #%i: %s", this->topic.c_str(), output->track_info->ssrc, ex.what());
                }
            } catch(const std::invalid_argument & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->node->get_logger(), "Error sending %s into track #%i: %s", this->topic.c_str(), output->track_info->ssrc, ex.what());
                }
            } catch(const std::exception & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->node->get_logger(), "Exception while sending %s into track #%i: %s", this->topic.c_str(), output->track_info->ssrc, ex.what());
                }
            }

            queue_lock.lock();
        }
        log(BLUE + "[" + getThreadId() + "] Track #" + std::to_string(output->track_info->ssrc) + " frames worker finished" /*worker_running=" + std::to_string(this->worker_running) + ", " */ + " output.active=" + std::to_string(output->active) + CLR); 
    }

    void TopicReaderH264::start() {
        std::lock_guard<std::mutex> lock(this->start_stop_mutex);
        std::lock_guard<std::mutex> introspection_lock(Introspection::mutex);
        if (this->subscriber_running)
            return;

        try {
            this->subscriber_running = true;

            if (this->create_node) {
                log(GREEN + "[" + this->topic + "] Creating node for "+ this->topic + CLR);
                uint highest_reader_no = 1;
                for (auto & r : readers) {
                    if (r.second->media_reader_node_no >= highest_reader_no) 
                        highest_reader_no = r.second->media_reader_node_no+1;
                }
                this->media_reader_node_no = highest_reader_no;
                this->node = std::make_shared<rclcpp::Node>("phntm_media_"+std::to_string(this->media_reader_node_no));

                log(GRAY + "[" + getThreadId() + "] Making executor " + this->topic + CLR);
                this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        
                this->executor->add_node(this->node);
            }

            // if (this->callback_group == nullptr) {
            //     this->callback_group = this->node->create_callback_group(
            //         rclcpp::CallbackGroupType::Reentrant
            //     );
            // }
            auto options = rclcpp::SubscriptionOptions();
            if (this->callback_group != nullptr && this->callback_group.get() != nullptr) {
                options.callback_group = this->callback_group;
            }

            if (isEncodedVideoType(this->msg_type)) {
                this->sub_enc = this->node->create_subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
                    this->topic,
                    this->qos,
                    std::bind(&TopicReaderH264::onEncodedFrame, this, std::placeholders::_1),
                    options
                );
            } else if (isImageType(this->msg_type)) {
                this->sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(
                    this->topic,
                    this->qos,
                    std::bind(&TopicReaderH264::onImageFrame, this, std::placeholders::_1),
                    options
                );
            } else if (isCompressedImageType(this->msg_type)) {
                this->sub_cmp = this->node->create_subscription<sensor_msgs::msg::CompressedImage>(
                    this->topic,
                    this->qos,
                    std::bind(&TopicReaderH264::onCompressedFrame, this, std::placeholders::_1),
                    options
                );
            } else {
                log("Invalid message type in H264 reader of " + this->topic+": " + this->msg_type, true);
                return;
            }

            log(GREEN + "[" + this->topic + "] Created subscriber" + CLR);
            
            if (this->create_node) { // spin in a new thread
                this->subscriber_thread = std::thread(&TopicReaderH264::spinSubscriber, this);
                this->subscriber_thread.detach();
            }

        } catch(const std::runtime_error & ex) {
            this->sub_enc.reset();
            this->sub_img.reset();
            this->sub_cmp.reset();
            log("Error creating subscriber for " + this->topic + " {"+ VIDEO_STREAM_MSG_TYPE +"}: " + ex.what(), true);
        }
    }

    void TopicReaderH264::spinSubscriber() { // when on its own node
        auto topic = this->topic;
        log(GRAY + "[" + getThreadId() + "] Subscriuber spinning for " + topic + CLR);

        while (rclcpp::ok() && this->subscriber_running) {
            std::lock_guard<std::mutex> lock(this->start_stop_mutex);
            if (this->executor.get() != nullptr)
                this->executor->spin_once(std::chrono::milliseconds(10));
        }

        log(BLUE + "[" + getThreadId() + "] Subscriber spinning finished for "+ topic + CLR);
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
            o->in_queue_cv.notify_one();
            // {
            //     std::lock_guard<std::mutex> output_lock (o->output_to_send_mutex);
            //     while (!o->output_to_send.empty()) {
            //         o->output_to_send.pop();
            //     }
            // }
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
            log(GRAY + "[destroy] Stopping reader for " + topic + CLR);
            readers[topic]->stop();
            log(GRAY + "Destroying reader for " + topic + CLR);
            readers.erase(topic);
            log(GRAY + "Destroyed reader " + topic + CLR);
        }
    }

    void TopicReaderH264::stop() {
        std::lock_guard<std::mutex> lock(this->start_stop_mutex);
        std::lock_guard<std::mutex> introspection_lock(Introspection::mutex);

        if (!this->subscriber_running)
            return;

        log(GRAY + "[stop] Stopping reader for " + this->topic + CLR);
        this->subscriber_running = false; 
  
        //     while (this->node.get() != nullptr) { // wait fo the thread to stop and destroy node
        //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //     }
        // }

        if (rclcpp::ok()) {
            if (this->sub_enc.get() != nullptr) {
                try {
                    this->sub_enc.reset(); // removes sub
                    log(BLUE + "[" + this->topic + "] Removed subscriber" + CLR);
                } catch (const std::exception & ex) {
                    log("Exception closing media subscriber: " + std::string(ex.what()), true);
                }    
            }
            if (this->sub_img.get() != nullptr) {
                try {
                    this->sub_img.reset(); // removes sub
                    log(BLUE + "[" + this->topic + "] Removed subscriber" + CLR);
                } catch (const std::exception & ex) {
                    log("Exception closing media subscriber: " + std::string(ex.what()), true);
                }    
            }
            if (this->sub_cmp.get() != nullptr) {
                try {
                    this->sub_cmp.reset(); // removes sub
                    log(BLUE + "[" + this->topic + "] Removed subscriber" + CLR);
                } catch (const std::exception & ex) {
                    log("Exception closing media subscriber: " + std::string(ex.what()), true);
                }    
            }
            log(GRAY + "[" + this->topic + "] Subs cleanup done" + CLR);
        }

        {
            log(GRAY + "[" + this->topic + "] Stopping outputs" + CLR);
            std::lock_guard<std::mutex> lock(this->outputs_mutex);
            for (auto & o : this->outputs) {
                o->active = false; //kill worker
                o->in_queue_cv.notify_one();
            }
            this->outputs.clear();
            log(GRAY + "[" + this->topic + "] Outputs clear" + CLR);
        }

        if (this->encoder.get() != nullptr) {
            log(GRAY + "[" + this->topic + "] Clearing encoder" + CLR);
            this->encoder.reset();
            log(GRAY + "[" + this->topic + "] Encoder clear" + CLR);
        }

        if (this->create_node) {
            log(GRAY + "[" + this->topic + "] Removing dedicated node from executor" + CLR);
            this->executor->remove_node(this->node);
        }
        // if (this->create_node) {
        log(GRAY + "[" + this->topic + "] Unreffing node" + CLR);
        this->node.reset();
        if (this->create_node) {
            log(GRAY + "[" + this->topic + "] Unreffing dedicated executor" + CLR);
            this->executor.reset();
        }

        log(GRAY + "[" + getThreadId() + "] All cleared for " + topic + CLR);
    }

     TopicReaderH264::~TopicReaderH264() {
        this->stop();
    }

    std::string TopicReaderH264::openMediaTrackForTopic(std::string topic, std::shared_ptr<WRTCPeer> peer) {

        rtc::SSRC ssrc = getRandomUInt();
        auto payload_type = 96;

        auto cname = topic;
        auto msid = generateId(16); // Unique per stream
       
        auto mid = generateId(16); // Unique per stream
        auto id_track = topic; //generateId(16); // Unique per track

        log(GREEN + "Opening media track for " + topic + ": "+ CLR);
        log("ssrc: "+ std::to_string(ssrc));
        log("cname: "+ cname);
        log("msid: "+ msid);
        log("id_track: "+ id_track);
        log("mid: "+ mid);

        rtc::Description::Video media(mid, rtc::Description::Direction::SendOnly);
        media.addSSRC(ssrc, cname, msid, id_track); //track id is used to identify streams by the client
		media.addH264Codec(payload_type); // Must match the payload type of the external h264 RTP stream
		auto track = peer->getPC()->addTrack(media);

        // create RTP configuration
        auto rtpConfig = std::make_shared<rtc::RtpPacketizationConfig>(ssrc, cname, payload_type, rtc::H264RtpPacketizer::ClockRate);
        // create packetizer
        auto packetizer = std::make_shared<rtc::H264RtpPacketizer>(rtc::NalUnit::Separator::StartSequence, rtpConfig);
        // add RTCP SR handler
        auto srReporter = std::make_shared<rtc::RtcpSrReporter>(rtpConfig);
        packetizer->addToChain(srReporter);
        // add RTCP NACK handler
        auto nackResponder = std::make_shared<rtc::RtcpNackResponder>();
        packetizer->addToChain(nackResponder);
        // set handler
        track->setMediaHandler(packetizer);

        // auto description = track->description();
        // description.addSSRC(ssrc, cname);
        // track->setDescription(std::move(description));

        track->onOpen([peer, topic, track](){
            log(GREEN + peer->toString() + "Media track open for " + topic + CLR);
            // auto description = track->description();
            // for (auto ssrc : description.getSSRCs()) {
            //     log(std::to_string(ssrc));
            // }
        });

        track->onClosed([peer, topic](){
            log(BLUE + peer->toString() + "Media track closed for "+topic + CLR);
        });

        auto track_info = std::make_shared<MediaTrackInfo>(MediaTrackInfo {
            track,
            msid,
            ssrc,
            true,  // in use
            false, // init incomplete
            rtpConfig,
            packetizer
        });
        peer->outbound_media_tracks.emplace(topic, track_info);

        return msid;
    }

     void TopicReaderH264::closeMediaTrackForTopic(std::string topic, std::shared_ptr<WRTCPeer> peer) {
        log(GRAY + peer->toString() + "Closing media track for " + topic + CLR);
        if (peer->outbound_media_tracks.at(topic)->track->isOpen()) {
            peer->outbound_media_tracks.at(topic)->track->close();
        }
        peer->outbound_media_tracks.erase(topic);
    }

}