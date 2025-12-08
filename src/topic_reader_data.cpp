#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/status_leds.hpp"
#include "phntm_bridge/topic_reader_data.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "rtc/peerconnection.hpp"
#include <stdexcept>
#include <string>

namespace phntm {

    std::map<std::string, std::shared_ptr<TopicReaderData>> TopicReaderData::readers;
    std::mutex TopicReaderData::readers_mutex;

    std::shared_ptr<TopicReaderData> TopicReaderData::getForTopic(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            return readers.at(topic);
        } else { // create subscriber
            auto tr = std::make_shared<TopicReaderData>(topic, msg_type, bridge_node, qos);
            readers.emplace(topic, tr);
            return tr;
        }
    }

    std::shared_ptr<TopicReaderData> TopicReaderData::getForTopic(std::string topic) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            return readers.at(topic);
        } else {
            return nullptr;
        }
    }

    void TopicReaderData::destroy(std::string topic) {
        std::lock_guard<std::mutex> lock(readers_mutex);
        if (readers.find(topic) != readers.end()) {
            log(GRAY + "Destroying reader for " + topic + CLR);
            readers.erase(topic);
        }
    }

    TopicReaderData::TopicReaderData(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos) : topic(topic), msg_type(msg_type), bridge_node(bridge_node), qos(qos) {
        this->is_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;
    }

    TopicReaderData::~TopicReaderData() {
        this->stop();
    }

    bool TopicReaderData::addOutput(std::shared_ptr<rtc::DataChannel> dc, std::shared_ptr<rtc::PeerConnection> pc) {
        std::lock_guard<std::mutex> outputs_lock(this->outputs_mutex);
        auto pos = std::find_if(
            this->outputs.begin(),
            this->outputs.end(),
            [&](const std::shared_ptr<Output> output) {
                return output->dc.get() == dc.get();
            }
        );
        if (pos != this->outputs.end())
            return false;

        auto output = std::make_shared<Output>();
        output->dc = dc;
        output->pc = pc;
        this->outputs.push_back(output);
        this->start();

        if (this->is_reliable) {
            sendLatestData(output);
        }

        return true;
    }

    void TopicReaderData::onPCSignalingStateChange(std::shared_ptr<rtc::PeerConnection> pc) {
        if (pc->signalingState() != rtc::PeerConnection::SignalingState::Stable)
            return;
            
        for (auto & tr : TopicReaderData::readers) {
            for (auto & output : tr.second->outputs) {
                if (output->pc.get() == pc.get()) {
                    if (!output->init_complete) {
                        output->init_complete = true;
                        log(GRAY + "Init complete for DC #" + std::to_string(output->dc->id().value()) + CLR);
                    }
                    break;
                }
            }
        }
    }

    bool TopicReaderData::removeOutput(std::shared_ptr<rtc::DataChannel> dc) {
        std::lock_guard<std::mutex> outputs_lock(this->outputs_mutex);
        auto pos = std::find_if(
            this->outputs.begin(),
            this->outputs.end(),
            [&](const std::shared_ptr<Output> output) {
                return output->dc.get() == dc.get();
            }
        );
        if (pos != this->outputs.end()) {
            auto o = *pos;
            o->active = false;
            this->outputs.erase(pos);    
        } else {
            log("DC not found in " + this->topic + " reader");
        }
        if (this->outputs.size() == 0) {
            this->stop();
            return true; // good to destoy
        }
        return false;
    }

    void TopicReaderData::onData(std::shared_ptr<rclcpp::SerializedMessage> data) {

        // save local copy
        const auto& msg = data->get_rcl_serialized_message();
        this->latest_payload_size = msg.buffer_length;
        this->latest_payload.resize(this->latest_payload_size);
        std::memcpy(this->latest_payload.data(), msg.buffer, this->latest_payload_size);

        if (!this->logged_receiving) {
            log(MAGENTA + "[" + getThreadId() + "] Receiving data from " + this->topic + " " + std::to_string(this->latest_payload_size) + " B" + CLR);
            this->logged_receiving = true;
        }

        auto msg_sent = false;
        std::lock_guard<std::mutex> outputs_lock(this->outputs_mutex);
        for (auto output : this->outputs) {
            if (!output->dc->isOpen()) {
                if (!output->logged_closed) {
                    output->logged_closed = true;
                    log(GRAY + "[" + getThreadId() + "] DC #" + std::to_string(output->dc->id().value()) + " is closed for " + this->topic + CLR);
                }
                continue;
            }
            if (!output->init_complete) {
                if (!output->logged_init_incomplete) {
                    output->logged_init_incomplete = true;
                    log(GRAY + "[" + getThreadId() + "] DC #" + std::to_string(output->dc->id().value()) + " not sending msg yet for " + this->topic + " (init incomplete)" + CLR);
                }
                continue;
            }
            try {
                if (!output->logged_sending) {
                    log(GRAY + "[" + getThreadId() + "] Sending " + std::to_string(this->latest_payload_size) +" B into DC " + std::to_string(output->dc->id().value()) + " for " + this->topic + CLR);
                    output->logged_sending = true;
                }

                if (!output->dc->send(this->latest_payload.data(), this->latest_payload_size)) {
                    if (!output->logged_error) {
                        output->logged_error = true;
                        RCLCPP_ERROR(this->bridge_node->get_logger(), "Sending %s into DC #%i failed", this->topic.c_str(), output->dc->id().value());
                    }
                } else {
                    output->num_sent++;
                    msg_sent = true; // flash LED

                    // reset
                    output->logged_closed = false;
                    output->logged_init_incomplete = false;
                    output->logged_error = false;
                    output->logged_exception = false;
                }
            } catch(const std::runtime_error & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->bridge_node->get_logger(), "Error sending %s into DC #%i: %s", this->topic.c_str(), output->dc->id().value(), ex.what());
                }
            } catch(const std::invalid_argument & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    RCLCPP_ERROR(this->bridge_node->get_logger(), "Error sending %s into DC #%i: %s", this->topic.c_str(), output->dc->id().value(), ex.what());
                }
            }
        }

        if (msg_sent)
            DataLED::once();
    }

    void TopicReaderData::sendLatestData(std::shared_ptr<Output> output) {

        std::thread sendLatestThread([this, output]() {
            while ((!output->dc->isOpen() || !output->init_complete) && output->active) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            while (this->latest_payload_size == 0 && output->num_sent == 0 && output->active) { // wait for first data in
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (!output->active || output->num_sent != 0)
                return;
            log(MAGENTA + "Sending latest " + this->topic + " into DC #" + std::to_string(output->dc->id().value())
                            + ", " + std::to_string(this->latest_payload_size)+ " B" + CLR
            );
            try {
                if (!output->dc->send(this->latest_payload.data(), this->latest_payload_size)) {
                    if (!output->logged_error) {
                        output->logged_error = true;
                        log(RED + "Sending latest " + this->topic + " into DC #" + std::to_string(output->dc->id().value()) + " failed" + CLR);
                    }
                } else {
                    output->num_sent++;

                    // reset
                    output->logged_closed = false;
                    output->logged_init_incomplete = false;
                    output->logged_error = false;
                    output->logged_exception = false;
                }
            } catch(const std::runtime_error & ex) {
                if (!output->logged_exception) {
                    output->logged_exception = true;
                    log(RED + "Error sending " + this->topic + " into DC #" + std::to_string(output->dc->id().value()) + "" + CLR);
                }
            }
        });
        sendLatestThread.detach();
    }

    void TopicReaderData::start() {
        if (this->sub != nullptr)
            return;
        try {
            this->sub = this->bridge_node->create_generic_subscription(
                this->topic,
                this->msg_type,
                this->qos,
                std::bind(&TopicReaderData::onData, this, std::placeholders::_1));
            log(GREEN + "[" + this->topic + "] Created subscriber" + CLR);
        } catch(const std::runtime_error & ex) {
            this->sub = nullptr;
            log("Error creating subscriber for " + this->topic + " {"+ this->msg_type +"}: " + ex.what(), true);
        }
    }

    void TopicReaderData::stop() {
        if (this->sub == nullptr)
            return; // already stopped
        if (!rclcpp::ok()) 
            return;
        try {
            this->sub.reset(); // removes sub
            this->sub = nullptr;
            this->logged_receiving = false;
            log(BLUE + "[" + this->topic + "] Removed subscriber" + CLR);
        } catch (const std::exception & ex) {
            log("Exception closing data subscriber: " + std::string(ex.what()), true);
        }
    }

}