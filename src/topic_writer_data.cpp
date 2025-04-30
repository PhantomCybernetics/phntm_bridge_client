#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/topic_writer_data.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "phntm_bridge/status_leds.hpp"
#include "rtc/peerconnection.hpp"
#include <string>

namespace phntm {

    std::map<std::string, std::shared_ptr<TopicWriterData>> TopicWriterData::writers;
    std::map<rtc::DataChannel *, std::shared_ptr<TopicWriterData>> TopicWriterData::dc_to_writer_map;

    std::shared_ptr<TopicWriterData> TopicWriterData::getForTopic(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos) {
        if (writers.find(topic) != writers.end()) {
            return writers.at(topic);
        } else { // create publisher
            auto tw = std::make_shared<TopicWriterData>(topic, msg_type, bridge_node, qos);
            writers.emplace(topic, tw);
            return tw;
        }
    }

    std::shared_ptr<TopicWriterData> TopicWriterData::getForTopic(std::string topic) {
        if (writers.find(topic) != writers.end()) {
            return writers.at(topic);
        } else {
            return nullptr;
        }
    }

    std::shared_ptr<TopicWriterData> TopicWriterData::getForDC(std::shared_ptr<rtc::DataChannel> dc) {
        auto dc_ptr= dc.get();
        if (dc_to_writer_map.find(dc_ptr) != dc_to_writer_map.end()) {
            return dc_to_writer_map.at(dc_ptr);
        }
        return nullptr;
    }

    TopicWriterData::TopicWriterData(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos) : topic(topic), msg_type(msg_type), bridge_node(bridge_node), qos(qos) {
    }

    TopicWriterData::~TopicWriterData() {
        log("Removing writer for "+this->topic);
        this->stop();
        log("Stopped writer for "+this->topic);
    }

    bool TopicWriterData::addInput(std::shared_ptr<rtc::DataChannel> dc) {
        std::lock_guard<std::mutex> lock(this->inputs_mutex);
        auto pos = std::find_if(
            this->inputs.begin(),
            this->inputs.end(),
            [&](const std::shared_ptr<Input> input) {
                return input->dc.get() == dc.get();
            }
        );
        if (pos != this->inputs.end())
            return false;

        auto input = std::make_shared<Input>();
        input->dc = dc;
        this->inputs.push_back(input);
        this->start();

        dc_to_writer_map.emplace(dc.get(), shared_from_this());

        return true;
    }

    bool TopicWriterData::removeInput(std::shared_ptr<rtc::DataChannel> dc) {
        std::lock_guard<std::mutex> lock(this->inputs_mutex);
        auto pos = std::find_if(
            this->inputs.begin(),
            this->inputs.end(),
            [&](const std::shared_ptr<Input> input) {
                return input->dc.get() == dc.get();
            }
        );
        if (pos == this->inputs.end()) {
            log("DC not found in " + this->topic + " writer");
            return false;
        }
        auto in = *pos;
        in->active = false;
        this->inputs.erase(pos);
        if (this->inputs.size() == 0) {
            this->stop();
        }

        auto pos_index = dc_to_writer_map.find(dc.get());
        if (pos_index != dc_to_writer_map.end()) {
            dc_to_writer_map.erase(pos_index);
        }

        return true;
    }

    bool TopicWriterData::onData(std::shared_ptr<WRTCPeer> peer, std::shared_ptr<rtc::DataChannel> dc, std::string topic, std::variant<rtc::binary, rtc::string> message) {
        if (!std::holds_alternative<rtc::binary>(message)) { // always bin
            log(RED + peer->toString() + "Got non-binary msg, ignoring" + CLR);
            return false;
        }

        rtc::binary& bin = std::get<rtc::binary>(message);
        auto tw = TopicWriterData::getForDC(dc);
        if (tw == nullptr)
            return false; // writer not found

        for (auto & in : tw->inputs) {
            if (in->dc.get() == dc.get()) {
                if (!in->logged_receiving)  {
                    in->logged_receiving = true;
                    log(GREEN + peer->toString() + "Receiving data from DC #" + std::to_string(dc->id().value()) + " for " + topic + ", " + std::to_string(bin.size()) + " B" + CLR);
                }
                in->num_received++;
                break;
            }
        }
        
        rclcpp::SerializedMessage serialized_msg(bin.size());
        std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, bin.data(), bin.size());
        serialized_msg.get_rcl_serialized_message().buffer_length = bin.size(); // Important!

        if (tw->pub) {
            tw->pub->publish(serialized_msg);
            DataLED::once();   
        }

        return true; // handled
    }

    void TopicWriterData::start() {
        if (this->pub != nullptr)
            return;
        try {
            this->pub = this->bridge_node->create_generic_publisher(this->topic, this->msg_type, this->qos);
            log(GREEN + "[" + this->topic + "] Created publisher" + CLR);
        } catch(const std::runtime_error & ex) {
            this->pub = nullptr;
            log("Error creating publisher for " + this->topic + " {"+ this->msg_type +"}: " + ex.what(), true);
        }
    }

    void TopicWriterData::stop() {
        if (this->pub == nullptr)
            return; //already stopped
        
        this->pub.reset(); // removes sub
        this->pub = nullptr;
        log(BLUE + "[" + this->topic + "] Removed publisher" + CLR);
    }

}