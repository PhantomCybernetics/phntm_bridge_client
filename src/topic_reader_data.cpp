#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/topic_reader_data.hpp"
#include "rtc/peerconnection.hpp"
#include <string>

std::map<std::string, std::shared_ptr<TopicReaderData>> TopicReaderData::readers;

std::shared_ptr<TopicReaderData> TopicReaderData::getForTopic(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos) {
    if (readers.find(topic) != readers.end()) {
        return readers.at(topic);
    } else { // create subscriber
        auto tr = std::make_shared<TopicReaderData>(topic, msg_type, bridge_node, qos);
        readers.emplace(topic, tr);
        return tr;
    }
}

std::shared_ptr<TopicReaderData> TopicReaderData::getForTopic(std::string topic) {
    if (readers.find(topic) != readers.end()) {
        return readers.at(topic);
    } else {
        return nullptr;
    }
}

// std::shared_ptr<TopicReaderData> TopicReaderData::getForDC(std::shared_ptr<rtc::DataChannel> dc) {
//     for (auto p : readers) {
//         for (auto dc_search : p.second->dcs) {
//             if (dc.get() == dc_search.get()) {
//                 return p.second;
//             }
//         }
//     }
//     return nullptr;
// }

TopicReaderData::TopicReaderData(std::string topic, std::string msg_type, std::shared_ptr<PhntmBridge> bridge_node, rclcpp::QoS qos) : topic(topic), msg_type(msg_type), bridge_node(bridge_node), qos(qos) {
    this->is_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;
    this->start();
}

TopicReaderData::~TopicReaderData() {
    this->stop();
}

bool TopicReaderData::addOutput(std::shared_ptr<rtc::DataChannel> dc, std::shared_ptr<rtc::PeerConnection> pc) {
    std::lock_guard<std::mutex> lock(this->outputs_mutex);
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
    output->num_sent = 0;
    output->active = true;
    this->outputs.push_back(output);
    this->start();

    if (this->is_reliable) {
        sendLatestData(output);
    }

    return true;
}

bool TopicReaderData::removeOutput(std::shared_ptr<rtc::DataChannel> dc) {
    std::lock_guard<std::mutex> lock(this->outputs_mutex);
    auto pos = std::find_if(
        this->outputs.begin(),
        this->outputs.end(),
        [&](const std::shared_ptr<Output> output) {
            return output->dc.get() == dc.get();
        }
    );
    if (pos == this->outputs.end()) {
        log("DC not found in " + this->topic + " reader");
        return false;
    }
    auto o = *pos;
    o->active = false;
    this->outputs.erase(pos);
    if (this->outputs.size() == 0) {
        this->stop();
    }
    return true;
}

void TopicReaderData::onData(std::shared_ptr<rclcpp::SerializedMessage> data) {

    // save local copy
    const auto& msg = data->get_rcl_serialized_message();
    this->latest_payload_size = msg.buffer_length;
    this->latest_payload.resize(this->latest_payload_size);
    std::memcpy(this->latest_payload.data(), msg.buffer, this->latest_payload_size);

    if (!this->logged_receiving) {
        log(MAGENTA + "Receiving data from " + this->topic + " " + std::to_string(this->latest_payload_size) + " B" + CLR);
        this->logged_receiving = true;
    }

    std::lock_guard<std::mutex> lock(this->outputs_mutex);
    for (auto output : this->outputs) {
        if (!output->dc->isOpen()) {
            log(GRAY + "DC #" + std::to_string(output->dc->id().value()) + " is closed for " + this->topic + CLR);
            continue;
        }
        if (output->num_sent == 0 && output->pc->signalingState() != rtc::PeerConnection::SignalingState::Stable) {
            log(GRAY + "DC #" + std::to_string(output->dc->id().value()) + " not sendiong yet for " + this->topic + CLR);
            continue;
        }
        if (!output->dc->send(this->latest_payload.data(), this->latest_payload_size)) {
            log(RED + "Sending " + this->topic + " into DC #" + std::to_string(output->dc->id().value()) + " failed" + CLR);
        } else {
            output->num_sent++;
        }
    }
}

void TopicReaderData::sendLatestData(std::shared_ptr<Output> output) {

    std::thread sendLatestThread([this, output]() {
        while (!output->dc->isOpen() && output->active) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        while (output->pc->signalingState() != rtc::PeerConnection::SignalingState::Stable && output->num_sent == 0 && output->active) { // after the first channel init, wait for signalling state
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        while (this->latest_payload_size == 0 && output->num_sent == 0 && output->active) { // wait for first data
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!output->active || output->num_sent != 0)
            return;
        log(MAGENTA + "Sending latest " + this->topic + " into DC #" + std::to_string(output->dc->id().value()) + ", " + std::to_string(this->latest_payload_size)+ " B"+ CLR);
        if (!output->dc->send(this->latest_payload.data(), this->latest_payload_size)) {
            log(RED + "Sending latest " + this->topic + " into DC #" + std::to_string(output->dc->id().value()) + " failed" + CLR);
        } else {
            output->num_sent++;
        }
    });
    sendLatestThread.detach();
}

void TopicReaderData::start() {
    if (this->sub != nullptr)
        return;
    this->sub = this->bridge_node->create_generic_subscription(this->topic, this->msg_type, this->qos, std::bind(&TopicReaderData::onData, this, std::placeholders::_1));
    log(GREEN + "[" + this->topic + "] Created subscriber" + CLR);
}

void TopicReaderData::stop() {
    if (this->sub == nullptr) {
        log("[" + this->topic + "] Sub not found");
        return;
    }
    log("[" + this->topic + "] Removing sub");
    this->sub.reset(); // removes sub
    this->sub = nullptr;
    log(BLUE + "[" + this->topic + "] Removed subscriber" + CLR);
}