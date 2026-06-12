#include "phntm_bridge/file_extractor.hpp"
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/sio.hpp"
#include "sio_message.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <json/json.h>
#include <curl/curl.h>
#include <fmt/core.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <string>

namespace phntm {

    std::shared_ptr<rclcpp::Publisher<phntm_interfaces::msg::FileExtractionRequest>> FileExtractor::requests_pub = nullptr;
    std::shared_ptr<rclcpp::Subscription<phntm_interfaces::msg::FileExtractionResult>> FileExtractor::results_sub = nullptr;
    std::shared_ptr<PhntmBridge> FileExtractor::node = nullptr;
    std::map<std::string, FileExtractor::FileExtractionRequest> FileExtractor::file_requests_in_progresss;

    void FileExtractor::init(const std::shared_ptr<PhntmBridge> node) {
        log("Starting FileExtractor");
        FileExtractor::node = node;
        rclcpp::QoS qos(rclcpp::KeepLast(1000));
        qos.reliable();
        qos.history(rclcpp::HistoryPolicy::KeepLast);

        requests_pub = node->create_publisher<phntm_interfaces::msg::FileExtractionRequest>(node->config->file_extraction_request_topic, qos);
        results_sub = node->create_subscription<phntm_interfaces::msg::FileExtractionResult>(node->config->file_extraction_result_topic, qos, FileExtractor::onResult);
    }

    void FileExtractor::stop() {
        log("Stopping FileExtractor");
        requests_pub.reset();
        requests_pub = nullptr;
        results_sub.reset();
        results_sub = nullptr;
    }

    // running in a separate thread
    void FileExtractor::onRequest(const std::shared_ptr<PhntmBridge> node, const std::string search_path, const int ack_msg_id) {

        auto extractors = Introspection::getFileExtractors();

        RCLCPP_INFO(node->get_logger(), "FileExtractor request: %s (%zu agents)", search_path.c_str(), extractors.size());

        if (FileExtractor::file_requests_in_progresss.find(search_path) != FileExtractor::file_requests_in_progresss.end()) {
            log("FileExtractor Warning: older request already in progress for '" + search_path + "'; dropping older", true);
            FileExtractor::file_requests_in_progresss.erase(search_path);
        }

        FileExtractor::FileExtractionRequest req_data;
        req_data.ack_msg_id = ack_msg_id;
        for (const auto &extractor : extractors) {
            req_data.agent_replies.emplace(extractor, -1);
        }
        FileExtractor::file_requests_in_progresss.emplace(search_path, req_data);

        phntm_interfaces::msg::FileExtractionRequest req_msg;
        req_msg.path = search_path;
        req_msg.id_robot = node->config->id_robot;
        requests_pub->publish(req_msg);
    }

    void FileExtractor::onResult(const phntm_interfaces::msg::FileExtractionResult res) {
        
        if (res.id_robot != FileExtractor::node->config->id_robot) {
            return; // ignore replies with other robot ids
        }

        if (FileExtractor::file_requests_in_progresss.find(res.path) == FileExtractor::file_requests_in_progresss.end()) {
            log("FileExtractor request not found for '" + res.path + "'", true);
            return;
        }

        auto req = FileExtractor::file_requests_in_progresss[res.path];
        auto success = res.result == phntm_interfaces::msg::FileExtractionResult::RESULT_UPLOADED;
        
        if (req.agent_replies.find(res.agent) != req.agent_replies.end()) {
            req.agent_replies[res.agent] = res.result;
        }

        auto all_done = true;
        for (const auto& agent_reply : req.agent_replies) {
            if (agent_reply.second < 0 || agent_reply.second == phntm_interfaces::msg::FileExtractionResult::RESULT_FOUND_SENDING_CHUNKS) 
                all_done = false;
        }

        switch (res.result) {
            case phntm_interfaces::msg::FileExtractionResult::RESULT_NOT_FOUND:
                log("FileExtractor result for '" + res.path + "' from " + res.agent + ": NOT FOUND");
                req.agent_messages.emplace(res.agent, "File not found");
                break;
            case phntm_interfaces::msg::FileExtractionResult::RESULT_UPLOADED:
                log("FileExtractor result for '" + res.path + "' from " + res.agent + ", UPLOADED, cached as " + res.cached_file_name);
                break;
            case phntm_interfaces::msg::FileExtractionResult::RESULT_FOUND_SENDING_CHUNKS:
                log("FileExtractor result for '" + res.path + "' from " + res.agent + ", FOUND, sending chunks");
                break;
            case phntm_interfaces::msg::FileExtractionResult::RESULT_EXTRACTION_DISABLED:
                log("FileExtractor result for '" + res.path + "' from " + res.agent + ", extraction disabled");
                req.agent_messages.emplace(res.agent, "File extraction disabled");
                break;
            case phntm_interfaces::msg::FileExtractionResult::RESULT_ERROR:
                log("FileExtractor got error for '" + res.path + "' from " + res.agent);
                req.agent_messages.emplace(res.agent, "Produced errord");
                break;
            case phntm_interfaces::msg::FileExtractionResult::RESULT_INVALID_ROBOT:
                log("FileExtractor got invalid robot for '" + res.path + "' from " + res.agent);
                break;
            default:
                log("FileExtractor result for '" + res.path + "' from " + res.agent + ", invalid state", true);
                req.agent_messages.emplace(res.agent, "Produced errord");
                break;
        }

        if (success || all_done) {
            auto ack = sio::object_message::create(); 
            if (success) {
                ack->get_map().emplace("fileName", sio::string_message::create(res.cached_file_name));
            } else {
                auto reply_msgs = sio::object_message::create();
                for (const auto& agent_reply_msg : req.agent_messages) {
                    reply_msgs->get_map().emplace(agent_reply_msg.first, sio::string_message::create(agent_reply_msg.second));
                }
                ack->get_map().emplace("msgs", reply_msgs);
            }
            BridgeSocket::ack(req.ack_msg_id, ack);
            FileExtractor::file_requests_in_progresss.erase(res.path);
        }
    }


    // service that requests the files cache on the cloud bridge to be cleared
    void FileExtractor::requestClearFileCache(const std::shared_ptr<PhntmBridge> node, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        CURL* curl = curl_easy_init();
        if (!curl) {
            std::string err = "Failed to initialize CURL for file cache clear";
            RCLCPP_ERROR(node->get_logger(), "%s", err.c_str());
            response->success = false;
            response->message = err;
            return;
        }

        std::string url = node->config->file_uploader_address + "/clear_cache";

        // JSON payload
        Json::Value json_data;
        json_data["idRobot"] = node->config->id_robot;
        json_data["key"] = node->config->auth_key;
        Json::FastWriter writer;
        std::string json_payload = writer.write(json_data);

        log(YELLOW + "Requesting clear server file cache at " + url + CLR);

        // response buffer
        std::string response_buffer;

        // CURL options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_payload.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json_payload.size());
        
        // set headers
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // set callback to capture the response
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CURLResponseCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_buffer);

        // perform the request
        CURLcode res = curl_easy_perform(curl);

        if (res != CURLE_OK) {
            std::string err = fmt::format("CURL error: {}", curl_easy_strerror(res));
            RCLCPP_ERROR(node->get_logger(), "%s", err.c_str());
            response->success = false;
            response->message = err;
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
            return;
        }

        long response_code = 0;
        CURLcode info_res = curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
        if(info_res != CURLE_OK) {
            log("Failed to get response code", true);
        }

        // cleanup
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (response_code == 200) { // ok
            log("Server replied: " + response_buffer + " (code " + std::to_string(response_code) + ")");
            response->success = true;
            response->message = response_buffer;
        } else {
            log("Server replied: " + response_buffer + " (code " + std::to_string(response_code) + ")", true);
            response->success = false;
            response->message = response_buffer;
        }
    }


}