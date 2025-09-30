#include "phntm_bridge/file_extractor.hpp"
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/sio.hpp"
#include "phntm_interfaces/srv/file_request.hpp"
#include "sio_message.h"

#include <chrono>
#include <json/json.h>
#include <curl/curl.h>
#include <fmt/core.h>
#include <memory>
#include <phntm_interfaces/msg/detail/file_chunk__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <string>

namespace phntm {

    std::shared_ptr<rclcpp::Subscription<phntm_interfaces::msg::FileChunk>> FileExtractor::chunks_sub = nullptr;
    std::shared_ptr<PhntmBridge> FileExtractor::node = nullptr;
    std::map<std::string, std::map<int, bool>> FileExtractor::uploaded_chunks;
    std::mutex FileExtractor::uploaded_chunks_mutex;

    // running in a separate thread
    void FileExtractor::findAndUploadFile(const std::shared_ptr<PhntmBridge> node, const std::string search_path, const int msg_id) {
        RCLCPP_INFO(node->get_logger(), "Requested file upload for %s", search_path.c_str());

        auto extractors = Introspection::getFileExtractors();
        if (extractors.empty()) {
            RCLCPP_ERROR(node->get_logger(), "No file extractors were discovered yet");
            auto ack = sio::null_message::create(); // = error
            BridgeSocket::ack(msg_id, ack);    
            return;
        }

        auto request = std::make_shared<phntm_interfaces::srv::FileRequest::Request>();
        request->path = search_path;
        bool success = false;

        {
            std::lock_guard<std::mutex> lock(FileExtractor::uploaded_chunks_mutex);
            if (FileExtractor::uploaded_chunks.find(search_path) != FileExtractor::uploaded_chunks.end()) {
                FileExtractor::uploaded_chunks.erase(search_path);
            }
            FileExtractor::uploaded_chunks.emplace(search_path, std::map<int, bool>()); // reset
        }

        std::vector<std::shared_ptr<std::thread>> extractor_threads;
        for (auto & e: extractors) { // node id => service client
            auto t = std::make_shared<std::thread>([&]() {
                auto node_name = e.first;
                // log(YELLOW + "Requesting file " + search_path + " from " + node_name + CLR);
                auto client = e.second;
                
                while (!client->wait_for_service(std::chrono::milliseconds(10))) {
                    if (!rclcpp::ok()) {
                        log("Client interrupted while waiting for service " + std::string(client->get_service_name()) + " to appear.", true);
                        return;
                    }
                }

                log("Requesting file " + search_path + " from " + node_name + " via " + std::string(client->get_service_name()));
                auto result_future = client->async_send_request(request);

                const auto timeout = std::chrono::milliseconds(10);
    
                // response wait loop
                while (rclcpp::ok() && result_future.valid()) {
                    const auto status = result_future.wait_for(timeout);
                    if (status == std::future_status::ready) break;
                }
                if (!rclcpp::ok()) { // wost-wait validation
                    log("Node shutdown during file response wait");
                    return;
                }
                if (!result_future.valid()) {
                    log("Future invalidated for " + node_name);
                    return;
                }

                try {
                    auto result = result_future.get();
                    if (!result->success) {
                        log("File not found by " + node_name + "");
                        return;
                    }

                    log(GREEN + "Success from " + node_name + ", awaiting " + std::to_string(result->total_bytes) + " B in "+ std::to_string(result->num_parts) + " chunks for " + search_path + CLR);
                    
                    while (FileExtractor::uploaded_chunks.at(search_path).size() < result->num_parts) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    int num_ok = 0, num_err = 0;
                    for (auto chunk : FileExtractor::uploaded_chunks.at(search_path)) {
                        if (chunk.second) {
                            num_ok++;
                        } else {
                            num_err++;
                        }
                    }

                    log("All parts of uploaded for " + search_path + " "+ GREEN + std::to_string(num_ok) + " ok" + CLR + " / " + RED + std::to_string(num_err) + " failed" + CLR + " / " + std::to_string(result->num_parts) + " total");

                    if (num_err != 0) {
                        success = false;
                        return;
                    }

                    CURL* curl = curl_easy_init();
                    if (!curl) {
                        std::string err = "Failed to initialize CURL for file complete";
                        RCLCPP_ERROR(node->get_logger(), "%s", err.c_str());
                        success = false;
                        return;
                    }

                    std::string url = node->config->uploader_address + "/complete";

                    // JSON payload
                    Json::Value json_data;
                    json_data["idRobot"] = node->config->id_robot;
                    json_data["authKey"] = node->config->auth_key;
                    json_data["fileUrl"] = search_path;
                    json_data["totalParts"] = result->num_parts;
                    Json::FastWriter writer;
                    std::string json_payload = writer.write(json_data);

                    log(YELLOW + "Requesting complete file " + search_path + " at " + url + CLR);

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

                    // cleanup
                    curl_slist_free_all(headers);
                    curl_easy_cleanup(curl);

                    if (res != CURLE_OK) {
                        std::string err = fmt::format("CURL error: {}", curl_easy_strerror(res));
                        RCLCPP_ERROR(node->get_logger(), "%s", err.c_str());
                        success = false;
                        return;
                    }

                    long response_code;
                    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

                    if (response_code == 200) { // ok
                        log("Server replied: " + response_buffer);
                        success = true;
                        Json::Reader reader;
                        Json::Value reply_json_value;
                        reader.parse(response_buffer, reply_json_value);
                        auto ack = sio::object_message::create(); 
                        if (reply_json_value.isMember("fileName"))
                            ack->get_map().emplace("fileName", sio::string_message::create(reply_json_value["fileName"].asString()));
                        BridgeSocket::ack(msg_id, ack);
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "Server replied: %s (%ld)", response_buffer.c_str(), response_code);
                        success = false;
                    }
    
                } catch (const std::exception& ex) {
                    log("Service error from " + e.first + ": " + ex.what());
                }
                
            });
            extractor_threads.push_back(t);
        }

        // wait for all search threads to finish
        for (auto & t : extractor_threads) {
            t->join();
        }
        extractor_threads.clear();
        log(GRAY + "All requests done, " + (success ? "success" : "file not found") + CLR);

        if (!success) {
            auto ack = sio::null_message::create(); // = error
            BridgeSocket::ack(msg_id, ack);
        }
    }

    void FileExtractor::markChunkResult(std::string file_path, const int chunk_number, bool success) {
        std::lock_guard<std::mutex> lock(FileExtractor::uploaded_chunks_mutex);
        if (FileExtractor::uploaded_chunks.find(file_path) != FileExtractor::uploaded_chunks.end()) {
            FileExtractor::uploaded_chunks.at(file_path).emplace(chunk_number, success);
        }
    }

    void FileExtractor::receiveChunk(const phntm_interfaces::msg::FileChunk chunk) {
        log("Received "+ chunk.file_path+", chunk " + std::to_string(chunk.chunk_number)
            + "/" +std::to_string(chunk.total_chunks)+ " " + std::to_string(chunk.data.size()) + " B");

        CURL* curl = curl_easy_init();
        if (!curl) {
            std::string err = "Failed to initialize CURL for chunk upload";
            RCLCPP_ERROR(node->get_logger(), "%s", err.c_str());
            markChunkResult(chunk.file_path, chunk.chunk_number, false);
            return;
        }
        
        std::string url = node->config->uploader_address + "/upload";
        
        // JSON payload
        Json::Value json_data;
        json_data["idRobot"] = node->config->id_robot;
        json_data["key"] = node->config->auth_key;
        json_data["fileUrl"] = chunk.file_path;
        Json::FastWriter writer;
        std::string json_payload = writer.write(json_data);

        std::string response_string;

        // Configure curl options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
       
        curl_mime* mime = curl_mime_init(curl);
        curl_mimepart* part;

        // add file part
        std::string filename = chunk.file_path + ".part" + std::to_string(chunk.chunk_number);
        part = curl_mime_addpart(mime);
        curl_mime_name(part, "file");
        curl_mime_filename(part, filename.c_str());
        curl_mime_data(part, reinterpret_cast<const char*>(chunk.data.data()), chunk.data.size());

        // JSON payload
        part = curl_mime_addpart(mime);
        curl_mime_name(part, "json");
        curl_mime_data(part, json_payload.c_str(), json_payload.size());

        // set JSON Content-Type header
        struct curl_slist* json_headers = nullptr;
        json_headers = curl_slist_append(json_headers, "Content-Type: application/json");
        curl_mime_headers(part, json_headers, 1);  // 1 = libcurl takes ownership

        curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);

        // Response handling
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CURLResponseCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

        // Execute request
        CURLcode res = curl_easy_perform(curl);

        // cleanup
        curl_slist_free_all(json_headers);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            std::string err = fmt::format("CURL error: {}", curl_easy_strerror(res));
            RCLCPP_ERROR(node->get_logger(), "%s", err.c_str());
            markChunkResult(chunk.file_path, chunk.chunk_number, false);
            return;
        }

        long response_code;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

        if (response_code == 200) {
            log("Uploaded "+ chunk.file_path+", chunk " + std::to_string(chunk.chunk_number));
            markChunkResult(chunk.file_path, chunk.chunk_number, true);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Server returned code %ld", response_code);
            markChunkResult(chunk.file_path, chunk.chunk_number, false);
        }
    }

    void FileExtractor::init(const std::shared_ptr<PhntmBridge> node) {
        log("Starting FileExtractor");
        FileExtractor::node = node;
        rclcpp::QoS qos(1);
        qos.reliable();
        qos.history(rclcpp::HistoryPolicy::KeepAll);
        chunks_sub = node->create_subscription<phntm_interfaces::msg::FileChunk>(node->config->file_chunks_topic, qos,
            FileExtractor::receiveChunk);
    }

    void FileExtractor::stop() {
        log("Stopping FileExtractor");
        chunks_sub.reset();
        chunks_sub = nullptr;
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

        std::string url = node->config->uploader_address + "/clear_cache";

        // JSON payload
        Json::Value json_data;
        json_data["idRobot"] = node->config->id_robot;
        json_data["authKey"] = node->config->auth_key;
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

        // cleanup
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            std::string err = fmt::format("CURL error: {}", curl_easy_strerror(res));
            RCLCPP_ERROR(node->get_logger(), "%s", err.c_str());
            response->success = false;
            response->message = err;
            return;
        }

        long response_code;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

        if (response_code == 200) { // ok
            log("Server replied: " + response_buffer + " (" + std::to_string(response_code) + ")");
            response->success = true;
            response->message = response_buffer;
        } else {
            log("Server replied: " + response_buffer + " (" + std::to_string(response_code) + ")", true);
            response->success = false;
            response->message = response_buffer;
        }
    }


}