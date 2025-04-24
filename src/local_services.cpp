#include "phntm_bridge/phntm_bridge.hpp"
#include <json/json.h>
#include <curl/curl.h>

namespace phntm {

    // set up local services of this node
    void PhntmBridge::setupLocalServices() {
        this->srv_clear_file_cache = this->create_service<std_srvs::srv::Trigger>(fmt::format("/{}/clear_cloud_file_cache", this->get_name()),
                                                                                  std::bind(&PhntmBridge::srvRequestClearFileCache, this, std::placeholders::_1, std::placeholders::_2));
    }

    // receives CURL response data
    size_t CURLResponseCallback(void* contents, size_t size, size_t nmemb, std::string* out) {
        size_t totalSize = size * nmemb;
        out->append((char*)contents, totalSize);
        return totalSize;
    }

    // service that requests the files cache on the cloud bridge to be cleared
    void PhntmBridge::srvRequestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        CURL* curl = curl_easy_init();
        if (!curl) {
            std::string err = "Failed to initialize CURL";
            RCLCPP_ERROR(get_logger(), "%s", err.c_str());
            response->success = false;
            response->message = err;
            return;
        }

        // url to call
        std::string url = this->config->uploader_address + "/clear_cache";

        // JSON payload
        Json::Value jsonData;
        jsonData["idRobot"] = this->config->id_robot;
        jsonData["authKey"] = this->config->auth_key;
        Json::FastWriter writer;
        std::string payload = writer.write(jsonData);

        log(YELLOW + "Requesting clear server file cache at " + url + CLR);

        // response buffer
        std::string responseBuffer;

        // CURL options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, payload.size());
        
        // set headers
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // set callback to capture the response
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CURLResponseCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &responseBuffer);

        // perform the request
        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            std::string err = fmt::format("CURL error: {}", curl_easy_strerror(res));
            RCLCPP_ERROR(get_logger(), "%s", err.c_str());
            response->success = false;
            response->message = err;
            return;
        }

        // cleanup
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        long responseCode;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &responseCode);

        if (responseCode == 200) { // ok
            log("Server replied: " + responseBuffer + " (" + std::to_string(responseCode) + ")");
            response->success = true;
            response->message = responseBuffer;
        } else {
            log("Server replied: " + responseBuffer + " (" + std::to_string(responseCode) + ")", true);
            response->success = false;
            response->message = responseBuffer;
        }
    }

}