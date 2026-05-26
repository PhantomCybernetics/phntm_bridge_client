#include <memory>
// #include <phntm_interfaces/msg/detail/file_chunk__struct.hpp>
#include "phntm_bridge/phntm_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "phntm_interfaces/msg/file_extraction_request.hpp"
#include "phntm_interfaces/msg/file_extraction_result.hpp"

#include "config.hpp"

namespace phntm {

    class PhntmBridge;

    class FileExtractor {

        public:
            static void requestClearFileCache(const std::shared_ptr<PhntmBridge> node, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            static void onRequest(const std::shared_ptr<PhntmBridge> node, const std::string search_path, const int ack_msg_id);
            static void onResult(const phntm_interfaces::msg::FileExtractionResult res);
            static void init(const std::shared_ptr<PhntmBridge> node);
            static void stop();
            static std::shared_ptr<rclcpp::Publisher<phntm_interfaces::msg::FileExtractionRequest>> requests_pub;
            static std::shared_ptr<rclcpp::Subscription<phntm_interfaces::msg::FileExtractionResult>> results_sub;
            static std::shared_ptr<PhntmBridge> node;

        private:
            struct FileExtractionRequest {
                std::map<std::string, int> agent_replies;
                std::map<std::string, std::string> agent_messages;
                int ack_msg_id;
            };
            static std::map<std::string, FileExtractionRequest> file_requests_in_progresss;
    };

}