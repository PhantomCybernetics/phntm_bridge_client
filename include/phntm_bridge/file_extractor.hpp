#include <memory>
#include <phntm_interfaces/msg/detail/file_chunk__struct.hpp>
#include "phntm_bridge/phntm_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "phntm_interfaces/msg/file_chunk.hpp"

#include "config.hpp"

namespace phntm {

    class PhntmBridge;

    class FileExtractor {
        public:
            static void requestClearFileCache(const std::shared_ptr<PhntmBridge> node, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            static void findAndUploadFile(const std::shared_ptr<PhntmBridge> node, const std::string search_path, const int msg_id);
            static void receiveChunk(const phntm_interfaces::msg::FileChunk chunk);
            static void init(const std::shared_ptr<PhntmBridge> node);
            static void stop();
            static std::shared_ptr<rclcpp::Subscription<phntm_interfaces::msg::FileChunk>> chunks_sub;
            static std::shared_ptr<PhntmBridge> node;
            static std::map<std::string, std::map<int, bool>> uploaded_chunks;
            static std::mutex uploaded_chunks_mutex;
        
        private:
            static void markChunkResult(std::string file_path, const int chunk_number, bool success);
    };

}