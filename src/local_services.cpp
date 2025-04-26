#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/file_extractor.hpp"

namespace phntm {

    // set up local services of this node
    void PhntmBridge::setupLocalServices() {
        this->srv_clear_file_cache = this->create_service<std_srvs::srv::Trigger>(fmt::format("/{}/clear_cloud_file_cache", this->get_name()),
                                                                                  std::bind(&PhntmBridge::srvRequestClearFileCache, this, std::placeholders::_1, std::placeholders::_2));
    }

    void PhntmBridge::srvRequestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto self = std::static_pointer_cast<PhntmBridge>(shared_from_this());
        FileExtractor::requestClearFileCache(self, response);
    }
}