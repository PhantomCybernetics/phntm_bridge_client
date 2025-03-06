#include "phntm_bridge/phntm_bridge.hpp"

// set up local services of this node
void PhntmBridge::setupLocalServices() {
    this->srv_clear_file_cache = this->create_service<std_srvs::srv::Trigger>(fmt::format("/{}/clear_cloud_file_cache", this->get_name()),
                                                                              std::bind(&PhntmBridge::srvRequestClearFileCache, this, std::placeholders::_1, std::placeholders::_2));
}

// local service that requests the files cache on the cloud bridge to be cleared
void PhntmBridge::srvRequestClearFileCache(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
std::cout << YELLOW << "Requesting clear server file cache" << CLR << std::endl;

(void)request;
(void)response;

// clear_data = {
//     'idRobot': id_robot,
//     'authKey': auth_key
// }
// clear_response = requests.post(f'{upload_host}/clear_cache',
// json=clear_data)

// if clear_response.status_code == 200:
//     logger.debug("Cache cleared")
//     response.success = True
//     response.message = clear_response.text
// else:
//     logger.error(f"Error clearing cache: {clear_response.text}")
//     response.success = False
//     response.message = clear_response.text
}