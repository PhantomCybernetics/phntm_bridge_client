#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/sio.hpp"

PhntmBridge::PhntmBridge(std::string node_name) : Node(node_name)
{   
    this->config = std::make_shared<BridgeConfig>();

    this->loadConfig();
    this->makeServices();

    this->config->ros_distro = std::getenv("ROS_DISTRO");
    std::cout << "ROS distro is: " << YELLOW << this->config->ros_distro << CLR << std::endl;

    this->readGitRepoHead("/ros2_ws/src/phntm_bridge");
    std::cout << "Git commit: " << YELLOW << this->config->git_head_sha << CLR << " Tag: " << YELLOW << (this->config->latest_git_tag.empty() ? "-" : this->config->latest_git_tag) << CLR << std::endl;

    // this->declare_parameter("topic_prefix", "/picam_ros2/camera_");
    // this->declare_parameter("log_message_every_sec", 5.0); // -1.0 = off
    // this->declare_parameter("log_scroll", false);
    // this->declare_parameter("calibration_frames_needed", 10);
    // this->declare_parameter("calibration_pattern_size", std::vector<int>{ 9, 6 });
    // this->declare_parameter("calibration_square_size_m", 0.019f);
    // this->declare_parameter("calibration_files", "/calibration/");
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::cout << "Launching the Bridge Node" << std::endl;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PhntmBridge>("phntm_bridge");
  auto sio = std::make_shared<BridgeSocket>(node->config);

  sio->connect();
  rclcpp::spin(node);
    
  std::cout << BLUE << "Shutting down..." << CLR << std::endl;

  sio->disconnect();
  rclcpp::shutdown();

  return 0;
}
