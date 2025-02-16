#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/config.hpp"
#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/git.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/extra_packages.hpp"
#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

PhntmBridge::PhntmBridge(std::string node_name, std::shared_ptr<BridgeConfig> config) : Node(node_name)
{   
    this->config = config;

    this->loadConfig();
    this->makeServices();

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

  if (installExtraPackages()) {
    std::cout << MAGENTA << "Restarting..." << CLR << std::endl;
    return 0;
  }

  std::cout << LIME << "Launching the Bridge Node" << CLR << std::endl;

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto config = std::make_shared<BridgeConfig>();

  config->ros_distro = std::getenv("ROS_DISTRO");
  std::cout << "ROS distro is: " << YELLOW << config->ros_distro << CLR << std::endl;
  readGitRepoHead("/ros2_ws/src/phntm_bridge", config);
  std::cout << "Git commit: " << YELLOW << config->git_head_sha << CLR << " Tag: " << YELLOW << (config->latest_git_tag.empty() ? "-" : config->latest_git_tag) << CLR << std::endl;

  auto base_node = std::make_shared<PhntmBridge>("phntm_bridge", config);
  // auto introspection_node = std::make_shared<rclcpp::Node>("phntm_introspection");
  executor.add_node(base_node);
  // executor.add_node(introspection_node);

  auto sio = std::make_shared<BridgeSocket>(config);
  auto introspection = std::make_shared<Introspection>(base_node, sio, config);
  sio->setIntrospection(introspection);

  introspection->start();
  sio->connect();

  //rclcpp::spin((base_node));
  executor.spin();
    
  std::cout << BLUE << "Shutting down..." << CLR << std::endl;

  introspection->stop();
  sio->disconnect();
  rclcpp::shutdown();

  return 0;
}
