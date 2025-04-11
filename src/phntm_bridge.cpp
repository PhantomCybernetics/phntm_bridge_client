#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/config.hpp"
#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/git.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/extra_packages.hpp"
#include "phntm_bridge/status_leds.hpp"

#include <iostream>
#include <ostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

PhntmBridge::PhntmBridge(std::string node_name, std::shared_ptr<BridgeConfig> config) : Node(node_name)
{   
    this->config = config;

    this->loadConfig(this->config);
    this->setupLocalServices();

    // this->declare_parameter("topic_prefix", "/picam_ros2/camera_");
    // this->declare_parameter("log_message_every_sec", 5.0); // -1.0 = off
    // this->declare_parameter("log_scroll", false);
    // this->declare_parameter("calibration_frames_needed", 10);
    // this->declare_parameter("calibration_pattern_size", std::vector<int>{ 9, 6 });
    // this->declare_parameter("calibration_square_size_m", 0.019f);
    // this->declare_parameter("calibration_files", "/calibration/");
}

std::atomic<bool> g_interrupt_requested(false);

void signal_handler(int signum) {
    std::cout << RED << "Signal handler got " << signum << CLR << std::endl;
    g_interrupt_requested.store(true);
}

PhntmBridge::~PhntmBridge() {
  this->clearServicesCache();
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
  rclcpp::uninstall_signal_handlers();

  std::signal(SIGINT, signal_handler);

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

  StatusLEDs::init(base_node, config);

  BridgeSocket::init(base_node, config);
  Introspection::init(base_node, config);
  Introspection::start();
  BridgeSocket::connect();

  while (!g_interrupt_requested.load() && rclcpp::ok()) {
    executor.spin_some();
  }
    
  std::cout << BLUE << "Shutting down..." << CLR << std::endl;

  Introspection::stop();
  StatusLEDs::clear();

  std::cout << "Spinning some more..." << std::endl;
  executor.spin_some(); // make sure we send out what we need before shutdown
  std::cout << "Done spinning" << std::endl;

  BridgeSocket::shutdown();

  rclcpp::shutdown();

  std::cout << BLUE << "Rclcpp down..." << CLR << std::endl;

  return 0;
}
