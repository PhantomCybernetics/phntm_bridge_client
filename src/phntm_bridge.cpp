#include "phntm_bridge/phntm_bridge.hpp"
#include "phntm_bridge/config.hpp"
#include "phntm_bridge/sio.hpp"
#include "phntm_bridge/git.hpp"
#include "phntm_bridge/introspection.hpp"
#include "phntm_bridge/extra_packages.hpp"
#include "phntm_bridge/status_leds.hpp"
#include "phntm_bridge/wrtc_peer.hpp"
#include "phntm_bridge/file_extractor.hpp"

#include <iostream>
#include <ostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <string>
#include <rcl/rcl.h>

namespace phntm {

  PhntmBridge::PhntmBridge(std::string node_name, rclcpp::NodeOptions node_options, std::shared_ptr<BridgeConfig> config)
    : Node(node_name, node_options)
  {   
      this->config = config;

      this->introspection_reentrant_group = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant
      );
      this->media_reentrant_group = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant
      );

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
      log(RED + "Signal handler got " + std::to_string(signum) + CLR);
      g_interrupt_requested.store(true);
  }

  PhntmBridge::~PhntmBridge() {
    this->clearServicesCache();
  }
}

int main(int argc, char ** argv)
{
  using namespace phntm;
  
  (void) argc;
  (void) argv;

  if (installExtraPackages()) {
    log(MAGENTA + "Restarting..." + CLR);
    return 0;
  }

  log(LIME + "Launching the Bridge Node" + CLR);

  rclcpp::init(argc, argv);
  rclcpp::uninstall_signal_handlers();

  std::signal(SIGINT, signal_handler);

  rclcpp::ExecutorOptions options;
  //options.number_of_threads = 4;  // Custom thread pool size
  rclcpp::executors::MultiThreadedExecutor executor(options, 0); // 0 = auto

  log("Executor using " + std::to_string(executor.get_number_of_threads())+ " threads");

  auto config = std::make_shared<BridgeConfig>();

  auto ros_distro = std::getenv("ROS_DISTRO");
  config->ros_distro = ros_distro == NULL || strlen(ros_distro) == 0 ? "unknown" : ros_distro;
  log("ROS distro is: " + YELLOW + config->ros_distro + CLR);
  readGitRepoHead("/ros2_ws/src/phntm_bridge", config);
  log("Git commit: " + YELLOW + config->git_head_sha + CLR + " Tag: "+ YELLOW + (config->latest_git_tag.empty() ? "-" : config->latest_git_tag) + CLR);

  auto rmw_implementation =  std::getenv("RMW_IMPLEMENTATION");
  config->rmw_implementation = rmw_implementation == NULL || strlen(rmw_implementation) == 0 ? "default" : rmw_implementation;
  log("RMW implementation is: " + YELLOW + config->rmw_implementation + CLR);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.allow_undeclared_parameters(true);
  auto base_node = std::make_shared<PhntmBridge>("phntm_bridge", node_options, config);
  base_node->loadConfig(config);
  base_node->setupLocalServices();
  // auto introspection_node = std::make_shared<rclcpp::Node>("phntm_introspection");
  executor.add_node(base_node);
  // executor.add_node(introspection_node);

  StatusLEDs::init(base_node, config);

  BridgeSocket::init(base_node, config);
  Introspection::init(base_node, config);
  Introspection::start();
  BridgeSocket::connect();

  FileExtractor::init(base_node);

  WRTCPeer::initLogging(config);

  while (!g_interrupt_requested.load() && rclcpp::ok()) {
    executor.spin_once(std::chrono::nanoseconds(100));
  }
    
  log(BLUE + "Shutting down..." + CLR);

  FileExtractor::stop();

  Introspection::stop();
  StatusLEDs::clear();
  BridgeSocket::shutdown();

  log("Spinning ROS node some more...");
  executor.spin_some(); // make sure we send out what we need before shutdown
  log("Done spinning");

  while (WRTCPeer::anyPeersConnected()) {
    log("Waiting for peers cleanup...");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();

  log(BLUE + "Rclcpp down..." + CLR);

  return 0;
}
