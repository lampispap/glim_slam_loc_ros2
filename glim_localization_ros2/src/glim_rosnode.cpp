#include <spdlog/spdlog.h>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim_ros/glim_ros.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto glim = std::make_shared<glim::GlimROS>(options);

  // MultiThreadedExecutor is required so the ~/global_localize service handler
  // can block on the HDL future while other callbacks (e.g. point cloud) keep running.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(std::static_pointer_cast<rclcpp::Node>(glim)->get_node_base_interface());

  try {
    exec.spin();
  }

  catch (std::exception& ex) {
  }

  rclcpp::shutdown();

  // std::string dump_path = "/tmp/dump";
  // // glim->declare_parameter<std::string>("dump_path", dump_path);
  // glim->get_parameter<std::string>("dump_path", dump_path);

  // glim->wait();
  // glim->save(dump_path);

  return 0;
}