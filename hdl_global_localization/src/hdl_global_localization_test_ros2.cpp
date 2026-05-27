#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <hdl_global_localization/srv/query_global_localization.hpp>
#include <hdl_global_localization/srv/set_global_localization_engine.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hdl_global_localization {

class GlobalLocalizationTestNode : public rclcpp::Node {
  public:
  GlobalLocalizationTestNode(rclcpp::NodeOptions& options)
      : rclcpp::Node("hdl_global_localization_test", options) {
    set_engine_client_ = this->create_client<srv::SetGlobalLocalizationEngine>(
            "/hdl_global_localization/set_engine");
    set_global_map_client_ = this->create_client<srv::SetGlobalMap>(
            "/hdl_global_localization/set_global_map");
    query_client_ = this->create_client<srv::QueryGlobalLocalization>(
            "/hdl_global_localization/query");

    // transient_local is the ROS2 equivalent of ROS1 latched publisher
    globalmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/globalmap", rclcpp::QoS(1).transient_local());
    points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/aligned_points", 1);
    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 1,
            std::bind(&GlobalLocalizationTestNode::points_callback, this,
                      std::placeholders::_1));
  }

  void set_engine(const std::string& engine_name) {
    while (!set_engine_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) return;
      RCLCPP_INFO(this->get_logger(), "Waiting for set_engine service...");
    }

    auto request =
            std::make_shared<srv::SetGlobalLocalizationEngine::Request>();
    request->engine_name.data = engine_name;

    auto future = set_engine_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Failed to set global localization engine");
    }
  }

  void set_global_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    while (!set_global_map_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) return;
      RCLCPP_INFO(this->get_logger(), "Waiting for set_global_map service...");
    }

    auto request = std::make_shared<srv::SetGlobalMap::Request>();
    pcl::toROSMsg(*cloud, request->global_map);

    auto future = set_global_map_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Failed to set global map");
    }

    sensor_msgs::msg::PointCloud2 globalmap_msg;
    pcl::toROSMsg(*cloud, globalmap_msg);
    globalmap_msg.header.frame_id = "map";
    globalmap_msg.header.stamp = this->now();
    globalmap_pub_->publish(globalmap_msg);
  }

  private:
  void points_callback(
          const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Callback");

    auto request = std::make_shared<srv::QueryGlobalLocalization::Request>();
    request->cloud = *cloud_msg;
    request->max_num_candidates = 1;

    query_client_->async_send_request(
            request,
            [this, cloud_msg](
                    rclcpp::Client<srv::QueryGlobalLocalization>::SharedFuture
                            future) {
              auto response = future.get();
              if (response->poses.empty()) {
                RCLCPP_INFO_STREAM(
                        this->get_logger(),
                        "Failed to find a global localization solution");
                return;
              }

              const auto& estimated = response->poses[0];
              Eigen::Quaternionf quat(
                      estimated.orientation.w, estimated.orientation.x,
                      estimated.orientation.y, estimated.orientation.z);
              Eigen::Vector3f trans(estimated.position.x, estimated.position.y,
                                    estimated.position.z);

              Eigen::Isometry3f transformation = Eigen::Isometry3f::Identity();
              transformation.linear() = quat.toRotationMatrix();
              transformation.translation() = trans;

              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                      new pcl::PointCloud<pcl::PointXYZ>);
              pcl::fromROSMsg(*cloud_msg, *cloud);

              pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(
                      new pcl::PointCloud<pcl::PointXYZ>);
              pcl::transformPointCloud(*cloud, *transformed, transformation);

              sensor_msgs::msg::PointCloud2 out_msg;
              pcl::toROSMsg(*transformed, out_msg);
              out_msg.header.frame_id = "map";
              out_msg.header.stamp = cloud_msg->header.stamp;
              points_pub_->publish(out_msg);
            });
  }

  rclcpp::Client<srv::SetGlobalLocalizationEngine>::SharedPtr
          set_engine_client_;
  rclcpp::Client<srv::SetGlobalMap>::SharedPtr set_global_map_client_;
  rclcpp::Client<srv::QueryGlobalLocalization>::SharedPtr query_client_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
};

}  // namespace hdl_global_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  const std::string global_map_path =
          "/home/lampis/study_ws/glim_loc_ws/test_maps/gz_playground_dense/"
          "pcd_map_points.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(
          new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(global_map_path, *global_map);

  rclcpp::NodeOptions options;
  auto node =
          std::make_shared<hdl_global_localization::GlobalLocalizationTestNode>(
                  options);
  // node->set_engine("FPFH_RANSAC");
  node->set_global_map(global_map);

  rclcpp::spin(node);

  return 0;
}
