#pragma once

#include <any>
#include <deque>
#include <random>
#include <map>
#include <memory>
#include <glim/mapping/sub_mapping_base.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
class PreintegratedImuMeasurements;
}  // namespace gtsam

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;

/**
 * @brief Sub mapping parameters
 */
struct SubMappingParams {
public:
  SubMappingParams();
  ~SubMappingParams();

public:
  bool enable_gpu;
  bool enable_imu;
  bool enable_optimization;
  // Keyframe update strategy params
  int max_num_keyframes;
  std::string keyframe_update_strategy;
  int keyframe_update_min_points;
  double keyframe_update_interval_rot;
  double keyframe_update_interval_trans;
  double keyframe_update_interval_time;
  double max_keyframe_overlap;

  bool create_between_factors;
  std::string between_registration_type;

  std::string registration_error_factor_type;
  double keyframe_randomsampling_rate;
  double keyframe_voxel_resolution;
  int keyframe_voxelmap_levels;
  double keyframe_voxelmap_scaling_factor;

  double submap_downsample_resolution;
  double submap_voxel_resolution;
  int submap_target_num_points;

  bool enable_gps;
  double gps_horizontal_weight;
  double gps_vertical_weight;
  double gps_max_hdop;
  double gps_max_distance;
};

/**
 * @brief Sub mapping
 */
class SubMapping : public SubMappingBase {
public:
  SubMapping(const SubMappingParams& params = SubMappingParams());
  virtual ~SubMapping() override;


  virtual void insert_gps(double stamp, const Eigen::Vector4d& lat_lon_alt) override;
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_frame(const EstimationFrame::ConstPtr& odom_frame) override;

  virtual std::vector<SubMap::Ptr> get_submaps() override;

  virtual std::vector<SubMap::Ptr> submit_end_of_sequence() override;

private:
  void insert_keyframe(const int current, const EstimationFrame::ConstPtr& odom_frame);
  gtsam::NonlinearFactorGraph create_gps_factor(const int current, gtsam::Values& new_values);

  SubMap::Ptr create_submap(bool force_create = false) const;

private:
  using Params = SubMappingParams;
  Params params;

  std::mt19937 mt;
  int submap_count;

  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  std::shared_ptr<void> stream;
  std::shared_ptr<void> stream_buffer_roundrobin;

  std::deque<EstimationFrame::ConstPtr> delayed_input_queue;
  std::vector<EstimationFrame::ConstPtr> odom_frames;
  std::map<double, int> gps_frames;

  std::vector<int> keyframe_indices;
  std::vector<EstimationFrame::Ptr> keyframes;

  std::unique_ptr<gtsam::Values> values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

  std::vector<SubMap::Ptr> submap_queue;

  std::shared_ptr<void> tbb_task_arena;

  GeographicLib::LocalCartesian geoConverter;
  std::deque<std::pair<double, Eigen::Vector4d>> gps_queue;
  bool geoconverter_initialized {false};

  Eigen::Isometry3d T_ned_odom;
  Eigen::Isometry3d T_world_odom;
  int gps_data_id;
};

}  // namespace glim
