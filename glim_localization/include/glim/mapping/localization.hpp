#pragma once

#include <any>
#include <memory>
#include <random>
#include <thread>
#include <map>
#include <boost/shared_ptr.hpp>
#include <glim/mapping/global_mapping_base.hpp>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/sub_map.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class ISAM2Ext;
class StreamTempBufferRoundRobin;
struct ISAM2ResultExt;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;

struct LocalizationParams: public GlobalMappingParams {
public:
  LocalizationParams();
  ~LocalizationParams();

public:
  double max_localization_distance;
  double min_localization_overlap;
  double linear_search_window;
  double angular_search_window;
  double relocalization_factor_weight;
  double loc_between_factor_weight;
  std::string output_debug_factor_graph;
  int num_keep_submaps;
};


class Localization : public GlobalMappingBase {
public:
  Localization(const LocalizationParams& params = LocalizationParams());
  virtual ~Localization();

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual void insert_submap(const SubMap::Ptr& submap) override;
  virtual void relocalize(EstimationFrame::ConstPtr latest, const Eigen::Isometry3d & initial_pose) override;

  // virtual void find_overlapping_submaps(double min_overlap) override;
  virtual void optimize() override;

  // virtual void save(const std::string& path) override;
  // virtual std::vector<Eigen::Vector4d> export_points() override;

  /**
   * @brief Load a mapping result from a dumped directory
   * @param path Input dump path
   */
  // bool load_pose_graph(const std::string& path);
  virtual bool load(const std::string& path) override;
  bool load_ply(const std::string& path);
protected:

private:

  void insert_submap(int current, const SubMap::Ptr& submap);
  void update_submaps();

  std::shared_ptr<gtsam::NonlinearFactorGraph> create_between_factors(int current) ;
  // std::shared_ptr<gtsam::NonlinearFactorGraph> create_matching_cost_factors(int current) ;
  std::shared_ptr<gtsam::NonlinearFactorGraph> create_map_matching_cost_factors(
    int current, const Eigen::Isometry3d& current_T_world_submap);
  std::shared_ptr<gtsam::NonlinearFactorGraph> create_relocalization_factors(
    int submap_id,
    gtsam_points::PointCloud::ConstPtr cloud, const Eigen::Isometry3d& submap_pose,
    const Eigen::Isometry3d& initial_pose);

  Eigen::Isometry3d find_best_candidate(
    const SubMap::Ptr& target_map, gtsam_points::PointCloud::ConstPtr cloud, const Eigen::Isometry3d& submap_pose,
    double linear_search_window, double angular_search_window,
    Eigen::Isometry3d initial_pose, double& overlap_score);

  gtsam_points::ISAM2ResultExt update_isam2(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values);

  void trim_submap(int submap_index);

private:
  using Params = LocalizationParams;
  Params params;

  std::mt19937 mt;

  std::unique_ptr<IMUIntegration> imu_integration;
  std::any stream_buffer_roundrobin;

  std::map<int, SubMap::Ptr> active_submaps;
  std::vector<SubMap::Ptr> prebuilt_submaps;
  std::map<int, gtsam_points::PointCloud::ConstPtr> subsampled_active_submaps;
  size_t next_active_submap_id;
  std::vector<gtsam_points::PointCloud::ConstPtr> prebuilt_subsampled_submaps;

  std::unique_ptr<gtsam::Values> new_values;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors;

  std::unique_ptr<gtsam_points::ISAM2Ext> isam2;

  std::shared_ptr<void> tbb_task_arena;

  bool do_relocalization = false;
  bool relocalized = false;
  Eigen::Isometry3d initial_pose_;
  EstimationFrame::ConstPtr latest_frame_;
  std::shared_ptr<gtsam::NonlinearFactorGraph> relocalization_factors_;
  std::shared_ptr<std::thread> relocalize_thread_;
  int target_submap_id_{-1};
  int query_submap_id_{-1};
  // Submap::Ptr relocalize_submap_;
};
}  // namespace glim