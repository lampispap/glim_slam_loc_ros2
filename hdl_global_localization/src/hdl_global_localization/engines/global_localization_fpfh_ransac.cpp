#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>

#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>
#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>
#include <pcl/search/impl/kdtree.hpp>

namespace hdl_global_localization {

GlobalLocalizationEngineFPFH_RANSAC::GlobalLocalizationEngineFPFH_RANSAC(
        const GlobalLocalizationEngineFPFH_RANSACParams& params)
    : params(params) {}

GlobalLocalizationEngineFPFH_RANSAC::~GlobalLocalizationEngineFPFH_RANSAC() {}

namespace {

// A descriptor is usable only if every histogram bin is finite. NaN/Inf bins
// arise when the underlying normal could not be estimated (too few neighbors).
bool is_finite_feature(const pcl::FPFHSignature33& feature) {
  for (int i = 0; i < pcl::FPFHSignature33::descriptorSize(); i++) {
    if (!std::isfinite(feature.histogram[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr
GlobalLocalizationEngineFPFH_RANSAC::extract_fpfh(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {
  double normal_estimation_radius = params.normal_estimation_radius;
  double search_radius = params.search_radius;

  // Drop NaN/Inf input points up front; they would yield NaN normals anyway.
  pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> kept_indices;
  pcl::removeNaNFromPointCloud(*cloud, *clean_cloud, kept_indices);
  if (clean_cloud->size() != cloud->size()) {
    spdlog::warn("Removed {} NaN/Inf input points before FPFH extraction",
                 cloud->size() - clean_cloud->size());
  }

  spdlog::info("Normal Estimation: Radius({})", normal_estimation_radius);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
  nest.setRadiusSearch(normal_estimation_radius);
  nest.setInputCloud(clean_cloud);
  nest.compute(*normals);

  spdlog::info("FPFH Extraction: Search Radius({})", search_radius);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(
          new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch(search_radius);
  fest.setInputCloud(clean_cloud);
  fest.setInputNormals(normals);
  fest.compute(*features);

  // Keep only finite descriptors and the points they correspond to, so the two
  // clouds stay index-aligned for RANSAC. A NaN descriptor reaching the FLANN
  // kd-tree's nearestKSearch would otherwise trip an assertion and abort.
  filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr filtered_features(
          new pcl::PointCloud<pcl::FPFHSignature33>);
  filtered_cloud->reserve(features->size());
  filtered_features->reserve(features->size());

  for (std::size_t i = 0; i < features->size(); i++) {
    if (is_finite_feature(features->at(i))) {
      filtered_cloud->push_back(clean_cloud->at(i));
      filtered_features->push_back(features->at(i));
    }
  }
  filtered_cloud->width = filtered_cloud->size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;
  filtered_features->width = filtered_features->size();
  filtered_features->height = 1;
  filtered_features->is_dense = true;

  const std::size_t dropped = features->size() - filtered_features->size();
  if (dropped) {
    spdlog::warn("Dropped {} non-finite FPFH descriptors ({} remaining)",
                 dropped, filtered_features->size());
  }

  return filtered_features;
}

void GlobalLocalizationEngineFPFH_RANSAC::set_global_map(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  global_map_features = extract_fpfh(cloud, filtered_cloud);
  global_map = filtered_cloud;

  ransac.reset(
          new RansacPoseEstimation<pcl::FPFHSignature33>(params.ransac_params));
  ransac->set_target(global_map, global_map_features);
}

GlobalLocalizationResults GlobalLocalizationEngineFPFH_RANSAC::query(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        int max_num_candidates) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr cloud_features =
          extract_fpfh(cloud, filtered_cloud);

  ransac->set_source(filtered_cloud, cloud_features);
  auto results = ransac->estimate();

  return results.sort(max_num_candidates);
}

}  // namespace hdl_global_localization