// The AG-LOAM project
// Hanzhe Teng, April 2022

#pragma once

#include <atomic>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <yaml-cpp/yaml.h>

#include "ag_loam/utils.hpp"
#include "mt_gicp/mt_gicp.h"

namespace ag_loam {

struct RegistrationResult {
  double time;     // unit: ms
  double fitness;  // unit: meter
  Eigen::Matrix4d transformation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class AgLoam {
 public:
  using PointT = pcl::PointXYZINormal;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
  using OctreeT = typename pcl::octree::OctreePointCloudSearch<PointT>;
  using OctreePtr = typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr;

  AgLoam(ros::NodeHandle& nh);
  ~AgLoam();

  void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void OdometryThread();

 protected:
  void AdaptiveVoxelGridFilter(const PointCloudPtr& cloud);
  void UpdateKeyframe(const PointCloudPtr& cloud, bool stable);
  void PublishOdometry(const Eigen::Matrix4d& pose, const std_msgs::Header& header);
  void PublishPointCloud(const ros::Publisher& pub, const PointCloudPtr& cloud, std::string& frame_id) const;
  RegistrationResult GICP(const PointCloudPtr& source, const PointCloudPtr& target, YAML::Node& node) const;

  bool CheckConsistency(const PointT& a, const PointT& b, float knn_dist_sqr);
  void ConsistencyFilter(const PointCloudPtr& cloud_current, const PointCloudPtr& cloud_prev,
                         const PointCloudPtr& cloud_next, PointCloudPtr& cloud_filtered);

  bool FindNearestNeighbors(const PointCloudPtr& cloud, const PointCloudPtr& neighbors);
  bool UpdateMap(const PointCloudPtr& cloud);

 private:
  // utilities
  ros::NodeHandle nh_;
  ros::Subscriber sub_laser_cloud_;
  ros::Publisher pub_cloud_frame_;
  ros::Publisher pub_map_;
  ros::Publisher pub_path_;
  ros::Publisher pub_odometry_;
  tf::TransformBroadcaster pub_tf_;
  pcl::VoxelGrid<PointT> voxel_filter_;
  pcl::NormalEstimation<PointT, PointT> normal_estimator_;
  pcl::search::KdTree<PointT>::Ptr kdtree_;
  std::thread thread_odometry_;

  // parameters
  YAML::Node params_;
  std::string child_frame_id_;
  std::string fixed_frame_id_;
  float voxel_size_;
  int num_threads_;
  bool initialized_;
  bool stable_last_frame_;
  double keyframe_translation_threshold_;
  double rotation_threshold_stability_;
  bool consistency_filter_use_normal_;
  double consistency_filter_threshold_;

  // state variables, data clouds
  Eigen::Matrix4d tf_incremental_;
  Eigen::Matrix4d tf_incremental_last_frame_;
  Eigen::Matrix4d tf_integrated_;
  Eigen::Matrix4d tf_integrated_last_keyframe_;
  nav_msgs::Path odom_path_;
  std::list<PointCloudPtr> cloud_frame_buffer_;
  std::mutex cloud_mutex_;
  PointCloudPtr last_cloud_frame_;
  PointCloudPtr last_last_cloud_frame_;
  PointCloudPtr map_;
  OctreePtr map_octree_;
};

}  // namespace ag_loam
