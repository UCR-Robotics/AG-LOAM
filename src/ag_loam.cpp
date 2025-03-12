// The AG-LOAM project
// Hanzhe Teng, April 2022

#include "ag_loam/ag_loam.h"

namespace ag_loam {

AgLoam::AgLoam(ros::NodeHandle& nh)
    : nh_(nh),
      initialized_(false),
      stable_last_frame_(false),
      voxel_size_(0.1),
      tf_incremental_(Eigen::Matrix4d::Identity()),
      tf_incremental_last_frame_(Eigen::Matrix4d::Identity()),
      tf_integrated_(Eigen::Matrix4d::Identity()),
      tf_integrated_last_keyframe_(Eigen::Matrix4d::Identity()),
      last_cloud_frame_(new PointCloudT),
      last_last_cloud_frame_(new PointCloudT),
      map_(new PointCloudT),
      kdtree_(new pcl::search::KdTree<PointT>) {
  std::string param_filename;
  nh_.getParam("/param_filename", param_filename);
  params_ = YAML::LoadFile(param_filename);

  // global settings
  child_frame_id_ = params_["child_frame_id"].as<std::string>();
  fixed_frame_id_ = params_["fixed_frame_id"].as<std::string>();
  normal_estimator_.setSearchMethod(kdtree_);
  normal_estimator_.setKSearch(params_["num_normal_neighbors"].as<int>());
  num_threads_ = params_["num_threads"].as<int>();
  map_->header.frame_id = fixed_frame_id_;
  map_octree_.reset(new OctreeT(params_["map_octree_resolution"].as<double>()));
  map_octree_->setInputCloud(map_);

  // pipeline params
  keyframe_translation_threshold_ = params_["pipeline"]["keyframe_translation_threshold"].as<double>();
  rotation_threshold_stability_ = params_["pipeline"]["rotation_threshold_stability"].as<double>();
  consistency_filter_use_normal_ = params_["pipeline"]["consistency_filter_use_normal_distance"].as<bool>();
  consistency_filter_threshold_ = params_["pipeline"]["consistency_filter_distance_threshold"].as<double>();

  // pcl voxel filter
  voxel_size_ = params_["voxel_filter"]["voxel_size"].as<double>();
  voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

  // ros utilities
  std::string lidar_topic = params_["lidar_topic"].as<std::string>();
  sub_laser_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100, &AgLoam::LidarCallback, this);
  pub_cloud_frame_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_frame", 10, true);
  pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/map", 10, true);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/path", 10, true);
  pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("/odometry", 10, true);

  // begin odometry thread
  thread_odometry_ = std::thread(&AgLoam::OdometryThread, this);
  ROS_INFO("Color-LOAM started");
}

AgLoam::~AgLoam() {
  if (thread_odometry_.joinable()) {
    thread_odometry_.join();
  }
}

void AgLoam::LidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  auto tic = std::chrono::steady_clock::now();
  PointCloudPtr cloud_frame(new PointCloudT);
  pcl::fromROSMsg(*cloud_msg, *cloud_frame);

  // downsample point cloud
  voxel_filter_.setInputCloud(cloud_frame);
  voxel_filter_.filter(*cloud_frame);
  if (params_["voxel_filter"]["adaptive_voxel_size"].as<bool>()) {
    AdaptiveVoxelGridFilter(cloud_frame);
  }

  // normal estimation
  normal_estimator_.setInputCloud(cloud_frame);
  normal_estimator_.compute(*cloud_frame);

  // save processed cloud
  cloud_mutex_.lock();
  cloud_frame_buffer_.push_back(cloud_frame);
  cloud_mutex_.unlock();
  auto toc = std::chrono::steady_clock::now();
  double time_elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() / 1000000.0;
  ROS_INFO_STREAM("Callback: cloud size = " << cloud_frame->size() << "; total time " << time_elapsed_ms << " ms");
}

void AgLoam::OdometryThread() {
  ROS_INFO("Odometry thread started");
  ros::Rate rate(100);
  while (ros::ok()) {
    // run odometry estimation once a new cloud frame is ready
    rate.sleep();
    cloud_mutex_.lock();
    int buffer_size = static_cast<int>(cloud_frame_buffer_.size());
    if (buffer_size == 0) {
      cloud_mutex_.unlock();
      continue;
    }
    PointCloudPtr cloud_frame = cloud_frame_buffer_.front();
    cloud_frame_buffer_.pop_front();
    cloud_mutex_.unlock();
    if (buffer_size > 3) {
      ROS_WARN_STREAM_THROTTLE(1.0, "Running behind! Current cloud buffer size = " << buffer_size);
    }

    // initialization
    if (!initialized_) {
      last_cloud_frame_ = cloud_frame;
      UpdateMap(cloud_frame);
      map_->header.stamp = cloud_frame->header.stamp;
      PublishPointCloud(pub_map_, map_, fixed_frame_id_);
      initialized_ = true;
      ROS_INFO("Initialization completed");
      continue;
    }
    auto tic = std::chrono::steady_clock::now();

    // scan-to-scan transformation estimation
    YAML::Node scan_gicp_params = params_["scan_gicp"];
    RegistrationResult scan_result = GICP(cloud_frame, last_cloud_frame_, scan_gicp_params);
    ROS_INFO_STREAM("Scan GICP: time = " << scan_result.time << " ms; fitness = " << scan_result.fitness);
    Eigen::Matrix4d tf_scan_incremental = scan_result.transformation;

    // check motion stability
    bool stable = true;
    if (params_["pipeline"]["check_motion_stability"].as<bool>()) {
      double angle = ag_loam::GetAngleDegree(tf_scan_incremental);
      if (angle > rotation_threshold_stability_) stable = false;
    }

    // transform points to fixed frame
    Eigen::Matrix4d tf_integrated_temp = tf_integrated_ * tf_scan_incremental;
    PointCloudPtr cloud_frame_transformed(new PointCloudT);
    pcl::transformPointCloudWithNormals(*cloud_frame, *cloud_frame_transformed, tf_integrated_temp);

    // find neighbor points in the map
    PointCloudPtr cloud_frame_neighbors(new PointCloudT);
    FindNearestNeighbors(cloud_frame_transformed, cloud_frame_neighbors);

    // transform points back to local sensor frame
    Eigen::Matrix4d tf_integrated_temp_inverse = ag_loam::GetInverseTransform(tf_integrated_temp);
    pcl::transformPointCloudWithNormals(*cloud_frame_neighbors, *cloud_frame_neighbors, tf_integrated_temp_inverse);

    // scan-to-sub transformation estimation
    YAML::Node map_gicp_params = params_["map_gicp"];
    RegistrationResult map_result = GICP(cloud_frame, cloud_frame_neighbors, map_gicp_params);
    ROS_INFO_STREAM("Map GICP: time = " << map_result.time << " ms; fitness = " << map_result.fitness);
    Eigen::Matrix4d tf_map_incremental = map_result.transformation;
    tf_incremental_ = tf_scan_incremental * tf_map_incremental;

    // integrate transform updates and publish current pose
    tf_integrated_ = tf_integrated_ * tf_incremental_;
    PublishOdometry(tf_integrated_, pcl_conversions::fromPCL(cloud_frame->header));
    PublishPointCloud(pub_cloud_frame_, cloud_frame, child_frame_id_);

    // check keyframe condition and update map accordingly
    UpdateKeyframe(cloud_frame, stable && stable_last_frame_);
    tf_incremental_last_frame_ = tf_incremental_;
    last_last_cloud_frame_ = last_cloud_frame_;
    last_cloud_frame_ = cloud_frame;
    stable_last_frame_ = stable;
    auto toc = std::chrono::steady_clock::now();
    double time_elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() / 1000000.0;
    ROS_INFO_STREAM("Odometry: End with total time " << time_elapsed_ms << " ms");
  }
}

void AgLoam::AdaptiveVoxelGridFilter(const PointCloudPtr& cloud) {
  double rate = static_cast<double>(cloud->points.size()) / params_["voxel_filter"]["desired_num_points"].as<double>();
  if (rate > 1.4) {
    voxel_size_ = voxel_size_ * 1.4;  // sqrt(2) = 1.414
    voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    ROS_WARN_STREAM("Adaptive VoxelGrid Filter: voxel size changed to " << voxel_size_ << " m");
  } else if (rate < 0.7) {
    voxel_size_ = voxel_size_ / 1.4;
    voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    ROS_WARN_STREAM("Adaptive VoxelGrid Filter: voxel size changed to " << voxel_size_ << " m");
  }
}

bool AgLoam::CheckConsistency(const PointT& a, const PointT& b, float knn_dist_sqr) {
  if (consistency_filter_use_normal_) {
    ROS_WARN_STREAM_COND(!a.normal_x && !a.normal_y && !a.normal_z, "Consistency filter: Point normal does not exist!");
    return ag_loam::DistanceAlongNormal(a, b) < consistency_filter_threshold_;
  } else {
    return knn_dist_sqr < consistency_filter_threshold_ * consistency_filter_threshold_;
  }
}

void AgLoam::ConsistencyFilter(const PointCloudPtr& cloud_current, const PointCloudPtr& cloud_prev,
                               const PointCloudPtr& cloud_next, PointCloudPtr& cloud_filtered) {
  std::vector<int> indices(1);
  std::vector<float> sqr_dists(1);

  pcl::KdTreeFLANN<PointT> kdtree_prev;
  kdtree_prev.setInputCloud(cloud_prev);
  pcl::KdTreeFLANN<PointT> kdtree_next;
  kdtree_next.setInputCloud(cloud_next);

  PointCloudPtr cloud_trans_to_prev(new PointCloudT);
  pcl::transformPointCloudWithNormals(*cloud_current, *cloud_trans_to_prev, tf_incremental_last_frame_);
  PointCloudPtr cloud_trans_to_next(new PointCloudT);
  pcl::transformPointCloudWithNormals(*cloud_current, *cloud_trans_to_next,
                                      ag_loam::GetInverseTransform(tf_incremental_));

  int prev_counter = 0;
  int next_counter = 0;
  for (size_t i = 0; i < cloud_current->points.size(); ++i) {
    bool prev = false;
    bool next = false;
    if (kdtree_prev.nearestKSearch(cloud_trans_to_prev->points[i], 1, indices, sqr_dists) > 0) {
      prev = CheckConsistency(cloud_trans_to_prev->points[i], cloud_prev->points[indices[0]], sqr_dists[0]);
    }
    if (kdtree_next.nearestKSearch(cloud_trans_to_next->points[i], 1, indices, sqr_dists) > 0) {
      next = CheckConsistency(cloud_trans_to_next->points[i], cloud_next->points[indices[0]], sqr_dists[0]);
    }
    prev_counter += prev ? 1 : 0;
    next_counter += next ? 1 : 0;
    if (prev || next) {
      cloud_filtered->push_back(cloud_current->points[i]);
    }
  }
  ROS_WARN_STREAM("Consistency filter point size: before = "
                  << cloud_current->points.size() << ", after = " << cloud_filtered->points.size()
                  << ", prev = " << prev_counter << ", next = " << next_counter);
}

void AgLoam::UpdateKeyframe(const PointCloudPtr& cloud_frame, bool stable) {
  auto tic = std::chrono::steady_clock::now();
  Eigen::Matrix4d delta = ag_loam::GetInverseTransform(tf_integrated_last_keyframe_) * tf_integrated_;
  double angle = ag_loam::GetAngleDegree(delta);
  double translation = ag_loam::GetTranslation(delta);
  if (translation > keyframe_translation_threshold_) {
    if (stable) {
      ROS_WARN_STREAM("Map update: translation " << translation << " and rotation " << angle << " deg");
      PointCloudPtr cloud_filtered(new PointCloudT);
      ConsistencyFilter(last_cloud_frame_, last_last_cloud_frame_, cloud_frame, cloud_filtered);
      PointCloudPtr cloud_to_map(new PointCloudT);
      pcl::transformPointCloudWithNormals(
          *cloud_filtered, *cloud_to_map,
          (tf_integrated_ * ag_loam::GetInverseTransform(tf_incremental_)).cast<float>());
      UpdateMap(cloud_to_map);
      map_->header.stamp = last_cloud_frame_->header.stamp;
      PublishPointCloud(pub_map_, map_, fixed_frame_id_);
      tf_integrated_last_keyframe_ = tf_integrated_;
    } else {
      ROS_WARN_STREAM("Map update postponed due to unstable motion");
    }
  }
  auto toc = std::chrono::steady_clock::now();
  double time_elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() / 1000000.0;
  ROS_WARN_STREAM_COND(time_elapsed_ms > 10.0, "Keyframe update is time consuming: " << time_elapsed_ms << " ms");
}

bool AgLoam::FindNearestNeighbors(const PointCloudPtr& cloud, const PointCloudPtr& neighbors) {
  neighbors->clear();
  neighbors->resize(cloud->points.size());
#pragma omp parallel for num_threads(num_threads_) schedule(dynamic, 1)
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    float sqr_dist = 0.f;
    int result_index = -1;
    map_octree_->approxNearestSearch(cloud->points[i], result_index, sqr_dist);
    if (result_index >= 0) {
      neighbors->points[i] = map_->points[result_index];
    }
  }
  return neighbors->points.size() > 0;
}

bool AgLoam::UpdateMap(const PointCloudPtr& cloud) {
  // Insert points if one does not already exist on the map
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (!map_octree_->isVoxelOccupiedAtPoint(cloud->points[i])) {
      map_octree_->addPointToCloud(cloud->points[i], map_);
    }
  }
  return true;
}

void AgLoam::PublishOdometry(const Eigen::Matrix4d& pose, const std_msgs::Header& header) {
  // prepare pose
  Eigen::Quaterniond pose_q(pose.block<3, 3>(0, 0));
  Eigen::Vector3d pose_t(pose.block<3, 1>(0, 3));

  // publish odometry
  nav_msgs::Odometry odometry_msg;
  odometry_msg.header = header;
  odometry_msg.header.frame_id = fixed_frame_id_;
  odometry_msg.child_frame_id = child_frame_id_;
  odometry_msg.pose.pose.position.x = pose_t.x();
  odometry_msg.pose.pose.position.y = pose_t.y();
  odometry_msg.pose.pose.position.z = pose_t.z();
  odometry_msg.pose.pose.orientation.x = pose_q.x();
  odometry_msg.pose.pose.orientation.y = pose_q.y();
  odometry_msg.pose.pose.orientation.z = pose_q.z();
  odometry_msg.pose.pose.orientation.w = pose_q.w();
  pub_odometry_.publish(odometry_msg);

  // publish path
  geometry_msgs::PoseStamped current_pose;
  current_pose.header = header;
  current_pose.pose.position.x = pose_t.x();
  current_pose.pose.position.y = pose_t.y();
  current_pose.pose.position.z = pose_t.z();
  current_pose.pose.orientation.x = pose_q.x();
  current_pose.pose.orientation.y = pose_q.y();
  current_pose.pose.orientation.z = pose_q.z();
  current_pose.pose.orientation.w = pose_q.w();
  odom_path_.header = header;
  odom_path_.header.frame_id = fixed_frame_id_;
  odom_path_.poses.push_back(current_pose);
  pub_path_.publish(odom_path_);

  // publish tf
  tf::Transform tf_pub;
  tf_pub.setOrigin(tf::Vector3(pose_t.x(), pose_t.y(), pose_t.z()));
  tf_pub.setRotation(tf::Quaternion(pose_q.x(), pose_q.y(), pose_q.z(), pose_q.w()));
  pub_tf_.sendTransform(tf::StampedTransform(tf_pub, header.stamp, fixed_frame_id_, child_frame_id_));
}

void AgLoam::PublishPointCloud(const ros::Publisher& pub, const PointCloudPtr& cloud, std::string& frame_id) const {
  if (pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);  // this will set header to be empty
    cloud_msg.header = pcl_conversions::fromPCL(cloud->header);
    cloud_msg.header.frame_id = frame_id;
    pub.publish(cloud_msg);
  }
}

RegistrationResult AgLoam::GICP(const PointCloudPtr& source, const PointCloudPtr& target, YAML::Node& node) const {
  auto tic = std::chrono::steady_clock::now();
  PointCloudT unused_output;
  pcl::MTGeneralizedIterativeClosestPoint<PointT, PointT> gicp;
  gicp.setMaximumIterations(node["max_iteration"].as<int>());
  gicp.setMaxCorrespondenceDistance(node["max_distance"].as<float>());
  gicp.setTransformationEpsilon(node["transform_epsilon"].as<float>());
  gicp.setEuclideanFitnessEpsilon(node["fitness_epsilon"].as<float>());
  gicp.setCorrespondenceRandomness(params_["num_normal_neighbors"].as<int>());
  gicp.computeSourceCovariancesFromNormals(node["src_cov_from_normal"].as<bool>());
  gicp.computeTargetCovariancesFromNormals(node["tgt_cov_from_normal"].as<bool>());
  gicp.setNumThreads(num_threads_);
  gicp.setInputSource(source);
  gicp.setInputTarget(target);
  gicp.align(unused_output);
  RegistrationResult result;
  result.transformation = gicp.getFinalTransformation().cast<double>();
  result.fitness = gicp.getFitnessScore(node["max_distance"].as<float>());
  auto toc = std::chrono::steady_clock::now();
  result.time = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() / 1000000.0;
  return result;
}

}  // namespace ag_loam
