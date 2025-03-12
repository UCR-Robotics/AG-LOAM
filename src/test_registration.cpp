// The AG-LOAM project
// Hanzhe Teng, April 2022

#include <iostream>
#include <string>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include "mt_gicp/mt_gicp.h"

class Registration {
 public:
  using PointT = pcl::PointXYZINormal;
  using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

  Registration(ros::NodeHandle& nh);
  ~Registration() {}

 protected:
  void visualizeRegistrationPCL(const PointCloudPtr& source, const PointCloudPtr& source_transformed,
                                const PointCloudPtr& target) const;
  void visualizeRegistrationRViz(const PointCloudPtr& source, const PointCloudPtr& source_transformed,
                                 const PointCloudPtr& target) const;
  Eigen::Matrix4d GICP(const PointCloudPtr& source, const PointCloudPtr& target, YAML::Node& node) const;

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_source_;
  ros::Publisher pub_target_;
  ros::Publisher pub_source_transformed_;
  YAML::Node params_;
};

Registration::Registration(ros::NodeHandle& nh) : nh_(nh) {
  // load parameters
  std::string param_filename;
  nh_.getParam("/param_filename", param_filename);
  params_ = YAML::LoadFile(param_filename);
  std::string source_name = params_["test_registration"]["source_cloud"].as<std::string>();
  std::string target_name = params_["test_registration"]["target_cloud"].as<std::string>();
  std::string visualization = params_["test_registration"]["visualization"].as<std::string>();
  YAML::Node scan_gicp_params = params_["scan_gicp"];

  // initialization
  pub_source_ = nh_.advertise<sensor_msgs::PointCloud2>("/diagnostics/source", 5, true);
  pub_target_ = nh_.advertise<sensor_msgs::PointCloud2>("/diagnostics/target", 5, true);
  pub_source_transformed_ = nh_.advertise<sensor_msgs::PointCloud2>("/diagnostics/source_transformed", 5, true);

  // run registration
  pcl::PointCloud<PointT>::Ptr source_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr target_cloud(new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile(source_name, *source_cloud);
  pcl::io::loadPCDFile(target_name, *target_cloud);

  float voxel_size = params_["voxel_size"].as<float>();
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_filter.setInputCloud(source_cloud);
  voxel_filter.filter(*source_cloud);
  voxel_filter.setInputCloud(target_cloud);
  voxel_filter.filter(*target_cloud);

  Eigen::Matrix4d transformation = GICP(source_cloud, target_cloud, scan_gicp_params);

  // visualization
  pcl::PointCloud<PointT>::Ptr source_cloud_transformed(new pcl::PointCloud<PointT>);
  pcl::transformPointCloudWithNormals(*source_cloud, *source_cloud_transformed, transformation);
  if (visualization == "pcl")
    visualizeRegistrationPCL(source_cloud, source_cloud_transformed, target_cloud);
  else if (visualization == "rviz")
    visualizeRegistrationRViz(source_cloud, source_cloud_transformed, target_cloud);
}

void Registration::visualizeRegistrationPCL(const PointCloudPtr& source, const PointCloudPtr& source_transformed,
                                            const PointCloudPtr& target) const {
  // Add point clouds to the viewer
  pcl::visualization::PCLVisualizer visualizer;
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_color_handler(source, 255, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_trans_color_handler(source_transformed, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_color_handler(target, 0, 255, 255);
  visualizer.addPointCloud(source, src_color_handler, "source cloud");
  visualizer.addPointCloud(source_transformed, src_trans_color_handler, "source cloud transformed");
  visualizer.addPointCloud(target, tgt_color_handler, "target cloud");

  while (!visualizer.wasStopped()) {
    visualizer.spinOnce();
    pcl_sleep(0.01);
  }
}

void Registration::visualizeRegistrationRViz(const PointCloudPtr& source, const PointCloudPtr& source_transformed,
                                             const PointCloudPtr& target) const {
  ros::Rate rate(1);
  while (ros::ok()) {
    rate.sleep();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*source, cloud_msg);  // this will set header to be empty
    cloud_msg.header = pcl_conversions::fromPCL(source->header);
    cloud_msg.header.frame_id = "diagnostics";
    pub_source_.publish(cloud_msg);

    pcl::toROSMsg(*target, cloud_msg);  // this will set header to be empty
    cloud_msg.header = pcl_conversions::fromPCL(target->header);
    cloud_msg.header.frame_id = "diagnostics";
    pub_target_.publish(cloud_msg);

    pcl::toROSMsg(*source_transformed, cloud_msg);  // this will set header to be empty
    cloud_msg.header = pcl_conversions::fromPCL(source_transformed->header);
    cloud_msg.header.frame_id = "diagnostics";
    pub_source_transformed_.publish(cloud_msg);
  }
}

Eigen::Matrix4d Registration::GICP(const PointCloudPtr& source, const PointCloudPtr& target, YAML::Node& node) const {
  std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

  pcl::PointCloud<PointT> registration_output;
  pcl::MTGeneralizedIterativeClosestPoint<PointT, PointT> gicp;
  gicp.setMaximumIterations(node["max_iteration"].as<int>());
  gicp.setMaxCorrespondenceDistance(node["max_distance"].as<float>());
  gicp.setTransformationEpsilon(node["transform_epsilon"].as<float>());
  gicp.setEuclideanFitnessEpsilon(node["fitness_epsilon"].as<float>());
  gicp.setNumThreads(node["num_threads"].as<int>());
  gicp.setInputSource(source);
  gicp.setInputTarget(target);
  gicp.align(registration_output);
  std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
  auto time_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic);
  return gicp.getFinalTransformation().cast<double>();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "maclo");
  ros::NodeHandle nh("~");
  Registration registration(nh);
  return 0;
}