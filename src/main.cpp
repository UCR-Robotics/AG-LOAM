// The AG-LOAM project
// Hanzhe Teng, April 2022

#include "ag_loam/ag_loam.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ag_loam");
  ros::NodeHandle nh("~");
  ag_loam::AgLoam LOAM(nh);
  ros::spin();
  return 0;
}
