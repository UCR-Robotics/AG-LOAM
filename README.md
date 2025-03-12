# AG-LOAM
## Introduction
AG-LOAM is a light-weight LiDAR-only odometry and mapping framework designed for autonomous mobile robots/vehicles operating in unstructured agricultural environments.
- The proposed framework consists of a robust LiDAR odometry algorithm based on dense GICP scan matching, and an adaptive mapping module that considers motion stability and point cloud consistency for selective map updates.
- The key design principle of this framework is to prioritize the incremental consistency of the map by rejecting motion-distorted points and sparse dynamic objects, which in turn leads to high accuracy in odometry estimated from scan matching against the map.
- The developed method has been validated on our field datasets over three phases (featuring various planting types, terrain types, and robot motion profiles), as well as public datasets such as the [TreeScope dataset](https://treescope.org/).

In addition to this source code, we also made publicly available
- our LiDAR odometry benchmark tools at [Odometry-Benchmark](https://github.com/UCR-Robotics/Odometry-Benchmark), and
- the datasets collected in this work (see the end of this README file for instructions on how to use/download).

This work was developed primarily in Summer 2022. The motivation behind it is to provide accurate localization for agricultural vechicles (used by many farmers and growers) with simplified LiDAR integration, eliminating the spatial and temporal synchronization step required by many LiDAR-inertial frameworks. A single LiDAR is all you need—just mount it and go!

**Authors:** Hanzhe Teng, Yipeng Wang, Dimitrios Chatziparaschis and Konstantinos Karydis from [ARCS Lab](https://sites.google.com/view/arcs-lab/) at [UC Riverside](https://www.ucr.edu/).

**Related Publications:**
H. Teng, Y. Wang, D. Chatziparaschis and K. Karydis, "Adaptive LiDAR Odometry and Mapping for Autonomous Agricultural Mobile Robots in Unmanned Farms", Computers and Electronics in Agriculture (COMPAG 2025). [[paper](https://www.sciencedirect.com/science/article/pii/S0168169925001292)] [[preprint](https://arxiv.org/abs/2412.02899)]
```
@article{teng2025adaptive,
  title={Adaptive LiDAR Odometry and Mapping for Autonomous Agricultural Mobile Robots in Unmanned Farms},
  author={Teng, Hanzhe and Wang, Yipeng and Chatziparaschis, Dimitrios and Karydis, Konstantinos},
  journal={Computers and Electronics in Agriculture},
  volume={232},
  pages={110023},
  year={2025}
}
```

## Installation
### Dependencies
- Ubuntu 20 and ROS Noetic
- CMake, PCL, Eigen, yaml-cpp, OpenMP (all in the default version in Ubuntu 20)
```
sudo apt install cmake libpcl-dev libeigen3-dev libyaml-cpp-dev libomp-dev
```

### Build and Run
Build this package in the catkin workspace (`catkin build` or `catkin_make`) and run
```
roslaunch ag_loam run.launch
```

Play rosbag in a separate terminal (make sure ROS topics are set properly in `config/params.yaml`)
```
rosbag play <filename>.bag
```

### Notes
To visualize the map, you can run with rviz enabled, but this may slow down the algorithm in large-scale environments. 
```
roslaunch ag_loam rviz.launch            # option 1
roslaunch ag_loam run.launch rviz:=true  # option 2
```
To monitor key updates from the algorithm, you can open `rqt_console` in a separate terminal and exclude `info` level messages.
```
rqt_console
```

### Coding Style
- Set up clang-format and git hooks.
  ```
  ./scripts/clang_setup/clang_setup.sh
  ```
- To fix coding style before every git commit, run
  ```
  git clang-format -f
  ```

## Dataset
AG-LOAM dataset has been released to facilitate the evaluation of LiDAR-based odometry algorithms in agricultural environments.
- It was collected by a wheeled mobile robot at the [Agricultural Experimental Station](https://cnas.ucr.edu/resources/agricultural-experiment-station) of the [University of California Riverside](https://www.ucr.edu/), during Winter 2022 and Winter 2023.
- It provides LiDAR point cloud data captured using a Velodyne VLP-16 sensor, along with ground-truth trajectories obtained from an RTK-GPS system.
- It consists of 18 sequences collected over three phases, covering diverse planting environments, terrain conditions, path patterns, and robot motion profiles.
- It spans a total operation time of 3 hours, covers a total distance of 7.5 km, and constitutes 150 GB of data.

### Folder Structure
We provide our dataset in both ROS1 and ROS2 versions. The folder structure is illustrated as follows.
```
ag-loam-dataset/ros1/
├── ground_truth/
│   ├── C1/
│   │   ├── gt.bag
│   │   └── gt.csv
│   ├── C2/
│   ├── C3/
│   └── ...
├── a1.bag
├── a2.bag
├── ...

ag-loam-dataset/ros2/
├── a1/
│   ├── metadata.yaml
│   └── a1.db3
├── a2/
├── ...
├── c1/
│   ├── metadata.yaml
│   ├── c1.db3
│   └── gt.csv
├── c2/
├── ...
```

### Download
We host [our dataset](https://registry.opendata.aws/ag-loam/) on Amazon Web Services (AWS), sponsored by AWS [Open Data program](https://aws.amazon.com/opendata/open-data-sponsorship-program/).
If you have a Ubuntu computer, you can download all data from the S3 bucket using AWS CLI tool (No AWS account required):
```
sudo apt install awscli
aws s3 sync --no-sign-request s3://ucr-robotics/ag-loam-dataset/ros1/ /path/to/local/directory
```
Alternatively, you may download the dataset from Google Drive:
- [Dataset in ROS1 format](https://drive.google.com/drive/folders/1Gi0Dn5dy4fnmKG0b8WryUPBYlGbNYHfY?usp=sharing)
- [Dataset in ROS2 format](https://drive.google.com/drive/folders/1q8WQcG558-dlwfYI4HoLwrBXTTDYARYE?usp=sharing)

## Acknowledgement
This work is supported in part by NSF, USDA-NIFA, ONR and the University of California UC-MRPI.
We thank Dr. Peggy Mauk and the staff team at UCR's Agricultural Experimental Station for their support in our work.
Furthermore, we thank the authors of [LOCUS](https://arxiv.org/abs/2012.14447) and [LOCUS 2.0](https://arxiv.org/abs/2205.11784), along with their [open-source repository](https://github.com/NeBula-Autonomy/LOCUS), which inspired this work.

## License
- The AG-LOAM open source code is licensed under the terms of MIT license.
- The AG-LOAM dataset is licensed under Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0).
