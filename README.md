# AG-LOAM
This is the repository for our work "Adaptive LiDAR Odometry and Mapping for Autonomous Agricultural
Mobile Robots in Unmanned Farms" on Computers and Electronics in Agriculture.

Preprint paper available on arXiv here: https://arxiv.org/abs/2412.02899

Source code to be released soon.


## Dataset
### Folder Structure
We provide our dataset in both ROS1 and ROS2 versions. The folder structure is illustrated as follows.
```
ag-loam-dataset-ros1/
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

ag-loam-dataset-ros2/
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
aws s3 sync --no-sign-request s3://ucr-robotics/ag-loam-dataset/ /path/to/local/directory
```
Alternatively, you may download the dataset from Google Drive:
- [Dataset in ROS1 format](https://drive.google.com/drive/folders/1Gi0Dn5dy4fnmKG0b8WryUPBYlGbNYHfY?usp=sharing)
- [Dataset in ROS2 format](https://drive.google.com/drive/folders/1q8WQcG558-dlwfYI4HoLwrBXTTDYARYE?usp=sharing)
