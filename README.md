# LLS-LOAM
Lidar Odometry and Mapping with Mutiple Metrics Linear Least Square ICP

## How to use

1. Install dependent 3rd libraries: 
PCL, OpenCV, Eigen, VTK, Glog, Gflags.

If you'd like to use your own data and want to know its absolute projected coordinate, install Proj.

2. Compile
```
mkdir build
cd build
cmake ..
make 
```

3. Run
```
cd ..
# If you'd like to test on kitti dataset
# See below for data preparation
sh script/kitti/kitti_xx.sh
# If you'd like to test on your own dataset, you need to create new shell files and then run it
```

## How to prepare the data
For kitti dataset, you may refer to https://github.com/YuePanEdward/kitti_bin2pcd.

After transforming the point cloud from .bin to .pcd, rename the point cloud folder HDL64.

Horizontally, create a folder called OXTS, and put the ground truth pose .txt file (which can be download from kitti's website) into it.

Then create a pcd file list:
```
touch file_list.txt
ls HDL64 >> filelist.txt
```

Then the data is ready for test.

For your own dataset, if the point cloud are in pcd format and the pose and imu information are in oxts format, then you can directly use this programme (LoadPcImuGnss Method). Or you need to transform the data youself.

## TO DO LIST

0.For the loop closure and pose graph optimization module, the code is not released.

The walkaround is here: https://github.com/YuePanEdward/CloudControlNet

1.Speed up

2.Weighting

3.Add ROS module

## Demo

### On KITTI dataset

![alt text](assets/kitti_00_demo.png)

![alt text](assets/kitti_02_demo.png)

![alt text](assets/kitti_05_demo.png)

![alt text](assets/kitti_07_demo.png)

### On own dataset

![alt text](assets/owndata_demo.png)
