# A-LOAM
## Advanced implementation of LOAM

A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED). Compared with origin code, this code is more clean and simpler without complicated mathematical derivation and redundant operations. This code also achieves more accurate and stable performance. It may be the best open-sourced implementation of LOAM on github.

<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti.png" width = 55% height = 55%/>


**Modifier:** [Tong Qin](http://www.qintonguav.com), [Shaozu Cao](https://github.com/shaozu)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).


## 2. Build A-LOAM
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Velodyne VLP-16 Example
Download [NSH indoor outdoor](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) to YOUR_DATASET_FOLDER. 

```
    roslaunch loam_velodyne loam_velodyne_16.launch
    rosbag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
```


## 4. KITTI Example (Velodyne HDL-64)
Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER and set the `dataset_folder` and `sequence_number` parameters in `kitti_helper.launch` file. Note you also convert KITTI dataset to bag file for easy use by setting proper parameters in `kitti_helper.launch`. 

```
    roslaunch loam_velodyne loam_velodyne_64.launch
    roslaunch loam_velodyne kitti_helper.launch
```
<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti_gif.gif" width = 720 height = 351 />

