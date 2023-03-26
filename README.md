# Livox-Horizon-LOAM
## LiDAR Odemetry and Mapping (LOAM) package for Livox Horizon LiDAR

![image-20230326122828997](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20230326122828997.png)

This repository serves as a storehouse for the deployment environment of the latest version, namely ROS2. For those who intend to use ROS1 (Noetic), kindly switch to the designated branch. The code will be subject to ongoing maintenance during leisure hours.


## 1. Prerequisites
- Ubuntu 22.04, ROS **Humble**  
- Ceres Solver
- PCL
- Eigen
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)


## 2. Build
Clone the repository and colcon build:

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/Natsu-Akatsuki/loam_horizon
$ cd ../
# 当livox_ros_driver2放置于本地工程时，需要添加如下变量ROS2和humble
$ colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble --symlink-install
$ source ~/catkin_ws/install/setup.bash
```
## 3. Directly run
Connect to your PC to Livox LiDAR (horizon) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```bash
$ ros2 launch livox_ros_driver livox_lidar_msg.launch
$ ros2 launch loam_horizon loam_livox_horizon.launch
```
If you want to use horizon's internal IMU to eliminate rotation distortion, run
```bash
# 尚未修改：
$ roslaunch livox_ros_driver livox_lidar_msg.launch
$ roslaunch loam_horizon loam_livox_horizon_imu.launch
```

## 4. Rosbag Example
### 4.1. **Common rosbag**
- Download [parking lot rosbag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/demo/2020_parking_lot.bag) or [outdoor scene rosbag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/demo/2020_open_road.bag) and then

```bash
# 尚未修改：
$ roslaunch loam_horizon loam_livox_horizon.launch
$ rosbag play YOUR_DOWNLOADED.bag
```
- If you want to use horizon's internal IMU to eliminate rotation distortion, run

```bash
# 尚未修改：
$ roslaunch loam_horizon loam_livox_horizon_imu.launch
$ rosbag play YOUR_DOWNLOADED.bag
```

### 4.2. **External IMU rosbag**
If you want to use an external IMU with horizon to eliminate rotation distortion, you need to manually obtain external parameters. For example, we record a rosbag containing external imu data and horizon data, and the extrinsic quaternion is (0, 1, 0, 0), then:
1. Download [external imu rosbag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/demo/imu-demo.bag).
2. Modify the `ExtIL` rosparam in [`loam_livox_horizon_ext_imu.launch`](https://github.com/Livox-SDK/livox_horizon_loam/blob/b43494f0217839b849fb6752a6dea4bd79bd3bb4/launch/loam_livox_horizon_ext_imu.launch#L3) in order`[q.w, q.x, q.y, q.z, t.x, t.y, t.z]`:
```bash
<rosparam param="ExtIL"> [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]</rosparam>
```
3. Run
```bash
# 尚未修改：
$ roslaunch loam_horizon loam_livox_horizon_ext_imu.launch
$ rosbag play imu-demo.bag
```

## 5.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

