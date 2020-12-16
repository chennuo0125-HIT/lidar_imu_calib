# lidar_imu_calib

### overview

when develop slam based on 3D lidar, we often use imu to provide priori for matching algorithm(icp, ndt), so the transform between lidar and imu need to be calibrated.For matching algorithm, attitude in transfom is more important than position in transform, and position often be set to 0. So this repo concentrate on calibrate attitude component in transform between lidar and imu.

### prerequisite

- [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [ndt_omp](https://github.com/koide3/ndt_omp) 

### step

1. use rosbag tool record imu and lidar data

   ```
   rosbag record /imu /lidar_points
   ```

2. config launch file

   ```
   lidar_topic: 雷达数据话题名
   imu_topic: imu数据话题名
   bag_file: 记录的bag数据包
   ```

   