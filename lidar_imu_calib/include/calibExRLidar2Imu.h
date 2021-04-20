/*
 * brief: calibrate extrincs rotation between multi-layer lidar and imu
 * author: chennuo0125@163.com
*/

#pragma once

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "../../ndt_omp/include/pclomp/ndt_omp.h"

using namespace std;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct LidarData
{
    double stamp;
    CloudT::Ptr cloud;
};

struct LidarFrame
{
    double stamp;
    Eigen::Matrix4d T;
    Eigen::Matrix4d gT;
    CloudT::Ptr cloud{nullptr};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuData
{
    double stamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
    Eigen::Quaterniond rot;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CalibExRLidarImu
{
public:
    CalibExRLidarImu();
    ~CalibExRLidarImu();

    //@brief: set init extrinsic if have
    void setInitExR(Eigen::Vector3d init_R);

    //@brief: add lidar data and calculate lidar odometry
    void addLidarData(const LidarData &data);

    //@brief: add imu data and cache
    void addImuData(const ImuData &data);

    //@brief: integration imu data, align lidar odom and imu
    Eigen::Vector3d calib(bool integration = false);

private:
    //@brief: interpolated attitude from start attitude to end attitude by scale
    Eigen::Quaterniond getInterpolatedAttitude(const Eigen::Quaterniond &q_s_w, const Eigen::Quaterniond &q_e_w, double scale);

    //@brief: update relative transform between neighbor lidar frame by aligned imu data
    void optimize();

    //@brief: solve least square answer by constraints
    Eigen::Quaterniond solve(const vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> &corres);

    Eigen::Vector3d init_R_{0.0, 0.0, 0.0};
    CloudT::Ptr last_lidar_cloud_{nullptr};                                 // last lidar cloud
    vector<LidarFrame> lidar_buffer_;                                       // record relative transform between neighbor lidar frame
    vector<ImuData> imu_buffer_;                                            // record raw imu datas
    vector<pair<LidarFrame, Eigen::Quaterniond>> aligned_lidar_imu_buffer_; // aligned lidar frame and interpolated imu attitude at lidar stamp
    Eigen::Quaterniond q_l_b_;                                              // result

    CloudT::Ptr local_map_{nullptr};                                              // local map
    pcl::VoxelGrid<PointT> downer_;                                               // downsample local map
    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr register_{nullptr}; // register object

    vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres1_;
    vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres2_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};