#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <time.h>
#include "calibExRLidar2Imu.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#define READ_BAGFILE true

using namespace std;

queue<sensor_msgs::PointCloud2ConstPtr> lidar_buffer;
queue<sensor_msgs::ImuConstPtr> imu_buffer;

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    lidar_buffer.push(msg);
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    imu_buffer.push(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calib_exR_lidar2imu_node");
    ros::NodeHandle nh, pnh("~");

    // read data topic
    string lidar_topic, imu_topic;
    if (!pnh.getParam("lidar_topic", lidar_topic) || !pnh.getParam("imu_topic", imu_topic))
    {
        cout << "please config param: lidar_topic, imu_topic !!!" << endl;
        return 0;
    }

    // initialize caliber
    CalibExRLidarImu caliber;

#if READ_BAGFILE
    // get local param
    string bag_file;
    if (!pnh.getParam("bag_file", bag_file))
    {
        cout << "please config param: bag_file !!!" << endl;
        return 0;
    }

    // open bagfile
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);
    vector<string> topics;
    topics.push_back(lidar_topic);
    topics.push_back(imu_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // read data and add data
    foreach (rosbag::MessageInstance const m, view)
    {
        // add lidar msg
        sensor_msgs::PointCloud2ConstPtr lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (lidar_msg != NULL)
        {
            ROS_INFO_STREAM_THROTTLE(5.0, "add lidar msg ......");

            CloudT::Ptr cloud(new CloudT);
            pcl::fromROSMsg(*lidar_msg, *cloud);
            LidarData data;
            data.cloud = cloud;
            data.stamp = lidar_msg->header.stamp.toSec();
            caliber.addLidarData(data);
        }

        // add imu msg
        sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg)
        {
            ImuData data;
            data.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x,
                                       imu_msg->linear_acceleration.y,
                                       imu_msg->linear_acceleration.z);
            data.gyr = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                       imu_msg->angular_velocity.y,
                                       imu_msg->angular_velocity.z);
            data.rot = Eigen::Quaterniond(imu_msg->orientation.w,
                                          imu_msg->orientation.x,
                                          imu_msg->orientation.y,
                                          imu_msg->orientation.z);
            data.stamp = imu_msg->header.stamp.toSec();
            caliber.addImuData(data);
        }
    }
#else
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud", 1000, lidarCallback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 10000, imuCallback);

    // add data
    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        ROS_INFO_STREAM_THROTTLE(5.0, "lidar buffer size " << lidar_buffer.size() << ", imu buffer size " << imu_buffer.size()); // monitor

        // add lidar data
        while (lidar_buffer.size() != 0)
        {
            CloudT::Ptr cloud(new CloudT);
            pcl::fromROSMsg(*(lidar_buffer.front()), *cloud);
            LidarData data;
            data.cloud = cloud;
            data.stamp = lidar_buffer.front()->header.stamp.toSec();
            caliber.addLidarData(data);
            lidar_buffer.pop();
        }

        // add imu data
        while (imu_buffer.size() != 0)
        {
            ImuData data;
            data.acc = Eigen::Vector3d(imu_buffer.front()->linear_acceleration.x,
                                       imu_buffer.front()->linear_acceleration.y,
                                       imu_buffer.front()->linear_acceleration.z);
            data.gyr = Eigen::Vector3d(imu_buffer.front()->angular_velocity.x,
                                       imu_buffer.front()->angular_velocity.y,
                                       imu_buffer.front()->angular_velocity.z);
            data.rot = Eigen::Quaterniond(imu_buffer.front()->orientation.w,
                                          imu_buffer.front()->orientation.x,
                                          imu_buffer.front()->orientation.y,
                                          imu_buffer.front()->orientation.z);
            data.stamp = imu_buffer.front()->header.stamp.toSec();
            caliber.addImuData(data);
            imu_buffer.pop();
        }

        loop_rate.sleep();
    }
#endif

    // calib
    Eigen::Vector3d rpy = caliber.calib();
    Eigen::Matrix3d rot = Eigen::Matrix3d(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));
    cout << "result euler angle(RPY) : " << rpy[0] << " " << rpy[1] << " " << rpy[2] << endl;
    cout << "result extrinsic rotation matrix : " << endl;
    cout << rot << endl;

    return 0;
}