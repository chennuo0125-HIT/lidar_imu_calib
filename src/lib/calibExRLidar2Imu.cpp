#include "calibExRLidar2Imu.h"
#include <omp.h>
#include <utility>
#include <tf/tf.h>
#include <pcl/io/pcd_io.h>

#define USE_SCAN_2_MAP true

CalibExRLidarImu::CalibExRLidarImu()
{
    imu_buffer_.clear();

    // init downsample object
    downer_.setLeafSize(0.1, 0.1, 0.1);

    // init register object
    register_.reset(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    register_->setResolution(1.0);
    int avalib_cpus = omp_get_max_threads();
    register_->setNumThreads(avalib_cpus);
    register_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
}

CalibExRLidarImu::~CalibExRLidarImu()
{
}

void CalibExRLidarImu::setInitExR(Eigen::Vector3d init_R)
{
    init_R_ = init_R;
}

void CalibExRLidarImu::addLidarData(const LidarData &data)
{
    if (!data.cloud || data.cloud->size() == 0)
    {
        cout << "no cloud in lidar data !!!" << endl;
        return;
    }

    if (!register_)
    {
        cout << "register no initialize !!!" << endl;
        return;
    }

#if USE_SCAN_2_MAP
    if (!local_map_)
    {
        local_map_.reset(new CloudT);
        *local_map_ += *(data.cloud);

        LidarFrame frame;
        frame.stamp = data.stamp;
        frame.T = Eigen::Matrix4d::Identity();
        frame.gT = Eigen::Matrix4d::Identity();
        lidar_buffer_.push_back(move(frame));

        return;
    }

    // downsample local map and lidar cloud for save align time
    CloudT::Ptr downed_map(new CloudT);
    downer_.setInputCloud(local_map_);
    downer_.filter(*downed_map);
    local_map_ = downed_map;

    CloudT::Ptr downed_cloud(new CloudT);
    downer_.setInputCloud(data.cloud);
    downer_.filter(*downed_cloud);

    // get transform between frame and local map
    register_->setInputSource(downed_cloud);
    register_->setInputTarget(local_map_);
    CloudT::Ptr aligned(new CloudT);
    register_->align(*aligned);
    if (!register_->hasConverged())
    {
        cout << "register cant converge, please check initial value !!!" << endl;
        return;
    }
    Eigen::Matrix4d T_l_m = (register_->getFinalTransformation()).cast<double>();

    // generate lidar frame
    LidarFrame frame;
    frame.stamp = data.stamp;
    frame.gT = T_l_m;
    Eigen::Matrix4d last_T_l_m = lidar_buffer_.back().gT;
    frame.T = last_T_l_m.inverse() * T_l_m;
    lidar_buffer_.push_back(move(frame));

    // update local map
    *local_map_ += *aligned;
#else
    // init first lidar frame and set it as zero pose
    if (!last_lidar_cloud_)
    {
        last_lidar_cloud_.reset(new CloudT);
        last_lidar_cloud_ = data.cloud;

        LidarFrame frame;
        frame.stamp = data.stamp;
        frame.T = Eigen::Matrix4d::Identity();
        frame.gT = Eigen::Matrix4d::Identity();
        lidar_buffer_.push_back(move(frame));

        return;
    }

    // get transform between neighbor frames
    register_->setInputSource(data.cloud_);
    register_->setInputTarget(last_lidar_cloud);
    CloudT::Ptr aligned(new CloudT);
    register_->align(*aligned);

    if (!register_->hasConverged())
    {
        cout << "register cant converge, please check initial value !!!" << endl;
        return;
    }
    Eigen::Matrix4d result_T = (register_->getFinalTransformation()).cast<double>();

    // generate lidar frame
    LidarFrame frame;
    frame.stamp = data.stamp;
    frame.T = result_T;
    Eigen::Matrix4d temp1 = lidar_buffer_.back().gT;
    Eigen::Matrix4d temp2 = result_T;
    frame.gT = temp1 * temp2;
    lidar_buffer_.push_back(move(frame));

    // debug
    // CloudT::Ptr g_cloud(new CloudT);
    // pcl::transformPointCloud(*(data.cloud), *g_cloud, lidar_buffer_.back().gT.cast<float>());
    // string pcd_file = "/home/cn/temp/" + to_string(lidar_buffer_.size()) + ".pcd";
    // pcl::io::savePCDFile(pcd_file, *g_cloud);
#endif
}

void CalibExRLidarImu::addImuData(const ImuData &data)
{
    imu_buffer_.push_back(data);
}

Eigen::Quaterniond CalibExRLidarImu::getInterpolatedAttitude(const Eigen::Quaterniond &q_s_w, const Eigen::Quaterniond &q_e_w, double scale)
{
    if (0 == scale || scale > 1)
        return move(Eigen::Quaterniond().Identity());

    // calculate angleaxis difference
    Eigen::Quaterniond q_e_s = q_s_w.inverse() * q_e_w;
    q_e_s.normalize();
    Eigen::AngleAxisd diff_angle_axis(q_e_s);

    // interpolated attitude by scale
    double interpolated_angle = diff_angle_axis.angle() * scale;
    Eigen::Quaterniond q_ie_s(Eigen::AngleAxisd(interpolated_angle, diff_angle_axis.axis()).toRotationMatrix());
    Eigen::Quaterniond q_ie_w = q_s_w * q_ie_s;
    q_ie_w.normalize();

    return move(q_ie_w);
}

Eigen::Quaterniond CalibExRLidarImu::solve(const vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> &corres)
{
    if (corres.size() == 0)
    {
        cout << "no constraint found !!!" << endl;
        return move(Eigen::Quaterniond().Identity());
    }

    cout << "constraints size " << corres.size() << endl;

    // transform quaternion to skew symmetric matrix
    auto toSkewSymmetric = [](const Eigen::Vector3d &q) -> Eigen::Matrix3d {
        Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
        mat(0, 1) = -q.z();
        mat(0, 2) = q.y();
        mat(1, 0) = q.z();
        mat(1, 2) = -q.x();
        mat(2, 0) = -q.y();
        mat(2, 1) = q.x();

        return move(mat);
    };

    // create homogeneous linear equations
    Eigen::MatrixXd A(corres.size() * 4, 4);
    for (int i = 0; i < corres.size(); i++)
    {
        // get relative transform
        const auto &q_l1_l2 = corres[i].first;
        const auto &q_b1_b2 = corres[i].second;

        // get left product matrix
        Eigen::Vector3d q_b1_b2_vec = q_b1_b2.vec();
        Eigen::Matrix4d left_Q_b1_b2 = Eigen::Matrix4d::Zero();
        left_Q_b1_b2.block<1, 3>(0, 1) = -q_b1_b2_vec.transpose();
        left_Q_b1_b2.block<3, 1>(1, 0) = q_b1_b2_vec;
        left_Q_b1_b2.block<3, 3>(1, 1) = toSkewSymmetric(q_b1_b2_vec);
        left_Q_b1_b2 += q_b1_b2.w() * Eigen::Matrix4d::Identity();

        // get right product matrix
        Eigen::Vector3d q_l1_l2_vec = q_l1_l2.vec();
        Eigen::Matrix4d right_Q_l1_l2 = Eigen::Matrix4d::Zero();
        right_Q_l1_l2.block<1, 3>(0, 1) = -q_l1_l2_vec.transpose();
        right_Q_l1_l2.block<3, 1>(1, 0) = q_l1_l2_vec;
        right_Q_l1_l2.block<3, 3>(1, 1) = -toSkewSymmetric(q_l1_l2_vec);
        right_Q_l1_l2 += q_l1_l2.w() * Eigen::Matrix4d::Identity();

        A.block<4, 4>(i * 4, 0) = left_Q_b1_b2 - right_Q_l1_l2;
    }

    // solve homogeneous linear equations by svd method
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
    Eigen::Quaterniond q_l_b(x(0), x(1), x(2), x(3));

    return move(q_l_b);
}

Eigen::Vector3d CalibExRLidarImu::calib(bool integration)
{
    // debug
    pcl::io::savePCDFile("/home/cn/temp/local_map.pcd", *local_map_);

    if (lidar_buffer_.size() == 0 || imu_buffer_.size() == 0)
    {
        cout << "no lidar data or imu data !!!" << endl;
        return init_R_;
    }

    cout << "total lidar buffer size " << lidar_buffer_.size() << ", imu buffer size " << imu_buffer_.size() << endl;

    // move invalid lidar frame which got before first imu frame
    auto invalid_lidar_it = lidar_buffer_.begin();
    for (; invalid_lidar_it != lidar_buffer_.end(); invalid_lidar_it++)
    {
        if (invalid_lidar_it->stamp >= imu_buffer_[0].stamp)
            break;
    }
    if (invalid_lidar_it != lidar_buffer_.begin())
        lidar_buffer_.erase(lidar_buffer_.begin(), invalid_lidar_it);
    if (lidar_buffer_.size() == 0)
    {
        cout << "no valid lidar frame !!!" << endl;
        return move(Eigen::Vector3d(0.0, 0.0, 0.0));
    }

    // get last imu frame which before first lidar frame
    auto last_imu_it = imu_buffer_.begin();
    for (; last_imu_it != imu_buffer_.end(); last_imu_it++)
    {
        if (last_imu_it->stamp >= lidar_buffer_[0].stamp)
            break;
    }
    if (last_imu_it != imu_buffer_.begin())
        last_imu_it--;

    // get time-aligned lidar odometry rotation and imu integration rotation
    vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres(0);
    for (int ei = 1; ei < lidar_buffer_.size(); ei++)
    {
        // get left lidar and right lidar information
        int si = ei - 1;
        const auto &left_lidar = lidar_buffer_[si];
        const auto &right_lidar = lidar_buffer_[ei];

        // get interpolated imu attitude at left lidar stamp
        auto left_imu_it1 = last_imu_it;
        auto left_imu_it2 = last_imu_it + 1;
        if (imu_buffer_.end() == left_imu_it2)
            break;
        assert(left_imu_it2->stamp >= left_lidar.stamp || left_imu_it1->stamp < left_imu_it2->stamp); // this shouldnt happen
        Eigen::Quaterniond left_q_i1_w = left_imu_it1->rot;
        Eigen::Quaterniond left_q_i2_w = left_imu_it2->rot;
        double left_scale = (left_lidar.stamp - left_imu_it1->stamp) / (left_imu_it2->stamp - left_imu_it1->stamp);
        Eigen::Quaterniond left_q_inter_w = getInterpolatedAttitude(left_q_i1_w, left_q_i2_w, left_scale);

        // get interpolated imu attitude at right lidar stamp
        last_imu_it = imu_buffer_.begin();
        for (; last_imu_it != imu_buffer_.end(); last_imu_it++)
        {
            if (last_imu_it->stamp >= right_lidar.stamp)
                break;
        }
        if (last_imu_it != imu_buffer_.begin())
            last_imu_it--;
        auto right_imu_it1 = last_imu_it;
        auto right_imu_it2 = last_imu_it + 1;
        if (imu_buffer_.end() == right_imu_it2)
            break;
        assert(right_imu_it2->stamp >= right_lidar.stamp || right_imu_it1->stamp < right_imu_it2->stamp);
        Eigen::Quaterniond right_q_i1_w = right_imu_it1->rot;
        Eigen::Quaterniond right_q_i2_w = right_imu_it2->rot;
        double right_scale = (right_lidar.stamp - right_imu_it1->stamp) / (right_imu_it2->stamp - right_imu_it1->stamp);
        Eigen::Quaterniond right_q_inter_w = getInterpolatedAttitude(right_q_i1_w, right_q_i2_w, right_scale);

        // get relative transform between interpolated imu attitudes at left and right lidar stamp
        Eigen::Quaterniond imu_q_right_left = left_q_inter_w.inverse() * right_q_inter_w;
        Eigen::Quaterniond lidar_q_right_left(right_lidar.T.block<3, 3>(0, 0));
        imu_q_right_left.normalize();
        lidar_q_right_left.normalize();
        corres.push_back(move(pair<Eigen::Quaterniond, Eigen::Quaterniond>(lidar_q_right_left, imu_q_right_left)));
    }

    // get result
    Eigen::Quaterniond q_l_b = solve(corres);
    tf::Matrix3x3 mat(tf::Quaternion(q_l_b.x(), q_l_b.y(), q_l_b.z(), q_l_b.w()));
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    return move(Eigen::Vector3d(roll, pitch, yaw));
}