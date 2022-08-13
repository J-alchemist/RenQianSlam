/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    // tf
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "velo_link");
    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/synced_cloud", "/velo_link", 100);              //发布对齐之后的数据
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/velo_link", 100);  

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();      //畸变矫正  匀速模型假设前提下，坐标 = 运动×时间
}

bool DataPretreatFlow::Run() { 
    if (!ReadData())        //数据同步
        return false;

    if (!InitCalibration())         //获取lidar-imu静态变换 
        return false;

    if (!InitGNSS())        //初始化gnss位姿
        return false;

    while(HasData()) {
        if (!ValidData())       //对齐的数据是否正常
            continue;

        TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() { 
    cloud_sub_ptr_->ParseData(cloud_data_buff_);    

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_; 

    imu_sub_ptr_->ParseData(unsynced_imu_);   // 将订阅的数据放入 为同步的deque里面
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if (cloud_data_buff_.size() == 0)
        return false;

    double cloud_time = cloud_data_buff_.front().time;       //以雷达数据时间为准，插值imu数据对齐
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);         
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataPretreatFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) { //获取imu到雷达的变换 
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition(); //gnss初始位姿
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();     //经纬度转ENU（米） 获取相对于gnss初始化点的坐标
    gnss_pose_(0,3) = current_gnss_data_.local_E; 
    gnss_pose_(1,3) = current_gnss_data_.local_N; 
    gnss_pose_(2,3) = current_gnss_data_.local_U;  
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();      //imu在gnss初始点的位置
    gnss_pose_ *= lidar_to_imu_;       //雷达转到gnss-map下  

    current_velocity_data_.TransformCoordinate(lidar_to_imu_);      //imu的速度转到雷达上，估计雷达的运动（车旋转时他们的速度并不一致） 
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);     // 运动速度进行畸变矫正
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);  

    return true; 
}

bool DataPretreatFlow::PublishData() {  
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);   //发布对齐后的雷达
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);    //雷达在map下的位姿  

    return true;
}
}


