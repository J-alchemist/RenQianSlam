/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:38:12
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "lidar_localization/sensor_data/velocity_data.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class DistortionAdjust {
  public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);//获取运动个时刻的角速度、线速度，前提是该时间已经同步了
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

  private:
    float scan_period_;     //雷达旋转一周的时间 kitti里面是100ms = 0.1s
    Eigen::Vector3f velocity_;  //imu3轴线速度
    Eigen::Vector3f angular_rate_; //imu3轴角速度
};
} // namespace lidar_slam
#endif