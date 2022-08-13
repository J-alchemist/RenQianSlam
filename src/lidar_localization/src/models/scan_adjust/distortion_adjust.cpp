/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:39:00
 */
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "glog/logging.h"

namespace lidar_localization { 
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) { 

    CloudData::CLOUD_PTR  origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));  //传入input_cloud_ptr
    output_cloud_ptr.reset(new CloudData::CLOUD());

    float start_orientation = atan2(origin_cloud_ptr->points[0].y,  origin_cloud_ptr->points[0].x);  //反正切计算方位角 y/x 计算每个点z轴的偏移 弧度返回
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());      
  //  Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f(0,0,1));
    Eigen::Matrix3f rotate_matrix = t_V.matrix();       //得到该点在雷达坐标系下的旋转角度  

    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();      
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();    
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);   

    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    float orientation_space = 2.0 * M_PI;       //雷达一周旋转360°，周期scan_period_ = 100ms
    float delete_space = 5.0 * M_PI / 180.0;
    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {

        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);    //计算每个点的角度

        if (orientation < 0.0)          //特殊处理
            orientation += 2.0 * M_PI;
    
        if (orientation < delete_space || (2.0*M_PI-orientation) < delete_space)
            continue;
        /*
        kitti2bag这个功能包在把数据转成bag文件的过程中，利用起始时刻和终止时刻取了个平均值，即中间时刻，
        作为这一帧点云的采集时刻，上面的方法其实是把点全都转到起始时刻上去，
        所以我们在计算的每个激光点采集时刻上再减去50ms，这样就相当于把一帧点云的坐标系转到中间时刻对应的坐标系上去了。
        */
        float real_time = ( fabs(orientation) / orientation_space * scan_period_ ) - scan_period_ / 2.0;    

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z); 

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time); 
        Eigen::Vector3f rotated_point = current_matrix * origin_point;  
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time; //矫正后的点
        CloudData::POINT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse()); 
    return true;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
}
} // namespace lidar_localization