/*
 * @Description: tf监听模块
 * @Author: Ren Qian
 * @Date: 2020-02-06 16:10:31
 */
#include "lidar_localization/tf_listener/tf_listener.hpp"

#include <Eigen/Geometry>

namespace lidar_localization {
TFListener::TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id) 
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
}

bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
    try {
        tf::StampedTransform transform;
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
        //tf变换转换出变换矩阵  
        TransformToMatrix(transform, transform_matrix); 
        return true;
    } catch (tf::TransformException &ex) {
        return false;
    }
}
/*
位移矩阵 (3X3)：Eigen::Translation3f  
旋转矩阵（3X3）:Eigen::Matrix3d 
旋转向量（3X1）:Eigen::AngleAxisd 
四元数（4X1）:Eigen::Quaterniond 
平移向量（3X1）:Eigen::Vector3d 
变换矩阵（4X4）:Eigen::Isometry3d 
*/
bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) {  //imu到lidar变换矩阵

    double roll, pitch, yaw;
    //平移
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //姿态
    tf::Matrix3x3( transform.getRotation() ).getEulerYPR(yaw, pitch, roll); //ros中的角，getRPY内部也是调用getEulerYPR
    //tf::Matrix3x3( transform.getRotation() ).getRPY(roll, pitch,yaw); //ros中的角

    Eigen::AngleAxisf rot_x_btol( roll, Eigen::Vector3f::UnitX() );     //转为Eigen中的旋转向量， 以x轴旋转roll弧度
    Eigen::AngleAxisf rot_y_btol( pitch, Eigen::Vector3f::UnitY() ); 
    Eigen::AngleAxisf rot_z_btol( yaw, Eigen::Vector3f::UnitZ() ); 

    //TF变换转换到变换矩阵 
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();  

    return true; 
}
}