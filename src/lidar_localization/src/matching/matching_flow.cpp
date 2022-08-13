/*
 * @Description: 地图匹配任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/matching/matching_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
MatchingFlow::MatchingFlow(ros::NodeHandle& nh) { 

//订阅 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);        //时间配准后的数据  
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);                               
//发布
    //topic
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);     //发布/global_map，坐标系id：/map  全局地图，在run()里面发布
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);       //发布/local_map    局部地图（从全局切割的）
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);     //发布/current_scan     当前帧
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);     //发布/laser_localization：/map和/lidar
    //tf
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");      //发布车辆在地图位姿
   // laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/base_link");   

//匹配
    matching_ptr_ = std::make_shared<Matching>();       //全局地图和局部地图初始化  
}

bool MatchingFlow::Run() {  
    
    if ( matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers() ) {        //是否发布全局地图、有人订阅 /global_map 

        CloudData::CLOUD_PTR  global_map_ptr(new CloudData::CLOUD());    //新对象，存储稀疏后全局地图
        matching_ptr_->GetGlobalMap(global_map_ptr);                //更新HasNewGlobalMap() 
        global_map_pub_ptr_->Publish(global_map_ptr);                   //1-发布全局地图 
    } 

    if ( matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers() )
        local_map_pub_ptr_->Publish( matching_ptr_->GetLocalMap() );     //2-发布局部地图

    ReadData();     //获取对齐后的数据
    while( HasData() ) {
        if ( !ValidData() )
            continue;

        if ( UpdateMatching()) {     // scan-to-map
            PublishData();      //发布匹配的位姿
        }    
    }

    return true;
}

bool MatchingFlow::ReadData() { 
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

bool MatchingFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (matching_ptr_->HasInited())
        return true;
    
    if (gnss_data_buff_.size() == 0)
        return false;
    
    return true;
}

bool MatchingFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    if (matching_ptr_->HasInited()) {
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();
        return true;
    }

    current_gnss_data_ = gnss_data_buff_.front();

    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool MatchingFlow::UpdateMatching() {
    if (!matching_ptr_->HasInited()) {               //使用gnss进行初始化，后续匹配不需要gnss
        matching_ptr_->SetGNSSPose(current_gnss_data_.pose);                //初始以gnss当前位姿 进行子图的分割 
    }

    return matching_ptr_->Update(current_cloud_data_, laser_odometry_);      //用当前帧进行估计，获取位姿矩阵， 输出laser_odometry_,判别是否需要重新分割局部地图
}

bool MatchingFlow::PublishData() { 

    laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time);    //发布tf
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);        //分布topic
    current_scan_pub_ptr_->Publish( matching_ptr_->GetCurrentScan() );

    return true;
}
}