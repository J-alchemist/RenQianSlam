/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/matching/matching.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/registration/icp_registration.hpp"

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

namespace lidar_localization {
Matching::Matching()
    :local_map_ptr_(new CloudData::CLOUD()),
     global_map_ptr_(new CloudData::CLOUD()),
     current_scan_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();   //初始化配置参数
    InitGlobalMap();    //加载全局地图进行滤波
    ResetLocalMap(0.0, 0.0, 0.0);   //局部地图分割          以gnss定位点进行子图分割 
}

bool Matching::InitWithConfig() { 
    std::string config_file_path = WORK_SPACE_PATH + "/config/matching.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path); 

    std::cout << "-----------------Map-Localization-Init-------------------" << std::endl;
    InitDataPath(config_node);      //读取地图路径
    InitRegistration(registration_ptr_, config_node);       //读取匹配方式
    InitFilter("global_map", global_map_filter_ptr_, config_node);      //读取全局地图滤波方式
    InitFilter("local_map", local_map_filter_ptr_, config_node);     //读取局部地图滤波方式 
    InitFilter("frame", frame_filter_ptr_, config_node);         //读取当前帧点云的滤波方式 
    InitBoxFilter(config_node);         //滑动窗口大小 

    return true;
}

bool Matching::InitDataPath(const YAML::Node& config_node) {
    map_path_ = config_node["map_path"].as<std::string>();
    return true;
}

bool Matching::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "Map-Matching-Method: " << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP"){
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);  
    }
    else {
        LOG(ERROR) << "Not Found Map-Matching-Method: " << registration_method;
        return false;
    }

    return true;
}

bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "Map-Matching " << filter_user << "filter-Method: " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") { 
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);    //config_node[voxel_filter][global_map]
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "Not Found Filter-Method: " << filter_mothod; 
        return false;
    }

    return true;
}

bool Matching::InitBoxFilter(const YAML::Node& config_node) {
    box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}

bool Matching::InitGlobalMap() { 
    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);    
    LOG(INFO) << "load global map size: " << global_map_ptr_->points.size();

   // global_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_); 
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);    //全局地图滤波
    LOG(INFO) << "filtered global map size: " << global_map_ptr_->points.size();

    has_new_global_map_ = true;     // 滤波完成后的全局地图  

    return true;
}

bool Matching::ResetLocalMap(float x, float y, float z) {
    std::vector<float> origin = {x, y, z};
    box_filter_ptr_->SetOrigin(origin);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);   // 从全局分割局部地图

    registration_ptr_->SetInputTarget(local_map_ptr_);       // 局部地图，作为目标点云，后续进行scan-to-map的匹配，在Update()函数中

    has_new_local_map_ = true;  

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    LOG(INFO) << "new local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;
} 
 
bool Matching::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {       
    std::vector<int> indices; 

    //注意：cloud_in->is_dense = true（意思：该点云为密集，不存在无效点云） 的点云不能使用removeNaNFromPointCloud去除无效点
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);    // 去除无效点：输入点云，滤除无效点的点云，存储保留点云的序列号

    CloudData::CLOUD_PTR  filtered_frame_cloud_ptr(new CloudData::CLOUD()); 
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_frame_cloud_ptr);        // 帧滤波

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();     
    static Eigen::Matrix4f last_pose = init_pose_;          //init_pose_ 由初始时刻gnss获取得到
    static Eigen::Matrix4f predict_pose = init_pose_;       //ndt的预测位姿，

    if (!has_inited_) {    
        predict_pose = current_gnss_pose_;       //gnss未初始化
    }

    // **************** 与地图匹配 **************** // 
    CloudData::CLOUD_PTR  result_cloud_ptr(new CloudData::CLOUD()); //存储匹配结果
    //只是将滤波后的帧 变换（配准）到了目标点云系下
    registration_ptr_->ScanMatch(filtered_frame_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);   // ctrl+in See！  ResetLocalMap()函数中给出了匹配的目标点云
    //利用估计的变换，将该帧所有点云变换过去 
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, cloud_pose);     // 源点云配准到目标点云坐标系下，使用cloud_pose，将cloud_data变换到current_scan

    // 里程计位姿递推 （核心理解！！！）
    step_pose = last_pose.inverse() * cloud_pose;   //   T(帧-局部地图)  * T(局部地图-帧) ，last_pose初始是gnss给的位姿作为基础，cloud_pose雷达里程计估计
    predict_pose = cloud_pose * step_pose;      //  T(局部地图-帧) * T(帧-帧)
    last_pose = cloud_pose;  

    // 匹配之后判断是否需要更新局部地图
    // 当前位置和立方体的边缘距离小于50米时，则以当前位置为原点，按长方体尺寸重新分割出一个局部地图
    std::vector<float> edge = box_filter_ptr_->GetEdge();  
    for (int i = 0; i < 3; i++) {
        if (fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0)
            continue;
        ResetLocalMap(cloud_pose(0,3), cloud_pose(1,3), cloud_pose(2,3)); 
        break;
    }

    return true;
}

bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose) {
    current_gnss_pose_ = gnss_pose;  

    static int gnss_cnt = 0; 
    if (gnss_cnt == 0) {            //初始gnss可能不稳定，需要多用
        SetInitPose(gnss_pose); 
    } else if (gnss_cnt > 3) { 
        has_inited_ = true; 
    }
    gnss_cnt ++; 
    return true; 
}

bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose) {    
 
    init_pose_ = init_pose;              // gnss位姿作为  匹配的基础位置
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));     //分割出局部地图
    
    return true; 
} 

void Matching::GetGlobalMap(CloudData::CLOUD_PTR& global_map) {
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map);        //源点云global_map_ptr_，滤波后存储：global_map
    has_new_global_map_ = false;
}

CloudData::CLOUD_PTR& Matching::GetLocalMap() {
    return local_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetCurrentScan() {
    return current_scan_ptr_;
}

bool Matching::HasInited() { 
    return has_inited_;
}

bool Matching::HasNewGlobalMap() {
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap() {
    return has_new_local_map_;
}
}