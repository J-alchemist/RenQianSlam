#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include <pcl/registration/icp.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization { 

  class ICPRegistration: public RegistrationInterface {
    public:
      ICPRegistration(const YAML::Node& node);    //通过yaml加载匹配参数
      ICPRegistration(float res, float step_size, float trans_eps, int max_iter);

      bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;   //输入点云
      bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                    const Eigen::Matrix4f& predict_pose, 
                    CloudData::CLOUD_PTR& result_cloud_ptr,
                    Eigen::Matrix4f& result_pose) override;       //匹配执行 
      float GetFitnessScore() override;
    
    private:
      bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter); //设置匹配参数

    private:
      pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
  };
}

#endif