/*
 * @Description: 地图匹配定位算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_MATCHING_HPP_
#define LIDAR_LOCALIZATION_MATCHING_MATCHING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"
#include "lidar_localization/models/registration/LeGO_registration.hpp"
#include "lidar_localization/models/cloud_filter/box_filter.hpp"
#include "lidar_localization/models/features/lego_loam_sc.hpp"

namespace lidar_localization {
class Matching {
  public:
    Matching(const YAML::Node& config_node);
    bool Update_lidar_only(const CloudData& cloud_data, 
                                              const CloudData& sharp_cloud_data,
                                              const CloudData& flat_cloud_data,
                                              Eigen::Matrix4f& pose_estimited);

    void GetGlobalFeatureMap(CloudData::CLOUD_PTR& global_sharp_map, 
                                                            CloudData::CLOUD_PTR& global_flat_map);
    void GetLocalFeatureMap(CloudData::CLOUD_PTR& local_sharp_map, 
                                                        CloudData::CLOUD_PTR& local_flat_map);
    CloudData::CLOUD_PTR& GetCurrentScan();
    bool HasNewGlobalFeatureMap();
    bool HasNewLocalFeatureMap();
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

  private:
    bool InitWithConfig(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitLOAMRegistration(const YAML::Node& config_node);
    bool InitBoxFilter(const YAML::Node& config_node);

    bool InitGlobalFeatureMap();
    bool ResetLocalFeatureMap(float x, float y, float z);

    void sc_relocation(const CloudData& cloud_data,Eigen::Matrix4f &guess_pose);

  private:
    std::string matching_method = "";
    std::string map_path_ = "";
    std::string sharp_map_path_ = "";
    std::string flat_map_path_ = "";
    std::string database_path_ = "";  //数据库路径（关键帧及其位姿）
    std::string possible_pose_path_ = "";

    std::string loop_closure_method_ = "";

    std::shared_ptr<LeGORegistration> loam_icp_ptr_; 
    std::shared_ptr<BoxFilter> box_filter_ptr_;

    CloudData::CLOUD_PTR current_scan_ptr_;
    // Current sharp scan
    CloudData::CLOUD_PTR current_sharp_scan_ptr_;
    CloudData::CLOUD_PTR current_flat_scan_ptr_;
    // Feature cloud map
    CloudData::CLOUD_PTR global_sharp_map_ptr_;
    CloudData::CLOUD_PTR global_flat_map_ptr_;
    CloudData::CLOUD_PTR local_sharp_map_ptr_;
    CloudData::CLOUD_PTR local_flat_map_ptr_;

    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    bool has_new_global_feature_map_ = false;
    bool has_new_local_feature_map_ = false;
    bool pre_init;
    bool has_inited_;
    
};
}

#endif