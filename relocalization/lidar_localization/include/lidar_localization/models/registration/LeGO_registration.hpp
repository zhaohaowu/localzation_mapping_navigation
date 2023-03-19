/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_LEGO_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_LEGO_REGISTRATION_HPP_

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/tools/tic_toc.hpp"

#include "lidar_localization/models/registration/lidar_factor.hpp"

#include <yaml-cpp/yaml.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <ceres/ceres.h>


namespace lidar_localization {

typedef CloudData::POINT PointType;

class LeGORegistration{

public:
    LeGORegistration(const YAML::Node& node);
    LeGORegistration(int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& target_sharp_map,
                                             const CloudData::CLOUD_PTR& target_flat_map);
    bool ScanMatch(const CloudData::CLOUD_PTR& input_sharp_cloud, 
                                    const CloudData::CLOUD_PTR& input_flat_cloud, 
                                    const Eigen::Matrix4f& predict_pose, 
                                    Eigen::Matrix4f& result_pose);
    float GetFitnessScore();
  
private:
    bool SetRegistrationParam(int max_iter);
    void pointAssociateToMap(PointType const *const pi, PointType *const po);
    void setCurrentPose(const Eigen::Matrix4f& pose);
private:
    int max_iter_times_ = 2;

    double parameters_[7] = {0, 0, 0, 1, 0, 0, 0};
    // Eigen::Map<Eigen::Quaterniond> q_w_curr_(parameters_);
    // Eigen::Map<Eigen::Vector3d> t_w_curr_(parameters_ + 4);

    Eigen::Quaterniond q_w_curr_;
    Eigen::Vector3d t_w_curr_;

    CloudData::CLOUD_PTR local_sharp_map_;
    CloudData::CLOUD_PTR local_flat_map_;

    pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_corner_map_;
    pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_surf_map_;    

    pcl::VoxelGrid<CloudData::POINT> sharp_voxel_filter_;
    pcl::VoxelGrid<CloudData::POINT> flat_voxel_filter_;

};


}

#endif