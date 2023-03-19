#ifndef _FEATURE_EXTRACTION_HPP_
#define _FEATURE_EXTRACTION_HPP_

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "lidar_localization/tools/cloud_info.h"

#include "lidar_localization/sensor_param/lidar_param.hpp"


namespace lidar_localization{
typedef pcl::PointXYZI PointType;

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtractor
{
public:
    // FeatureExtractor() = default;
    FeatureExtractor(const LidarParam& lidar_param);
    ~FeatureExtractor();

    void runExtactorOnce(pcl::PointCloud<PointType>::Ptr segmented_cloud_in, cloud_msgs::cloud_info& info_in);
    // void runExtactor(std::vector<pcl::PointCloud<PointType>::Ptr> cloud_ptr_vec, cloud_msgs::cloud_info& info_vec);

    pcl::PointCloud<PointType>::Ptr getCornerPointsSharp();
    pcl::PointCloud<PointType>::Ptr getCornerPointsLessSharp();
    pcl::PointCloud<PointType>::Ptr getSurfPointsFlat();
    pcl::PointCloud<PointType>::Ptr getSurfPointsLessFlat();

private:
    // param:
    int edgeFeatureNum = 2;
    int surfFeatureNum = 4;
    int sectionsTotal = 6;
    float edge_threshold_ = 0.1;
    float surf_threshold = 0.1;
    float nearestFeatureSearchSqDist = 25;

    // members
    LidarParam lidar_param_;

    cloud_msgs::cloud_info seg_info_;

    std::vector<smoothness_t> cloud_smoothness_;

    pcl::PointCloud<PointType>::Ptr segmented_cloud_;

    float *cloud_curvature_;
    int *cloud_neighbor_picked_;
    int *cloud_label_;

    // feature cloud out
    pcl::PointCloud<PointType>::Ptr corner_points_sharp_;
    pcl::PointCloud<PointType>::Ptr corner_points_less_sharp_;
    pcl::PointCloud<PointType>::Ptr surf_points_flat_;
    pcl::PointCloud<PointType>::Ptr surf_points_less_flat_;

    // temp cloud
    pcl::PointCloud<PointType>::Ptr surf_points_flat_scan_;
    pcl::PointCloud<PointType>::Ptr surf_points_flat_scan_DS_;  // 降采样后的scan

    pcl::VoxelGrid<PointType> down_size_filter_;
private:
    void initExtrator();
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeatures();
};

}

#endif
