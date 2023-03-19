#ifndef LIDAR_LOCALIZATION_IMAGE_PROJECTION_IMAGE_PROJECTION_HPP_
#define LIDAR_LOCALIZATION_IMAGE_PROJECTION_IMAGE_PROJECTION_HPP_

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <ostream>
#include <yaml-cpp/yaml.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include "lidar_localization/sensor_param/lidar_param.hpp"
#include "lidar_localization/tools/cloud_info.h"


using namespace std;

namespace lidar_localization{

    
// typedef  CloudData::POINT PointType;
typedef pcl::PointXYZI PointType;

class ImageProjection{

public:

    ImageProjection(const LidarParam& lidar_param);

    void processCloud(pcl::PointCloud<PointType>::Ptr cloud_in);
    void removeGround(pcl::PointCloud<PointType>::Ptr cloud_in);

    inline void reset(){
        resetParameters();
    }
    
    inline void isMakeGroundCloud(const bool &flag){
        make_ground_cloud_flag_ = flag;
    };

    inline void isMakePureCloud(const bool &flag){
        make_pure_cloud_flag_ = flag;
    }

    pcl::PointCloud<PointType>::Ptr getFullCloud();
    pcl::PointCloud<PointType>::Ptr getFullInfoCloud();
    pcl::PointCloud<PointType>::Ptr getGroundCloud();
    pcl::PointCloud<PointType>::Ptr getGroundRemovedCloud();
    pcl::PointCloud<PointType>::Ptr getSegmentedCloud();
    pcl::PointCloud<PointType>::Ptr getSegmentedCloudPure();
    cloud_msgs::cloud_info getSegmentedCloudInfo();
    pcl::PointCloud<PointType>::Ptr getOutlierCloud();
    
    
private:
    // lidar param
    LidarParam lidar_param_;
    
    // int N_SCAN = 64;
    // int Horizon_SCAN = 2083;
    // float ang_res_x = 360.0/float(Horizon_SCAN);
    // float ang_res_y = 26.8/float(N_SCAN-1);
    // float ang_bottom = 24.8;
    // int ground_scan_id_ = 55;

    // image projection param
    float sensor_min_range_ = 1.0;
    float sensor_segment_range_ = 1.5;
    float sensor_mount_angle_ = 0.0;   //传感器的安装角度
    float segment_theta_ = 60.0/180.0*M_PI; // decrese this value may improve accuracy
    int segment_valid_point_num_ = 5;
    int segment_valid_line_num_ = 3;
    float segment_alpha_x_ = lidar_param_.ang_res_x_ / 180.0 * M_PI;
    float segment_alpha_y_ = lidar_param_.ang_res_y_ / 180.0 * M_PI;


    pcl::PointCloud<PointType>::Ptr laser_cloud_in_;
    // pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr full_cloud_; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr full_info_cloud_; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr ground_removed_cloud_;
    pcl::PointCloud<PointType>::Ptr ground_cloud_;
    pcl::PointCloud<PointType>::Ptr segmented_cloud_;
    pcl::PointCloud<PointType>::Ptr segmented_cloud_pure_;
    pcl::PointCloud<PointType>::Ptr outlier_cloud_;

    PointType nan_point_; // fill in fullCloud at each iteration

    cv::Mat range_mat_; // range matrix for range image
    cv::Mat label_mat_; // label matrix for segmentaiton marking
    cv::Mat ground_mat_; // ground matrix for ground cloud marking
    int label_count_;

    float start_orientation_;
    float end_orientation_;

    cloud_msgs::cloud_info seg_msg_;

    uint16_t *queue_ind_x_;  // array for breadth-first search process of segmentation, for speed
    uint16_t *queue_ind_y_;

    uint16_t *all_pushed_ind_x_; // array for tracking points of a segmented object
    uint16_t *all_pushed_ind_y_;

    std::vector<std::pair<int8_t, int8_t> > neighbor_iterator_; // neighbor iterator for segmentaiton process

    bool make_ground_cloud_flag_ = false;
    bool make_pure_cloud_flag_ = false;

    // custom param

private:
    void initParam();

    void allocateMemory();
    void resetParameters();

    void findStartEndAngle();
    void  projectPointCloud();
    void groundRemoval();
    void cloudSegmentation();
    void labelComponents(int row, int col);


};


}
#endif
