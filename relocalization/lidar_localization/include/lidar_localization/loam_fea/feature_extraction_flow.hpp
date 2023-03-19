#ifndef LIDAR_LOCALIZATION_feature_extraction_FEATURE_EXTRACTION_FLOW_HPP_
#define LIDAR_LOCALIZATION_feature_extraction_FEATURE_EXTRACTION_FLOW_HPP_


#include <ros/ros.h>
#include "lidar_localization/sensor_param/lidar_param.hpp"
#include "lidar_localization/loam_fea/feature_extraction.hpp"

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/cloud_msgs_subcriber.hpp"

#include "lidar_localization/publisher/cloud_publisher.hpp"

#include "glog/logging.h"

namespace lidar_localization{

class FeatureExtractorFlow{

public:
    FeatureExtractorFlow(ros::NodeHandle& nh);

    bool run();

private:
    bool readData();
    bool hasData();
    bool validData();
    bool extractFeatures();
    // bool checkSubcriber();
    bool publishData();

private:
    std::shared_ptr<CloudSubscriber> segmented_cloud_sub_ptr_;
    std::shared_ptr<CloudMsgsSubcriber> cloud_info_sub_ptr_;

    std::shared_ptr<CloudPublisher> sharp_cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> less_sharp_cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> flat_cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> less_flat_cloud_pub_ptr_;

    std::deque<CloudData> segmented_cloud_buff_;
    std::deque<cloud_msgs::cloud_info> cloud_info_buff_;

    CloudData current_segmented_cloud_;
    cloud_msgs::cloud_info current_cloud_info_;

    std::shared_ptr<LidarParam> lidar_param_;
    std::shared_ptr<FeatureExtractor> feature_extraction_;

    CloudData::CLOUD_PTR sharp_cloud_ptr_;
    CloudData::CLOUD_PTR less_sharp_cloud_ptr_;
    CloudData::CLOUD_PTR flat_cloud_ptr_;
    CloudData::CLOUD_PTR less_flat_cloud_ptr_;
};

}


#endif
