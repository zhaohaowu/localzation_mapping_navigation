
#ifndef LIDAR_LOCALIZATION_IMAGE_PROJECTION_IMAGE_PROJECTION_FLOW_HPP_
#define LIDAR_LOCALIZATION_IMAGE_PROJECTION_IMAGE_PROJECTION_FLOW_HPP_

#include <ros/ros.h>

#include "lidar_localization/sensor_param/lidar_param.hpp"
#include "lidar_localization/loam_fea/image_projection.hpp"

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/cloud_msgs_publisher.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "glog/logging.h"

namespace lidar_localization{

class ImageProjectionFlow{

    public:
        ImageProjectionFlow(ros::NodeHandle& nh, std::string cloud_topic);

        bool run();

    private:
        bool readData();
        bool hasData();
        bool validData();
        bool checkSubcriber();
        bool publishData();

    private:


        std::shared_ptr<CloudSubscriber>   cloud_sub_ptr_;
        
        std::shared_ptr<CloudPublisher>     full_cloud_pub_ptr_;
        std::shared_ptr<CloudPublisher>     full_info_cloud_pub_ptr_;
        std::shared_ptr<CloudPublisher>     ground_cloud_pub_ptr_;
        std::shared_ptr<CloudPublisher>     ground_removed_cloud_pub_ptr_;
        std::shared_ptr<CloudPublisher>     segmented_cloud_pub_ptr_;
        std::shared_ptr<CloudPublisher>     segmented_cloud_pure_pub_ptr_;
        std::shared_ptr<CloudPublisher>     outlier_cloud_pub_ptr_;
        std::shared_ptr<CloudMsgsPublisher>     segmented_cloud_info_pub_ptr_;

        std::deque<CloudData> cloud_data_buff_;

        CloudData current_cloud_data_;

        std::shared_ptr<LidarParam> lidar_param_;
        std::shared_ptr<ImageProjection> image_projector_;
};




}



#endif