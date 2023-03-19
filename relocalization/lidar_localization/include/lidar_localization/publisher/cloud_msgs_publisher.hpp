#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_MSGS_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_MSGS_PUBLISHER_HPP_


#include <ros/ros.h>
#include "lidar_localization/tools/cloud_info.h"

namespace lidar_localization{

class CloudMsgsPublisher{
    public:
        CloudMsgsPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
        CloudMsgsPublisher() = default;

    void Publish(cloud_msgs::cloud_info &msgs_input, double time);
    void Publish(cloud_msgs::cloud_info & msgs_input);

    bool HasSubcribers();

    private:
        void PublishData(cloud_msgs::cloud_info &msgs_input, ros::Time &time);
        
    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;

};



}

#endif
