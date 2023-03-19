#ifndef LIDAR_LOCALIZATION_SUBCRIBER_CLOUD_MSGS_SUBCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBCRIBER_CLOUD_MSGS_SUBCRIBER_HPP_


#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "lidar_localization/tools/cloud_info.h"


namespace lidar_localization{

class CloudMsgsSubcriber{

    public:
    CloudMsgsSubcriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    CloudMsgsSubcriber() = default;

    void ParseData(std::deque<cloud_msgs::cloud_info> &cloud_msgs_buff);

    private:
        void msgs_callback(const cloud_msgs::cloud_info& cloud_msgs);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<cloud_msgs::cloud_info> new_msgs_data_;

    std::mutex buff_mutex_;
};









}


#endif
