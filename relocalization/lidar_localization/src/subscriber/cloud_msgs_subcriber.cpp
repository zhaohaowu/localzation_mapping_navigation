#include "lidar_localization/subscriber/cloud_msgs_subcriber.hpp"

namespace lidar_localization{

CloudMsgsSubcriber::CloudMsgsSubcriber(ros::NodeHandle &nh, 
                                                                                        std::string topic_name, 
                                                                                        size_t buff_size)
        : nh_(nh){
            subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudMsgsSubcriber::msgs_callback, this);

    }

void CloudMsgsSubcriber::msgs_callback(const cloud_msgs::cloud_info& cloud_msgs){
    buff_mutex_.lock();
    new_msgs_data_.push_back(cloud_msgs);
    buff_mutex_.unlock();
}

void CloudMsgsSubcriber::ParseData(std::deque<cloud_msgs::cloud_info> &cloud_msgs_buff){
    buff_mutex_.lock();
    if(new_msgs_data_.size()  > 0 ){
        cloud_msgs_buff.insert(cloud_msgs_buff.end(), new_msgs_data_.begin(), new_msgs_data_.end());
        new_msgs_data_.clear();
    }
    buff_mutex_.unlock();
}











}
