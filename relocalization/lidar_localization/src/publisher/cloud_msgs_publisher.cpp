#include "lidar_localization/publisher/cloud_msgs_publisher.hpp"

namespace lidar_localization{

CloudMsgsPublisher::CloudMsgsPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id){
        publisher_ = nh_.advertise<cloud_msgs::cloud_info>(topic_name, buff_size);
}

void CloudMsgsPublisher::Publish(cloud_msgs::cloud_info &msgs_input, double time){
    ros::Time ros_time(time);
    PublishData(msgs_input, ros_time);
}

void CloudMsgsPublisher::Publish(cloud_msgs::cloud_info & msgs_input){
    ros::Time ros_time = ros::Time::now();
    PublishData(msgs_input, ros_time);
}

bool CloudMsgsPublisher::HasSubcribers(){
    return publisher_.getNumSubscribers() != 0;
}


void CloudMsgsPublisher::PublishData(cloud_msgs::cloud_info &msgs_input, ros::Time &time){
    msgs_input.header.stamp = time;
    msgs_input.header.frame_id = frame_id_;
    publisher_.publish(msgs_input);
}

} // namespace lidar_localization
