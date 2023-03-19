#include <ros/ros.h>
#include "lidar_localization/loam_fea/image_projection_flow.hpp"


#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"

using namespace lidar_localization;         

int main(int argc, char *argv[]){

    ros::init(argc, argv, "image_projection_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    //nh.param<std::string>("cloud_topic", cloud_topic, "/rslidar_points");
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    std::shared_ptr<ImageProjectionFlow> image_projection_flow_ptr = std::make_shared<ImageProjectionFlow>(nh, cloud_topic);

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Node Loaded.");

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();

        image_projection_flow_ptr->run();

        rate.sleep();
    }

    return 0;
}