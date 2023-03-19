#include <ros/ros.h>

#include "lidar_localization/loam_fea/feature_extraction_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "feature_extraction_node");
    ros::NodeHandle nh;

    std::shared_ptr<FeatureExtractorFlow> feature_extraction_flow_ptr = std::make_shared<FeatureExtractorFlow>(nh);

    ROS_INFO("\033[1;32m---->\033[0m Feature Extractor Node Loaded.");

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        feature_extraction_flow_ptr->run();

        rate.sleep();
    }

    return 0;
}