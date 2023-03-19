/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
// publishe
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
// models

namespace lidar_localization {
class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh);

    bool run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();
    //mxzz
    bool CreateFile(std::ofstream& ofs, std::string file_path);

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<CloudSubscriber> vlp16_sub_ptr_;

    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;

    CloudData current_cloud_data_;
    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();

    //mxzz
    bool mapping_mode;
    std::ofstream map_zero_ofs_;
};
}

#endif