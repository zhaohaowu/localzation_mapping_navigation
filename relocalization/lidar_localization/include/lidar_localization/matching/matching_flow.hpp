/*
 * @Description: matching 模块任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_MATCHING_FLOW_HPP_
#define LIDAR_LOCALIZATION_MATCHING_MATCHING_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// publisher
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"
// matching
#include "lidar_localization/matching/matching.hpp"
#include <nav_msgs/GridCells.h>

//#include "lidar_localization/models/features/lego_loam_sc.hpp"

namespace lidar_localization {
class MatchingFlow {
  public:
    MatchingFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool InitWithConfig();
    bool Run_lidar_only();
    bool ReadData_lidar_only();
    bool HasData_lidar_only();
    bool ValidData_lidar_only();
    bool UpdateMatching_lidar_only();
    bool PublishData();
    bool CreateFile(std::ofstream& ofs, std::string file_path);
    bool SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose);
    bool readPose(std::ifstream &fin, Eigen::Matrix4f& latest_pose);
    bool readLatestPose(Eigen::Matrix4f &latest_pose);

    

  private:
    // subscriber 
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::unique_ptr<CloudSubscriber> sharp_cloud_sub_ptr_;
    std::unique_ptr<CloudSubscriber> flat_cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
    std::unique_ptr<OdometrySubscriber> initial_odom_sub_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;

    std::unique_ptr<CloudPublisher> global_sharp_map_pub_ptr_;
    std::unique_ptr<CloudPublisher> global_flat_map_pub_ptr_;
    std::unique_ptr<CloudPublisher> local_sharp_map_pub_ptr_;
    std::unique_ptr<CloudPublisher> local_flat_map_pub_ptr_;

    ros::Publisher possible_cell_pub_;
    int once;
    std::vector<Eigen::Matrix4f> possible_pose_;
    
    // matching
    std::shared_ptr<Matching> matching_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> initial_odom_buff_;
    std::deque<CloudData> sharp_cloud_buff_;
    std::deque<CloudData> flat_cloud_buff_;
    std::deque<PoseData> gnss_data_buff_;

    CloudData current_cloud_data_;
    CloudData current_sharp_cloud_data_;
    CloudData current_flat_cloud_data_;
    PoseData current_gnss_data_;
    PoseData current_initial_odom_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

    std::string sensor_type;
    std::string latest_pose_path_;
};
}

#endif