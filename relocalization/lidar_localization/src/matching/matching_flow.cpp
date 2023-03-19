/*
 * @Description: 地图匹配任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/matching/matching_flow.hpp"
#include "glog/logging.h"
#include "bits/stdc++.h"
using namespace std;
namespace lidar_localization {
MatchingFlow::MatchingFlow(ros::NodeHandle& nh) {

    InitWithConfig();
    // subscriber:
    // a. undistorted Velodyne measurement: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);

    sharp_cloud_sub_ptr_ = std::make_unique<CloudSubscriber>(nh, "/less_sharp_cloud", 100000);
    flat_cloud_sub_ptr_ = std::make_unique<CloudSubscriber>(nh, "/less_flat_cloud", 100000);

    // b. lidar pose in map frame:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    // publisher:
    // a. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    // b. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);

    // c. current scan:/vehicle_link
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "map", 100);
    // d. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("map", "vehicle_link");

    // for feature map
    global_sharp_map_pub_ptr_ = std::make_unique<CloudPublisher>(nh, "/global_sharp_map", "/map", 100);
    global_flat_map_pub_ptr_ = std::make_unique<CloudPublisher>(nh, "/global_flat_map", "/map", 100);
    local_sharp_map_pub_ptr_ = std::make_unique<CloudPublisher>(nh, "/local_sharp_map", "/map", 100);
    local_flat_map_pub_ptr_ = std::make_unique<CloudPublisher>(nh, "/local_flat_map", "/map", 100);

    possible_cell_pub_ = nh.advertise<nav_msgs::GridCells>("/possible_cell",1);
    once = 0;
    //TODO重定位
    initial_odom_sub_ptr_ = std::make_unique<OdometrySubscriber>(nh, "/initial_odom", 100000);
}

bool MatchingFlow::InitWithConfig() {
    char abs_path[PATH_MAX];
    realpath(__FILE__, abs_path);
    std::string dirpath(abs_path);
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    std::string config_file_path = dirpath + "/config/matching/matching.yaml";
    latest_pose_path_ = dirpath + "/slam_data/latest_pose.txt";

    YAML::Node config_node = YAML::LoadFile(config_file_path);

    matching_ptr_ = std::make_shared<Matching>(config_node);
    
    sensor_type = config_node["sensor_type"].as<std::string>();
    }


bool MatchingFlow::Run_lidar_only(){
    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    if (matching_ptr_->HasNewGlobalFeatureMap()){
        CloudData::CLOUD_PTR global_sharp_map, global_flat_map;
        matching_ptr_->GetGlobalFeatureMap(global_sharp_map, global_flat_map);
        global_sharp_map_pub_ptr_->Publish(global_sharp_map);
        global_flat_map_pub_ptr_->Publish(global_flat_map);
    }

    if  (matching_ptr_->HasNewLocalFeatureMap())
    {
        CloudData::CLOUD_PTR local_sharp_map, local_flat_map;
        matching_ptr_->GetLocalFeatureMap(local_sharp_map, local_flat_map);
        local_sharp_map_pub_ptr_->Publish(local_sharp_map);
        local_flat_map_pub_ptr_->Publish(local_flat_map);
    }

    ReadData_lidar_only();

    while(HasData_lidar_only()) {
        if (!ValidData_lidar_only()) {
            LOG(INFO) << "Invalid data. Skip matching" << std::endl;
            continue;
        }

        if (UpdateMatching_lidar_only()) {
            PublishData();
                chrono::steady_clock::time_point end_time = chrono::steady_clock::now();
    auto delta_time = chrono::duration_cast<chrono::duration<double>>(end_time - start_time);
    // std::cout << "time:" << delta_time.count() << "s" << std::endl;
        }
    }

    return true;
}

bool MatchingFlow::Run() {
   
    if (sensor_type == "lidar_only"){
        ROS_INFO_ONCE("use lidar only.");
        Run_lidar_only();
    }
    else if (sensor_type == "lidar_gnss"){
        ROS_INFO_ONCE("use lidar gnss.");
    }
    else{
        std::cout << "wrong sensor type.";
    }

    return true;
}


bool MatchingFlow::ReadData_lidar_only() {
    // pipe lidar measurements and pose into buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    sharp_cloud_sub_ptr_->ParseData(sharp_cloud_buff_);
    flat_cloud_sub_ptr_->ParseData(flat_cloud_buff_);
    return true;
}

bool MatchingFlow::HasData_lidar_only() {
    if (cloud_data_buff_.size() == 0)
        return false;

    if(sharp_cloud_buff_.size() == 0)
        return false;

    if(flat_cloud_buff_.size() == 0)
        return false;
    
    /* if (matching_ptr_->HasInited())
        return true; */

    return true;
}


bool MatchingFlow::ValidData_lidar_only() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_sharp_cloud_data_ = sharp_cloud_buff_.front();
    current_flat_cloud_data_ = flat_cloud_buff_.front();

    double diff_sharp_time = current_cloud_data_.time - current_sharp_cloud_data_.time;
    double diff_flat_time = current_cloud_data_.time - current_flat_cloud_data_.time;

    if (diff_sharp_time < -0.05 || diff_flat_time < -0.05) 
    {
        cloud_data_buff_.pop_front();
        return false;
    }

    if(diff_sharp_time > 0.05)
    {
        sharp_cloud_buff_.pop_front();
        return false;
    }

    if(diff_flat_time > 0.05)
    {
        flat_cloud_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    sharp_cloud_buff_.pop_front();
    flat_cloud_buff_.pop_front();

    return true;
}


bool MatchingFlow::UpdateMatching_lidar_only() {
    
    //TODO重定位
    initial_odom_sub_ptr_->ParseData(initial_odom_buff_);
    current_initial_odom_data_ = initial_odom_buff_.front();
    static int inited = 1;
    if (inited == 1 && initial_odom_buff_.size()!=0) {
        // matching_ptr_->SetGNSSPose(current_gnss_odom_data_.pose);
        // matching_ptr_->SetScanContextPose(current_cloud_data_);
        matching_ptr_->SetInitPose(current_initial_odom_data_.pose);
        inited = 0;
    }
    else if(inited == 0){
        // std::cout << "current_initial_odom_data_.pose:" << std::endl
        //           << current_initial_odom_data_.pose << std::endl;
        // std::cout << "initial_odom_buff_.size():" << initial_odom_buff_.size() << std::endl;
        matching_ptr_->Update_lidar_only(current_cloud_data_, current_sharp_cloud_data_, current_flat_cloud_data_,
                                                                    laser_odometry_);
    }

    return inited == 0;
}

bool MatchingFlow::PublishData() {
    auto R2ypr = [](const Eigen::Matrix3f &R){
        Eigen::Vector3f n = R.col(0);
        Eigen::Vector3f o = R.col(1);
        Eigen::Vector3f a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr;
    };
    // laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time);
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    Eigen::Vector3f eulerAngle = laser_odometry_.block<3,3>(0,0).eulerAngles(2, 1, 0);		//roll , pitch , yaw
    auto euler = R2ypr(laser_odometry_.block<3,3>(0,0));
    auto yaw = euler(0);
    // std::cout   << "x:" << laser_odometry_(0,3) << std::endl
    //             << "y:" << laser_odometry_(1,3) << std::endl
    //             // << "yaw:" << eulerAngle(0) << std::endl;
    //             << "yaw:" << yaw + 6.28 << endl;
    pcl::transformPointCloud(*current_cloud_data_.cloud_ptr, *current_cloud_data_.cloud_ptr, laser_odometry_);
    current_scan_pub_ptr_->Publish(current_cloud_data_.cloud_ptr);



    // std::ofstream ofs_temp;
    // CreateFile(ofs_temp, latest_pose_path_);
    // SavePose(ofs_temp, laser_odometry_);

    return true;
}

bool MatchingFlow::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.close();
    boost::filesystem::remove(file_path.c_str());

    ofs.open(file_path.c_str(), std::ios::out);
    if (!ofs) {
        //LOG(WARNING) << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }

    return true;
}

bool MatchingFlow::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ofs << pose(i, j);
                
                if (i == 2 && j == 3) {
                    ofs << std::endl;
                } else {
                    ofs << " ";
                }
            }
        }
        return true;
    }

bool MatchingFlow::readPose(std::ifstream &fin, Eigen::Matrix4f& latest_pose){

    std::string temp;
    getline(fin, temp);

    std::vector<float> data_vec;
    float data;

    std::istringstream iss(temp);
    while(iss  >> data)
        data_vec.push_back(data);
    assert(data_vec.size() == 12);

    int temp_cnt = 0;
    for(int i = 0; i < 3; i++){
            for(int j = 0; j < 4; j++){
                latest_pose(i, j) = static_cast<double>(data_vec[temp_cnt++]);
            }
    }
    latest_pose(3,0) = 0;
    latest_pose(3,1) = 0;
    latest_pose(3,2) = 0;
    latest_pose(3,3) = 1;

    return true;
}

bool MatchingFlow::readLatestPose(Eigen::Matrix4f &latest_pose){

    std::ifstream fin_latest_pose(latest_pose_path_);

    if(!fin_latest_pose){
        std::cout << "Cannot load latest pose file at : " << latest_pose_path_ << std::endl;
        return false;
    }

    readPose(fin_latest_pose, latest_pose);

    return true;
}

}