#ifndef LIDAR_LOCALIZATION_MODELS_RW_HPP_
#define LIDAR_LOCALIZATION_MODELS_RW_HPP_

#include <iostream>
#include <string>
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/models/rw/rw_interface.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "glog/logging.h"
#include <Eigen/Core>

namespace lidar_localization{

class RW{

public:
    RW(const YAML::Node& node);
    RW();

    bool CreateFile(std::ofstream& ofs, std::string file_path);

    bool readCloudData(CloudData::CLOUD_PTR& cloud_ptr_, const std::string data_path);
    bool readPoseData(std::vector<Eigen::Matrix4f>& key_pose_lsit, const std::string data_path);

private:
    

};

}

#endif
