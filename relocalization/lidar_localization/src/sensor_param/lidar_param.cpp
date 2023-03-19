#include "lidar_localization/sensor_param/lidar_param.hpp"
#include <yaml-cpp/yaml.h>

namespace lidar_localization
{

LidarParam::LidarParam(const std::string& lidar_name)
{
  setLidarParamFromYAML(lidar_name);
}

LidarParam::LidarParam(const int& lidar_type)
{
  setLidarParam(lidar_type);
}

bool LidarParam::setLidarParamFromYAML(const std::string& lidar_name)
{
    char abs_path[PATH_MAX];
    realpath(__FILE__, abs_path);
    std::string dirpath(abs_path);
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
  std::string yaml_file_path = dirpath + "/config/sensor_param/lidar_param/" + lidar_name + ".yaml";
  YAML::Node config_node = YAML::LoadFile(yaml_file_path);
  if(config_node.IsNull())
  {
    //std::cout << "Cannot load " + lidar_name + ".yaml.  Using default Param. (RS16)";
    setLidarParam(LidarType::RS_16);
    return false;
  }

  N_SCAN = config_node["N_SCAN"].as<int>();
  Horizon_SCAN = config_node["Horizon_SCAN"].as<int>();
  ang_top_ = config_node["angle_top"].as<float>();
  ang_bottom_ = config_node["angle_bottom"].as<float>();
  ang_res_x_ = 360.0/float(Horizon_SCAN);
  ang_res_y_ = 30.0/float(N_SCAN-1);
  ground_scan_id_ = config_node["ground_scan_id"].as<int>();
  
  return true;
}

void LidarParam::setLidarParam(const int& lidar_type)
{
    if(lidar_type == LidarType::RS_16)
  {
    N_SCAN = 16;
    Horizon_SCAN = 1800;
    ang_top_ = 15.0f;
    ang_bottom_ = -15.0f;
    ang_res_x_ = 360.0/float(Horizon_SCAN);
    ang_res_y_ = 30.0/float(N_SCAN-1);
    ground_scan_id_ = 11;
  }
  else if(lidar_type == LidarType::VLP_32)
  {
    N_SCAN = 32;
    Horizon_SCAN = 1800;
    ang_res_x_ = 360.0/float(Horizon_SCAN);
    ang_res_y_ = 40.0/float(N_SCAN-1);
    ang_top_ = 15.0;
    ang_bottom_ = -25.0;
    ground_scan_id_ = 12;
  }
  else if(lidar_type == LidarType::HDL_64)
  {
    N_SCAN = 64;
    Horizon_SCAN = 2083;
    ang_res_x_ = 360.0/float(Horizon_SCAN);
    ang_res_y_ = 26.8/float(N_SCAN-1);
    ang_top_ = 2.0;
    ang_bottom_ = -24.8;
    ground_scan_id_ = 55;
  }
}


    

}