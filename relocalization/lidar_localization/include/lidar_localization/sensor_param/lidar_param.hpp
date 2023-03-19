#ifndef _SENSOR_PARAM_LIDAR_PARAM_HPP_
#define _SENSOR_PARAM_LIDAR_PARAM_HPP_
#include <bits/stdc++.h>
namespace lidar_localization
{

typedef enum 
{
    UNKNOW = 0,
    RS_16 = 1,
    VLP_32 = 2,
    HDL_64 = 3
}LidarType;


class LidarParam
{
public:
    LidarParam() = default;
    LidarParam(const std::string& lidar_name);
    LidarParam(const int& lidar_type);
    void setLidarParam(const int& lidar_type);
    bool setLidarParamFromYAML(const std::string& lidar_file_name);

public:
    // lidar param
    // 雷达线数
    int N_SCAN; 
    // 雷达每条线具有的激光点数
    int Horizon_SCAN;
    // 水平方向的分辨率
    float ang_res_x_;
    // 垂直方向上的分辨率
    float ang_res_y_;
    //  最想面线束的角度
    float ang_top_;
    // 最下面线束的角度
    float ang_bottom_;
    // 小于该id的线束用于判断地面点
    int ground_scan_id_;
  
};






}



#endif