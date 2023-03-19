#ifndef LIDAR_LOCALIZATION_MODELS_RW_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_RW_INTERFACE_HPP_

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include <yaml-cpp/yaml.h>

namespace lidar_localizationn{

class RWInterface{

public:
    virtual ~RWInterface() = default;

};
}

#endif