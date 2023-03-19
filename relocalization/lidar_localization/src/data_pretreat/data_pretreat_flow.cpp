#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"
namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    vlp16_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 100000);
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/synced_cloud", "/rslidar", 100);

}

bool DataPretreatFlow::run(){

    if (!ReadData())
        return false;

    while(HasData()) {
    
        if (!ValidData())
            continue;
        
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData(){
    
    vlp16_sub_ptr_->ParseData(cloud_data_buff_);

    if (cloud_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::HasData(){
    if (cloud_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::ValidData(){
    current_cloud_data_ = cloud_data_buff_.front();

    cloud_data_buff_.pop_front();

    return true;


}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);

    return true;
}
}