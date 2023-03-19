#include "lidar_localization/loam_fea/image_projection_flow.hpp"

namespace lidar_localization{


ImageProjectionFlow::ImageProjectionFlow(ros::NodeHandle& nh, std::string cloud_topic){
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 10000);
    full_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/full_cloud_projected", "/rslidar",1);
    full_info_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/full_info_cloud", "/rslidar",1);

    ground_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/ground_cloud", "/rslidar",1);
    ground_removed_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/ground_removed_cloud", "/rslidar",1);
    segmented_cloud_pub_ptr_ =  std::make_shared<CloudPublisher>(nh, "/segmented_cloud", "/rslidar",1);
    segmented_cloud_pure_pub_ptr_ =  std::make_shared<CloudPublisher>(nh, "/segmented_cloud_pure", "/rslidar",1);
    segmented_cloud_info_pub_ptr_ = std::make_shared<CloudMsgsPublisher>(nh, "/segmented_cloud_info", "/rslidar",1);
    outlier_cloud_pub_ptr_ =  std::make_shared<CloudPublisher>(nh, "/outlier_cloud", "/rslidar",1);


    lidar_param_ = std::make_shared<LidarParam>(LidarType::RS_16);
    image_projector_ = std::make_shared<ImageProjection>(*lidar_param_);
}

bool ImageProjectionFlow::run(){
    if(!readData())
        return false;

    while(hasData()){
        if(!validData())
            continue;
        
        checkSubcriber();

        image_projector_->processCloud(current_cloud_data_.cloud_ptr);
        publishData();
        image_projector_->reset();
    }
}

bool ImageProjectionFlow::readData(){
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool ImageProjectionFlow::hasData(){
    return cloud_data_buff_.size() > 0;
}


bool ImageProjectionFlow::validData(){
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();
    return true;
}

bool ImageProjectionFlow::checkSubcriber(){

    image_projector_->isMakeGroundCloud(ground_cloud_pub_ptr_->HasSubscribers());
    image_projector_->isMakePureCloud(segmented_cloud_pure_pub_ptr_->HasSubscribers());
    
    return true;
}

bool ImageProjectionFlow::publishData(){
    double time  = current_cloud_data_.time;

    auto full_cloud = image_projector_->getFullCloud();
    full_cloud_pub_ptr_->Publish(full_cloud, time);

    auto full_info_cloud = image_projector_->getFullInfoCloud();
    full_info_cloud_pub_ptr_->Publish(full_info_cloud, time);

    if(ground_cloud_pub_ptr_->HasSubscribers()){
        auto ground_cloud = image_projector_->getGroundCloud();
        ground_cloud_pub_ptr_->Publish(ground_cloud, time);
    }

    if(ground_removed_cloud_pub_ptr_->HasSubscribers()){
        auto ground_removed_cloud = image_projector_->getGroundRemovedCloud();
        ground_removed_cloud_pub_ptr_->Publish(ground_removed_cloud, time);
    }

    auto segmented_cloud = image_projector_->getSegmentedCloud();
    segmented_cloud_pub_ptr_->Publish(segmented_cloud, time);

    if(segmented_cloud_pure_pub_ptr_->HasSubscribers()){
        auto segmented_cloud_pure = image_projector_->getSegmentedCloudPure();
        segmented_cloud_pure_pub_ptr_->Publish(segmented_cloud_pure, time);
    }


    auto outlier_cloud = image_projector_->getOutlierCloud();
    outlier_cloud_pub_ptr_->Publish(outlier_cloud, time);

    auto segmented_cloud_info = image_projector_->getSegmentedCloudInfo();
    segmented_cloud_info_pub_ptr_->Publish(segmented_cloud_info, time);

    return true;
}

}