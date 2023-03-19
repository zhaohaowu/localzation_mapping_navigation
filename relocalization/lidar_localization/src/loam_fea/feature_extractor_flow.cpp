#include "lidar_localization/loam_fea/feature_extraction_flow.hpp"

namespace lidar_localization{

FeatureExtractorFlow::FeatureExtractorFlow(ros::NodeHandle& nh)
{
    segmented_cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/segmented_cloud", 10000);
    cloud_info_sub_ptr_ = std::make_shared<CloudMsgsSubcriber>(nh, "/segmented_cloud_info", 10000);

    sharp_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/sharp_cloud", "/rslidar", 1);
    less_sharp_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/less_sharp_cloud", "/rslidar", 1);
    flat_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/flat_cloud", "/rslidar", 1);
    less_flat_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/less_flat_cloud", "/rslidar", 1);

    lidar_param_ = std::make_shared<LidarParam>(LidarType::RS_16);
    feature_extraction_ = std::make_shared<FeatureExtractor>(*lidar_param_);
}


bool FeatureExtractorFlow::run(){
    if(!readData())
        return false;

    while(hasData()){
        if(!validData())
            continue;

        extractFeatures();

        publishData();
    }
}

bool FeatureExtractorFlow::extractFeatures()
{
    feature_extraction_->runExtactorOnce(current_segmented_cloud_.cloud_ptr, current_cloud_info_);
    
    sharp_cloud_ptr_ = feature_extraction_->getCornerPointsSharp();
    less_sharp_cloud_ptr_ = feature_extraction_->getCornerPointsLessSharp();
    flat_cloud_ptr_ = feature_extraction_->getSurfPointsFlat();
    less_flat_cloud_ptr_ = feature_extraction_->getSurfPointsLessFlat();
    
    return true;
}

bool FeatureExtractorFlow::publishData()
{
    //ROS_WARN("sharp_cloud_ptr_->points.size():%d",sharp_cloud_ptr_->points.size());
    sharp_cloud_pub_ptr_->Publish(sharp_cloud_ptr_, current_segmented_cloud_.time);
    less_sharp_cloud_pub_ptr_->Publish(less_sharp_cloud_ptr_, current_segmented_cloud_.time);
    flat_cloud_pub_ptr_->Publish(flat_cloud_ptr_, current_segmented_cloud_.time);
    less_flat_cloud_pub_ptr_->Publish(less_flat_cloud_ptr_, current_segmented_cloud_.time);

    return true;
}


bool FeatureExtractorFlow::readData(){
    segmented_cloud_sub_ptr_->ParseData(segmented_cloud_buff_);
    cloud_info_sub_ptr_->ParseData(cloud_info_buff_);
    return true;
}

bool FeatureExtractorFlow::hasData(){
    if(segmented_cloud_buff_.size() == 0)
        return false;
    if(cloud_info_buff_.size() == 0)
        return false;
    return true;
}

bool FeatureExtractorFlow::validData(){
    current_segmented_cloud_ = segmented_cloud_buff_.front();
    current_cloud_info_ = cloud_info_buff_.front();
    double diff_time = current_segmented_cloud_.time - current_cloud_info_.header.stamp.toSec();
    // std::cout << "diff_time:" << diff_time << std::endl;
    if(diff_time < -0.05){
        segmented_cloud_buff_.pop_front();
        return false;
    }

    if(diff_time > 0.05){
        cloud_info_buff_.pop_front();
        return false;
    }

    segmented_cloud_buff_.pop_front();
    cloud_info_buff_.pop_front();
    return true;
}






}