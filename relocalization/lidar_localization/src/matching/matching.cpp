/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/matching/matching.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

namespace lidar_localization {
Matching::Matching(const YAML::Node& config_node)
    :   global_sharp_map_ptr_(new CloudData::CLOUD()),
        global_flat_map_ptr_ (new CloudData::CLOUD()),
        local_sharp_map_ptr_(new CloudData::CLOUD()),
        local_flat_map_ptr_(new CloudData::CLOUD()),
        current_sharp_scan_ptr_(new CloudData::CLOUD()),
        current_flat_scan_ptr_(new CloudData::CLOUD()),
      current_scan_ptr_(new CloudData::CLOUD()) 
{
    
    InitWithConfig(config_node);

}

bool Matching::InitWithConfig(const YAML::Node& config_node) {

    matching_method = config_node["matching_method"].as<std::string>();
    pre_init = config_node["pre_init"].as<bool>();
    std::cout << matching_method << std::endl;
    InitDataPath(config_node);
    //InitRelocateDescriptor(config_node);
    has_inited_ = false;
    
    if (matching_method == "loam_fea"){
        
        InitLOAMRegistration(config_node);
        InitBoxFilter(config_node);
        //InitGlobalMap();
        InitGlobalFeatureMap();
        ResetLocalFeatureMap(0.0, 0.0, 0.0);
        return true;
    }
    else if (matching_method == "ori_fea"){

        // InitRegistration(registration_ptr_, config_node);
        // // a. global map filter -- downsample point cloud map for visualization:
        // InitFilter("global_map", global_map_filter_ptr_, config_node);
        // // b. local map filter -- downsample & ROI filtering for scan-map matching:
        // InitBoxFilter(config_node);
        // InitFilter("local_map", local_map_filter_ptr_, config_node);
        // // c. scan filter -- 
        // InitFilter("frame", frame_filter_ptr_, config_node);

        // InitGlobalMap();

        // ResetLocalMap(0.0, 0.0, 0.0);

        return true;
    }else{
        std::cout << "wrong feature type." << std::endl;
    }

    //InitScanContextManager(config_node);

    return true;
}

bool Matching::InitDataPath(const YAML::Node& config_node) {

    char abs_path[PATH_MAX];
    realpath(__FILE__, abs_path);
    std::string dirpath(abs_path);
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    sharp_map_path_ = dirpath + "/lio_sam/data_park/sharp_cloud_map.pcd";
    flat_map_path_ = dirpath + "/lio_sam/data_park/flat_cloud_map.pcd";
    return true;
}

bool Matching::InitLOAMRegistration(const YAML::Node& config_node)
{
    loam_icp_ptr_ = std::make_shared<LeGORegistration>(config_node["LOAM"]);
    return true;
}

bool Matching::InitBoxFilter(const YAML::Node& config_node) {
    box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}

bool Matching::InitGlobalFeatureMap()
{
    pcl::io::loadPCDFile(sharp_map_path_, *global_sharp_map_ptr_);
    pcl::io::loadPCDFile(flat_map_path_, *global_flat_map_ptr_);
    LOG(INFO) << "load global sharp map size:" << global_sharp_map_ptr_->points.size();
    LOG(INFO) << "load global flat map size:" << global_flat_map_ptr_->points.size();

    has_new_global_feature_map_ = true;

    return true;
}

bool Matching::ResetLocalFeatureMap(float x, float y, float z) {
    // TicToc reset_lmap_tic;

    std::vector<float> origin = {x, y, z};
    //std::cout << "x:" << x << "y:" << y <<std::endl;
    box_filter_ptr_->SetOrigin(origin);

    box_filter_ptr_->Filter(global_sharp_map_ptr_, local_sharp_map_ptr_);
    box_filter_ptr_->Filter(global_flat_map_ptr_, local_flat_map_ptr_);
    // kdtree_local_sharp_map_->setInputCloud(local_sharp_map_ptr_);
    // kdtree_local_flat_map_->setInputCloud(local_flat_map_ptr_);
    
    loam_icp_ptr_->SetInputTarget(local_sharp_map_ptr_, local_flat_map_ptr_);


    has_new_local_feature_map_ = true;

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    LOG(INFO) << "new local feature map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl;

    // reset_lmap_time_.push_back (static_cast<float>(reset_lmap_tic.toc()));
    return true;
}

void Matching::sc_relocation(const CloudData& cloud_data,Eigen::Matrix4f &guess_pose){

    //read his info
    SCManager sc_guess;
    std::vector<Eigen::MatrixXd> memory_ringkey;
    std::vector<Eigen::MatrixXd> memory_sc;
    std::vector<Eigen::Matrix4f> memory_pose;
    sc_guess.processHistoryKeyFrame(memory_ringkey, memory_sc, memory_pose);
    std::vector<Eigen::Matrix4f> candidate_poses;

    Eigen::MatrixXd cur_sc = sc_guess.makeScancontext(*cloud_data.cloud_ptr);
    Eigen::MatrixXd cur_ringkey = sc_guess.makeRingkeyFromScancontext(cur_sc);
    sc_guess.exploreCandidates(cur_sc, memory_ringkey, memory_sc, memory_pose, candidate_poses);

    guess_pose = candidate_poses.back();
}
bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    // ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));

    return true;
}

bool Matching::Update_lidar_only(const CloudData& cloud_data, 
                                              const CloudData& sharp_cloud_data,
                                              const CloudData& flat_cloud_data,
                                              Eigen::Matrix4f& pose_estimited)
{
    if(!has_inited_)
    {
        ResetLocalFeatureMap(init_pose_(0,3), init_pose_(1,3), init_pose_(2,3));
        // 用于匹配的位姿
        pose_estimited = init_pose_;
        
        has_inited_ = true;
        
    }

    static Eigen::Matrix4f last_predict_step_pose = Eigen::Matrix4f::Identity();  //预测位姿与估计位姿last之间的变换
    static Eigen::Matrix4f last_pose = pose_estimited; //上次估计的位姿
    static Eigen::Matrix4f predict_pose = pose_estimited; // 作为里程计配准的初始位姿

    //the feature cloud of currrent cloud which is used in matching
    current_sharp_scan_ptr_ = sharp_cloud_data.cloud_ptr;
    current_flat_scan_ptr_ = flat_cloud_data.cloud_ptr;

    loam_icp_ptr_->ScanMatch(current_sharp_scan_ptr_, current_flat_scan_ptr_, predict_pose, predict_pose);

    last_predict_step_pose = last_pose.inverse() * predict_pose;
    // std::cout << "predict_pose:" << predict_pose << std::endl;
    // std::cout << "last_pose:" << last_pose << std::endl;
    // std::cout << "last_predict_step_pose:" << last_predict_step_pose << std::endl;
    pose_estimited = predict_pose;
    last_pose = pose_estimited;
    predict_pose = pose_estimited * last_predict_step_pose;             

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if (fabs(pose_estimited(i, 3) - edge.at(2 * i)) > 30.0 &&
            fabs(pose_estimited(i, 3) - edge.at(2 * i + 1)) > 30.0)
            continue;
        ResetLocalFeatureMap(pose_estimited(0,3), pose_estimited(1,3), pose_estimited(2,3));
        break;
    }                                

    return true;
}


void Matching::GetGlobalFeatureMap(CloudData::CLOUD_PTR& global_sharp_map, 
                                                                            CloudData::CLOUD_PTR& global_flat_map)
{
    // has_new_global_feature_map_ = false;
    global_sharp_map = global_sharp_map_ptr_;
    global_flat_map = global_flat_map_ptr_;
}

void Matching::GetLocalFeatureMap(CloudData::CLOUD_PTR& local_sharp_map, 
                                                                        CloudData::CLOUD_PTR& local_flat_map)
{
    // has_new_local_feature_map_ = false;
    local_sharp_map = local_sharp_map_ptr_;
    local_flat_map = local_flat_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetCurrentScan() {
    return current_scan_ptr_;
}

bool Matching::HasNewGlobalFeatureMap(){
    return has_new_global_feature_map_;
}

bool Matching::HasNewLocalFeatureMap(){
    return has_new_local_feature_map_;
}

}