#include "lidar_localization/models/rw/rw.hpp"

namespace lidar_localization{

RW::RW(const YAML::Node& node){

}

RW::RW(){

}

bool RW::CreateFile(std::ofstream& ofs, std::string file_path) 
{
    ofs.close();
    boost::filesystem::remove(file_path.c_str());

    ofs.open(file_path.c_str(), std::ios::out);
    if (!ofs) {
        //LOG(WARNING) << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }

    return true;
}

bool RW::readCloudData(CloudData::CLOUD_PTR& cloud_ptr_, const std::string data_path){

    if (pcl::io::loadPCDFile(data_path, *cloud_ptr_)){
        std::cerr << "failed to open: " << data_path << std::endl;
    }
    std::cout << "read cloud data from:" << data_path << std::endl;
    std::cout << "the cloud size is:" << cloud_ptr_->points.size() << std::endl;
    return true;
}

bool RW::readPoseData(std::vector<Eigen::Matrix4f>& key_pose_lsit, const std::string data_path){

    std::ifstream fin_pose(data_path);
    if (!fin_pose){
        std::cerr << "Cannot load pose file at : " << data_path << std::endl;
        return false;
    }

    std::string temp;
    //std::vector<std::vector<float>> key_pose_list;
    while (getline(fin_pose, temp)){
        std::vector<float> data_vec;
        float data;

        std::istringstream iss(temp);
        while (iss >> data){
            data_vec.push_back(data);
        }
        assert(data_vec.size() == 12);
        //key_pose_list.push_back(data_vec);

        Eigen::Matrix4f keyframe_pose(Eigen::Matrix4f::Identity());
        int temp_cnt = 0;
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 4; j++){
                keyframe_pose(i, j) = static_cast<double>(data_vec[temp_cnt++]);
            }
        }
        key_pose_lsit.push_back(keyframe_pose);
        
    }

    std::cout << "key_pose_lsit.size():"<< key_pose_lsit.size() << std::endl;

    /* std::cout << "key_pose_list.size():" << key_pose_list.size() << std::endl;
    for (int i = 0; i < key_pose_list.size(); i++){
        TOOLB::printVector(key_pose_list[i]);
    } */

    return true;
}


}