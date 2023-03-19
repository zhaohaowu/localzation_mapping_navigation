#include "lidar_localization/models/features/lego_loam_sc.hpp"

namespace lidar_localization{

SCManager::SCManager( ){
    setParam();

    rw_ptr = std::make_shared<RW>();
}

void SCManager::setParam(){
    char abs_path[PATH_MAX];
    realpath(__FILE__, abs_path);
    std::string dirpath(abs_path);
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    std::string config_path = dirpath + "/config/matching/lego_loam_sc.yaml";
    YAML::Node config_node = YAML::LoadFile(config_path);

    sc_type = config_node["sc_type"].as<std::string>();

    LIDAR_HEIGHT = config_node["LIDAR_HEIGHT"].as<double>(); // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.
    PC_NUM_RING = config_node["PC_NUM_RING"].as<int>(); // 20 in the original paper (IROS 18)
    PC_NUM_SECTOR = config_node["PC_NUM_SECTOR"].as<int>(); // 60 in the original paper (IROS 18)
    PC_MAX_RADIUS = config_node["PC_MAX_RADIUS"].as<double>(); // 80 meter max in the original paper (IROS 18)
    // tree
    NUM_EXCLUDE_RECENT = config_node["NUM_EXCLUDE_RECENT"].as<int>(); // simply just keyframe gap, but node position distance-based exclusion is ok. 
    NUM_CANDIDATES_FROM_TREE = config_node["NUM_CANDIDATES_FROM_TREE"].as<int>();

    level_num = config_node["level_num"].as<int>();
    max_hight = config_node["max_hight"].as<double>();
    level_contain_num = config_node["level_contain_num"].as<int>();
}

MatrixXd SCManager::makeScancontext(CloudData::CLOUD & _scan_down ){

    MatrixXd desc;
    if (sc_type == "max_height"){
        std::cout << "max height sc used." << std::endl;
        maxHeightInterface(_scan_down, desc);
    }else if (sc_type == "average_height"){
        std::cout << "average height sc used." << std::endl;
    }else if (sc_type == "multi_height"){
        std::cout << "multi height sc used." << std::endl;
        multiLevelHeightInterface(_scan_down, desc);
    }
    else{
        std::cout << "can not find appropriate sc!" << std::endl;
    }
    

    return desc;
}

MatrixXd SCManager::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc ){

    /* 
     * summary: rowwise mean vector
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }

    return invariant_key;
}

MatrixXd SCManager::makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc ){

    /* 
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
}

void SCManager::maxHeightInterface(CloudData::CLOUD & _scan_down, Eigen::MatrixXd& max_height_sc){

    int num_pts_scan_down = _scan_down.points.size();
    // main
    const int NO_POINT = -1000;
    max_height_sc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
    CloudData::POINT pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++){
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;

        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
        sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );

        // taking maximum z 
        if ( max_height_sc(ring_idx-1, sctor_idx-1) < pt.z ) // -1 means cpp starts from 0
            max_height_sc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)
    for ( int row_idx = 0; row_idx < max_height_sc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < max_height_sc.cols(); col_idx++ )
            if( max_height_sc(row_idx, col_idx) == NO_POINT )
                max_height_sc(row_idx, col_idx) = 0;
    
}

void SCManager::multiLevelHeightInterface(CloudData::CLOUD & _scan_down, 
        Eigen::MatrixXd& multi_height_sc){

    int num_pts_scan_down = _scan_down.points.size();
    // main
    const int NO_POINT = -1000;
    //multi_height_sc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
    multi_height_sc = MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);
    std::vector<Eigen::MatrixXd> level_vec;
    for (int i = 0; i < level_num; i++){
        Eigen::MatrixXd matrix_count = MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);
        level_vec.push_back(matrix_count);
    }
    
    CloudData::POINT pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++){
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT;

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;

        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
        sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );

        double level_height = max_hight/level_num;
        if (pt.z < 0.3){
            continue;
        }
        int cur_level = pt.z/level_height;
        if (cur_level > level_num-1){
            continue;
        }
        level_vec[cur_level](ring_idx-1, sctor_idx-1) = level_vec[cur_level](ring_idx-1, sctor_idx-1) + 1;
    }

    for (int i = 0; i < level_num; i++){
        for ( int row_idx = 0; row_idx < level_vec[i].rows(); row_idx++ ){
            for ( int col_idx = 0; col_idx < level_vec[i].cols(); col_idx++ ){
                if( level_vec[i](row_idx, col_idx) > level_contain_num){
                    multi_height_sc(row_idx, col_idx) = multi_height_sc(row_idx, col_idx) + 1;
                }
            }
        }
        
    }
}

void SCManager::processHistoryKeyFrame(std::vector<Eigen::MatrixXd>& memory_ringkey, 
            std::vector<Eigen::MatrixXd>& memory_sc, std::vector<Eigen::Matrix4f>& memory_pose){
    char abs_path[PATH_MAX];
    realpath(__FILE__, abs_path);
    std::string dirpath(abs_path);
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    std::string key_pose_list_path = dirpath + "/slam_data/sc/key_poses.txt";
    std::string key_cloud_path = dirpath + "/slam_data/sc/key_frames";

    rw_ptr->readPoseData(memory_pose, key_pose_list_path);
    int file_num = memory_pose.size();
    std::cout << "file_num:"<< file_num << std::endl;
    CloudData::CLOUD_PTR temp_cloud(new CloudData::CLOUD());
    Eigen::MatrixXd temp_sc;
    Eigen::MatrixXd temp_ringkey;
    Eigen::MatrixXd temp_sectorkey;
    std::ofstream ofs;
    for (int i = 0; i < file_num; i++){
        rw_ptr->readCloudData(temp_cloud, key_cloud_path + "/key_cloud_" + std::to_string(i) + ".pcd");
        temp_sc = makeScancontext(*temp_cloud);
        temp_ringkey = makeRingkeyFromScancontext(temp_sc);
        memory_sc.push_back(temp_sc);
        memory_ringkey.push_back(temp_ringkey);
    }

}

void SCManager::processHistoryKeyFrame(){
    char abs_path[PATH_MAX];
    realpath(__FILE__, abs_path);
    std::string dirpath(abs_path);
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    std::string key_pose_list_path = dirpath + "/slam_data/sc/key_poses.txt";
    std::string key_cloud_path = dirpath + "/slam_data/sc/key_frames";
    std::string save_sc_path = dirpath + "/slam_data/sc";
    
    rw_ptr->readPoseData(history_pose_vec, key_pose_list_path);
    int file_num = history_pose_vec.size();
    std::cout << "file_num:"<< file_num << std::endl;
    CloudData::CLOUD_PTR temp_cloud(new CloudData::CLOUD());
    Eigen::MatrixXd temp_sc;
    Eigen::MatrixXd temp_ringkey;
    Eigen::MatrixXd temp_sectorkey;
    std::ofstream ofs;
    for (int i = 0; i < file_num; i++){
        rw_ptr->readCloudData(temp_cloud, key_cloud_path + "/key_cloud_" + std::to_string(i) + ".pcd");
        temp_sc = makeScancontext(*temp_cloud);
        temp_ringkey = makeRingkeyFromScancontext(temp_sc);
        temp_sectorkey = makeSectorkeyFromScancontext(temp_sc);
        rw_ptr->CreateFile(ofs, save_sc_path + "/sc/" + std::to_string(i) + ".txt");
        SaveScanContext(ofs, temp_sc);
        rw_ptr->CreateFile(ofs, save_sc_path + "/ringkey/" + std::to_string(i) + ".txt");
        SaveScanContext(ofs, temp_ringkey);
        rw_ptr->CreateFile(ofs, save_sc_path + "/sectorkey/" + std::to_string(i) + ".txt");
        SaveScanContext(ofs, temp_ringkey);
    }
}

bool SCManager::readSCinfo(std::string sc_info_path, Eigen::MatrixXd& sc_info, int m, int n){

    std::ifstream fin_sc_info(sc_info_path);
    if (!fin_sc_info){
        std::cerr << "Cannot load sc info file at : " << sc_info_path << std::endl;
        return false;
    }

    sc_info.resize(m,n);

    std::string temp;
    int line_count = 0;
    while (getline(fin_sc_info, temp)){
        std::vector<float> data_vec;
        float data;

        std::istringstream iss(temp);
        while (iss >> data){
            data_vec.push_back(data);
        }
        assert(data_vec.size() == n);

        for (int i = 0; i < n; i++){
            sc_info(line_count, i) = static_cast<double>(data_vec[i]);
        }
        line_count++;
    }
}

void SCManager::exploreCandidates(Eigen::MatrixXd &cur_sc, std::vector<Eigen::MatrixXd>& memory_ringkey, 
            std::vector<Eigen::MatrixXd>& memory_sc, std::vector<Eigen::Matrix4f>& memory_pose, 
                    std::vector<Eigen::Matrix4f>& candidates){

    //与当前帧最近似的K个历史关键帧的索引
    std::vector<size_t> candidates_indexes(NUM_CANDIDATES_FROM_TREE);
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

    KeyMat polarcontext_invkeys_to_search_;
    std::vector<float> polarcontext_invkey_vec_temp;
    KeyMat temp_load;
    //memory_ringkey里面存储的是历史关键帧对应的ringkey描述子，即一个由Eigen::MatrixXd构成的向量
    for (size_t i = 0; i < memory_ringkey.size(); i++){
        polarcontext_invkey_vec_temp = eig2stdvec( memory_ringkey[i] );
        temp_load.push_back(polarcontext_invkey_vec_temp);
    }
    //polarcontext_invkeys_to_search_即由历史关键帧对应的ringkey描述子构成的2维向量
    //std::vector<std::vector<float>>
    polarcontext_invkeys_to_search_.assign(temp_load.begin(), 
                    temp_load.end() - NUM_EXCLUDE_RECENT);

    Eigen::MatrixXd cur_ringkey = makeRingkeyFromScancontext(cur_sc);
    std::vector<float> cur_invkey = eig2stdvec(cur_ringkey);

    //基于历史ringkey描述子向量构建搜索树
    std::unique_ptr<InvKeyTree> polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, temp_load, 10 /* max leaf */ );
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    knnsearch_result.init( &candidates_indexes[0], &out_dists_sqr[0] );
    polarcontext_tree_->index->findNeighbors( knnsearch_result, &cur_invkey[0] /* query */, nanoflann::SearchParams(10) ); 

    //获取关键帧位姿
    for (size_t i = 0; i < candidates_indexes.size(); i++){
        candidates.push_back(memory_pose[candidates_indexes[i]]);
    }

    //return;

    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;
    int initial_id { -1 };
    for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ ){

        MatrixXd polarcontext_candidate = memory_sc[ candidates_indexes[candidate_iter_idx] ];
        std::pair<double, int> sc_dist_result = distanceBtnScanContext( cur_sc, polarcontext_candidate ); 
        
        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_idx = candidates_indexes[candidate_iter_idx];
        }
    }

    

    /* 
     * initial pose estimate
     */
    if( min_dist < SC_DIST_THRES )
    {
        initial_id = nn_idx; 
    
        // std::cout.precision(3); 
        std::cout << "[possible initial pose found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << std::endl;
        std::cout << "[possible initial pose found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << std::endl;
    }
    else
    {
        std::cout.precision(3); 
        std::cout << "[Not found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << std::endl;
        std::cout << "[Not found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << std::endl;
    }

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = nn_align * PC_UNIT_SECTORANGLE * M_PI / 180.0;
    std::pair<int, float> result {initial_id, yaw_diff_rad};

    
    std::cout << "PC_UNIT_SECTORANGLE:" << PC_UNIT_SECTORANGLE << std::endl;
    std::cout << "modify interval:" << nn_align << std::endl;
    std::cout << "most competitive index:" << nn_idx << std::endl;
    double modify_angle = nn_align * 360/PC_NUM_SECTOR;
    std::cout << "yaw_diff_rad:" << modify_angle << std::endl;
    modify_angle = modify_angle/180*M_PI;

    Eigen::Matrix3f modify_r;
    modify_r << cos(modify_angle), -sin(modify_angle), 0, sin(modify_angle), cos(modify_angle), 0,
            0, 0, 1;
    Eigen::Matrix4f modify_pose;
    modify_pose = memory_pose[nn_idx];
    modify_pose.topLeftCorner<3,3>() =  modify_r.inverse() * modify_pose.topLeftCorner<3,3>();
    candidates.push_back(modify_pose);
    //candidates.push_back(memory_pose[nn_idx]);
}

double SCManager::distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);
        
        if( col_sc1.norm() == 0 | col_sc2.norm() == 0 )
            continue; // don't count this sector pair. 

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }
    
    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

} // distDirectSC

MatrixXd SCManager::circshift( MatrixXd &_mat, int _num_shift )
{
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return 
    }

    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // circshift

int SCManager::fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
    {
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

} // fastAlignUsingVkey

std::pair<double, int> SCManager::distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 ){
    // 1. fast align using variant key (not in original IROS18)
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff 
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for ( int num_shift: shift_idx_search_space )
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return std::make_pair(min_sc_dist, argmin_shift);
}

float SCManager::xy2theta( const float & _x, const float & _y )
{
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
} // xy2theta

std::vector<float> SCManager::eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
} // eig2stdvec

bool SCManager::SaveScanContext(std::ofstream& ofs, const Eigen::MatrixXd & sc){

    /* ofs << "[";
    for (int i = 0; i < sc.rows(); ++i){
        
        for (int j = 0; j < sc.cols(); ++j){
            ofs << "[";
            ofs << std::to_string(i);
            ofs << ",";
            ofs << std::to_string(j);
            ofs << ",";
            ofs << sc(i, j);
            ofs << "],";
            
        }
    }
    ofs << "]"; */
    for (int i = 0; i < sc.rows(); ++i){
        for (int j = 0; j < sc.cols(); ++j){
            ofs << sc(i, j);
            ofs << " ";
        }
        ofs << std::endl;
    }
    
}

}