#ifndef LIDAR_LOCALIZATION_MODELS_FEATURES_LEGO_LOAM_SC_HPP_
#define LIDAR_LOCALIZATION_MODELS_FEATURES_LEGO_LOAM_SC_HPP_

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>
#include <math.h> 

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/models/lego_sc/KDTreeVectorOfVectorsAdaptor.h"
#include "lidar_localization/models/lego_sc/nanoflann.hpp"

//#include "tictoc.h"

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include "lidar_localization/models/rw/rw.hpp"

using namespace Eigen;
using namespace nanoflann;

using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;

namespace lidar_localization{

class SCManager{

public:
    SCManager( );

    Eigen::MatrixXd makeScancontext(CloudData::CLOUD & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    bool SaveScanContext(std::ofstream& ofs, const Eigen::MatrixXd & sc);

    void processHistoryKeyFrame();
    void processHistoryKeyFrame(std::vector<Eigen::MatrixXd>& memory_ringkey, 
            std::vector<Eigen::MatrixXd>& memory_sc, std::vector<Eigen::Matrix4f>& memory_pose);
    void exploreCandidates(Eigen::MatrixXd &cur_sc, std::vector<Eigen::MatrixXd>& memory_ringkey, 
            std::vector<Eigen::MatrixXd>& memory_sc, std::vector<Eigen::Matrix4f>& memory_pose, 
                    std::vector<Eigen::Matrix4f>& candidates);
    bool readSCinfo(std::string sc_info_path, Eigen::MatrixXd& sc_info, int m, int n);

public:
    // hyper parameters ()
    int PC_NUM_RING; // 20 in the original paper (IROS 18)
    int PC_NUM_SECTOR; // 60 in the original paper (IROS 18)

    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // loop thres
    const double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    const double SC_DIST_THRES = 0.5; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config 
    const int    TREE_MAKING_PERIOD_ = 10; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / in the LeGO-LOAM integration, it is synchronized with the loop detection callback (which is 1Hz) so it means the tree is updated evrey 10 sec. But you can use the smaller value because it is enough fast ~ 5-50ms wrt N.
    int          tree_making_period_conter = 0;

    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    //KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

private:

    void maxHeightInterface(CloudData::CLOUD & _scan_down, Eigen::MatrixXd& max_height_sc);
    void multiLevelHeightInterface(CloudData::CLOUD & _scan_down,
            Eigen::MatrixXd& multi_height_sc);
    void setParam();
        std::string sc_type;
        double LIDAR_HEIGHT; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.
        double PC_MAX_RADIUS; // 80 meter max in the original paper (IROS 18)
        // tree
        int NUM_EXCLUDE_RECENT; // simply just keyframe gap, but node position distance-based exclusion is ok. 
        int NUM_CANDIDATES_FROM_TREE; // 10 is enough. (refer the IROS 18 paper)

        int level_num;
        double max_hight;
        int level_contain_num;

    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 );
    MatrixXd circshift( MatrixXd &_mat, int _num_shift );
    int fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2);
    std::pair<double, int> distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 );

    float xy2theta( const float & _x, const float & _y );
    std::vector<float> eig2stdvec( MatrixXd _eigmat );

private:
    std::shared_ptr<RW> rw_ptr;
    std::vector<Eigen::Matrix4f> history_pose_vec;

};

}

#endif