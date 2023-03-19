#include <ros/ros.h>
#include <bits/stdc++.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
using namespace std;
using namespace Eigen;
class Sc_pose{
private:
    ros::NodeHandle n;
    ros::Publisher pub_pose;
    ros::Subscriber sub_cloud;
    string path;
    string dirpath;
    int num = 143;
    vector<MatrixXd> polarcontexts_;
    vector<MatrixXd> polarcontext_invkeys_;
    vector<MatrixXd> polarcontext_vkeys_;
    vector<vector<float>> polarcontext_invkeys_mat_;
    std::unique_ptr<KDTreeVectorOfVectorsAdaptor<vector<vector<float>>, float>> polarcontext_tree_;
    deque<sensor_msgs::PointCloud2> q_key_cloud;
    const int    PC_NUM_RING = 20;
    const int    PC_NUM_SECTOR = 60;
    const double LIDAR_HEIGHT = 2.0;
    const double PC_MAX_RADIUS = 80.0;
    const int    NUM_CANDIDATES_FROM_TREE = 1;
    const double SEARCH_RATIO = 0.1;
    const double SC_DIST_THRES = 0.3; 
    const double PC_UNIT_SECTORANGLE = 6.0;
public:
    Sc_pose(){
        char abs_path[PATH_MAX];
        realpath(__FILE__, abs_path);
        dirpath = abs_path;
        dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
        dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
        path = dirpath + "/data_park/";
        
        pub_pose = n.advertise<nav_msgs::Odometry>("/initial_odom", 1);
        sub_cloud = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &Sc_pose::call_back_cloud, this);
    }
    void call_back_cloud(const sensor_msgs::PointCloud2ConstPtr& msgCloud){
        chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
        auto scd2matrix = [](string path, int a, int b){
            ifstream data(path);
            MatrixXd m_matrix(a, b);
            VectorXd hang(b);
            for (int j = 0; j < a; j++){
                for (int i = 0; i < b; i++){
                    data >> hang(i);
                }
                m_matrix.row(j) = hang;
            }
            return m_matrix;
        };
        static int init = 0;
        if(init==0){
            for(int i=0; i<num; i++){
                string old_string = to_string(i);
                string new_string = string(6 - old_string.length(), '0') + old_string;
                MatrixXd sc = scd2matrix(path+"SCDs/"+new_string+".scd", PC_NUM_RING, PC_NUM_SECTOR);
                MatrixXd ringkey = makeRingkeyFromScancontext(sc);
                vector<float> polarcontext_invkey_vec(ringkey.data(), ringkey.data() + ringkey.size());
                polarcontexts_.push_back(sc);
                polarcontext_invkeys_.push_back(ringkey);
                polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
            }

            init = 1;
        }
        
        q_key_cloud.push_back(*msgCloud);
        sensor_msgs::PointCloud2 current_key_cloud = q_key_cloud.front();
        // q_key_cloud.pop_front();
        pcl::PointCloud<pcl::PointXYZI> scan;
        pcl::fromROSMsg(current_key_cloud, scan);
        MatrixXd curr_desc = makeScancontext(scan);
        MatrixXd curr_ringkey = makeRingkeyFromScancontext(curr_desc);
        vector<float> curr_key(curr_ringkey.data(), curr_ringkey.data() + curr_ringkey.size());

        polarcontext_tree_.reset(); 
        polarcontext_tree_ = std::make_unique<KDTreeVectorOfVectorsAdaptor<vector<vector<float>>, float>>(PC_NUM_RING, polarcontext_invkeys_mat_, 10);
        double min_dist = 10000000;
        int nn_align = 0;
        int nn_idx = 0;
        vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
        vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );
        nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
        knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
        polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0], nanoflann::SearchParams(10) ); 
        for (int i = 0; i < NUM_CANDIDATES_FROM_TREE; i++){
            MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[i] ];
            std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
            double candidate_dist = sc_dist_result.first;
            int candidate_align = sc_dist_result.second;
            if( candidate_dist < min_dist ){
                min_dist = candidate_dist;
                nn_align = candidate_align;
                nn_idx = candidate_indexes[i];
            }
        }
        // nn_idx = candidate_indexes[0];
        
        if( min_dist < SC_DIST_THRES ){
            // cout << "重定位的关键帧索引：" << nn_idx << endl;
            // cout << "角度差异：" << nn_align * PC_UNIT_SECTORANGLE << "度" << endl;
            float yaw_diff_rad = nn_align * PC_UNIT_SECTORANGLE * M_PI / 180.0;
            nav_msgs::Odometry msgOdom;
            MatrixXd matrixIn = scd2matrix(path+"pose.txt", num, 6);
            msgOdom.header.frame_id = "map";
            msgOdom.pose.pose.position.x = matrixIn(nn_idx, 0);
            msgOdom.pose.pose.position.y = matrixIn(nn_idx, 1);
            
            msgOdom.pose.pose.position.z = matrixIn(nn_idx, 2);
            double roll = matrixIn(nn_idx, 3);
            double pitch = matrixIn(nn_idx, 4);
            double yaw = matrixIn(nn_idx, 5) - yaw_diff_rad;
            
            cout << "sc_x:" << matrixIn(nn_idx, 0) << endl;
            cout << "sc_y:" << matrixIn(nn_idx, 1) << endl;
            cout << "sc_yaw:" << 6.28 + yaw << endl;
            
            tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
            msgOdom.pose.pose.orientation.x = q.x();
            msgOdom.pose.pose.orientation.y = q.y();
            msgOdom.pose.pose.orientation.z = q.z();
            msgOdom.pose.pose.orientation.w = q.w();
            pub_pose.publish(msgOdom);
            chrono::steady_clock::time_point end_time = chrono::steady_clock::now();
            auto delta_time = chrono::duration_cast<chrono::duration<double>>(end_time - start_time);
            cout << "time:" << delta_time.count() << "s" << endl;
        }
    }
    MatrixXd makeScancontext(pcl::PointCloud<pcl::PointXYZI> _scan_down){
        int num_pts_scan_down = _scan_down.points.size();
        const int NO_POINT = -1000;
        MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
        pcl::PointXYZI pt;
        float azim_angle, azim_range;
        int ring_idx, sctor_idx;
        for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
        {
            pt.x = _scan_down.points[pt_idx].x;
            pt.y = _scan_down.points[pt_idx].y;
            pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT;
            azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
            auto xy2theta = [](const float& _x, const float& _y){
                if((_x >= 0) & (_y >= 0)) 
                    return (180/M_PI) * atan(_y/_x);
                if ((_x < 0) & (_y >= 0)) 
                    return 180 - ((180/M_PI) * atan(_y / (-_x)));
                if ( (_x < 0) & (_y < 0)) 
                    return 180 + ((180/M_PI) * atan(_y / _x));
                if ( (_x >= 0) & (_y < 0))
                    return 360 - ((180/M_PI) * atan((-_y) / _x));
            };
            azim_angle = xy2theta(pt.x, pt.y);
            if(azim_range > PC_MAX_RADIUS) continue;
            ring_idx = max(min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
            sctor_idx = max(min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);
            if(desc(ring_idx-1, sctor_idx-1) < pt.z ) desc(ring_idx-1, sctor_idx-1) = pt.z;
        }
        for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
            for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
                if( desc(row_idx, col_idx) == NO_POINT )
                    desc(row_idx, col_idx) = 0;
        return desc;
    };
    MatrixXd makeRingkeyFromScancontext(MatrixXd& _desc){
        MatrixXd invariant_key(_desc.rows(), 1);
        for(int row_idx = 0; row_idx < _desc.rows(); row_idx++){
            MatrixXd curr_row = _desc.row(row_idx);
            invariant_key(row_idx, 0) = curr_row.mean();
        }
        return invariant_key;
    }
    MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc){
        Eigen::MatrixXd variant_key(1, _desc.cols());
        for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ ){
            Eigen::MatrixXd curr_col = _desc.col(col_idx);
            variant_key(0, col_idx) = curr_col.mean();
        }
        return variant_key;
    }
    pair<double, int> distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 ){
        MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
        MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
        int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );
        const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
        std::vector<int> shift_idx_search_space { argmin_vkey_shift };
        for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ ){
            shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
            shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
        }
        std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());
        int argmin_shift = 0;
        double min_sc_dist = 10000000;
        for ( int num_shift: shift_idx_search_space ){
            MatrixXd sc2_shifted = circshift(_sc2, num_shift);
            double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
            if( cur_sc_dist < min_sc_dist ){
                argmin_shift = num_shift;
                min_sc_dist = cur_sc_dist;
            }
        }
        return make_pair(min_sc_dist, argmin_shift);
    }
    int fastAlignUsingVkey(MatrixXd & _vkey1, MatrixXd & _vkey2){
        int argmin_vkey_shift = 0;
        double min_veky_diff_norm = 10000000;
        for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ ){
            MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
            MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
            double cur_diff_norm = vkey_diff.norm();
            if(cur_diff_norm < min_veky_diff_norm){
                argmin_vkey_shift = shift_idx;
                min_veky_diff_norm = cur_diff_norm;
            }
        }
        return argmin_vkey_shift;
    }
    MatrixXd circshift(MatrixXd& _mat, int _num_shift){
        assert(_num_shift >= 0);
        if( _num_shift == 0 ){
            MatrixXd shifted_mat( _mat );
            return shifted_mat; // Early return 
        }
        MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
        for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ ){
            int new_location = (col_idx + _num_shift) % _mat.cols();
            shifted_mat.col(new_location) = _mat.col(col_idx);
        }
        return shifted_mat;
    }
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ){
        int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
        double sum_sector_similarity = 0;
        for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ ){
            VectorXd col_sc1 = _sc1.col(col_idx);
            VectorXd col_sc2 = _sc2.col(col_idx);
            if( (col_sc1.norm() == 0) | (col_sc2.norm() == 0) )
                continue;
            double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());
            sum_sector_similarity = sum_sector_similarity + sector_similarity;
            num_eff_cols = num_eff_cols + 1;
        }
        double sc_sim = sum_sector_similarity / num_eff_cols;
        return 1.0 - sc_sim;
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sc_pose"); //初始化节点
    Sc_pose sc_pose;
    ros::spin();
    return 0;
}
