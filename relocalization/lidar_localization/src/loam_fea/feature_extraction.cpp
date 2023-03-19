#include "lidar_localization/loam_fea/feature_extraction.hpp"

namespace lidar_localization{

FeatureExtractor::FeatureExtractor(const LidarParam& lidar_param)
    : lidar_param_(lidar_param)
{
    initExtrator();
}


FeatureExtractor::~FeatureExtractor()
{
    // cloud_curvature_.release();
    // delete cloud_neighbor_picked_;
    // delete cloud_label_;

}

void FeatureExtractor::initExtrator()
{
    int N_SCAN = lidar_param_.N_SCAN;
    int Horizon_SCAN = lidar_param_.Horizon_SCAN;

    N_SCAN = lidar_param_.N_SCAN;
    Horizon_SCAN = lidar_param_.Horizon_SCAN;

    cloud_curvature_ = new float[N_SCAN*Horizon_SCAN];
    cloud_neighbor_picked_ = new int[N_SCAN*Horizon_SCAN];
    cloud_label_ = new int[N_SCAN*Horizon_SCAN];

    cloud_smoothness_.resize(N_SCAN*Horizon_SCAN);

    down_size_filter_.setLeafSize(0.2, 0.2, 0.2);

    corner_points_sharp_.reset( new pcl::PointCloud<PointType>() );
    corner_points_less_sharp_.reset( new pcl::PointCloud<PointType>() );
    surf_points_flat_.reset( new pcl::PointCloud<PointType>() );
    surf_points_less_flat_.reset( new pcl::PointCloud<PointType>() );

    surf_points_flat_scan_.reset( new pcl::PointCloud<PointType>() );
    surf_points_flat_scan_DS_.reset( new pcl::PointCloud<PointType>() );
}


void FeatureExtractor::runExtactorOnce(pcl::PointCloud<PointType>::Ptr segmented_cloud_in, cloud_msgs::cloud_info& info_in)
{
    segmented_cloud_ = segmented_cloud_in;
    //std::cout << segmented_cloud_in->points.size() << std::endl;
    seg_info_ = info_in;

    calculateSmoothness();
    markOccludedPoints();
    extractFeatures();
}



void FeatureExtractor::calculateSmoothness()
{
    int cloudSize = segmented_cloud_->points.size();
    for (int i = 5; i < cloudSize - 5; i++) {

        float diffRange = seg_info_.segmentedCloudRange[i-5] + seg_info_.segmentedCloudRange[i-4]
                        + seg_info_.segmentedCloudRange[i-3] + seg_info_.segmentedCloudRange[i-2]
                        + seg_info_.segmentedCloudRange[i-1] - seg_info_.segmentedCloudRange[i] * 10
                        + seg_info_.segmentedCloudRange[i+1] + seg_info_.segmentedCloudRange[i+2]
                        + seg_info_.segmentedCloudRange[i+3] + seg_info_.segmentedCloudRange[i+4]
                        + seg_info_.segmentedCloudRange[i+5];           

        cloud_curvature_[i] = diffRange*diffRange;
        cloud_neighbor_picked_[i] = 0;
        cloud_label_[i] = 0;

        cloud_smoothness_[i].value = cloud_curvature_[i];
        cloud_smoothness_[i].ind = i;
    }
}

void FeatureExtractor::markOccludedPoints()
{
    int cloud_size = segmented_cloud_->points.size();

    for(int i=5; i < cloud_size - 6; i++)
    {
        // 相邻两个点的range
        float depth1 = seg_info_.segmentedCloudRange[i];
        float depth2 = seg_info_.segmentedCloudRange[i+1];
        // 两个点之间的列id之差（因为去掉了外点，导致了点在id上的不连续，需要作判断）
        int column_diff = std::abs( static_cast<int>(seg_info_.segmentedCloudColInd[i+1] - seg_info_.segmentedCloudColInd[i]) );

        // 两点的列id相差小于10
        if(column_diff < 10)
        {
            // 相邻两点range相差0.3的，说明极有可能是断层点，即两个点不在同一个平面
            // 哪个点所在的平面在前，就对哪个点边缘的5个点进行mark处理
            if(depth1 - depth2 > 0.3)
            {
                cloud_neighbor_picked_[i - 5] = 1;
                cloud_neighbor_picked_[i - 4] = 1;
                cloud_neighbor_picked_[i - 3] = 1;
                cloud_neighbor_picked_[i - 2] = 1;
                cloud_neighbor_picked_[i - 1] = 1;
            }
            else if (depth2 - depth1 > 0.3) 
            {
                cloud_neighbor_picked_[i + 1] = 1;
                cloud_neighbor_picked_[i + 2] = 1;
                cloud_neighbor_picked_[i + 3] = 1;
                cloud_neighbor_picked_[i + 4] = 1;
                cloud_neighbor_picked_[i + 5] = 1;
                cloud_neighbor_picked_[i + 6] = 1;
            }
        }
        // 满足下面条件的点，也不会是特征点，mark掉
        // 即该点与相邻点的距离之差过大
        float diff1 = std::abs(static_cast<float>(seg_info_.segmentedCloudRange[i-1] - seg_info_.segmentedCloudRange[i]));
        float diff2 = std::abs(static_cast<float>(seg_info_.segmentedCloudRange[i+1] - seg_info_.segmentedCloudRange[i]));
        
        if (diff1 > 0.02 * seg_info_.segmentedCloudRange[i] && diff2 > 0.02 * seg_info_.segmentedCloudRange[i])
        cloud_neighbor_picked_[i] = 1;
    }
}


void FeatureExtractor::extractFeatures()
{
    corner_points_sharp_->clear();
    corner_points_less_sharp_->clear();
    surf_points_flat_->clear();
    surf_points_less_flat_->clear();

    for(int i=0; i < lidar_param_.N_SCAN; i++)
    {
        surf_points_flat_scan_->clear();
        // 将1个scan划分为6个区域
        for(int j = 0; j < 6; j++)
        {
            // 该区域的起始点和终止点
            int sp = (seg_info_.startRingIndex[i] * (6 - j)    + seg_info_.endRingIndex[i] * j) / 6;
            int ep = (seg_info_.startRingIndex[i] * (5 - j)    + seg_info_.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;
            // 按照曲率从小到大排列
            std::sort(cloud_smoothness_.begin()+sp, cloud_smoothness_.begin()+ep, by_value());

            // **选取 Edge Points
            int largest_picked_num = 0;
            for(int k = ep; k > sp; k--)
            {
                int ind = cloud_smoothness_[k].ind;
                // 判断条件 a.该点没有被mark掉  b.该点的曲率大于阈值  c. 该点不是地面点
                if(cloud_neighbor_picked_[ind] == 0 &&
                    cloud_curvature_[ind] > edge_threshold_ && 
                    seg_info_.segmentedCloudGroundFlag[ind] == false)
                {
                    largest_picked_num++;

                    if(largest_picked_num <= 2)
                    {
                        cloud_label_[ind] = 2;  // edge points 的label 为 2
                        corner_points_sharp_->push_back(segmented_cloud_->points[ind]);
                        corner_points_less_sharp_->push_back(segmented_cloud_->points[ind]);
                    }
                    else if(largest_picked_num <= 20)
                    {
                        cloud_label_[ind] = 1;  // less sharp points  =  1
                        corner_points_less_sharp_->push_back(segmented_cloud_->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloud_neighbor_picked_[ind] = 1;  // 该点被选过了
                    // mark该点前后的5个点（这个操作与LOAM相同）
                    for(int l=1; i<=5; l++)
                    {
                        int column_diff = std::abs( static_cast<int>(seg_info_.segmentedCloudColInd[ind + l] - seg_info_.segmentedCloudColInd[ind + l - 1]) );
                        if (column_diff > 10)
                            break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }

                    for (int l = -1; l >= -5; l--) 
                    {
                        int columnDiff = std::abs(int(seg_info_.segmentedCloudColInd[ind + l] - seg_info_.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                } // end of if 
            } // end of for

            // **选取Planar Points
            int smallest_picked_num = 0;
            for(int k = sp; k <= ep; k++)
            {
                int ind = cloud_smoothness_[k].ind;
                // 判断条件 a.该点没有被mark掉  b.该点的曲率大于阈值  c. 该点是地面点
                if(cloud_neighbor_picked_[ind] == 0 &&
                    cloud_curvature_[ind] < surf_threshold && 
                    seg_info_.segmentedCloudGroundFlag[ind] == true)
                {
                    cloud_label_[ind] = -1; // flat point label == -1
                    surf_points_flat_->push_back(segmented_cloud_->points[ind]);

                    smallest_picked_num ++;
                    if(smallest_picked_num >= 4)
                        break;
                    
                    cloud_neighbor_picked_[ind] = 1;
                    // mark该点前后的5个点（这个操作与LOAM相同）
                    for(int l=1; i<=5; l++)
                    {
                        int column_diff = std::abs( static_cast<int>(seg_info_.segmentedCloudColInd[ind + l] - seg_info_.segmentedCloudColInd[ind + l - 1]) );
                        if (column_diff > 10)
                            break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }

                    for (int l = -1; l >= -5; l--) 
                    {
                        int columnDiff = std::abs(int(seg_info_.segmentedCloudColInd[ind + l] - seg_info_.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                }
            }

            for(int k = sp; k <= ep; k++)
            {
                // 不是 sharp point的，都是surf points flat scan（label == 0 || label == -1）
                // 这里是一条激光，scan
                if(cloud_label_[k] <= 0)
                {
                    surf_points_flat_scan_->push_back(segmented_cloud_->points[k]);
                }
            }

        } // 检测完六个区域的点

        // 对当前scan进行滤波处理
        surf_points_flat_scan_DS_->clear();
        down_size_filter_.setInputCloud(surf_points_flat_scan_);
        down_size_filter_.filter(*surf_points_flat_scan_DS_);

        *surf_points_less_flat_ += *surf_points_flat_scan_DS_;
   }
}

pcl::PointCloud<PointType>::Ptr FeatureExtractor::getCornerPointsSharp()
{
    return corner_points_sharp_;
}

pcl::PointCloud<PointType>::Ptr FeatureExtractor::getCornerPointsLessSharp()
{
    return corner_points_less_sharp_;
}

pcl::PointCloud<PointType>::Ptr FeatureExtractor::getSurfPointsFlat()
{
    return surf_points_flat_;
}

pcl::PointCloud<PointType>::Ptr FeatureExtractor::getSurfPointsLessFlat()
{
    return surf_points_less_flat_;
}

}   // end of namespace
