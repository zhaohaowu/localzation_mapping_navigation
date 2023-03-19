/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/LeGO_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

LeGORegistration::LeGORegistration(const YAML::Node& node)
    :   local_sharp_map_(new CloudData::CLOUD()),
        local_flat_map_(new CloudData::CLOUD()),
        kdtree_corner_map_(new pcl::KdTreeFLANN<CloudData::POINT>()),
        kdtree_surf_map_(new pcl::KdTreeFLANN<CloudData::POINT>()) {

    sharp_voxel_filter_.setLeafSize(0.4, 0.4, 0.4);
    flat_voxel_filter_.setLeafSize(0.8, 0.8, 0.8);
    // float res = node["res"].as<float>();
    // float step_size = node["step_size"].as<float>();
    // float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_iter);
}

LeGORegistration::LeGORegistration(int max_iter)
    :   local_sharp_map_(new CloudData::CLOUD()),
        local_flat_map_(new CloudData::CLOUD()),
        kdtree_corner_map_(new pcl::KdTreeFLANN<CloudData::POINT>()),
        kdtree_surf_map_(new pcl::KdTreeFLANN<CloudData::POINT>()) {

    // sharp_voxel_filter_ptr_ = std::make_unique<pcl::VoxelGrid<CloudData::POINT>>();
    // flat_voxel_filter_ptr_ = std::make_unique<pcl::VoxelGrid<CloudData::POINT>>();
    sharp_voxel_filter_.setLeafSize(0.4, 0.4, 0.4);
    flat_voxel_filter_.setLeafSize(0.8, 0.8, 0.8);

    SetRegistrationParam(max_iter);
}

bool LeGORegistration::SetRegistrationParam(int max_iter) {

    max_iter_times_ = max_iter;

    std::cout << "LOAM-ICP 的匹配参数为：" << std::endl
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

void LeGORegistration::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr_ * point_curr + t_w_curr_;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
}

// void LeGORegistration::setCurrentPose(const Eigen::Matrix4f& pose)
// {
//     Eigen::Matrix3d rot_mat = pose.block<3,3>(0,0).cast<double>();
//     Eigen::Vector3d trans(pose(0,3), pose(1,3), pose(2,3));
//     q_w_curr_ = Eigen::Quaterniond(rot_mat);
//     t_w_curr_ = trans;
// }


bool LeGORegistration::SetInputTarget(const CloudData::CLOUD_PTR& target_sharp_map,
                                                                                const CloudData::CLOUD_PTR& target_flat_map) 
{
    // TODO: add voxel filter
    if(! (target_sharp_map->size() > 20 && target_flat_map->size() > 100) )
    {
        LOG(ERROR) << "Feature maps do not have enough points" << std::endl;
        return false;
    }
    TicToc t_tree;
    
    sharp_voxel_filter_.setInputCloud(target_sharp_map);
    sharp_voxel_filter_.filter(*local_sharp_map_);
    flat_voxel_filter_.setInputCloud(target_flat_map);
    flat_voxel_filter_.filter(*local_flat_map_);

    // local_sharp_map_ = target_sharp_map;
    // local_flat_map_ = target_flat_map;


    kdtree_corner_map_->setInputCloud(local_sharp_map_);
    kdtree_surf_map_->setInputCloud(local_flat_map_);
    printf("build tree time %f ms \n", t_tree.toc());
    return true;
}

/* bool LeGORegistration::ScanMatch(const CloudData::CLOUD_PTR& input_sharp_cloud, 
                                                                    const CloudData::CLOUD_PTR& input_flat_cloud, 
                                                                    const Eigen::Matrix4f& predict_pose, 
                                                                    Eigen::Matrix4f& result_pose)
{

    Eigen::Map<Eigen::Quaterniond> q_w_curr_map(parameters_);
    Eigen::Map<Eigen::Vector3d> t_w_curr_map(parameters_ + 4);

    Eigen::Matrix3d predict_rot_mat = predict_pose.block<3,3>(0,0).cast<double>();
    Eigen::Vector3d predict_trans(predict_pose(0,3), predict_pose(1,3), predict_pose(2,3));
    q_w_curr_map = Eigen::Quaterniond(predict_rot_mat);
    t_w_curr_map = predict_trans;

    q_w_curr_ = q_w_curr_map;
    t_w_curr_ = t_w_curr_map;

    // setCurrentPose(predict_pose);

    for(int iter_count = 0; iter_count < max_iter_times_; iter_count++)
    {
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_paramterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(parameters_, 4, q_paramterization); // rotation
        problem.AddParameterBlock(parameters_ + 4, 3); // translation

        // pcl::transformPointCloud(*input_sharp_cloud, *input_sharp_cloud, predict_pose);
        // pcl::transformPointCloud(*input_flat_cloud, *input_flat_cloud, predict_pose);
        // TicToc t_data;

        CloudData::POINT point_ori, point_sel;
        std::vector<int> point_search_ind;
        std::vector<float> point_search_dist; 

        int corner_count = 0;

        int laser_corner_num = input_sharp_cloud->size();
        for(int i = 0; i < laser_corner_num; i++)
        {
            point_ori = input_sharp_cloud->points[i];
            pointAssociateToMap(&point_ori, &point_sel);
 
            kdtree_corner_map_->nearestKSearch(point_sel, 5, point_search_ind, point_search_dist);

            if(point_search_dist[4] < 1.0)
            {
                std::vector<Eigen::Vector3d> near_corners;
                Eigen::Vector3d center(0, 0, 0);

                for(int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d temp(local_sharp_map_->points[point_search_ind[j]].x,
                                                                local_sharp_map_->points[point_search_ind[j]].y,
                                                                local_sharp_map_->points[point_search_ind[j]].z);
                    center += temp;
                    near_corners.push_back(temp);
                }
                center /= 5.0;

                Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
                for(int j = 0; j < 5; j++)
                {
                    Eigen::Matrix<double, 3, 1> temp_zero_mean = near_corners[j] - center; // 坐标去质心
                    cov_mat = cov_mat + temp_zero_mean * temp_zero_mean.transpose(); // 这5个点的协方差
                }

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);
                // if is indeed line feature
                // note Eigen library sort eigenvalues in increasing order
                // PCA求这五个点的主方向（特征值分解，最大特征值对应的特征向量）
                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2); // 这些edge points的主方向向量
                Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) // 这里相当于判断这些点的分布足够成线状分布
                { 
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                        // 相当于拟合出了这条线的两个端点 （中心点坐标 ± 0.1m * 主方向向量）
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;
                    // P.S. 这里点到直线的costfunction与odometry中的是一样的
                    // 计算点到直线距离作为误差函数，这里没有运动畸变，不需要插值
                    // cout << curr_point.transpose() << endl << point_a.transpose() << endl << point_b.transpose() << endl;
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4);
                    corner_count++;	
                }		
            }
        } // end for sharp points association

        int surf_count = 0;
        int laser_surf_num = input_flat_cloud->size();
        for(int i = 0; i < laser_surf_num; i++)
        {
            point_ori = input_flat_cloud->points[i];
            pointAssociateToMap(&point_ori, &point_sel);
            kdtree_surf_map_->nearestKSearch(point_sel, 5, point_search_ind, point_search_dist);
            
            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

            if(point_search_dist[4] < 1.0)
            {
                for(int j = 0; j < 5; j ++)
                {
                    matA0(j, 0) = local_flat_map_->points[point_search_ind[j]].x;
                    matA0(j, 1) = local_flat_map_->points[point_search_ind[j]].y;
                    matA0(j, 2) = local_flat_map_->points[point_search_ind[j]].z;
                }

                // find the norm of plane 得到这个拟合平面的法向量
                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0); // 平面方程的a,b,c
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();

                // Here n(pa, pb, pc) is unit norm of plane
                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    // if OX * n > 0.2, then plane is not fit well 判断法向量的拟合效果，即拟合原方程ax+by+cz+1=0的效果
                    if (fabs(norm(0) * local_flat_map_->points[point_search_ind[j]].x +
                                norm(1) * local_flat_map_->points[point_search_ind[j]].y +
                                norm(2) * local_flat_map_->points[point_search_ind[j]].z + negative_OA_dot_norm) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }
                Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
                if (planeValid)
                {
                    // P.S. 这里点到平面的距离和Odometry中的是不同的
                    ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4);
                    surf_count++;
                }
            }
        } // end for surf point association
        // printf("mapping data assosiation time %f ms \n", t_data.toc());

        // TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // printf("LOAM registration solver time %f ms \n", t_solver.toc());

        q_w_curr_ = q_w_curr_map;
        t_w_curr_ = t_w_curr_map;
    } // end for iter

    // get transform
    q_w_curr_.normalize();
    Eigen::Matrix3d rot_mat(q_w_curr_);
    result_pose.block<3,3>(0,0) = rot_mat.cast<float>();
    result_pose(0,3) = t_w_curr_(0);
    result_pose(1,3) = t_w_curr_(1);
    result_pose(2,3) = t_w_curr_(2);

    // cout << result_pose.matrix() << endl;

// pcl::transformPointCloud(*input_sharp_cloud, *result_cloud_ptr, predict_pose);

    return true;
} */

bool LeGORegistration::ScanMatch(const CloudData::CLOUD_PTR& input_sharp_cloud, 
                                                                    const CloudData::CLOUD_PTR& input_flat_cloud, 
                                                                    const Eigen::Matrix4f& predict_pose, 
                                                                    Eigen::Matrix4f& result_pose)
{

    Eigen::Map<Eigen::Quaterniond> q_w_curr_map(parameters_);
    Eigen::Map<Eigen::Vector3d> t_w_curr_map(parameters_ + 4);

    Eigen::Matrix3d predict_rot_mat = predict_pose.block<3,3>(0,0).cast<double>();
    Eigen::Vector3d predict_trans(predict_pose(0,3), predict_pose(1,3), predict_pose(2,3));
    q_w_curr_map = Eigen::Quaterniond(predict_rot_mat);
    t_w_curr_map = predict_trans;

    q_w_curr_ = q_w_curr_map;
    t_w_curr_ = t_w_curr_map;

    // setCurrentPose(predict_pose);

    for(int iter_count = 0; iter_count < max_iter_times_; iter_count++)
    {
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_paramterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(parameters_, 4, q_paramterization); // rotation
        problem.AddParameterBlock(parameters_ + 4, 3); // translation

        // pcl::transformPointCloud(*input_sharp_cloud, *input_sharp_cloud, predict_pose);
        // pcl::transformPointCloud(*input_flat_cloud, *input_flat_cloud, predict_pose);
        // TicToc t_data;

        CloudData::POINT point_ori, point_sel;
        std::vector<int> point_search_ind;
        std::vector<float> point_search_dist; 

        int corner_count = 0;

        int laser_corner_num = input_sharp_cloud->size();
        for(int i = 0; i < laser_corner_num; i++)
        {
            point_ori = input_sharp_cloud->points[i];
            pointAssociateToMap(&point_ori, &point_sel);
 
            kdtree_corner_map_->nearestKSearch(point_sel, 5, point_search_ind, point_search_dist);

            if(point_search_dist[4] < 1.0)
            {
                std::vector<Eigen::Vector3d> near_corners;
                Eigen::Vector3d center(0, 0, 0);

                for(int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d temp(local_sharp_map_->points[point_search_ind[j]].x,
                                                                local_sharp_map_->points[point_search_ind[j]].y,
                                                                local_sharp_map_->points[point_search_ind[j]].z);
                    center += temp;
                    near_corners.push_back(temp);
                }
                center /= 5.0;

                Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
                for(int j = 0; j < 5; j++)
                {
                    Eigen::Matrix<double, 3, 1> temp_zero_mean = near_corners[j] - center; // 坐标去质心
                    cov_mat = cov_mat + temp_zero_mean * temp_zero_mean.transpose(); // 这5个点的协方差
                }

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);
                // if is indeed line feature
                // note Eigen library sort eigenvalues in increasing order
                // PCA求这五个点的主方向（特征值分解，最大特征值对应的特征向量）
                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2); // 这些edge points的主方向向量
                Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) // 这里相当于判断这些点的分布足够成线状分布
                { 
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                        // 相当于拟合出了这条线的两个端点 （中心点坐标 ± 0.1m * 主方向向量）
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;
                    // P.S. 这里点到直线的costfunction与odometry中的是一样的
                    // 计算点到直线距离作为误差函数，这里没有运动畸变，不需要插值
                    // cout << curr_point.transpose() << endl << point_a.transpose() << endl << point_b.transpose() << endl;
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4);
                    corner_count++;	
                }		
            }
        } // end for sharp points association

        int surf_count = 0;
        int laser_surf_num = input_flat_cloud->size();
        for(int i = 0; i < laser_surf_num; i++)
        {
            point_ori = input_flat_cloud->points[i];
            pointAssociateToMap(&point_ori, &point_sel);
            kdtree_surf_map_->nearestKSearch(point_sel, 5, point_search_ind, point_search_dist);
            
            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

            if(point_search_dist[4] < 1.0)
            {
                for(int j = 0; j < 5; j ++)
                {
                    matA0(j, 0) = local_flat_map_->points[point_search_ind[j]].x;
                    matA0(j, 1) = local_flat_map_->points[point_search_ind[j]].y;
                    matA0(j, 2) = local_flat_map_->points[point_search_ind[j]].z;
                }

                // find the norm of plane 得到这个拟合平面的法向量
                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0); // 平面方程的a,b,c
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();

                // Here n(pa, pb, pc) is unit norm of plane
                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    // if OX * n > 0.2, then plane is not fit well 判断法向量的拟合效果，即拟合原方程ax+by+cz+1=0的效果
                    if (fabs(norm(0) * local_flat_map_->points[point_search_ind[j]].x +
                                norm(1) * local_flat_map_->points[point_search_ind[j]].y +
                                norm(2) * local_flat_map_->points[point_search_ind[j]].z + negative_OA_dot_norm) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }
                Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
                if (planeValid)
                {
                    // P.S. 这里点到平面的距离和Odometry中的是不同的
                    ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                    problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4);
                    surf_count++;
                }
            }
        } // end for surf point association
        // printf("mapping data assosiation time %f ms \n", t_data.toc());

        // TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // printf("LOAM registration solver time %f ms \n", t_solver.toc());

        q_w_curr_ = q_w_curr_map;
        t_w_curr_ = t_w_curr_map;
    } // end for iter

    // get transform
    q_w_curr_.normalize();
    Eigen::Matrix3d rot_mat(q_w_curr_);
    result_pose.block<3,3>(0,0) = rot_mat.cast<float>();
    result_pose(0,3) = t_w_curr_(0);
    result_pose(1,3) = t_w_curr_(1);
    result_pose(2,3) = t_w_curr_(2);

    // cout << result_pose.matrix() << endl;

// pcl::transformPointCloud(*input_sharp_cloud, *result_cloud_ptr, predict_pose);

    return true;
}

float LeGORegistration::GetFitnessScore() {
    // return ndt_ptr_->getFitnessScore();
    return float(0.0);
}
}