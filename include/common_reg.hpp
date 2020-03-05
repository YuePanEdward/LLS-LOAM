//
// This file is for the common implement of various registration methods
// It would cover the classic fine registration method icp and its variants
// And the feature points based registration methods 
// Dependent 3rd Libs: PCL (>=1.7) , Eigen     
//

#ifndef _INCLUDE_COMMON_REG_H_
#define _INCLUDE_COMMON_REG_H_

#include <cmath>
#include <memory>
#include <glog/logging.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

#include <Eigen/Dense>

#include <ceres/rotation.h>

#include "types.h"
#include "pca.hpp"
#include "map_viewer.hpp"

namespace lls_loam
{

enum point_type
{
    GROUND,
    EDGE,
    PLANAR,
    SPHERE
}; 
enum transform_estimator_type
{
    SVD,
    LM,
    LLS
}; //SVD: Singular Value Decomposition ; LM: Levenberg-Marquat ; LLS: Linear Lest Square
enum correspondence_estimator_type
{
    NN,
    NS
}; //NN: Nearest Neighbor ; NS: Normal Shooting
enum metrics_type
{
    Point2Point,
    Point2Plane,
    Plane2Plane
};

template <typename PointType>
class CRegistration
{
public:
    typedef pcl::PointXYZINormal PointT;

    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcXYZRGBPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB> pcXYZRGB;
    typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;
    typedef pcl::PointCloud<pcl::Normal> Normals;
    typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeaturePtr;
    typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

    // Pairwise registration between frames
    // There are three initial guess options
    // a. gnss pose difference ; b. uniform motion model ; c. imu preintegration
    // return code -- code=1::success; code=-1::Total correspondence number is too small; code=-2::Too little Non-ground features found;
    // code=-3::Too little Ground features found; code=-4::Too large translation or rotation for one iteration
    // Fix it later using try - catch
    //! WARNING: FrameData.noise_gnss.pose will be used as initial guess, make sure it is INIT correctly before you call this function
    int PairwiseReg(FrameData &group1, FrameData &group2, Pose3d &pose, InitialGuessType initial_type) {
        
        int max_iter_num = 12;
        float thre_dis_ground = 0.7f; 
        float thre_dis_edge = 0.65f;   
        float thre_dis_planar = 0.65f; 
        float thre_dis_sphere = 0.55f;  

        // Preprocessing: Initialize Feature Point Cloud (Fix it later by removing pcl dependency)
        pcl::PointCloud<PointT>::Ptr Source_Ground(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Edge(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Planar(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Sphere(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Ground(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Edge(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Planar(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Sphere(new pcl::PointCloud<PointT>);
        
        ExtractFeaturePointsFromFullPointCloud(group2.raw_frame.cld_lidar_ptr, group2.raw_frame.ground_down_index, Source_Ground); 
        ExtractFeaturePointsFromFullPointCloud(group2.raw_frame.cld_lidar_ptr, group2.raw_frame.edge_down_index, Source_Edge); 
        ExtractFeaturePointsFromFullPointCloud(group2.raw_frame.cld_lidar_ptr, group2.raw_frame.planar_down_index, Source_Planar); 
        ExtractFeaturePointsFromFullPointCloud(group2.raw_frame.cld_lidar_ptr, group2.raw_frame.sphere_down_index, Source_Sphere); 
        ExtractFeaturePointsFromFullPointCloud(group1.raw_frame.cld_lidar_ptr, group1.raw_frame.ground_index, Target_Ground); 
        ExtractFeaturePointsFromFullPointCloud(group1.raw_frame.cld_lidar_ptr, group1.raw_frame.edge_index, Target_Edge); 
        ExtractFeaturePointsFromFullPointCloud(group1.raw_frame.cld_lidar_ptr, group1.raw_frame.planar_index, Target_Planar); 
        ExtractFeaturePointsFromFullPointCloud(group1.raw_frame.cld_lidar_ptr, group1.raw_frame.sphere_index, Target_Sphere); 

        //Source point cloud initial guess
        pcl::PointCloud<PointT>::Ptr Source_Ground_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Edge_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Planar_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Sphere_g(new pcl::PointCloud<PointT>);
        
        Eigen::Matrix4d transformation2to1g;

        switch (initial_type)
        {
            case GNSSINSPoseDiff:
            {
                // Provide initial guess by OXTS pose
                // Pose3d pose2to1g(group1.raw_gnss.pose.quat.conjugate() * group2.raw_gnss.pose.quat,
                //                  group1.raw_gnss.pose.quat.conjugate() * (group2.raw_gnss.pose.trans - group1.raw_gnss.pose.trans));

                Pose3d pose2to1g(group1.noise_gnss.pose.quat.conjugate() * group2.noise_gnss.pose.quat,
                group1.noise_gnss.pose.quat.conjugate() * (group2.noise_gnss.pose.trans - group1.noise_gnss.pose.trans));

                transformation2to1g = pose2to1g.GetMatrix();
                LOG(INFO) << "Initial Guess of OXTS \n"
                          << transformation2to1g;
                break;
            }
            case UniformMotion:
            {
                // Provide initial guess by uniform motion model
                transformation2to1g = group1.raw_frame.last_transform.GetMatrix();
                Pose3d pose2to1g;
                pose2to1g.SetPose(transformation2to1g);
                LOG(INFO) << "Initial Guess of Unifrom Motion \n"
                          << transformation2to1g;
                break;
            }
            case IMUPreintegration:
            {
                // Provide initial guess by imu preintegration (TO DO) 
                // InitialGuessByIMUPreintergration();
                break;
            }
            default:
            {
                break;
            }   
        }

        //Apply intial guess
        pcl::transformPointCloudWithNormals(*Source_Ground, *Source_Ground_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Edge, *Source_Edge_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Planar, *Source_Planar_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Sphere, *Source_Sphere_g, transformation2to1g);

        //Do registration
        Eigen::Matrix4f transformation2to1;
        //Eigen::Matrix4f transformation2to1f;
        Eigen::Matrix<float, 6, 6> reg_information_matrix;
        double trans_x, trans_y, trans_z;
        //Feature point efficient registration
        int code = FeaturePointsRegistration(transformation2to1, reg_information_matrix,
                                            Source_Ground_g, Source_Edge_g, Source_Planar_g, Source_Sphere_g,
                                            Target_Ground, Target_Edge, Target_Planar, Target_Sphere,
                                            max_iter_num, thre_dis_ground, thre_dis_edge, thre_dis_planar, thre_dis_sphere, 0);
        if (code == 1) {
            trans_x = transformation2to1(0, 3);
            trans_y = transformation2to1(1, 3);
            trans_z = transformation2to1(2, 3);

            //Get final registration result
            transformation2to1 = transformation2to1.template cast<float>() * transformation2to1g.template cast<float>();
            LOG(INFO) << "Final Registration result:" << std::endl
                      << transformation2to1 << std::endl;
            // LOG(INFO) << "Final Information Matrix:" << std::endl
            //           << reg_information_matrix << std::endl;
            pose.SetPose(transformation2to1.template cast<double>());
            
        } 
        else {
            LOG(ERROR) << "Registration may encounter some problem, use default transformation, pcd is " << group1.raw_frame.pcd_file_name;
        }

#if 0   // For display
        static int i = 0;
        static double trans_thre = 0.25;
        if (code != 1 || trans_x > trans_thre || trans_x < -trans_thre || trans_y > trans_thre || trans_y < -trans_thre || trans_z > trans_thre || trans_z < -trans_thre)
        {
            //transform for display
            std::shared_ptr<PointCloud> frame2_pointcloud_ptr(new PointCloud);
            transformPointCloud(group2.raw_frame.cld_lidar_ptr, frame2_pointcloud_ptr, pose2to1g);

            std::shared_ptr<PointCloud> frame2_pointcloud_after_reg_ptr(new PointCloud);

            Pose3d pose2to1;
            pose2to1.SetPose(transformation2to1.template cast<double>());
            transformPointCloud(group2.raw_frame.cld_lidar_ptr, frame2_pointcloud_after_reg_ptr, pose2to1);

            MapViewer<PointType> viewer;

            // viewer.DispalyFeatureFrameRegistration(frame2_pointcloud_ptr, group2.raw_frame.ground_down_index, group2.raw_frame.edge_down_index,
            //                                           group2.raw_frame.planar_down_index, group2.raw_frame.sphere_down_index,
            //                                           group1.raw_frame.cld_lidar_ptr, group1.raw_frame.ground_index, group1.raw_frame.edge_index,
            //                                           group1.raw_frame.planar_index, group1.raw_frame.sphere_index, "Initial Guess Result - Feature Points");

            // viewer.Dispaly2Clouds(group1.raw_frame.cld_lidar_ptr, frame2_pointcloud_ptr, "Initial Guess Result - All Points", 1);

            viewer.DispalyFeatureFrameRegistration(frame2_pointcloud_after_reg_ptr,
                                                      group2.raw_frame.ground_down_index, group2.raw_frame.edge_down_index,
                                                      group2.raw_frame.planar_down_index, group2.raw_frame.sphere_down_index,
                                                      group1.raw_frame.cld_lidar_ptr,
                                                      group1.raw_frame.ground_index, group1.raw_frame.edge_index,
                                                      group1.raw_frame.planar_index, group1.raw_frame.sphere_index,
                                                      "After registration - Feature Points");

            // viewer.Dispaly2Clouds(group1.raw_frame.cld_lidar_ptr, frame2_pointcloud_after_reg_ptr, "After registration - All Points", 1);

            viewer.Dispaly2CloudsCompare(group1.raw_frame.cld_lidar_ptr, frame2_pointcloud_ptr,
                                          group1.raw_frame.cld_lidar_ptr, frame2_pointcloud_after_reg_ptr,
                                          "Left:before registration, Right:after registration", 1);

            frame2_pointcloud_ptr = std::make_shared<point_cloud_t<PointType>>();
            frame2_pointcloud_after_reg_ptr = std::make_shared<point_cloud_t<PointType>>();
        }
        i++;
#endif
        
        return code;
    }

    // Pairwise registration between frames by initial guess
    bool PairwiseInitialGuess(FrameData &group1, FrameData &group2) {
        Pose3d pose2to1g(group1.noise_gnss.pose.quat.conjugate() * group2.noise_gnss.pose.quat,
                         group1.noise_gnss.pose.quat.conjugate() * (group2.noise_gnss.pose.trans - group1.noise_gnss.pose.trans));
        Eigen::Matrix4d transformation2to1 = pose2to1g.GetMatrix();
        group2.raw_frame.pose.SetPose(group1.raw_frame.pose.GetMatrix() * transformation2to1);
        return 1;
    }
    
    // Scan to local map registration
    bool PairwiseReg(Submap &localmap, FrameData &currentscan, Pose3d &pose) {
        clock_t t1, t2;

        int max_iter_num = 6;
        float thre_dis_ground = 0.5f;
        float thre_dis_edge = 0.5f;
        float thre_dis_planar = 0.5f;
        float thre_dis_sphere = 0.5f;

        //LOG(INFO) << "submap1.cld_lidar_ptr size " << submap1.cld_lidar_ptr->points.size();
        //LOG(INFO) << "submap2.cld_lidar_ptr size " << submap2.cld_lidar_ptr->points.size();

        //Preprocessing: Initialize Feature Point Cloud (Fix it later by removing pcl dependency)
        pcl::PointCloud<PointT>::Ptr Source_Ground(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Edge(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Planar(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Sphere(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Ground(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Edge(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Planar(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Sphere(new pcl::PointCloud<PointT>);

        t1 = clock();
        
        ExtractFeaturePointsFromFullPointCloud(currentscan.raw_frame.cld_lidar_ptr, currentscan.raw_frame.ground_down_index, Source_Ground); 
        ExtractFeaturePointsFromFullPointCloud(currentscan.raw_frame.cld_lidar_ptr, currentscan.raw_frame.edge_down_index, Source_Edge); 
        ExtractFeaturePointsFromFullPointCloud(currentscan.raw_frame.cld_lidar_ptr, currentscan.raw_frame.planar_down_index, Source_Planar); 
        ExtractFeaturePointsFromFullPointCloud(currentscan.raw_frame.cld_lidar_ptr, currentscan.raw_frame.sphere_down_index, Source_Sphere); 
        ExtractFeaturePointsFromFullPointCloud(localmap.cld_feature_ptr, localmap.ground_index, Target_Ground); 
        ExtractFeaturePointsFromFullPointCloud(localmap.cld_feature_ptr, localmap.edge_index, Target_Edge); 
        ExtractFeaturePointsFromFullPointCloud(localmap.cld_feature_ptr, localmap.planar_index, Target_Planar); 
        ExtractFeaturePointsFromFullPointCloud(localmap.cld_feature_ptr, localmap.sphere_index, Target_Sphere); 

        //Source point cloud initial guess
        pcl::PointCloud<PointT>::Ptr Source_Ground_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Edge_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Planar_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Sphere_g(new pcl::PointCloud<PointT>);

        //Get initial guess
        Pose3d pose2to1g;

        //initial guess from submap 2's first frame to submap 1's local frame
        Eigen::Matrix4d transformation2to1g = localmap.pose.GetMatrix().inverse() * currentscan.raw_frame.pose.GetMatrix();

        pose2to1g.SetPose(transformation2to1g);
        LOG(INFO) << "Initial Guess of OXTS\n"
                  << transformation2to1g;

        //Apply intial guess
        pcl::transformPointCloudWithNormals(*Source_Ground, *Source_Ground_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Edge, *Source_Edge_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Planar, *Source_Planar_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Sphere, *Source_Sphere_g, transformation2to1g);

        //Do registration
        Eigen::Matrix4f transformation2to1;
        Eigen::Matrix<float, 6, 6> reg_information_matrix;
        double trans_x, trans_y, trans_z;

        int code = FeaturePointsRegistration(transformation2to1, reg_information_matrix,
                                             Source_Ground_g, Source_Edge_g, Source_Planar_g, Source_Sphere_g,
                                             Target_Ground, Target_Edge, Target_Planar, Target_Sphere,
                                             max_iter_num, thre_dis_ground, thre_dis_edge,
                                             thre_dis_planar, thre_dis_sphere, 1);
        //Feature point efficient registration
        if (code == 1) {
            trans_x = transformation2to1(0, 3);
            trans_y = transformation2to1(1, 3);
            trans_z = transformation2to1(2, 3);
            
            //Get final registration result
            transformation2to1 = transformation2to1.template cast<float>() * transformation2to1g.template cast<float>();
            LOG(INFO) << "Final Registration result:" << std::endl
                      << transformation2to1 << std::endl;

            // LOG(INFO) << "Final Information Matrix:" << std::endl
            //           << reg_information_matrix << std::endl;
            pose.SetPose(transformation2to1.template cast<double>());
        } else  {
            LOG(ERROR) << "Registration may encounter some problem, use default transformation.";
            
            pose.SetPose(transformation2to1g.template cast<double>());
        }

        t2 = clock();
#if 0   
        //For display
        static double trans_thre = 0.4;
        //if (transformation2to1(0, 3)<trans_thre && transformation2to1(1, 3)<trans_thre && transformation2to1(2, 3)<trans_thre)
        if (code < 0 || trans_x > trans_thre || trans_x < -trans_thre || trans_y > trans_thre || trans_y < -trans_thre || trans_z > trans_thre || trans_z < -trans_thre)
        {
            std::shared_ptr<PointCloud> currentscan_transform_cld_ptr(new PointCloud);
            std::shared_ptr<PointCloud> currentscan_transform_cld_ptr_reg(new PointCloud);
            transformPointCloud(currentscan.raw_frame.cld_lidar_ptr, currentscan_transform_cld_ptr, pose2to1g);
            transformPointCloud(currentscan.raw_frame.cld_lidar_ptr, currentscan_transform_cld_ptr_reg, pose);

            MapViewer<PointType> viewer;
            //viewer.Dispaly2Clouds(localmap.cld_feature_ptr, currentscan_transform_cld_ptr, "Initial Guess Result", 1);

            //viewer.Dispaly2Clouds(localmap.cld_feature_ptr, currentscan_transform_cld_ptr_reg, "After registration", 1);

            viewer.Dispaly2CloudsCompare(localmap.cld_lidar_ptr, currentscan_transform_cld_ptr,
                                         localmap.cld_lidar_ptr, currentscan_transform_cld_ptr_reg,
                                         "Left:before registration, Right:after registration", 1);

            currentscan_transform_cld_ptr = std::shared_ptr<PointCloud>();
            currentscan_transform_cld_ptr_reg = std::shared_ptr<PointCloud>();
        }
#endif
        return true;
    }



    // Pairwise registration between submaps
    bool PairwiseReg(Edge &edge, Submap &submap1, Submap &submap2) {
        clock_t t1, t2;

        int max_iter_num = 15;
        float thre_dis_ground = 1.25f;
        float thre_dis_edge = 0.95f;
        float thre_dis_planar = 0.95f;
        float thre_dis_sphere = 0.8f;

        //LOG(INFO) << "submap1.cld_lidar_ptr size " << submap1.cld_lidar_ptr->points.size();
        //LOG(INFO) << "submap2.cld_lidar_ptr size " << submap2.cld_lidar_ptr->points.size();

        //Preprocessing: Initialize Feature Point Cloud (Fix it later by removing pcl dependency)
        pcl::PointCloud<PointT>::Ptr Source_Ground(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Edge(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Planar(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Sphere(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Ground(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Edge(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Planar(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Sphere(new pcl::PointCloud<PointT>);

        t1 = clock();
        
        ExtractFeaturePointsFromFullPointCloud(submap2.cld_feature_ptr, submap2.ground_index, Source_Ground); 
        ExtractFeaturePointsFromFullPointCloud(submap2.cld_feature_ptr, submap2.edge_index, Source_Edge); 
        ExtractFeaturePointsFromFullPointCloud(submap2.cld_feature_ptr, submap2.planar_index, Source_Planar); 
        ExtractFeaturePointsFromFullPointCloud(submap2.cld_feature_ptr, submap2.sphere_index, Source_Sphere); 
        ExtractFeaturePointsFromFullPointCloud(submap1.cld_feature_ptr, submap1.ground_index, Target_Ground); 
        ExtractFeaturePointsFromFullPointCloud(submap1.cld_feature_ptr, submap1.edge_index, Target_Edge); 
        ExtractFeaturePointsFromFullPointCloud(submap1.cld_feature_ptr, submap1.planar_index, Target_Planar); 
        ExtractFeaturePointsFromFullPointCloud(submap1.cld_feature_ptr, submap1.sphere_index, Target_Sphere); 

        int k; //local frame id in submap 1
        //find local frame as the nearest neighbor between two submaps
        if (edge.edge_type == Edge::Adjacent)
            k = submap1.frame_number - 1;
        // Revisit edge
        else {
            int k_temp;
            double dis_temp = DBL_MAX;
            for (int i = 0; i < submap1.frame_number; i++) {
                for (int j = 0; j < submap2.frame_number; j++) {
                    double dis = (submap1.raw_data_group[i].raw_gnss.pose.trans - submap2.raw_data_group[j].raw_gnss.pose.trans).norm();
                    if (dis < dis_temp) {
                        dis_temp = dis;
                        k_temp = i;
                    }
                }
            }
            k = k_temp;
        }
        LOG(INFO) << "the local referenced frame of submap1 for edge registration is " << k;

        Eigen::Matrix4d transformation1to1l = submap1.raw_data_group[k].raw_frame.pose.GetMatrix().inverse() * submap1.pose.GetMatrix();

        //Target point cloud to a close local frame
        pcl::PointCloud<PointT>::Ptr Target_Ground_l(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Edge_l(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Planar_l(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Target_Sphere_l(new pcl::PointCloud<PointT>);

        //Apply local frame transform
        pcl::transformPointCloudWithNormals(*Target_Ground, *Target_Ground_l, transformation1to1l);
        pcl::transformPointCloudWithNormals(*Target_Edge, *Target_Edge_l, transformation1to1l);
        pcl::transformPointCloudWithNormals(*Target_Planar, *Target_Planar_l, transformation1to1l);
        pcl::transformPointCloudWithNormals(*Target_Sphere, *Target_Sphere_l, transformation1to1l);

        //Source point cloud initial guess
        pcl::PointCloud<PointT>::Ptr Source_Ground_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Edge_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Planar_g(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr Source_Sphere_g(new pcl::PointCloud<PointT>);

        //Get initial guess
        Pose3d pose2to1g;

        //initial guess from submap 2's first frame to submap 1's local frame
        Eigen::Matrix4d transformation2to1g = submap1.raw_data_group[k].raw_gnss.pose.GetMatrix().inverse() * submap2.pose.GetMatrix();

        pose2to1g.SetPose(transformation1to1l.inverse() * transformation2to1g);
        LOG(INFO) << "Initial Guess of OXTS\n"
                  << pose2to1g.GetMatrix();

        //Apply intial guess
        pcl::transformPointCloudWithNormals(*Source_Ground, *Source_Ground_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Edge, *Source_Edge_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Planar, *Source_Planar_g, transformation2to1g);
        pcl::transformPointCloudWithNormals(*Source_Sphere, *Source_Sphere_g, transformation2to1g);

        //Do registration
        Eigen::Matrix4f transformation2to1;
        Eigen::Matrix<float, 6, 6> reg_information_matrix;
        double trans_x, trans_y, trans_z;

        int code = FeaturePointsRegistration(transformation2to1, reg_information_matrix,
                                             Source_Ground_g, Source_Edge_g, Source_Planar_g, Source_Sphere_g,
                                             Target_Ground_l, Target_Edge_l, Target_Planar_l, Target_Sphere_l,
                                             max_iter_num, thre_dis_ground, thre_dis_edge,
                                             thre_dis_planar, thre_dis_sphere, 1);
        //Feature point efficient registration
        if (code == 1) {
            trans_x = transformation2to1(0, 3);
            trans_y = transformation2to1(1, 3);
            trans_z = transformation2to1(2, 3);
            edge.pose.SetPose(transformation1to1l.inverse() * (transformation2to1.template cast<double>() * transformation2to1g)); // 2 to 1
            edge.information_matrix = reg_information_matrix.template cast<double>();
        } else  {
            LOG(ERROR) << "Registration may encounter some problem, use default transformation, submap1 is " << submap1.submap_id << "submap2 is " << submap2.submap_id;
            edge.pose.SetPose(transformation1to1l.inverse() * transformation2to1g);
            // Submap Registration Failed, Edge Information Matrix will be set Zero
            edge.information_matrix = Eigen::Matrix<double, 6, 6>::Zero(6, 6);
        }

        t2 = clock();
#if 0
        //For display
        static double trans_thre = 1.4;
        //if (edge.edge_type == Edge::Intra && edge.submap_idx.first_submap.submap_id % 20 == 0)
        if (!successful_registration || trans_x > trans_thre || trans_x < -trans_thre || trans_y > trans_thre || trans_y < -trans_thre || trans_z > trans_thre || trans_z < -trans_thre)
        {
            std::shared_ptr<PointCloud> submap2_transform_cld_ptr(new PointCloud);
            std::shared_ptr<PointCloud> submap2_transform_cld_ptr_reg(new PointCloud);
            transformPointCloud(submap2.cld_feature_ptr, submap2_transform_cld_ptr, pose2to1g);
            transformPointCloud(submap2.cld_feature_ptr, submap2_transform_cld_ptr_reg, edge.pose);

            MapViewer<PointType> viewer;
            //viewer.Dispaly2Clouds(submap1.cld_feature_ptr, submap2_transform_cld_ptr, "Initial Guess Result", 1);

            //viewer.Dispaly2Clouds(submap1.cld_feature_ptr, submap2_transform_cld_ptr, "After registration", 1);

            viewer.Dispaly2CloudsCompare(submap1.cld_feature_ptr, submap2_transform_cld_ptr,
                                          submap1.cld_feature_ptr, submap2_transform_cld_ptr_reg,
                                          "Left:before registration, Right:after registration", 1);

            submap2_transform_cld_ptr = std::shared_ptr<PointCloud>();
            submap2_transform_cld_ptr_reg = std::shared_ptr<PointCloud>();
        }
#endif
        LOG(INFO) << "Total edge registration time: " << (float(t2 - t1) / CLOCKS_PER_SEC * 1000) << "ms";
        LOG(INFO) << "Final Registration result\n"
                  << edge.pose.GetMatrix();
        LOG(INFO) << "Information Matrix\n"
                  << edge.information_matrix;

        return true;
    }

// Test it 
#if 0
    bool InitialGuessByIMUPreintergration(FrameData & last_frame_data, Eigen::Vector3d & initial_velocity, Eigen::Matrix4d & initial_guess)
    {
        //Use fixed time step  //Fix it later
        double dt = 0.01; //unit:s
        double time_step = 0.1;

        // check delta time
        // if (dt <= -0.1 || dt > 2.0) {
        //_imu_state_stable = false;
        // _update_imu_count = 0;

        // delta measurements, position/velocity/rotation(matrix)

        Eigen::Vector3d delta_P; // P_k + 1 = P_k + V_k * dt + R_k * a_k * dt * dt / 2
        Eigen::Vector3d delta_V; // V_k+1 = V_k + R_k * a_k * dt
        Eigen::Matrix3d delta_R; // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

        Eigen::Matrix3d delta_R_cul;

        Eigen::Vector3d last_P;
        Eigen::Vector3d last_V;
        Eigen::Matrix3d last_R;

        delta_P.setZero();
        delta_V.setZero();
        delta_R.setZero();
        delta_R_cul.setIdentity();

        last_P.setZero();
        last_V = initial_velocity; 
        last_R = last_frame_data.raw_gnss.pose.GetMatrix().block<3, 3>(0, 0);

        Eigen::Vector3d acc;
        Eigen::Vector3d omega;
        Eigen::Vector3d bias_acc;
        Eigen::Vector3d bias_omega;

        bias_acc.setZero();
        bias_omega.setZero();

        Eigen::Vector3d gravity(0, 0, -9.8); // fix it
        double dt2;

        //Preintegration
        for (int i = 0; i < last_frame_data.raw_imu.imu_infos.size(); i++)
        {
            //get measurement
            acc(0) = last_frame_data.raw_imu.imu_infos[i].ax;
            acc(1) = last_frame_data.raw_imu.imu_infos[i].ay;
            acc(2) = last_frame_data.raw_imu.imu_infos[i].az;
            omega(0) = last_frame_data.raw_imu.imu_infos[i].wx; // already rad
            omega(1) = last_frame_data.raw_imu.imu_infos[i].wy;
            omega(2) = last_frame_data.raw_imu.imu_infos[i].wz;

            //  acc = axis_rot * acc;
            //  omega = axis_rot * omega;

            // TODO: get bias
            
            dt2 = dt * dt;
            
            double omega_temp[3];
            omega_temp[0]=(omega(0) - bias_omega(0)) * dt;
            omega_temp[1]=(omega(1) - bias_omega(1)) * dt;
            omega_temp[2]=(omega(2) - bias_omega(2)) * dt;
            
            ceres::AngleAxisToRotationMatrix(&omega_temp, &delta_R);

            delta_R = normalizeRotationM(Expmap((omega - bias_omega) * dt));
            
            //Pose3d delta_r_pose;

            delta_R_cul = delta_R_cul * delta_R;

            delta_V = gravity * dt + last_R * (acc - bias_acc) * dt;

            delta_P = last_V * dt + 0.5 * gravity * dt2 + 0.5 * last_R * (acc - bias_acc) * dt2;

            last_V += delta_V;

            last_P += delta_P;

            last_R = last_R * delta_R;
        }

        //last_linear_velocity_ = last_V * time_step; //Fix it

        initial_guess.setIdentity();

        initial_guess.block<3, 3>(0, 0) = delta_R_cul;

        initial_guess.block<3, 1>(0, 3) = last_P;

        // printf("IMU preintergration result for current frame \n %lf %lf % lf %lf \n %lf %lf % lf %lf \n %lf %lf % lf %lf \n",
        // pre_mat(0,0),pre_mat(0,1),pre_mat(0,2),pre_mat(0,3),
        // pre_mat(1,0),pre_mat(1,1),pre_mat(1,2),pre_mat(1,3),
        // pre_mat(2,0),pre_mat(2,1),pre_mat(2,2),pre_mat(2,3));

        return true;
    }
#endif

#if 0 
    //Point cloud Transformation with Normal
    bool TransformPointCloud(std::shared_ptr<PointCloud> point_cloud_ori_ptr,
                             std::shared_ptr<PointCloud> point_cloud_trgt_ptr,
                             const Pose3d &pose) const
    {
        clock_t t0, t1;
        t0 = clock();

        if (!point_cloud_trgt_ptr->points.empty())
            point_cloud_trgt_ptr->points.clear();
        for (size_t i = 0; i < point_cloud_ori_ptr->points.size(); ++i)
        {
            PointType point;
            point.intensity = point_cloud_ori_ptr->points[i].intensity;
            Eigen::Vector3d coord_origin;
            Eigen::Vector3d coord_transform;
            coord_origin << point_cloud_ori_ptr->points[i].x,
                point_cloud_ori_ptr->points[i].y,
                point_cloud_ori_ptr->points[i].z;
            coord_transform = pose.quat * coord_origin + pose.trans;
            point.x = coord_transform[0];
            point.y = coord_transform[1];
            point.z = coord_transform[2];
            //If the point normal is not 0,0,0 (the initialized value), then we transform it
            if (point_cloud_ori_ptr->points[i].nx != 0 ||
                point_cloud_ori_ptr->points[i].ny != 0 ||
                point_cloud_ori_ptr->points[i].nz != 0)
            {
                Eigen::Vector3d normal_origin;
                Eigen::Vector3d normal_transform;
                normal_origin << point_cloud_ori_ptr->points[i].nx,
                    point_cloud_ori_ptr->points[i].ny,
                    point_cloud_ori_ptr->points[i].nz;
                normal_transform = pose.quat * normal_origin;
                normal_transform.normalize();
                point.nx = normal_transform[0];
                point.ny = normal_transform[1];
                point.nz = normal_transform[2];
            }
            point_cloud_trgt_ptr->points.push_back(point);
        }
        t1 = clock();
        LOG(INFO) << "PointCloud Transform Done, " << point_cloud_ori_ptr->points.size() << " points are transformed in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";
    }
#endif

#if 1
    bool transformPointCloud(std::shared_ptr<PointCloud> point_cloud_ori_ptr,
                             pcl::PointCloud<pcl::PointXYZINormal>::Ptr &point_cloud_trgt_ptr,
                             const Pose3d &pose) const {
        if (!point_cloud_trgt_ptr->points.empty()) {
            point_cloud_trgt_ptr->points.clear();
        }
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZINormal>);
        Eigen::Matrix4d trans = pose.GetMatrix();
//        LOG(ERROR) << "Transform is " << trans;
//        LOG(ERROR) << "Transform is " << pose.GetMatrix();

        for (size_t i = 0; i < point_cloud_ori_ptr->points.size(); ++i)
        {
            pcl::PointXYZINormal pt;
            pt.x = point_cloud_ori_ptr->points[i].x;
            pt.y = point_cloud_ori_ptr->points[i].y;
            pt.z = point_cloud_ori_ptr->points[i].z;
            pt.intensity = point_cloud_ori_ptr->points[i].intensity;
            pt.normal_x = point_cloud_ori_ptr->points[i].nx;
            pt.normal_y = point_cloud_ori_ptr->points[i].ny;
            pt.normal_z = point_cloud_ori_ptr->points[i].nz;
            cloud_in->points.push_back(pt);
        }
        pcl::transformPointCloudWithNormals(*cloud_in, *point_cloud_trgt_ptr, trans);
        return true;
    }

    //Point cloud Transformation with Normal  //[With pcl]
    bool transformPointCloud(std::shared_ptr<PointCloud> point_cloud_ori_ptr,
                             std::shared_ptr<PointCloud> point_cloud_trgt_ptr,
                             const Pose3d &pose) const
    {
        clock_t t0, t1, t2, t3;
        t0 = clock();
        
        if (!point_cloud_trgt_ptr->points.empty())
        {
            point_cloud_trgt_ptr->points.clear();
        }

        if (pose.GetMatrix()==Eigen::Matrix4d::Identity(4,4)) {
             point_cloud_trgt_ptr=point_cloud_ori_ptr;
             return 1;
        }
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZINormal>);
        transformPointCloud(point_cloud_ori_ptr, cloud_out, pose);

        for (size_t i = 0; i < cloud_out->points.size(); ++i)
        {
            pointXYZINormal_t pt;
            pt.x = cloud_out->points[i].x;
            pt.y = cloud_out->points[i].y;
            pt.z = cloud_out->points[i].z;
            pt.intensity = cloud_out->points[i].intensity;
            pt.nx = cloud_out->points[i].normal_x;
            pt.ny = cloud_out->points[i].normal_y;
            pt.nz = cloud_out->points[i].normal_z;
            point_cloud_trgt_ptr->points.push_back(pt);
        }

        //LOG(INFO) << "PointCloud Transform Done, " << point_cloud_ori_ptr->points.size() << " points are transformed in " << (float(t3 - t0) / CLOCKS_PER_SEC * 1000) << "ms";
        //LOG(INFO) << "pcl transform time: " << (float(t2 - t1) / CLOCKS_PER_SEC * 1000) << "ms";
    }
#endif

    // Feature points multi-metric ICP Estimated by LLS (Entrance Function)
    // tag : edge_tag = 1 for registration between submaps , edge_tag = 0 for registration between frames
    // return code -- code=1::success; code=-1::Total correspondence number is too small; code=-2::Too little Non-ground features found;
    // code=-3::Too little Ground features found; code=-4::Too large translation or rotation for one iteration
    // Fix it later using try - catch
    int FeaturePointsRegistration(Eigen::Matrix4f &transformationS2T,
                                  Eigen::Matrix<float, 6, 6> &information_matrix,
                                  pcl::PointCloud<PointT>::Ptr &Source_Ground, pcl::PointCloud<PointT>::Ptr &Source_Edge,
                                  pcl::PointCloud<PointT>::Ptr &Source_Planar, pcl::PointCloud<PointT>::Ptr &Source_Sphere,
                                  pcl::PointCloud<PointT>::Ptr &Target_Ground, pcl::PointCloud<PointT>::Ptr &Target_Edge,
                                  pcl::PointCloud<PointT>::Ptr &Target_Planar, pcl::PointCloud<PointT>::Ptr &Target_Sphere,
                                  int max_iter_num, float dis_thre_ground, float dis_thre_edge, float dis_thre_planar, float dis_thre_sphere,
                                  bool edge_tag) const {

        clock_t t0, t1, t2, t3, t4;
        int code = 1;

        //Parameters
        int K_min = 20;          //Feature points min number
        int min_cor_number = 50; //correspondences' min number

        float max_bearable_translation, max_bearable_rotation;
        if (edge_tag)
        {
            max_bearable_translation = 1.35;
            max_bearable_rotation = 12.0 / 180.0 * M_PI;
        }
        else
        {
            max_bearable_translation = 0.6;
            max_bearable_rotation = 4.0 / 180.0 * M_PI;
        }

        float converge_translation = 0.0002;
        float converge_rotation = 0.01 / 180.0 * M_PI;

        int ground_unground_ratio = 20;

        bool successful = true;

        float accu_time_1 = 0;
        float accu_time_2 = 0;
        float accu_time_3 = 0;

        t0 = clock();

        transformationS2T = Eigen::Matrix4f::Identity(4, 4);

        //Iteration
        for (int i = 0; i < max_iter_num; i++)
        {
            t1 = clock();
            //LOG(INFO) << "--------------- < Registration Iteration " << i << " > ---------------";
            //Estimate Correspondence
            boost::shared_ptr<pcl::Correspondences> corrs_Ground(new pcl::Correspondences);
            boost::shared_ptr<pcl::Correspondences> corrs_Edge(new pcl::Correspondences);
            boost::shared_ptr<pcl::Correspondences> corrs_Planar(new pcl::Correspondences);
            boost::shared_ptr<pcl::Correspondences> corrs_Sphere(new pcl::Correspondences);
            boost::shared_ptr<pcl::Correspondences> corrs_Ground_f(new pcl::Correspondences);
            boost::shared_ptr<pcl::Correspondences> corrs_Edge_f(new pcl::Correspondences);
            boost::shared_ptr<pcl::Correspondences> corrs_Planar_f(new pcl::Correspondences);
            boost::shared_ptr<pcl::Correspondences> corrs_Sphere_f(new pcl::Correspondences);

            typename pcl::registration::CorrespondenceEstimation<PointT, PointT> corr_est;
            pcl::registration::CorrespondenceRejectorDistance corr_rej_dist;

            //For Ground points
            if (Source_Ground->points.size() >= K_min && Target_Ground->points.size() >= K_min)
            {
                corr_est.setInputCloud(Source_Ground);
                corr_est.setInputTarget(Target_Ground);
                corr_est.determineCorrespondences(*corrs_Ground);
                corr_rej_dist.setInputCorrespondences(corrs_Ground);
                corr_rej_dist.setMaximumDistance(dis_thre_ground);
                corr_rej_dist.getCorrespondences(*corrs_Ground_f);
            }

            //For Edge points
            if (Source_Edge->points.size() >= K_min && Target_Edge->points.size() >= K_min)
            {
                corr_est.setInputCloud(Source_Edge);
                corr_est.setInputTarget(Target_Edge);
                corr_est.determineCorrespondences(*corrs_Edge);
                corr_rej_dist.setInputCorrespondences(corrs_Edge);
                corr_rej_dist.setMaximumDistance(dis_thre_edge);
                corr_rej_dist.getCorrespondences(*corrs_Edge_f);
            }

            //For Planar points
            if (Source_Planar->points.size() >= K_min && Target_Planar->points.size() >= K_min)
            {
                corr_est.setInputCloud(Source_Planar);
                corr_est.setInputTarget(Target_Planar);
                corr_est.determineCorrespondences(*corrs_Planar);
                corr_rej_dist.setInputCorrespondences(corrs_Planar);
                corr_rej_dist.setMaximumDistance(dis_thre_planar);
                corr_rej_dist.getCorrespondences(*corrs_Planar_f);
            }

            //For Sphere points
            if (Source_Sphere->points.size() >= K_min && Target_Sphere->points.size() >= K_min)
            {
                corr_est.setInputCloud(Source_Sphere);
                corr_est.setInputTarget(Target_Sphere);
                corr_est.determineCorrespondences(*corrs_Sphere);
                corr_rej_dist.setInputCorrespondences(corrs_Sphere);
                corr_rej_dist.setMaximumDistance(dis_thre_sphere);
                corr_rej_dist.getCorrespondences(*corrs_Sphere_f);
            }

            dis_thre_ground = max_(dis_thre_ground / 1.1, 0.4);
            dis_thre_edge = max_(dis_thre_edge / 1.1, 0.3);
            dis_thre_planar = max_(dis_thre_planar / 1.1, 0.3);
            dis_thre_sphere = max_(dis_thre_sphere / 1.1, 0.3);

            if (i == 0)
            {
                LOG(INFO) << "Found correspondence # [G:" << (*corrs_Ground).size() << " E:" << (*corrs_Edge).size() << " P:" << (*corrs_Planar).size() << " S:" << (*corrs_Sphere).size() << "]";
            }

            LOG(INFO) << "Iter " << i << " - Correspondence after filtering # [G:" << (*corrs_Ground_f).size() << " E:" << (*corrs_Edge_f).size()
                      << " P:" << (*corrs_Planar_f).size() << " S:" << (*corrs_Sphere_f).size() << "]";

            if ((*corrs_Ground_f).size() < min_cor_number && (*corrs_Edge_f).size() < min_cor_number && (*corrs_Planar_f).size() < min_cor_number && (*corrs_Sphere_f).size() < min_cor_number)
            {
                code = -1;
                LOG(WARNING) << "Total correspondence number is too small";
                break;
            }

            if ((*corrs_Ground_f).size() > ground_unground_ratio * ((*corrs_Edge_f).size() + (*corrs_Planar_f).size() + (*corrs_Sphere_f).size()))
            {
                code = -2;
                LOG(WARNING) << "Too little Non-ground features found.";
                break;
            }

            if (ground_unground_ratio * (*corrs_Ground_f).size() < ((*corrs_Edge_f).size() + (*corrs_Planar_f).size() + (*corrs_Sphere_f).size()))
            {
                code = -3;
                LOG(WARNING) << "Too little Ground features found.";
                break;
            }

            t2 = clock();
            accu_time_1 += (float(t2 - t1) / CLOCKS_PER_SEC);

            //Estimate Transformation
            Eigen::Matrix4f TempTran;
            Eigen::Matrix<float, 6, 1> xlist;
            MultiMetricsLLSEstimation(Source_Ground, Target_Ground, corrs_Ground_f,
                                      Source_Edge, Target_Edge, corrs_Edge_f,
                                      Source_Planar, Target_Planar, corrs_Planar_f,
                                      Source_Sphere, Target_Sphere, corrs_Sphere_f,
                                      xlist, information_matrix);

            if (xlist(0) > max_bearable_translation || xlist(1) > max_bearable_translation || xlist(2) > max_bearable_translation ||
                xlist(3) > max_bearable_rotation || xlist(4) > max_bearable_rotation || xlist(5) > max_bearable_rotation)
            {
                code = -4;
                LOG(WARNING) << "Too large translation or rotation for one iteration";
                break;
            }

            //Judge converged or not
            if (i > 2 && xlist(0) < converge_translation && xlist(1) < converge_translation && xlist(2) < converge_translation &&
                xlist(3) < converge_rotation && xlist(4) < converge_rotation && xlist(5) < converge_rotation)
            {
                code = 1;
                LOG(INFO) << "Converged. Break out.";
                double sigma2;
                if (edge_tag == 1) //registration between submaps , update the information matrix by time (1/sigma^2)
                {
                    MultiMetricsLLSCalculateResidual(Source_Ground, Target_Ground, corrs_Ground_f,
                                                     Source_Edge, Target_Edge, corrs_Edge_f,
                                                     Source_Planar, Target_Planar, corrs_Planar_f,
                                                     Source_Sphere, Target_Sphere, corrs_Sphere_f,
                                                     xlist, sigma2);
                    information_matrix = (1.0 / sigma2) * information_matrix;
                }

                constructTransformation(xlist(0), xlist(1), xlist(2), xlist(3), xlist(4), xlist(5), TempTran);
                transformationS2T = TempTran * transformationS2T;

                break; //OUT
            }

            if (edge_tag == 1 && i == max_iter_num - 1) //registration between submaps , update the information matrix by time (1/sigma^2)
            {
                double sigma2;
                MultiMetricsLLSCalculateResidual(Source_Ground, Target_Ground, corrs_Ground_f,
                                                 Source_Edge, Target_Edge, corrs_Edge_f,
                                                 Source_Planar, Target_Planar, corrs_Planar_f,
                                                 Source_Sphere, Target_Sphere, corrs_Sphere_f,
                                                 xlist, sigma2);
                information_matrix = (1.0 / sigma2) * information_matrix;
            }

            constructTransformation(xlist(0), xlist(1), xlist(2), xlist(3), xlist(4), xlist(5), TempTran);

            t3 = clock();
            accu_time_2 += (float(t3 - t2) / CLOCKS_PER_SEC);

            //Update the Source pointcloud
            if (i < max_iter_num - 1)
            {
                pcl::transformPointCloudWithNormals(*Source_Ground, *Source_Ground, TempTran);
                pcl::transformPointCloudWithNormals(*Source_Edge, *Source_Edge, TempTran);
                pcl::transformPointCloudWithNormals(*Source_Planar, *Source_Planar, TempTran);
                pcl::transformPointCloudWithNormals(*Source_Sphere, *Source_Sphere, TempTran);
            }

            transformationS2T = TempTran * transformationS2T;

            t4 = clock();
            accu_time_3 += (float(t4 - t3) / CLOCKS_PER_SEC);
        }

        LOG(INFO) << "SCloud point [G:" << Source_Ground->points.size() << " E:" << Source_Edge->points.size() << " P:" << Source_Planar->points.size() << " S:" << Source_Sphere->points.size() << "] "
                  << "TCloud point [G:" << Target_Ground->points.size() << " E:" << Target_Edge->points.size() << " P:" << Target_Planar->points.size() << " S:" << Target_Sphere->points.size() << "]\n";
        LOG(INFO) << "Find Correspondence in " << accu_time_1 * 1000.0 << "ms, "
                  << "Alignment in " << accu_time_2 * 1000.0 << "ms, "
                  << "Update in " << accu_time_3 * 1000.0 << "ms.";
        LOG(INFO) << "Registration Result\n"
                  << transformationS2T;

        return code;
    }
    

    // Using Linear Least Square Optimization to estimate the transformation using multiple metrics for different kind of feature point correspondence
    // When the rotation angle is small, sin(alpha) ~ alpha, so we can do the linearization
    // We use point-to-plane metrics for ground and facade planar correspondences
    // We use point-to-line metrics for edge correspondecnes
    // We use point-to-point metrics for sphere correspondences
    // For function Ax=b , [ x= tx,ty,tz,alpha,beta,gamma ], we can slove x^ = (ATA)^(-1)*(ATb)
    bool MultiMetricsLLSEstimation(const typename pcl::PointCloud<PointT>::Ptr &Source_Ground, const typename pcl::PointCloud<PointT>::Ptr &Target_Ground,
                                      boost::shared_ptr<pcl::Correspondences> &Corr_Ground, const typename pcl::PointCloud<PointT>::Ptr &Source_Edge,
                                      const typename pcl::PointCloud<PointT>::Ptr &Target_Edge, boost::shared_ptr<pcl::Correspondences> &Corr_Edge,
                                      const typename pcl::PointCloud<PointT>::Ptr &Source_Planar, const typename pcl::PointCloud<PointT>::Ptr &Target_Planar,
                                      boost::shared_ptr<pcl::Correspondences> &Corr_Planar, const typename pcl::PointCloud<PointT>::Ptr &Source_Sphere,
                                      const typename pcl::PointCloud<PointT>::Ptr &Target_Sphere, boost::shared_ptr<pcl::Correspondences> &Corr_Sphere,
                                      Eigen::Matrix<float, 6, 1> &x, Eigen::Matrix<float, 6, 6> &information_matrix) const
    {
        typedef Eigen::Matrix<float, 6, 1> Vector6f;
        typedef Eigen::Matrix<float, 6, 6> Matrix6f;

        Matrix6f ATA;
        Vector6f ATb;
        ATA.setZero();
        ATb.setZero();

#if 1
        float w_ground, w_planar, w_edge_x, w_edge_y, w_edge_z, w_sphere;

        float balance_ratio = 1.0 * (*Corr_Ground).size() / ((*Corr_Planar).size() + (*Corr_Edge).size() + (*Corr_Sphere).size());
        int Corr_total_num = (*Corr_Ground).size() + (*Corr_Planar).size() + (*Corr_Edge).size() + (*Corr_Sphere).size();

        w_ground = 3.0 / balance_ratio;

        //LOG(INFO) << "Begin accumulation";
        //Ground to Ground (Plane to Plane)
        for (int i = 0; i < (*Corr_Ground).size(); i++)
        {
            int s_index, t_index;
            s_index = (*Corr_Ground)[i].index_query;
            t_index = (*Corr_Ground)[i].index_match;
            if (t_index != -1)
            {
                float px = Source_Ground->points[s_index].x;
                float py = Source_Ground->points[s_index].y;
                float pz = Source_Ground->points[s_index].z;
                float qx = Target_Ground->points[t_index].x;
                float qy = Target_Ground->points[t_index].y;
                float qz = Target_Ground->points[t_index].z;
                float ntx = Target_Ground->points[t_index].normal_x;
                float nty = Target_Ground->points[t_index].normal_y;
                float ntz = Target_Ground->points[t_index].normal_z;
                float w = w_ground;

                float a = ntz * py - nty * pz;
                float b = ntx * pz - ntz * px;
                float c = nty * px - ntx * py;

                //    0  1  2  3  4  5
                //    6  7  8  9 10 11
                //   12 13 14 15 16 17
                //   18 19 20 21 22 23
                //   24 25 26 27 28 29
                //   30 31 32 33 34 35

                ATA.coeffRef(0) += w * ntx * ntx;
                ATA.coeffRef(1) += w * ntx * nty;
                ATA.coeffRef(2) += w * ntx * ntz;
                ATA.coeffRef(3) += w * a * ntx;
                ATA.coeffRef(4) += w * b * ntx;
                ATA.coeffRef(5) += w * c * ntx;
                ATA.coeffRef(7) += w * nty * nty;
                ATA.coeffRef(8) += w * nty * ntz;
                ATA.coeffRef(9) += w * a * nty;
                ATA.coeffRef(10) += w * b * nty;
                ATA.coeffRef(11) += w * c * nty;
                ATA.coeffRef(14) += w * ntz * ntz;
                ATA.coeffRef(15) += w * a * ntz;
                ATA.coeffRef(16) += w * b * ntz;
                ATA.coeffRef(17) += w * c * ntz;
                ATA.coeffRef(21) += w * a * a;
                ATA.coeffRef(22) += w * a * b;
                ATA.coeffRef(23) += w * a * c;
                ATA.coeffRef(28) += w * b * b;
                ATA.coeffRef(29) += w * b * c;
                ATA.coeffRef(35) += w * c * c;

                float d = ntx * qx + nty * qy + ntz * qz - ntx * px - nty * py - ntz * pz;

                ATb.coeffRef(0) += w * d * ntx;
                ATb.coeffRef(1) += w * d * nty;
                ATb.coeffRef(2) += w * d * ntz;
                ATb.coeffRef(3) += w * d * a;
                ATb.coeffRef(4) += w * d * b;
                ATb.coeffRef(5) += w * d * c;
            }
        }
        //LOG(INFO) << "Ground OK";

        //Planar to Planar (Plane to Plane) Facade
        for (int i = 0; i < (*Corr_Planar).size(); i++)
        {
            int s_index, t_index;
            s_index = (*Corr_Planar)[i].index_query;
            t_index = (*Corr_Planar)[i].index_match;

            if (t_index != -1)
            {
                float px = Source_Planar->points[s_index].x;
                float py = Source_Planar->points[s_index].y;
                float pz = Source_Planar->points[s_index].z;
                float qx = Target_Planar->points[t_index].x;
                float qy = Target_Planar->points[t_index].y;
                float qz = Target_Planar->points[t_index].z;
                float ntx = Target_Planar->points[t_index].normal_x;
                float nty = Target_Planar->points[t_index].normal_y;
                float ntz = Target_Planar->points[t_index].normal_z;

                // if (i % 100 == 0)
                //     LOG(INFO) << "Normal (x,y,z): " << ntx << "," << nty << "," << ntz;

                if (Corr_total_num < 15000)
                    w_planar = 1.0 + 1.0 * min_(sqrt(qx * qx + qy * qy) / 20.0, 2.0); //distance weighted  //1.0 ~ 3.0  [Frame to frame reg]
                else
                    w_planar = 2.0;
                float w = w_planar;

                float a = ntz * py - nty * pz;
                float b = ntx * pz - ntz * px;
                float c = nty * px - ntx * py;

                //    0  1  2  3  4  5
                //    6  7  8  9 10 11
                //   12 13 14 15 16 17
                //   18 19 20 21 22 23
                //   24 25 26 27 28 29
                //   30 31 32 33 34 35

                ATA.coeffRef(0) += w * ntx * ntx;
                ATA.coeffRef(1) += w * ntx * nty;
                ATA.coeffRef(2) += w * ntx * ntz;
                ATA.coeffRef(3) += w * a * ntx;
                ATA.coeffRef(4) += w * b * ntx;
                ATA.coeffRef(5) += w * c * ntx;
                ATA.coeffRef(7) += w * nty * nty;
                ATA.coeffRef(8) += w * nty * ntz;
                ATA.coeffRef(9) += w * a * nty;
                ATA.coeffRef(10) += w * b * nty;
                ATA.coeffRef(11) += w * c * nty;
                ATA.coeffRef(14) += w * ntz * ntz;
                ATA.coeffRef(15) += w * a * ntz;
                ATA.coeffRef(16) += w * b * ntz;
                ATA.coeffRef(17) += w * c * ntz;
                ATA.coeffRef(21) += w * a * a;
                ATA.coeffRef(22) += w * a * b;
                ATA.coeffRef(23) += w * a * c;
                ATA.coeffRef(28) += w * b * b;
                ATA.coeffRef(29) += w * b * c;
                ATA.coeffRef(35) += w * c * c;

                float d = ntx * qx + nty * qy + ntz * qz - ntx * px - nty * py - ntz * pz;

                ATb.coeffRef(0) += w * d * ntx;
                ATb.coeffRef(1) += w * d * nty;
                ATb.coeffRef(2) += w * d * ntz;
                ATb.coeffRef(3) += w * d * a;
                ATb.coeffRef(4) += w * d * b;
                ATb.coeffRef(5) += w * d * c;
            }
        }
        //LOG(INFO) << "Planar OK";

        //Edge to Edge (Line to Line)
        for (int i = 0; i < (*Corr_Edge).size(); i++)
        {
            int s_index, t_index;
            s_index = (*Corr_Edge)[i].index_query;
            t_index = (*Corr_Edge)[i].index_match;

            if (t_index != -1)
            {
                float px = Source_Edge->points[s_index].x;
                float py = Source_Edge->points[s_index].y;
                float pz = Source_Edge->points[s_index].z;
                float qx = Target_Edge->points[t_index].x;
                float qy = Target_Edge->points[t_index].y;
                float qz = Target_Edge->points[t_index].z;
                float ntx = Target_Edge->points[t_index].normal_x;
                float nty = Target_Edge->points[t_index].normal_y;
                float ntz = Target_Edge->points[t_index].normal_z;
                float nsx = Source_Edge->points[s_index].normal_x;
                float nsy = Source_Edge->points[s_index].normal_y;
                float nsz = Source_Edge->points[s_index].normal_z;

                // if (i % 100 == 0)
                // {
                //     LOG(INFO) << "Normal T (x,y,z): " << ntx << "," << nty << "," << ntz;
                //     LOG(INFO) << "Normal S (x,y,z): " << nsx << "," << nsy << "," << nsz;
                // }

                float wx, wy, wz;

                if (Corr_total_num < 15000)
                    wx = 0.5 + 0.5 * min_(sqrt(qx * qx + qy * qy) / 40.0, 1.0); //1.5 - 3  [Frame to Frame Reg, Distance Weighted]
                else
                    wx = 0.8;

                wy = wx;
                wz = 0.5 * wx;

                float dx = px - qx;
                float dy = py - qy;
                float dz = pz - qz;

                float nx = nty * nsz - ntz * nsy;
                float ny = ntz * nsx - ntx * nsz;
                float nz = ntx * nsy - nty * nsx;
                float nd = sqrt(nx * nx + ny * ny + nz * nz);
                nx /= nd;
                ny /= nd;
                nz /= nd;

                // if (nz >0.7)
                // {
                // 	wx=1.25;
                // 	wy=1.25;
                // 	wz=0.25;
                // }
                // else if (nz<0.25)
                // {
                // 	wx=0.5;
                // 	wy=0.5;
                // 	wz=1.25;
                // }
                // else
                // {
                // 	wx=0.8;
                // 	wy=0.8;
                // 	wz=0.8;
                // }

                double nxy = nx * ny;
                double nxz = nx * nz;
                double nyz = ny * nz;
                double nx2 = nx * nx;
                double ny2 = ny * ny;
                double nz2 = nz * nz;
                double px2 = px * px;
                double py2 = py * py;
                double pz2 = pz * pz;
                double pxy = px * py;
                double pxz = px * pz;
                double pyz = py * pz;

                //    0  1  2  3  4  5
                //    6  7  8  9 10 11
                //   12 13 14 15 16 17
                //   18 19 20 21 22 23
                //   24 25 26 27 28 29
                //   30 31 32 33 34 35

                ATA.coeffRef(0) += (wy * nz2 + wz * ny2);
                ATA.coeffRef(1) += (-wz * nxy);
                ATA.coeffRef(2) += (-wy * nxz);
                ATA.coeffRef(3) += (-wy * nxz * py + wz * nxy * pz);
                ATA.coeffRef(4) += (wy * nxz * px + wy * nz2 * pz + wz * ny2 * pz);
                ATA.coeffRef(5) += (-wy * nz2 * py - wz * ny2 * py + wz * nxy * px);
                ATA.coeffRef(7) += (wx * nz2 + wz * nx2);
                ATA.coeffRef(8) += (-wx * nyz);
                ATA.coeffRef(9) += (-wx * nz2 * pz - wx * nyz * py - wz * nz2 * pz);
                ATA.coeffRef(10) += (wx * nyz * px - wz * nxy * pz);
                ATA.coeffRef(11) += (wx * nz2 * px + wz * nxy * py + wz * nx2 * px);
                ATA.coeffRef(14) += (wx * ny2 + wy * nx2);
                ATA.coeffRef(15) += (wx * nyz * pz + wx * ny2 * py + wy * nx2 * py);
                ATA.coeffRef(16) += (-wx * ny2 * px - wy * nx2 * px - wy * nxz * pz);
                ATA.coeffRef(17) += (-wx * nyz * px + wy * nxz * py);
                ATA.coeffRef(21) += (wx * (nz2 * pz2 + ny2 * py2 + 2 * nyz * pyz) + wy * nx2 * py2 + wz * nx2 * pz2);
                ATA.coeffRef(22) += (-wx * (nyz * pxz + ny2 * pxy) - wy * (nx2 * pxy + nxz * pyz) + wz * nxy * pz2);
                ATA.coeffRef(23) += (-wx * (nz2 * pxz + nyz * pxy) + wy * nxz * py2 - wz * (nxy * pyz + nx2 * pxz));
                ATA.coeffRef(28) += (wx * ny2 * px2 + wy * (nx2 * px2 + nz2 * pz2 + 2 * nxz * pxz) + wz * ny2 * pz2);
                ATA.coeffRef(29) += (wx * nyz * px2 - wy * (nxz * pxy + nz2 * pyz) - wz * (ny2 * pyz + nxy * pxz));
                ATA.coeffRef(35) += (wx * nz2 * px2 + wy * nz2 * py2 + wz * (ny2 * py2 + nx2 * px2 + 2 * nxy * pxy));

                ATb.coeffRef(0) += (wy * (nxz * dz - nz2 * dx) + wz * (-ny2 * dx + nxy * dy));
                ATb.coeffRef(1) += (wx * (-nz2 * dy + nyz * dz) + wz * (nxy * dx - nx2 * dy));
                ATb.coeffRef(2) += (wx * (nyz * dy - ny2 * dz) + wy * (-nx2 * dz + nxz * dx));
                ATb.coeffRef(3) += (wx * (nz * pz * ny * py) * (nz * dy - ny * dz) + wy * nx * py * (-nx * dz + nz * dx) + wz * nx * pz * (-ny * dx + nx * dy));
                ATb.coeffRef(4) += (wx * ny * px * (-nz * dy + ny * dz) + wy * (nx * px + nz * pz) * (nx * dz - nz * dx) + wz * ny * pz * (-ny * dx + nx * dy));
                ATb.coeffRef(5) += (wx * nz * px * (-nz * dy + ny * dz) + wy * nz * py * (-nx * dz + nz * dx) + wz * (ny * py + nx * px) * (ny * dx - nx * dy));
            }
        }
        //LOG(INFO) << "Edge OK";

        //Sphere to Sphere (Point to Point)
        for (int i = 0; i < (*Corr_Sphere).size(); i++)
        {
            int s_index, t_index;
            s_index = (*Corr_Sphere)[i].index_query;
            t_index = (*Corr_Sphere)[i].index_match;

            if (t_index != -1)
            {
                float px = Source_Sphere->points[s_index].x;
                float py = Source_Sphere->points[s_index].y;
                float pz = Source_Sphere->points[s_index].z;
                float qx = Target_Sphere->points[t_index].x;
                float qy = Target_Sphere->points[t_index].y;
                float qz = Target_Sphere->points[t_index].z;

                if (Corr_total_num < 15000)
                    w_sphere = 0.5 + 0.5 * min_(sqrt(qx * qx + qy * qy) / 40.0, 1.0); //1.5 - 3  [Frame to Frame Reg, Distance Weighted]
                else
                    w_sphere = 0.8;

                float wx = w_sphere;
                float wy = w_sphere;
                float wz = w_sphere;

                float dx = px - qx;
                float dy = py - qy;
                float dz = pz - qz;
           
                //    0  1  2  3  4  5
                //    6  7  8  9 10 11
                //   12 13 14 15 16 17
                //   18 19 20 21 22 23
                //   24 25 26 27 28 29
                //   30 31 32 33 34 35

                ATA.coeffRef(0) += wx;
                ATA.coeffRef(1) += 0;
                ATA.coeffRef(2) += 0;
                ATA.coeffRef(3) += 0;
                ATA.coeffRef(4) += wx * pz;
                ATA.coeffRef(5) += (-wx * py);
                ATA.coeffRef(7) += wy;
                ATA.coeffRef(8) += 0;
                ATA.coeffRef(9) += (-wy * pz);
                ATA.coeffRef(10) += 0;
                ATA.coeffRef(11) += wy * px;
                ATA.coeffRef(14) += wz;
                ATA.coeffRef(15) += wz * py;
                ATA.coeffRef(16) += (-wz * px);
                ATA.coeffRef(17) += 0;
                ATA.coeffRef(21) += wy * pz * pz + wz * py * py;
                ATA.coeffRef(22) += (-wz * px * py);
                ATA.coeffRef(23) += (-wy * px * pz);
                ATA.coeffRef(28) += wx * pz * pz + wz * px * px;
                ATA.coeffRef(29) += (-wx * py * pz);
                ATA.coeffRef(35) += wx * py * py + wy * px * px;

                ATb.coeffRef(0) += (-wx * dx);
                ATb.coeffRef(1) += (-wy * dy);
                ATb.coeffRef(2) += (-wz * dz);
                ATb.coeffRef(3) += wy * pz * dy - wz * py * dz;
                ATb.coeffRef(4) += wz * px * dz - wx * pz * dx;
                ATb.coeffRef(5) += wx * py * dx - wy * px * dy;
            }
        }
        //LOG(INFO) << "Sphere OK";

        ATA.coeffRef(6) = ATA.coeff(1);
        ATA.coeffRef(12) = ATA.coeff(2);
        ATA.coeffRef(13) = ATA.coeff(8);
        ATA.coeffRef(18) = ATA.coeff(3);
        ATA.coeffRef(19) = ATA.coeff(9);
        ATA.coeffRef(20) = ATA.coeff(15);
        ATA.coeffRef(24) = ATA.coeff(4);
        ATA.coeffRef(25) = ATA.coeff(10);
        ATA.coeffRef(26) = ATA.coeff(16);
        ATA.coeffRef(27) = ATA.coeff(22);
        ATA.coeffRef(30) = ATA.coeff(5);
        ATA.coeffRef(31) = ATA.coeff(11);
        ATA.coeffRef(32) = ATA.coeff(17);
        ATA.coeffRef(33) = ATA.coeff(23);
        ATA.coeffRef(34) = ATA.coeff(29);

        // Solve A*x = b  x= (ATA)^(-1)ATb
        // x: tx ty tz alpha beta gamma
        x = static_cast<Vector6f>(ATA.inverse() * ATb);

        information_matrix = 0.0001 * ATA;

#endif

        return 1;
    }
    

    bool MultiMetricsLLSCalculateResidual(const typename pcl::PointCloud<PointT>::Ptr &Source_Ground, const typename pcl::PointCloud<PointT>::Ptr &Target_Ground,
                                          boost::shared_ptr<pcl::Correspondences> &Corr_Ground, const typename pcl::PointCloud<PointT>::Ptr &Source_Edge,
                                          const typename pcl::PointCloud<PointT>::Ptr &Target_Edge, boost::shared_ptr<pcl::Correspondences> &Corr_Edge,
                                          const typename pcl::PointCloud<PointT>::Ptr &Source_Planar, const typename pcl::PointCloud<PointT>::Ptr &Target_Planar,
                                          boost::shared_ptr<pcl::Correspondences> &Corr_Planar, const typename pcl::PointCloud<PointT>::Ptr &Source_Sphere,
                                          const typename pcl::PointCloud<PointT>::Ptr &Target_Sphere, boost::shared_ptr<pcl::Correspondences> &Corr_Sphere,
                                          const Eigen::Matrix<float, 6, 1> &transform_x, double &sigma2) const
    {
        double MSE = 0;

        int Corr_total_num = (*Corr_Ground).size() + (*Corr_Planar).size() + (*Corr_Edge).size() + (*Corr_Sphere).size();
        //Ground to Ground (Plane to Plane)
        for (int i = 0; i < (*Corr_Ground).size(); i++)
        {
            int s_index, t_index;
            s_index = (*Corr_Ground)[i].index_query;
            t_index = (*Corr_Ground)[i].index_match;
            if (t_index != -1)
            {
                float px = Source_Ground->points[s_index].x;
                float py = Source_Ground->points[s_index].y;
                float pz = Source_Ground->points[s_index].z;
                float qx = Target_Ground->points[t_index].x;
                float qy = Target_Ground->points[t_index].y;
                float qz = Target_Ground->points[t_index].z;
                float ntx = Target_Ground->points[t_index].normal_x;
                float nty = Target_Ground->points[t_index].normal_y;
                float ntz = Target_Ground->points[t_index].normal_z;

                float a = ntz * py - nty * pz;
                float b = ntx * pz - ntz * px;
                float c = nty * px - ntx * py;
                float d = ntx * qx + nty * qy + ntz * qz - ntx * px - nty * py - ntz * pz;

                float residual = ntx * transform_x(0) + nty * transform_x(1) + ntz * transform_x(2) + a * transform_x(3) + b * transform_x(4) + c * transform_x(5) - d;

                MSE += residual * residual;
            }
        }
        //LOG(INFO) << "Ground OK";

        //Planar to Planar (Plane to Plane) Facade
        for (int i = 0; i < (*Corr_Planar).size(); i++)
        {
            int s_index, t_index;
            s_index = (*Corr_Planar)[i].index_query;
            t_index = (*Corr_Planar)[i].index_match;

            if (t_index != -1)
            {
                float px = Source_Planar->points[s_index].x;
                float py = Source_Planar->points[s_index].y;
                float pz = Source_Planar->points[s_index].z;
                float qx = Target_Planar->points[t_index].x;
                float qy = Target_Planar->points[t_index].y;
                float qz = Target_Planar->points[t_index].z;
                float ntx = Target_Planar->points[t_index].normal_x;
                float nty = Target_Planar->points[t_index].normal_y;
                float ntz = Target_Planar->points[t_index].normal_z;

                float a = ntz * py - nty * pz;
                float b = ntx * pz - ntz * px;
                float c = nty * px - ntx * py;
                float d = ntx * qx + nty * qy + ntz * qz - ntx * px - nty * py - ntz * pz;

                float residual = ntx * transform_x(0) + nty * transform_x(1) + ntz * transform_x(2) + a * transform_x(3) + b * transform_x(4) + c * transform_x(5) - d;

                MSE += residual * residual;
            }
        }
        //LOG(INFO) << "Planar OK";

        //Edge to Edge (Line to Line)
        for (int i = 0; i < (*Corr_Edge).size(); i++)
        {
            int s_index, t_index;
            s_index = (*Corr_Edge)[i].index_query;
            t_index = (*Corr_Edge)[i].index_match;

            if (t_index != -1)
            {
                float px = Source_Edge->points[s_index].x;
                float py = Source_Edge->points[s_index].y;
                float pz = Source_Edge->points[s_index].z;
                float qx = Target_Edge->points[t_index].x;
                float qy = Target_Edge->points[t_index].y;
                float qz = Target_Edge->points[t_index].z;
                float ntx = Target_Edge->points[t_index].normal_x;
                float nty = Target_Edge->points[t_index].normal_y;
                float ntz = Target_Edge->points[t_index].normal_z;
                float nsx = Source_Edge->points[s_index].normal_x;
                float nsy = Source_Edge->points[s_index].normal_y;
                float nsz = Source_Edge->points[s_index].normal_z;

                float dx = px - qx;
                float dy = py - qy;
                float dz = pz - qz;

                float nx = ntx * nsz - ntz * nsy;
                float ny = ntz * nsx - ntx * nsz;
                float nz = ntx * nsy - nty * nsx;
                float nd = sqrt(nx * nx + ny * ny + nz * nz);
                nx /= nd;
                ny /= nd;
                nz /= nd;

                double nxy = nx * ny;
                double nxz = nx * nz;
                double nyz = ny * nz;
                double nx2 = nx * nx;
                double ny2 = ny * ny;
                double nz2 = nz * nz;
                double px2 = px * px;
                double py2 = py * py;
                double pz2 = pz * pz;
                double pxy = px * py;
                double pxz = px * pz;
                double pyz = py * pz;

                Eigen::Matrix<float, 3, 6> A_Matrix;
                Eigen::Matrix<float, 3, 1> b_vector;
                Eigen::Matrix<float, 3, 1> residual_vector;

                A_Matrix << 0, nz, -ny, -nz * pz - ny * py, ny * px, nz * px,
                    -nz, 0, nx, nx * py, -nx * px - nz * pz, nz * py,
                    ny, -nx, 0, nx * pz, ny * pz, -ny * py - nx * px;
                b_vector << -nz * dy + ny * dz, -nx * dz + nz * dx, -ny * dx + nx * dy;

                residual_vector = A_Matrix * transform_x - b_vector;

                MSE += (residual_vector(0) * residual_vector(0) + residual_vector(1) * residual_vector(1) + residual_vector(2) * residual_vector(2));
            }
        }
        //LOG(INFO) << "Edge OK";

        //Sphere to Sphere (Point to Point)
        for (int i = 0; i < (*Corr_Sphere).size(); i++)
        {
            int s_index, t_index;
            s_index = (*Corr_Sphere)[i].index_query;
            t_index = (*Corr_Sphere)[i].index_match;

            if (t_index != -1)
            {
                float px = Source_Sphere->points[s_index].x;
                float py = Source_Sphere->points[s_index].y;
                float pz = Source_Sphere->points[s_index].z;
                float qx = Target_Sphere->points[t_index].x;
                float qy = Target_Sphere->points[t_index].y;
                float qz = Target_Sphere->points[t_index].z;

                float dx = px - qx;
                float dy = py - qy;
                float dz = pz - qz;

                Eigen::Matrix<float, 3, 6> A_Matrix;
                Eigen::Matrix<float, 3, 1> b_vector;
                Eigen::Matrix<float, 3, 1> residual_vector;

                A_Matrix << 1, 0, 0, 0, pz, -py,
                    0, 1, 0, -pz, 0, px,
                    0, 0, 1, py, -px, 0;
                b_vector << -dx, -dy, -dz;

                residual_vector = A_Matrix * transform_x - b_vector;

                MSE += (residual_vector(0) * residual_vector(0) + residual_vector(1) * residual_vector(1) + residual_vector(2) * residual_vector(2));
            }
        }
        //LOG(INFO) << "Sphere OK";

        MSE /= (Corr_total_num - 1);
        sigma2 = MSE;

        LOG(INFO) << "The sigma's square is " << sigma2;

        return 1;
    }

    void constructTransformation(const float &tx, const float &ty, const float &tz,
                                 const float &alpha, const float &beta, const float &gamma,
                                 Eigen::Matrix4f &transformation_matrix) const
    {
        // Construct the transformation matrix from rotation and translation
        transformation_matrix = Eigen::Matrix<float, 4, 4>::Zero();
        // From euler angle to rotation matrix
        transformation_matrix(0, 0) = static_cast<float>(cos(gamma) * cos(beta));
        transformation_matrix(0, 1) = static_cast<float>(-sin(gamma) * cos(alpha) + cos(gamma) * sin(beta) * sin(alpha));
        transformation_matrix(0, 2) = static_cast<float>(sin(gamma) * sin(alpha) + cos(gamma) * sin(beta) * cos(alpha));
        transformation_matrix(1, 0) = static_cast<float>(sin(gamma) * cos(beta));
        transformation_matrix(1, 1) = static_cast<float>(cos(gamma) * cos(alpha) + sin(gamma) * sin(beta) * sin(alpha));
        transformation_matrix(1, 2) = static_cast<float>(-cos(gamma) * sin(alpha) + sin(gamma) * sin(beta) * cos(alpha));
        transformation_matrix(2, 0) = static_cast<float>(-sin(beta));
        transformation_matrix(2, 1) = static_cast<float>(cos(beta) * sin(alpha));
        transformation_matrix(2, 2) = static_cast<float>(cos(beta) * cos(alpha));

        transformation_matrix(0, 3) = static_cast<float>(tx);
        transformation_matrix(1, 3) = static_cast<float>(ty);
        transformation_matrix(2, 3) = static_cast<float>(tz);
        transformation_matrix(3, 3) = static_cast<float>(1);
    }
    
    // From the point cloud raw data, Given feature points index, extract those feature points and group them into the pcl PointCloud 
    // For further registration
    void ExtractFeaturePointsFromFullPointCloud(const std::shared_ptr<point_cloud_t<PointType>> & full_pointcloud,
                                                const std::vector<unsigned int> & feature_point_index,
                                                pcl::PointCloud<PointT>::Ptr & feature_pointcloud_pcl) {
        for (int i = 0; i < feature_point_index.size(); i++) {
            PointT pt;
            pt.x = full_pointcloud->points[feature_point_index[i]].x;
            pt.y = full_pointcloud->points[feature_point_index[i]].y;
            pt.z = full_pointcloud->points[feature_point_index[i]].z;
            pt.normal_x = full_pointcloud->points[feature_point_index[i]].nx;
            pt.normal_y = full_pointcloud->points[feature_point_index[i]].ny;
            pt.normal_z = full_pointcloud->points[feature_point_index[i]].nz;
            feature_pointcloud_pcl->points.push_back(pt);
        }
    }
    
    // A common icp registration function grouping pcl's icp implement
    // You can select from different kind of icp variants
    // Different metrics, correspondence estimators, rejectors and transformation estimators
    // This one use radius nearest neighbor to estimate the normal
    double ICPRegistrationPCL(const typename pcl::PointCloud<PointType>::Ptr &SourceCloud,
                              const typename pcl::PointCloud<PointType>::Ptr &TargetCloud,
                              typename pcl::PointCloud<PointType>::Ptr &TransformedSource,
                              Eigen::Matrix4f &transformationS2T,
                              metrics_type metrics, correspondence_estimator_type ce, transform_estimator_type te,
                              bool use_reciprocal_correspondence, bool use_trimmed_rejector,
                              int max_iter, float thre_dis, float neighbor_radius)
    {

        clock_t t0, t1, t2;
        t0 = clock();

        double mae_nn;
        pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr trimmed_cr(
            new pcl::registration::CorrespondenceRejectorVarTrimmed);

        switch (metrics)
        {
        case Point2Point:
        {
            t1 = clock();
            pcl::IterativeClosestPoint<PointT, PointT> icp;

            icp.setInputSource(SourceCloud);
            icp.setInputTarget(TargetCloud);

            typename pcl::registration::TransformationEstimationSVD<PointType, PointType, float>::Ptr te_svd(
                new pcl::registration::TransformationEstimationSVD<PointType, PointType, float>);
            typename pcl::registration::TransformationEstimationLM<PointType, PointType, float>::Ptr te_lm(
                new pcl::registration::TransformationEstimationLM<PointType, PointType, float>);

            switch (te)
            {
            case SVD:
                icp.setTransformationEstimation(te_svd); //Use SVD
                break;
            case LM:
                icp.setTransformationEstimation(te_lm); //Use L-M Non-Linear Optimization
                break;
            default: //Default svd
                break;
            }

            // Use Reciprocal Correspondences or not? [a -> b && b -> a]
            icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

            // Trimmed or not?
            if (use_trimmed_rejector)
                icp.addCorrespondenceRejector(trimmed_cr);
            else
                icp.setMaxCorrespondenceDistance(thre_dis);

            // Converge criterion ( 'Or' Relation )
            // Set the maximum number of iterations [ n>x ] (criterion 1)
            icp.setMaximumIterations(max_iter); //Most likely to happen
            // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
            icp.setTransformationEpsilon(1e-8); //Quite hard to happen
            // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
            icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

            icp.align(*TransformedSource);

            transformationS2T = icp.getFinalTransformation();

            t2 = clock();

            if (use_trimmed_rejector)
                thre_dis = trimmed_cr->getTrimmedDistance();
            printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

            mae_nn = icp.getFitnessScore(
                thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

            // Commented these out if you don't want to output the registration log
            std::cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "
                      << TargetCloud->points.size() << std::endl;
            std::cout << "Point to Point ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << std::endl;
            std::cout << "The fitness score of this registration is " << mae_nn << std::endl
                      << transformationS2T << std::endl;

            break;
        }
        case Point2Plane:
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr SourceCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            copyPointCloud(*SourceCloud, *SourceCloudXYZ);
            copyPointCloud(*TargetCloud, *TargetCloudXYZ);
            // In this case, The Cloud's Normal hasn't been calculated yet.

            pcl::PointCloud<pcl::PointNormal>::Ptr SourceNormal(new pcl::PointCloud<pcl::PointNormal>());
            pcl::PointCloud<pcl::PointNormal>::Ptr TargetNormal(new pcl::PointCloud<pcl::PointNormal>());
            pcl::PointCloud<pcl::PointNormal>::Ptr TransformedSourceN(new pcl::PointCloud<pcl::PointNormal>());

            LOG(INFO) << "prepare for registration";

            //Estimate Normal Multi-thread
            PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator;

            //Radius search
            pca_estimator.CalculatePointCloudWithNormal_Radius(SourceCloudXYZ, neighbor_radius, SourceNormal);
            pca_estimator.CalculatePointCloudWithNormal_Radius(TargetCloudXYZ, neighbor_radius, TargetNormal);
            //Or
            //KNN search
            //pca_estimator.CalculatePointCloudWithNormal_KNN(SourceCloud, covariance_K, SourceNormal);
            //pca_estimator.CalculatePointCloudWithNormal_KNN(TargetCloud, covariance_K, TargetNormal);

            LOG(INFO) << "prepare for registration done";

            t1 = clock();

            pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;

            icp.setInputSource(SourceNormal);
            icp.setInputTarget(TargetNormal);

            if (ce == NS) //Normal Shooting
            {
                pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>::Ptr ns_est(
                    new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>);
                ns_est->setInputSource(SourceNormal);
                ns_est->setSourceNormals(SourceNormal);
                ns_est->setInputTarget(TargetNormal);
                ns_est->setKSearch(5);
                icp.setCorrespondenceEstimation(ns_est);
            }

            pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal, float>::Ptr te_lmn(
                new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal, float>);
            pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal, float>::Ptr te_lls(
                new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal, float>);
            pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<pcl::PointNormal, pcl::PointNormal, float>::Ptr te_lls_weight(
                new pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<pcl::PointNormal, pcl::PointNormal, float>);
            switch (te)
            {
            case LLS:
                icp.setTransformationEstimation(te_lls); //Use Linear Least Square
                break;
            case LM:
                icp.setTransformationEstimation(te_lmn); //Use L-M Non-Linear Optimization
                break;
            default: //Default lls
                break;
            }

            // Use Reciprocal Correspondences or not? [a -> b && b -> a]
            icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

            // Trimmed or not?
            if (use_trimmed_rejector)
                icp.addCorrespondenceRejector(trimmed_cr);
            else
                icp.setMaxCorrespondenceDistance(thre_dis);

            // Converge criterion ( 'Or' Relation )
            // Set the maximum number of iterations [ n>x ] (criterion 1)
            icp.setMaximumIterations(max_iter); //Most likely to happen
            // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
            icp.setTransformationEpsilon(1e-8); //Quite hard to happen
            // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
            icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

            icp.align(*TransformedSourceN);

            transformationS2T = icp.getFinalTransformation();

            t2 = clock();

            if (use_trimmed_rejector)
                thre_dis = trimmed_cr->getTrimmedDistance();
            printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

            mae_nn = icp.getFitnessScore(
                thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

            // Commented these out if you don't want to output the registration log
            std::cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "
                      << TargetCloud->points.size() << std::endl;
            std::cout << "Point-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << std::endl;
            std::cout << "Normal Estimation in " << float(t1 - t0) / CLOCKS_PER_SEC << " s, "
                      << "registration in "
                      << float(t2 - t1) / CLOCKS_PER_SEC << " s." << std::endl;
            std::cout << "The fitness score of this registration is " << mae_nn << std::endl
                      << transformationS2T << std::endl;

            copyPointCloud(*TransformedSourceN, *TransformedSource);
            pcl::PointCloud<pcl::PointNormal>().swap(*TransformedSourceN); // Free the Memory
            pcl::PointCloud<pcl::PointNormal>().swap(*SourceNormal);       // Free the Memory
            pcl::PointCloud<pcl::PointNormal>().swap(*TargetNormal);       // Free the Memory
            pcl::PointCloud<pcl::PointXYZ>().swap(*SourceCloudXYZ);        // Free the Memory
            pcl::PointCloud<pcl::PointXYZ>().swap(*TargetCloudXYZ);        // Free the Memory

            break;
        }
        case Plane2Plane:
        {
            t1 = clock();
            pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;

            // Set the number of points used to calculated the covariance of a point
            // icp.setCorrespondenceRandomness(covariance_K);
            icp.setCorrespondenceRandomness(10);

            icp.setInputSource(SourceCloud);
            icp.setInputTarget(TargetCloud);

            // Use Reciprocal Correspondences or not? [a -> b && b -> a]
            icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

            // Trimmed or not?
            if (use_trimmed_rejector)
                icp.addCorrespondenceRejector(trimmed_cr);
            else
                icp.setMaxCorrespondenceDistance(thre_dis);

            icp.setMaximumOptimizerIterations(10);

            // Converge criterion ( 'Or' Relation )
            // Set the maximum number of iterations [ n>x ] (criterion 1)
            icp.setMaximumIterations(max_iter); //Most likely to happen
            // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
            icp.setTransformationEpsilon(1e-8); //Quite hard to happen
            // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
            icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

            icp.align(*TransformedSource);

            transformationS2T = icp.getFinalTransformation();

            t2 = clock();

            if (use_trimmed_rejector)
                thre_dis = trimmed_cr->getTrimmedDistance();
            printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

            mae_nn = icp.getFitnessScore(
                thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

            // Commented these out if you don't want to output the registration log
            std::cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "
                      << TargetCloud->points.size() << std::endl;
            std::cout << "Plane-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << std::endl;
            std::cout << "The fitness score of this registration is " << mae_nn << std::endl
                      << transformationS2T << std::endl;
            break;
        }
        default:
            return -1;
        }

        return mae_nn;
    }
    
    // A common icp registration function grouping pcl's icp implement
    // You can select from different kind of icp variants
    // Different metrics, correspondence estimators, rejectors and transformation estimators
    // This one use K nearest neighbor to estimate the normal
    double ICPRegistrationPCL(const typename pcl::PointCloud<PointType>::Ptr &SourceCloud,
                             const typename pcl::PointCloud<PointType>::Ptr &TargetCloud,
                             typename pcl::PointCloud<PointType>::Ptr &TransformedSource,
                             Eigen::Matrix4f &transformationS2T,
                             metrics_type metrics, correspondence_estimator_type ce, transform_estimator_type te,
                             bool use_reciprocal_correspondence, bool use_trimmed_rejector,
                             int max_iter, float thre_dis, int neighbor_K)
    {
        clock_t t0, t1, t2;
        t0 = clock();

        double mae_nn;
        pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr trimmed_cr(
            new pcl::registration::CorrespondenceRejectorVarTrimmed);

        switch (metrics)
        {
        case Point2Point:
        {
            t1 = clock();
            pcl::IterativeClosestPoint<PointType, PointType> icp;

            icp.setInputSource(SourceCloud);
            icp.setInputTarget(TargetCloud);

            typename pcl::registration::TransformationEstimationSVD<PointType, PointType, float>::Ptr te_svd(
                new pcl::registration::TransformationEstimationSVD<PointType, PointType, float>);
            typename pcl::registration::TransformationEstimationLM<PointType, PointType, float>::Ptr te_lm(
                new pcl::registration::TransformationEstimationLM<PointType, PointType, float>);

            switch (te)
            {
            case SVD:
                icp.setTransformationEstimation(te_svd); //Use SVD
                break;
            case LM:
                icp.setTransformationEstimation(te_lm); //Use L-M Non-Linear Optimization
                break;
            default: //Default svd
                break;
            }

            // Use Reciprocal Correspondences or not? [a -> b && b -> a]
            icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

            // Trimmed or not?
            if (use_trimmed_rejector)
                icp.addCorrespondenceRejector(trimmed_cr);
            else
                icp.setMaxCorrespondenceDistance(thre_dis);

            // Converge criterion ( 'Or' Relation )
            // Set the maximum number of iterations [ n>x ] (criterion 1)
            icp.setMaximumIterations(max_iter); //Most likely to happen
            // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
            icp.setTransformationEpsilon(1e-8); //Quite hard to happen
            // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
            icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

            icp.align(*TransformedSource);

            transformationS2T = icp.getFinalTransformation();

            t2 = clock();

            if (use_trimmed_rejector)
                thre_dis = trimmed_cr->getTrimmedDistance();
            printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

            mae_nn = icp.getFitnessScore(
                thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

            // Commented these out if you don't want to output the registration log
            std::cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "
                      << TargetCloud->points.size() << std::endl;
            std::cout << "Point to Point ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << std::endl;
            std::cout << "The fitness score of this registration is " << mae_nn << std::endl
                      << transformationS2T << std::endl;

            break;
        }
        case Point2Plane:
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr SourceCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            copyPointCloud(*SourceCloud, *SourceCloudXYZ);
            copyPointCloud(*TargetCloud, *TargetCloudXYZ);
            // In this case, The Cloud's Normal hasn't been calculated yet.

            pcl::PointCloud<pcl::PointNormal>::Ptr SourceNormal(new pcl::PointCloud<pcl::PointNormal>());
            pcl::PointCloud<pcl::PointNormal>::Ptr TargetNormal(new pcl::PointCloud<pcl::PointNormal>());
            pcl::PointCloud<pcl::PointNormal>::Ptr TransformedSourceN(new pcl::PointCloud<pcl::PointNormal>());

            //Estimate Normal Multi-thread
            PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator;

            //Radius search
            // pca_estimator.CalculatePointCloudWithNormal_Radius(SourceCloudXYZ, neighbor_radius, SourceNormal);
            // pca_estimator.CalculatePointCloudWithNormal_Radius(TargetCloudXYZ, neighbor_radius, TargetNormal);
            //Or
            //KNN search
            pca_estimator.CalculatePointCloudWithNormal_KNN(SourceCloudXYZ, neighbor_K, SourceNormal);
            pca_estimator.CalculatePointCloudWithNormal_KNN(TargetCloudXYZ, neighbor_K, TargetNormal);

            LOG(INFO) << "prepare for registration done";

            t1 = clock();

            pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;

            icp.setInputSource(SourceNormal);
            icp.setInputTarget(TargetNormal);

            if (ce == NS) //Normal Shooting
            {
                pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>::Ptr ns_est(
                    new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>);
                ns_est->setInputSource(SourceNormal);
                ns_est->setSourceNormals(SourceNormal);
                ns_est->setInputTarget(TargetNormal);
                ns_est->setKSearch(1);
                icp.setCorrespondenceEstimation(ns_est);
            }

            pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal, float>::Ptr te_lmn(
                new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal, float>);
            pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal, float>::Ptr te_lls(
                new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal, float>);
            switch (te)
            {
            case LLS:
                icp.setTransformationEstimation(te_lls); //Use Linear Least Square
                break;
            case LM:
                icp.setTransformationEstimation(te_lmn); //Use L-M Non-Linear Optimization
                break;
            default: //Default lls
                break;
            }

            // Use Reciprocal Correspondences or not? [a -> b && b -> a]
            icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

            // Trimmed or not?
            if (use_trimmed_rejector)
                icp.addCorrespondenceRejector(trimmed_cr);
            else
                icp.setMaxCorrespondenceDistance(thre_dis);

            // Converge criterion ( 'Or' Relation )
            // Set the maximum number of iterations [ n>x ] (criterion 1)
            icp.setMaximumIterations(max_iter); //Most likely to happen
            // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
            icp.setTransformationEpsilon(1e-8); //Quite hard to happen
            // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
            icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

            icp.align(*TransformedSourceN);

            transformationS2T = icp.getFinalTransformation();

            t2 = clock();

            if (use_trimmed_rejector)
                thre_dis = trimmed_cr->getTrimmedDistance();
            printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

            mae_nn = icp.getFitnessScore(
                thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

            // Commented these out if you don't want to output the registration log
            std::cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "
                      << TargetCloud->points.size() << std::endl;
            std::cout << "Point-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << std::endl;
            std::cout << "Normal Estimation in " << float(t1 - t0) / CLOCKS_PER_SEC << " s, "
                      << "registration in "
                      << float(t2 - t1) / CLOCKS_PER_SEC << " s." << std::endl;
            std::cout << "The fitness score of this registration is " << mae_nn << std::endl
                      << transformationS2T << std::endl;

            copyPointCloud(*TransformedSourceN, *TransformedSource);
            pcl::PointCloud<pcl::PointNormal>().swap(*TransformedSourceN); // Free the Memory
            pcl::PointCloud<pcl::PointNormal>().swap(*SourceNormal);       // Free the Memory
            pcl::PointCloud<pcl::PointNormal>().swap(*TargetNormal);       // Free the Memory
            pcl::PointCloud<pcl::PointXYZ>().swap(*SourceCloudXYZ);        // Free the Memory
            pcl::PointCloud<pcl::PointXYZ>().swap(*TargetCloudXYZ);        // Free the Memory

            break;
        }
        case Plane2Plane:
        {
            t1 = clock();
            pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;

            // Set the number of points used to calculated the covariance of a point
            // icp.setCorrespondenceRandomness(covariance_K);
            icp.setCorrespondenceRandomness(neighbor_K);

            icp.setInputSource(SourceCloud);
            icp.setInputTarget(TargetCloud);

            // Use Reciprocal Correspondences or not? [a -> b && b -> a]
            icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

            // Trimmed or not?
            if (use_trimmed_rejector)
                icp.addCorrespondenceRejector(trimmed_cr);
            else
                icp.setMaxCorrespondenceDistance(thre_dis);

            icp.setMaximumOptimizerIterations(10);

            // Converge criterion ( 'Or' Relation )
            // Set the maximum number of iterations [ n>x ] (criterion 1)
            icp.setMaximumIterations(max_iter); //Most likely to happen
            // Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
            icp.setTransformationEpsilon(1e-8); //Quite hard to happen
            // Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
            icp.setEuclideanFitnessEpsilon(1e-5); //Quite hard to happen

            icp.align(*TransformedSource);

            transformationS2T = icp.getFinalTransformation();

            t2 = clock();

            if (use_trimmed_rejector)
                thre_dis = trimmed_cr->getTrimmedDistance();
            printf("Estimated trimmed distance threshold is %lf.\n", thre_dis);

            mae_nn = icp.getFitnessScore(
                thre_dis); //Get the Mean Absolute Error (MAE) after registration calculated from Nearest Neighbor Search

            // Commented these out if you don't want to output the registration log
            std::cout << "SCloud point # " << SourceCloud->points.size() << " , TCloud point # "
                      << TargetCloud->points.size() << std::endl;
            std::cout << "Plane-to-Plane ICP done in " << float(t2 - t0) / CLOCKS_PER_SEC << " s" << std::endl;
            std::cout << "The fitness score of this registration is " << mae_nn << std::endl
                      << transformationS2T << std::endl;
            break;
        }
        default:
            return -1;
        }

        return mae_nn;
    }


    /**
    * \brief Estimated the approximate overlapping ratio for Cloud1 considering Cloud2
    * \param[in]  Cloud1 : A pointer of the Point Cloud used for overlap ratio calculation
    * \param[in]  Cloud2 : A pointer of the Point Cloud overlapped with Cloud1
    * \param[out] thre_dis : It acts as the search radius of overlapping estimation
    * \return : The estimated overlap ratio [from 0 to 1]
    */
    float calOverlap(const typename pcl::PointCloud<PointT>::Ptr &Cloud1,
                     const typename pcl::PointCloud<PointT>::Ptr &Cloud2,
                     float thre_dis)
    {
        int overlap_point_num = 0;
        float overlap_ratio;

        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(Cloud2);

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        for (int i = 0; i < Cloud1->size(); i++)
        {
            if (kdtree.radiusSearch(Cloud1->points[i], thre_dis, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
                0)
                overlap_point_num++;
        }

        overlap_ratio = (0.01 + overlap_point_num) / Cloud1->size();
        //cout << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio << endl;
        LOG(INFO) << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio;

        return overlap_ratio;
    }

    /**
    * \brief Transform a Point Cloud using a given transformation matrix
    * \param[in]  Cloud : A pointer of the Point Cloud before transformation
    * \param[out] TransformedCloud : A pointer of the Point Cloud after transformation
    * \param[in]  transformation : A 4*4 transformation matrix
    */
    void TransformCloudByMatrix(typename pcl::PointCloud<PointT>::Ptr &Cloud,
                                typename pcl::PointCloud<PointT>::Ptr &TransformedCloud,
                                Eigen::Matrix4f &transformation)
    {
        Eigen::Matrix4Xf PC;
        Eigen::Matrix4Xf TPC;
        PC.resize(4, Cloud->size());
        TPC.resize(4, Cloud->size());
        for (int i = 0; i < Cloud->size(); i++)
        {
            PC(0, i) = Cloud->points[i].x;
            PC(1, i) = Cloud->points[i].y;
            PC(2, i) = Cloud->points[i].z;
            PC(3, i) = 1;
        }
        TPC = transformation * PC;
        for (int i = 0; i < Cloud->size(); i++)
        {
            PointT pt;
            pt.x = TPC(0, i);
            pt.y = TPC(1, i);
            pt.z = TPC(2, i);

            //Comment it if there's no intensity
            pt.intensity = Cloud->points[i].intensity;
            TransformedCloud->points.push_back(pt);
        }
        //cout << "Transform done ..." << endl;
    }

    /**
    * \brief Get the Inverse (Not Mathimatically) of a giving 4*4 Transformation Matrix
    * \param[in]  transformation : A 4*4 transformation matrix ( from Cloud A to Cloud A' )
    * \param[out] invtransformation : Inversed 4*4 transformation matrix ( from Cloud A' to Cloud A )
    */
    void invTransform(const Eigen::Matrix4f &transformation,
                      Eigen::Matrix4f &invtransformation)
    {
        invtransformation.block<3, 3>(0, 0) = (transformation.block<3, 3>(0, 0)).transpose();
        invtransformation(0, 3) = -transformation(0, 3);
        invtransformation(1, 3) = -transformation(1, 3);
        invtransformation(2, 3) = -transformation(2, 3);
        invtransformation(3, 0) = 0;
        invtransformation(3, 1) = 0;
        invtransformation(3, 2) = 0;
        invtransformation(3, 3) = 1;
    }

#if 0
    //brief: Compute fpfh_feature
    void compute_fpfh_feature(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
                              fpfhFeaturePtr &cloud_fpfh,
                              float search_radius) {
    
        // Calculate the Point Normal
        NormalsPtr cloud_normal(new Normals);
        calNormal(input_cloud, cloud_normal, search_radius);
    
        // Estimate FPFH Feature
        pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
        est_fpfh.setNumberOfThreads(4);
        est_fpfh.setInputCloud(input_cloud);
        est_fpfh.setInputNormals(cloud_normal);
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        est_fpfh.setSearchMethod(tree);
        //est_fpfh.setKSearch(20);
        est_fpfh.setRadiusSearch(2.0 * search_radius);
        est_fpfh.compute(*cloud_fpfh);
        }

    //brief: Accomplish Coarse registration using FPFH SAC
    void Coarsereg_FPFHSAC(const typename pcl::PointCloud<PointT>::Ptr &SourceCloud,
                           const typename pcl::PointCloud<PointT>::Ptr &TargetCloud,
                           typename pcl::PointCloud<PointT>::Ptr &TransformedSource,
                           Eigen::Matrix4f &transformationS2T,
                           float search_radius);
#endif

    //Brief: Using the Gauss-Newton Least Square Method to solve 4 Degree of Freedom Transformation from no less than 2 points
    //cp_number is the control points' number for LLS calculation, the rest of points are used to check the accuracy;
    //X Y Z yaw
    bool LLS_4DOF(const std::vector<std::vector<double>> &coordinatesA,
                  const std::vector<std::vector<double>> &coordinatesB, Eigen::Matrix4d &TransMatrixA2B, int cp_number,
                  double theta0_degree)
    {
        Eigen::Vector4d transAB;
        Eigen::Vector4d temp_trans;

        int iter_num = 0;

        double theta, theta0, dtheta, eps; // in rad
        double yaw_degree;                 // in degree
        double tx, ty, tz;                 // in meter
        double RMSE_check, sum_squaredist;
        int pointnumberA, pointnumberB, pointnumbercheck;
        //std::vector <std::vector<double>> coordinatesAT;

        //cout << "Input the approximate yaw angle in degree" << endl;
        //cin >> theta0_degree;

        dtheta = 9999;
        eps = 1e-9;

        theta0 = theta0_degree / 180 * M_PI; //Original Guess

        pointnumberA = coordinatesA.size();
        pointnumberB = coordinatesB.size();

        sum_squaredist = 0;

        if (cp_number < 2)
        {
            std::cout << "Error ! Not enough control point number ..." << std::endl;
            return 0;
        }

        while (abs(dtheta) > eps)
        {

            Eigen::MatrixXd A_;
            Eigen::VectorXd b_;

            A_.resize(cp_number * 3, 4);
            b_.resize(cp_number * 3, 1);

            for (int j = 0; j < cp_number; j++)
            {
                // A Matrix
                A_(j * 3, 0) = -coordinatesA[j][0] * sin(theta0) - coordinatesA[j][1] * cos(theta0);
                A_(j * 3, 1) = 1;
                A_(j * 3, 2) = 0;
                A_(j * 3, 3) = 0;

                A_(j * 3 + 1, 0) = coordinatesA[j][0] * cos(theta0) - coordinatesA[j][1] * sin(theta0);

                A_(j * 3 + 1, 1) = 0;
                A_(j * 3 + 1, 2) = 1;
                A_(j * 3 + 1, 3) = 0;

                A_(j * 3 + 2, 0) = 0;
                A_(j * 3 + 2, 1) = 0;
                A_(j * 3 + 2, 2) = 0;
                A_(j * 3 + 2, 3) = 1;

                //b Vector
                b_(j * 3, 0) = coordinatesB[j][0] - coordinatesA[j][0] * cos(theta0) + coordinatesA[j][1] * sin(theta0);
                b_(j * 3 + 1, 0) =
                    coordinatesB[j][1] - coordinatesA[j][0] * sin(theta0) - coordinatesA[j][1] * cos(theta0);
                b_(j * 3 + 2, 0) = coordinatesB[j][2] - coordinatesA[j][2];
            }

            //x=(ATA)-1(ATb)
            temp_trans = ((A_.transpose() * A_).inverse()) * A_.transpose() * b_;
            dtheta = temp_trans(0, 0);

            theta0 += dtheta;

            iter_num++;

            //cout << "Result for iteration " << iter_num << " is " << endl
            //<< temp_trans(1, 0) << " , " << temp_trans(2, 0) << " , " << temp_trans(3, 0) << " , " << theta0 * 180 / M_PI <<endl;
        }

        transAB = temp_trans;

        theta = theta0;
        yaw_degree = theta * 180 / M_PI;

        tx = transAB(1, 0);
        ty = transAB(2, 0);
        tz = transAB(3, 0);

        std::cout.setf(std::ios::showpoint);
        std::cout.precision(12);

        std::cout << "Calculated by Linear Least Square" << std::endl
                  << "Converged in " << iter_num << " iterations ..." << std::endl
                  << "Station B 's Coordinate and Orientation in A's System is:" << std::endl
                  << "X: " << tx << " m" << std::endl
                  << "Y: " << ty << " m" << std::endl
                  << "Z: " << tz << " m" << std::endl
                  << "yaw: " << yaw_degree << " degree" << std::endl;

        TransMatrixA2B(0, 0) = cos(theta);
        TransMatrixA2B(0, 1) = -sin(theta);
        TransMatrixA2B(0, 2) = 0;
        TransMatrixA2B(0, 3) = tx;

        TransMatrixA2B(1, 0) = sin(theta);
        TransMatrixA2B(1, 1) = cos(theta);
        TransMatrixA2B(1, 2) = 0;
        TransMatrixA2B(1, 3) = ty;

        TransMatrixA2B(2, 0) = 0;
        TransMatrixA2B(2, 1) = 0;
        TransMatrixA2B(2, 2) = 1;
        TransMatrixA2B(2, 3) = tz;

        TransMatrixA2B(3, 0) = 0;
        TransMatrixA2B(3, 1) = 0;
        TransMatrixA2B(3, 2) = 0;
        TransMatrixA2B(3, 3) = 1;

        std::cout << "The Transformation Matrix from Coordinate System A to B is: " << std::endl
                  << TransMatrixA2B << std::endl;

        // Checking
        if (pointnumberA >= pointnumberB)
            pointnumbercheck = pointnumberB;
        else
            pointnumbercheck = pointnumberA;

        if (pointnumbercheck <= cp_number)
            std::cout << "Not enough points for check ..." << std::endl;
        else
        {
            pointnumbercheck -= cp_number;
            for (int j = 0; j < pointnumbercheck; j++)
            {
                double X_tran, Y_tran, Z_tran, squaredist;
                X_tran = cos(theta) * coordinatesA[j + cp_number][0] - sin(theta) * coordinatesA[j + cp_number][1] + tx;
                Y_tran = sin(theta) * coordinatesA[j + cp_number][0] + cos(theta) * coordinatesA[j + cp_number][1] + ty;
                Z_tran = coordinatesA[j + cp_number][2] + tz;

                squaredist = (X_tran - coordinatesB[j + cp_number][0]) * (X_tran - coordinatesB[j + cp_number][0]) +
                             (Y_tran - coordinatesB[j + cp_number][1]) * (Y_tran - coordinatesB[j + cp_number][1]) +
                             (Z_tran - coordinatesB[j + cp_number][2]) * (Z_tran - coordinatesB[j + cp_number][2]);
                sum_squaredist += squaredist;
            }

            RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

            std::cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << std::endl;
        }

        return 1;
    }
    //X Y Z roll pitch yaw
    bool SVD_6DOF(const std::vector<std::vector<double>> &coordinatesA,
                  const std::vector<std::vector<double>> &coordinatesB, Eigen::Matrix4d &TransMatrixA2B,
                  int cp_number)
    {
        Eigen::Matrix4f transAB2D;
        pcl::PointCloud<PointT> Points2D_A, Points2D_B;
        double ZAB_mean, ZAB_sum;
        int pointnumberA, pointnumberB, pointnumbercheck;
        double RMSE_check, sum_squaredist;

        pointnumberA = coordinatesA.size();
        pointnumberB = coordinatesB.size();
        ZAB_sum = 0;
        sum_squaredist = 0;

        for (size_t i = 0; i < cp_number; i++)
        {
            PointT PtA, PtB;
            PtA.x = coordinatesA[i][0];
            PtA.y = coordinatesA[i][1];
            PtA.z = coordinatesA[i][2];

            PtB.x = coordinatesB[i][0];
            PtB.y = coordinatesB[i][1];
            PtB.z = coordinatesB[i][2];

            Points2D_A.push_back(PtA);
            Points2D_B.push_back(PtB);
            ZAB_sum += (coordinatesB[i][2] - coordinatesA[i][2]);
        }

        ZAB_mean = ZAB_sum / cp_number;

        if (cp_number < 2)
        {
            std::cout << "Error ! Not enough control point number ..." << std::endl;
            return 0;
        }

        pcl::registration::TransformationEstimationSVD<PointT, PointT> svd_estimator;
        svd_estimator.estimateRigidTransformation(Points2D_A, Points2D_B, transAB2D);

        TransMatrixA2B = transAB2D.cast<double>();

        double tx, ty, tz, yaw_rad, yaw_degree;

        tx = TransMatrixA2B(0, 3);
        ty = TransMatrixA2B(1, 3);
        tz = TransMatrixA2B(2, 3);
        yaw_rad = acos(TransMatrixA2B(0, 0));
        if (TransMatrixA2B(1, 0) < 0)
            yaw_rad = -yaw_rad;
        yaw_degree = yaw_rad / M_PI * 180;

        std::cout << "Calculated by SVD" << std::endl
                  << "Station B 's Coordinate and Orientation in A's System is:" << std::endl
                  << "X: " << tx << " m" << std::endl
                  << "Y: " << ty << " m" << std::endl
                  << "Z: " << tz << " m" << std::endl
                  << "yaw: " << yaw_degree << " degree" << std::endl;

        std::cout << "The Transformation Matrix from Coordinate System A to B is: " << std::endl
                  << TransMatrixA2B << std::endl;

        // Checking
        if (pointnumberA >= pointnumberB)
            pointnumbercheck = pointnumberB;
        else
            pointnumbercheck = pointnumberA;

        if (pointnumbercheck <= cp_number)
            std::cout << "Not enough points for check ..." << std::endl;
        else
        {
            pointnumbercheck -= cp_number;
            for (int j = 0; j < pointnumbercheck; j++)
            {
                double X_tran, Y_tran, Z_tran, squaredist;
                X_tran = TransMatrixA2B(0, 0) * coordinatesA[j + cp_number][0] +
                         TransMatrixA2B(0, 1) * coordinatesA[j + cp_number][1] +
                         TransMatrixA2B(0, 2) * coordinatesA[j + cp_number][2] + tx;
                Y_tran = TransMatrixA2B(1, 0) * coordinatesA[j + cp_number][0] +
                         TransMatrixA2B(1, 1) * coordinatesA[j + cp_number][1] +
                         TransMatrixA2B(1, 2) * coordinatesA[j + cp_number][2] + ty;
                Z_tran = TransMatrixA2B(2, 0) * coordinatesA[j + cp_number][0] +
                         TransMatrixA2B(2, 1) * coordinatesA[j + cp_number][1] +
                         TransMatrixA2B(2, 2) * coordinatesA[j + cp_number][2] + tz;

                squaredist = (X_tran - coordinatesB[j + cp_number][0]) * (X_tran - coordinatesB[j + cp_number][0]) +
                             (Y_tran - coordinatesB[j + cp_number][1]) * (Y_tran - coordinatesB[j + cp_number][1]) +
                             (Z_tran - coordinatesB[j + cp_number][2]) * (Z_tran - coordinatesB[j + cp_number][2]);
                sum_squaredist += squaredist;
            }

            RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

            std::cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << std::endl;
        }
    }

    // X Y yaw scale
    bool CSTRAN_4DOF(const std::vector<std::vector<double>> &coordinatesA,
                     const std::vector<std::vector<double>> &coordinatesB, std::vector<double> &transpara,
                     int cp_number)
    {
        double tx, ty, a, b; // 4 parameters
        double s, rot_rad, rot_degree;
        double RMSE_check, sum_squaredist;
        int pointnumberA, pointnumberB, pointnumbercheck;
        Eigen::Vector4d transAB;
        transpara.resize(5);

        pointnumberA = coordinatesA.size();
        pointnumberB = coordinatesB.size();

        sum_squaredist = 0;

        if (cp_number < 3)
        {
            std::cout << "Error ! Not enough control point number ..." << std::endl;
            return 0;
        }

        Eigen::MatrixXd A_;
        Eigen::VectorXd b_;

        A_.resize(cp_number * 2, 4);
        b_.resize(cp_number * 2, 1);

        for (int j = 0; j < cp_number; j++)
        {
            // A Matrix
            A_(j * 2, 0) = 1;
            A_(j * 2, 1) = 0;
            A_(j * 2, 2) = coordinatesA[j][0];
            A_(j * 2, 3) = -coordinatesA[j][1];

            A_(j * 2 + 1, 0) = 0;
            A_(j * 2 + 1, 1) = 1;
            A_(j * 2 + 1, 2) = coordinatesA[j][1];
            A_(j * 2 + 1, 3) = coordinatesA[j][0];

            //b Vector
            b_(j * 2, 0) = coordinatesB[j][0];
            b_(j * 2 + 1, 0) = coordinatesB[j][1];
        }
        transAB = ((A_.transpose() * A_).inverse()) * A_.transpose() * b_;

        tx = transAB(0, 0);
        ty = transAB(1, 0);
        a = transAB(2, 0);
        b = transAB(3, 0);
        s = sqrt(a * a + b * b);

        transpara[0] = tx;
        transpara[1] = ty;
        transpara[2] = s;
        transpara[3] = b / s; //sin (ang)
        transpara[4] = a / s; //cos (ang)

        std::cout.setf(std::ios::showpoint); 
        std::cout.precision(12);             

        std::cout << "Estimated Transformation From A to B" << std::endl
                  << "tx: " << tx << " m" << std::endl
                  << "ty: " << ty << " m" << std::endl
                  << "scale: " << s << std::endl;

        // Checking
        if (pointnumberA >= pointnumberB)
            pointnumbercheck = pointnumberB;
        else
            pointnumbercheck = pointnumberA;

        if (pointnumbercheck <= cp_number)
            std::cout << "Not enough points for check ..." << std::endl;
        else
        {
            pointnumbercheck -= cp_number;
            for (int j = 0; j < pointnumbercheck; j++)
            {
                double X_tran, Y_tran, squaredist;
                X_tran = transpara[2] * transpara[4] * coordinatesA[j + cp_number][0] -
                         transpara[2] * transpara[3] * coordinatesA[j + cp_number][1] + transpara[0];
                Y_tran = transpara[2] * transpara[3] * coordinatesA[j + cp_number][0] +
                         transpara[2] * transpara[4] * coordinatesA[j + cp_number][1] + transpara[1];
                squaredist = (X_tran - coordinatesB[j + cp_number][0]) * (X_tran - coordinatesB[j + cp_number][0]) +
                             (Y_tran - coordinatesB[j + cp_number][1]) * (Y_tran - coordinatesB[j + cp_number][1]);
                sum_squaredist += squaredist;
            }

            RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

            std::cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << std::endl;
        }

        return 1;
    }
    // X Y Z roll pitch yaw scale
    bool CSTRAN_7DOF(const std::vector<std::vector<double>> &coordinatesA,
                     const std::vector<std::vector<double>> &coordinatesB, std::vector<double> &transpara,
                     int cp_number)
    {
        double RMSE_check, sum_squaredist;
        int pointnumberA, pointnumberB, pointnumbercheck;
        Eigen::VectorXd transAB;
        transAB.resize(7);
        transpara.resize(7);

        pointnumberA = coordinatesA.size();
        pointnumberB = coordinatesB.size();

        sum_squaredist = 0;

        if (cp_number < 4)
        {
            std::cout << "Error ! Not enough control point number ..." << std::endl;
            return 0;
        }

        Eigen::MatrixXd A_;
        Eigen::VectorXd b_;

        A_.resize(cp_number * 3, 7);
        b_.resize(cp_number * 3, 1);

        for (int j = 0; j < cp_number; j++)
        {
            // A Matrix   tx ty tz rx ry rz s
            A_(j * 3, 0) = 1;
            A_(j * 3, 1) = 0;
            A_(j * 3, 2) = 0;
            A_(j * 3, 3) = 0;
            A_(j * 3, 4) = -coordinatesA[j][2];
            A_(j * 3, 5) = coordinatesA[j][1];
            A_(j * 3, 6) = coordinatesA[j][0];

            A_(j * 3 + 1, 0) = 0;
            A_(j * 3 + 1, 1) = 1;
            A_(j * 3 + 1, 2) = 0;
            A_(j * 3 + 1, 3) = coordinatesA[j][2];
            A_(j * 3 + 1, 4) = 0;
            A_(j * 3 + 1, 5) = -coordinatesA[j][0];
            A_(j * 3 + 1, 6) = coordinatesA[j][1];

            A_(j * 3 + 2, 0) = 0;
            A_(j * 3 + 2, 1) = 0;
            A_(j * 3 + 2, 2) = 1;
            A_(j * 3 + 2, 3) = -coordinatesA[j][1];
            A_(j * 3 + 2, 4) = coordinatesA[j][0];
            A_(j * 3 + 2, 5) = 0;
            A_(j * 3 + 2, 6) = coordinatesA[j][2];

            //b Vector
            b_(j * 3, 0) = coordinatesB[j][0];
            b_(j * 3 + 1, 0) = coordinatesB[j][1];
            b_(j * 3 + 2, 0) = coordinatesB[j][2];
        }
        transAB = ((A_.transpose() * A_).inverse()) * A_.transpose() * b_;

        transpara[0] = transAB(0);
        transpara[1] = transAB(1);
        transpara[2] = transAB(2);
        transpara[3] = transAB(3);
        transpara[4] = transAB(4);
        transpara[5] = transAB(5);
        transpara[6] = transAB(6);

        std::cout.setf(std::ios::showpoint); 
        std::cout.precision(10);            

        std::cout << "Estimated Transformation From A to B" << std::endl
                  << "tx: " << transpara[0] << " m" << std::endl
                  << "ty: " << transpara[1] << " m" << std::endl
                  << "tz: " << transpara[2] << " m" << std::endl
                  << "rx: " << transpara[3] << std::endl
                  << "ry: " << transpara[4] << std::endl
                  << "rz: " << transpara[5] << std::endl
                  << "scale: " << transpara[6] << std::endl;

        // Checking
        if (pointnumberA >= pointnumberB)
            pointnumbercheck = pointnumberB;
        else
            pointnumbercheck = pointnumberA;

        if (pointnumbercheck <= cp_number)
            std::cout << "Not enough points for check ..." << std::endl;
        else
        {
            pointnumbercheck -= cp_number;
            for (int j = 0; j < pointnumbercheck; j++)
            {
                double X_tran, Y_tran, Z_tran, squaredist;
                X_tran = transpara[0] + transpara[6] * coordinatesA[j + cp_number][0] +
                         transpara[5] * coordinatesA[j + cp_number][1] - transpara[4] * coordinatesA[j + cp_number][2];
                Y_tran = transpara[1] + transpara[6] * coordinatesA[j + cp_number][1] -
                         transpara[5] * coordinatesA[j + cp_number][0] + transpara[3] * coordinatesA[j + cp_number][2];
                Z_tran = transpara[2] + transpara[6] * coordinatesA[j + cp_number][2] +
                         transpara[4] * coordinatesA[j + cp_number][0] - transpara[3] * coordinatesA[j + cp_number][1];
                squaredist = (X_tran - coordinatesB[j + cp_number][0]) * (X_tran - coordinatesB[j + cp_number][0]) +
                             (Y_tran - coordinatesB[j + cp_number][1]) * (Y_tran - coordinatesB[j + cp_number][1]) +
                             (Z_tran - coordinatesB[j + cp_number][2]) * (Z_tran - coordinatesB[j + cp_number][2]);
                sum_squaredist += squaredist;
            }

            RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

            std::cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << std::endl;
        }

        return 1;
    }

    void Perturbation(const typename pcl::PointCloud<PointT>::Ptr &Cloud,
                      typename pcl::PointCloud<PointT>::Ptr &CloudAfterPerturbation, float pertubate_value,
                      std::vector<float> &pertubate_vector)
    {
        pertubate_vector.resize(3);
        pertubate_vector[0] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);
        pertubate_vector[1] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);
        pertubate_vector[2] = 0.5 * pertubate_value - pertubate_value * ((double)rand() / RAND_MAX);

        for (size_t i = 0; i < Cloud->size(); i++)
        {
            PointT pt;
            pt.x = Cloud->points[i].x + pertubate_vector[0];
            pt.y = Cloud->points[i].y + pertubate_vector[1];
            pt.z = Cloud->points[i].z + pertubate_vector[2];
            CloudAfterPerturbation->push_back(pt);
        }
        LOG(INFO) << "The perturbation vector is:  X " << pertubate_vector[0] << " , Y " << pertubate_vector[1]
                  << " , Z " << pertubate_vector[2];
    }

    
protected:

private:

};
} // namespace lls_loam

#endif // _INCLUDE_COMMON_REG_H_