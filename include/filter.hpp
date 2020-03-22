//
// This file is for the common implement of various basic point cloud filtering and segmentation methods
// It would cover the downsampling, ground filter and feature point detection functions
// Dependent 3rd Libs: PCL (>=1.7) , Eigen     
//

#ifndef _INCLUDE_FILTER_H_
#define _INCLUDE_FILTER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/normal_space.h>

#include <vector>
#include <limits>
#include <iostream>

#include "types.h"
#include "map_viewer.hpp"
#include "pca.hpp"

namespace lls_loam
{

template <typename PointT>
class CFilter //: public CloudUtility<PointT>
{
public:
    CFilter() {} //Constructor

    ~CFilter() {} //Destructor

    struct idpair_t
    { // ID pair for voxel downsampling
        idpair_t() : idx(0), voxel_idx(0) {}

        unsigned long long voxel_idx;
        unsigned int idx;

        bool operator<(const idpair_t &pair) { return voxel_idx < pair.voxel_idx; }
    };

    struct grid_t
    { // Grid for ground filter
        std::vector<int> point_id;
        float min_z;
        float max_z;
        float dertaz;
        float min_z_x; //X of Lowest Point in the Voxel;
        float min_z_y; //Y of Lowest Point in the Voxel;
        float NeighborMin_z;
        int PointsNumber;
        float mean_z;
        bool is_curb;
        bool is_road;

        grid_t()
        {
            min_z =  0.f;
            min_z_x = 0.f;
            min_z_y = 0.f;
            NeighborMin_z = 0.f;
            mean_z = 0.f;
            PointsNumber = 0;
            dertaz = 0.0;
            is_curb = 0;
            is_road = 0;
        }
    };

    struct simplifiedvoxel_t
    {
        std::vector<int> point_id;
        float max_curvature;
        int max_curvature_point_id;
        bool has_keypoint;

        simplifiedvoxel_t()
        {
            has_keypoint = false;
        }
    };

    struct voxel_t
    {
        // Voxel for downsample
        float center_point_x;
        float center_point_y;
        float center_point_z;
        float min_distance_square; // min distance between point in Voxel and the center of the Voxel
        int point_id;
    };

    // Voxel Downsampling of the point cloud
    // Keep only one point (which is the closest point to the voxel center) in a voxel
    bool VoxelDownsample(const std::shared_ptr<point_cloud_t<PointT>> cloud_in,
                         std::shared_ptr<point_cloud_t<PointT>> cloud_out, float voxel_size) const
    {
        clock_t t1, t2;
        t1 = clock();

        float inverse_voxel_size = 1.0f / voxel_size;

        Eigen::Vector4f min_p, max_p;
        bounds_t bounds;
        getCloudBound(*cloud_in, bounds);
        min_p << bounds.min_x, bounds.min_y, bounds.min_z, 0;
        max_p << bounds.max_x, bounds.max_y, bounds.max_z, 0;

        Eigen::Vector4f gap_p; //boundingbox gap;
        gap_p = max_p - min_p;

        unsigned long long max_vx = ceil(gap_p.coeff(0) * inverse_voxel_size);
        unsigned long long max_vy = ceil(gap_p.coeff(1) * inverse_voxel_size);
        unsigned long long max_vz = ceil(gap_p.coeff(2) * inverse_voxel_size);

        if (max_vx * max_vy * max_vz >= std::numeric_limits<unsigned long long>::max())
        {
            std::cout << "Filtering Failed: The number of box exceed the limit." << std::endl;
            return 0;
        }

        unsigned long long mul_vx = max_vy * max_vz;
        unsigned long long mul_vy = max_vz;
        unsigned long long mul_vz = 1;

        std::vector<voxel_t> voxels(max_vx * max_vy * max_vz);
        std::map<unsigned long long, int> voxels_has_point;

        /*//!< Keep Only The First Point In Voxel For Quick Downsample
        // Want Quicker? use unordered map
        std::vector<IDPair> id_pairs(voxels.size());
        for (int i = 0; i < cloud_in->points.size(); ++i)
        {
            if (voxels_has_point.size() == voxels.size()) {
                break;
            }

            unsigned long long vx = floor((cloud_in->points[i].x - min_p.coeff(0)) * inverse_voxel_size);
            unsigned long long vy = floor((cloud_in->points[i].y - min_p.coeff(1)) * inverse_voxel_size);
            unsigned long long vz = floor((cloud_in->points[i].z - min_p.coeff(2)) * inverse_voxel_size);

            unsigned long long voxel_idx = vx * mul_vx + vy * mul_vy + vz * mul_vz;

            if (voxels_has_point.find(voxel_idx) == voxels_has_point.end()) {
                voxels_has_point.insert({voxel_idx, i});
            }
        }

        cloud_out->points.clear();
        for (auto voxel_itr = voxels_has_point.cbegin(); voxel_itr != voxels_has_point.cend(); ++voxel_itr) {
            cloud_out->points.push_back(cloud_in->points[voxel_itr->second]);
        }

        t1 = clock();
        LOG(INFO) << "Quick [" << cloud_out->points.size() << " | " << cloud_in->points.size() << "] in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";*/

        //!< Keep the point which is the cloest to voxel center point
        // for each voxel, calculate its center point
        for (int i = 0; i < voxels.size(); ++i)
        {
            unsigned long long vx = i / mul_vx;
            unsigned long long vyvz = i % mul_vx;
            unsigned long long vy = vyvz / mul_vy;
            unsigned long long vz = vyvz % mul_vy;

            voxels[i].center_point_x = vx * voxel_size + 0.5 * voxel_size;
            voxels[i].center_point_y = vy * voxel_size + 0.5 * voxel_size;
            voxels[i].center_point_z = vz * voxel_size + 0.5 * voxel_size;
            voxels[i].min_distance_square = FLT_MAX;
            voxels[i].point_id = -1;
        }

        // for each point, update corresponding voxel
        for (int i = 0; i < cloud_in->points.size(); ++i)
        {
            unsigned long long vx = floor((cloud_in->points[i].x - min_p.coeff(0)) * inverse_voxel_size);
            unsigned long long vy = floor((cloud_in->points[i].y - min_p.coeff(1)) * inverse_voxel_size);
            unsigned long long vz = floor((cloud_in->points[i].z - min_p.coeff(2)) * inverse_voxel_size);

            unsigned long long voxel_idx = vx * mul_vx + vy * mul_vy + vz * mul_vz;
            float distance = (cloud_in->points[i].x - voxels[voxel_idx].center_point_x) * (cloud_in->points[i].x - voxels[voxel_idx].center_point_x) +
                             (cloud_in->points[i].y - voxels[voxel_idx].center_point_y) * (cloud_in->points[i].y - voxels[voxel_idx].center_point_y) +
                             (cloud_in->points[i].z - voxels[voxel_idx].center_point_z) * (cloud_in->points[i].z - voxels[voxel_idx].center_point_z);
            if (voxels[voxel_idx].min_distance_square > distance)
            {
                voxels[voxel_idx].min_distance_square = distance;
                voxels[voxel_idx].point_id = i;
            }
        }

        cloud_out->points.clear();
        for (int i = 0; i < voxels.size(); ++i)
        {
            if (voxels[i].point_id != -1)
            {
                cloud_out->points.push_back(cloud_in->points[voxels[i].point_id]);
            }
        }

        t2 = clock();
        LOG(INFO) << "Voxel downsampling done [" << cloud_out->points.size() << " | " << cloud_in->points.size() << "] in " << (float(t2 - t1) / CLOCKS_PER_SEC * 1000) << "ms";
        return 1;
    }

    // Random Downsampling of the point cloud (high intensity points would be kept ignoring the downsample)
    bool RandomDownsample(const std::shared_ptr<point_cloud_t<PointT>> cloud_in,
                          std::shared_ptr<point_cloud_t<PointT>> cloud_out, 
                          int downsample_step_low_intensity, int downsample_step_high_intensity, float intensity_thre) const {
        clock_t t0, t1;
        t0 = clock();

        cloud_out->points.clear();

        for (size_t i = 0; i < cloud_in->points.size(); i++)
        {
            if (cloud_in->points[i].intensity > intensity_thre)
            {
                if (i % downsample_step_high_intensity == 0)
                    cloud_out->points.push_back(cloud_in->points[i]);
            }
            else
            {
                if (i % downsample_step_low_intensity == 0)
                    cloud_out->points.push_back(cloud_in->points[i]);
            }
        }

        t1 = clock();
        //LOG(INFO) << "Random downsampling done in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";
        return 1;
    }

    // TODO [pcl points' random downsampling]
    bool RandomDownsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                          typename pcl::PointCloud<PointT>::Ptr &cloud_out, int downsample_ratio)
    {
        cloud_out->points.clear();

        for (size_t i = 0; i < cloud_in->points.size(); i++)
        {

            if (i % downsample_ratio == 0)
                cloud_out->points.push_back(cloud_in->points[i]);
        }
    }
// Normal Space downsampling by pcl
#if 0
        //Normal estimation by Radius Neighbor Search
        bool NormalDownsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                              typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                              float neighbor_radius, float downsample_ratio) {
            clock_t t0, t1;
            t0 = clock();

            pcl::PointCloud<pcl::PointXYZ>::Ptr CloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            copyPointCloud(*cloud_in, *CloudXYZ);
            // In this case, The Cloud's Normal hasn't been calculated yet.

            pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal(new pcl::PointCloud<pcl::PointNormal>());
            pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal_out(new pcl::PointCloud<pcl::PointNormal>());

            //Estimate Normal Multi-thread
            PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator;

            //Radius search
            pca_estimator.CalculatePointCloudWithNormal_Radius(CloudXYZ, neighbor_radius, CloudNormal);

            pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> nss;
            nss.setInputCloud(CloudNormal);
            nss.setNormals(CloudNormal);
            nss.setBins(4, 4, 4);
            nss.setSeed(0);
            nss.setSample(static_cast<unsigned int> (downsample_ratio * CloudNormal->size()));

            nss.filter(*CloudNormal_out);
            copyPointCloud(*CloudNormal_out, *cloud_out);

            t1 = clock();
            std::cout << "Normal space downsampling done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << std::endl;
            return 1;
        }

        //Normal estimation by KNN Neighbor Search
        bool NormalDownsample(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                              typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                              int K, float downsample_ratio) {
            clock_t t0, t1;
            t0 = clock();

            pcl::PointCloud<pcl::PointXYZ>::Ptr CloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            copyPointCloud(*cloud_in, *CloudXYZ);
            // In this case, The Cloud's Normal hasn't been calculated yet.

            pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal(new pcl::PointCloud<pcl::PointNormal>());
            pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormal_out(new pcl::PointCloud<pcl::PointNormal>());

            //Estimate Normal Multi-thread
            PrincipleComponentAnalysis<pcl::PointXYZ> pca_estimator;

            //K search
            pca_estimator.CalculatePointCloudWithNormal_KNN(CloudXYZ, K, CloudNormal);

            pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> nss;
            nss.setInputCloud(CloudNormal);
            nss.setNormals(CloudNormal);
            nss.setBins(4, 4, 4);
            nss.setSeed(0);
            nss.setSample(static_cast<unsigned int> (downsample_ratio * CloudNormal->size()));

            nss.filter(*CloudNormal_out);
            copyPointCloud(*CloudNormal_out, *cloud_out);

            t1 = clock();
            std::cout << "Normal space downsampling done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << std::endl;
            return 1;
        }
#endif

// Statistic Outlier Removing (SOR) filter by pcl
#if 0
    bool SORFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                       typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                       int MeanK, double std) {
            // Create the filtering object
            pcl::StatisticalOutlierRemoval<PointT> sor;

            sor.setInputCloud(cloud_in);
            sor.setMeanK(MeanK);         //50
            sor.setStddevMulThresh(std); //2.0
            sor.filter(*cloud_out);

            return 1;
        }
#endif

    void FilterCldWithIdx(const std::shared_ptr<point_cloud_t<PointType>> cld_ori_ptr,
                          std::shared_ptr<point_cloud_t<PointType>> cld_filtered_ptr,
                          const std::vector<unsigned int> &indexes) const
    {
        cld_filtered_ptr->points.resize(indexes.size());
        for (size_t i = 0; i < indexes.size(); ++i)
        {
            cld_filtered_ptr->points[i] = cld_ori_ptr->points[indexes[i]];
        }
    }

    // Filter the point cloud using distance threshold
    bool DisFilter(const std::shared_ptr<point_cloud_t<PointT>> cloud_in,
                   std::shared_ptr<point_cloud_t<PointT>> cloud_out,
                   double xy_dis_max, double z_min, double z_max) const
    {
        clock_t t0, t1;
        t0 = clock();
        double dis_square;
        for (size_t i = 0; i < cloud_in->points.size(); i++)
        {
            dis_square = cloud_in->points[i].x * cloud_in->points[i].x +
                         cloud_in->points[i].y * cloud_in->points[i].y;
            if (dis_square < xy_dis_max * xy_dis_max && cloud_in->points[i].z < z_max &&
                cloud_in->points[i].z > z_min)
            {
                cloud_out->points.push_back(cloud_in->points[i]);
            }
        }
        t1 = clock();
        //        LOG(INFO) << "DisFilter: " << cloud_out->points.size() << " points are extracted in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms" << std::endl;
        return 1;
    }

    // Filter the point cloud using a given bounding box
    bool BoxFilter(const std::shared_ptr<point_cloud_t<PointT>> cloud_in,
                   std::shared_ptr<point_cloud_t<PointT>> cloud_out,
                   const bounds_t &interested_box) const
    {
        clock_t t0, t1;
        t0 = clock();

        for (size_t i = 0; i < cloud_in->points.size(); i++)
        {

            if (cloud_in->points[i].z < interested_box.max_z && cloud_in->points[i].z > interested_box.min_z &&
                cloud_in->points[i].x < interested_box.max_x && cloud_in->points[i].x > interested_box.min_x &&
                cloud_in->points[i].y < interested_box.max_y && cloud_in->points[i].y > interested_box.min_y)

            {
                cloud_out->points.push_back(cloud_in->points[i]);
            }
        }
        t1 = clock();
        //        LOG(INFO) << "BoxFilter: " << cloud_out->points.size() << " points are extracted in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms" << std::endl;
        return 1;
    }
 
    // Remove detected active objects saving in the bounding box (given by deep learning based prediction)
    bool ActiveObjectFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                            typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                            std::vector<bounds_t> &active_bbxs)
    {
        std::vector<bool> is_static(cloud_in->points.size(), 1);
        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            for (int j = 0; j < active_bbxs.size(); j++)
            {
                // In the bounding box
                if (cloud_in->points[i].x > active_bbxs[j].min_x && cloud_in->points[i].x < active_bbxs[j].max_x &&
                    cloud_in->points[i].y > active_bbxs[j].min_y && cloud_in->points[i].y < active_bbxs[j].max_y &&
                    cloud_in->points[i].z > active_bbxs[j].min_z && cloud_in->points[i].z < active_bbxs[j].max_z)
                {
                    is_static[i] = 0;
                    break;
                }
            }
            if (is_static[i])
                cloud_out->points.push_back(cloud_in->points[i]);
        }

        return 1;
    }

    // Two threshold Fast Ground filter
    // Two-step adaptive Ground Filter
    // Reference: Two-step adaptive extraction method for ground points and breaklines from lidar point clouds, Bisheng Yang, Ronggang Huang, et al. ISPRS Journal of Photogrammetry and Remote Sensing
    // 1.Construct 2D grid
    // 2.Calculate the Minimum Z value in each grid
    // 3.For each grid, if its 8 neighbor grids' Minimum Z value is less than current grid's Minimum Z minus threshold1, then all the points in current grid would be seen as unground points
    // 4.Or, points whose Z value is larger than grid's Minimum Z plus threshold2 would be regarded as unground points. The rest points in the grid would be ground points.
    // (Estimate Ground Points' normal at the same time)
    bool FastGroundFilter(std::shared_ptr<point_cloud_t<PointT>> &cloud_in,
                          std::vector<unsigned int> &ground_idx, std::vector<unsigned int> &ground_down_idx, std::vector<unsigned int> &unground_idx,
                          const int min_grid_num, const float grid_resolution, const float max_height_difference,
                          const float neighbor_height_diff, const float max_ground_height,
                          const int grid_ground_num_first, const int grid_ground_num_second,
                          const int nonground_random_downsample_rate) const {
        clock_t t0, t1, t2;
        t0 = clock();

        static float vehicle_lidar_height = 1.8;

        // Parameters
        float curb_delta_z_max, curb_delta_z_min, curb_z_min, curb_z_min_min;
        curb_delta_z_max = 6.0;
        curb_delta_z_min = 0.08;
        curb_z_min = 0.35 - vehicle_lidar_height;
        curb_z_min_min = -0.75 - vehicle_lidar_height;

        bounds_t bounds;
        getCloudBound(*cloud_in, bounds);

        // Construct Grid
        int row, col, center_col, num_grid;
        row = ceil((bounds.max_y - bounds.min_y) / grid_resolution);
        col = ceil((bounds.max_x - bounds.min_x) / grid_resolution);
        center_col = ceil(-bounds.min_x / grid_resolution);
        num_grid = row * col;

        grid_t *grid = new grid_t[num_grid];

        // Each grid
        for (int i = 0; i < num_grid; i++)
        {
            grid[i].min_z = FLT_MAX;
            grid[i].NeighborMin_z = FLT_MAX;
            grid[i].max_z = -FLT_MAX;
        }

        // Each point
        for (int j = 0; j < cloud_in->points.size(); j++)
        {
            int temp_row, temp_col, temp_id;
            temp_col = floor((cloud_in->points[j].x - bounds.min_x) / grid_resolution);
            temp_row = floor((cloud_in->points[j].y - bounds.min_y) / grid_resolution);
            temp_id = temp_row * col + temp_col;
            if (temp_id >= 0 && temp_id < num_grid)
            {
                grid[temp_id].PointsNumber++;
                if (cloud_in->points[j].z > max_ground_height)
                {
                    unground_idx.push_back(j);
                }
                else
                {
                    grid[temp_id].point_id.push_back(j);
                    if (cloud_in->points[j].z < grid[temp_id].min_z)
                    {
                        grid[temp_id].min_z = cloud_in->points[j].z;
                        grid[temp_id].NeighborMin_z = cloud_in->points[j].z;
                    }
                    if (cloud_in->points[j].z > grid[temp_id].max_z)
                    {
                        grid[temp_id].max_z = cloud_in->points[j].z;
                    }
                }
            }
        }

        // Each grid
        for (int i = 0; i < num_grid; i++)
        {
            int temp_row, temp_col;
            temp_row = i / col;
            temp_col = i % col;
            if (temp_row >= 1 && temp_row <= row - 2 && temp_col >= 1 && temp_col <= col - 2)
            {
                for (int j = -1; j <= 1; j++) //row
                {
                    for (int k = -1; k <= 1; k++) //col
                    {
                        if (grid[i].NeighborMin_z > grid[i + j * col + k].min_z)
                        {
                            grid[i].NeighborMin_z = grid[i + j * col + k].min_z;
                        }
                    }
                }
            }
        }
        // Curb Detection and Road grids Determining 
#if 0
        // Judge the road grids
        int left_curb_memory = -1;
        int right_curb_memory = -1;
        int row_memory = -1;

        for (int i = 1; i < row - 1; i++) {
            int left_curb = -1;  //Not found
            int right_curb = -1; //Not found

            for (int j = 1; j < col - 1; j++) {

                // Judge Road curb grids
                if (grid[i * col + j].max_z - grid[i * col + j].min_z < curb_delta_z_max &&
                    grid[i * col + j].max_z - grid[i * col + j].min_z > curb_delta_z_min &&
                    grid[i * col + j].min_z < curb_z_min && grid[i * col + j].min_z > curb_z_min_min) {
            
                    grid[i * col + j].is_curb = 1;
                    if (j < center_col) {
                        left_curb = j;
                    }
                    else {        
                        right_curb = j;
                        break;
                    }
                }
                if (j - center_col <= 2 || j - center_col >= -2) //Narrow area on the road's vehicle would be regarded as road points
                    grid[i * col + j].is_road = 1;
            }
            // Both found
            if (left_curb >= 0 && right_curb >= 0) {
                if ((left_curb_memory - left_curb <= 1 || left_curb_memory - left_curb >= -1) &&
                   (right_curb_memory - right_curb <= 1 || right_curb_memory - right_curb >= -1)) {    
                    for (int kk = row_memory + 1; kk <= i; kk++) {
                        for (int j = max_(left_curb, left_curb_memory) + 1; j < min_(right_curb, right_curb_memory); j++) { 
                            grid[kk * col + j].is_road = 1;
                        }
                    }
                }
                else {
                    for (int j = left_curb + 1; j < right_curb; j++) //Those between the curbs would be regard as road {  
                        grid[i * col + j].is_road = 1;
                    }
                }
                left_curb_memory = left_curb;
                right_curb_memory = right_curb;
                row_memory = i;
            }
        }

#endif
        // Each grid
        for (int i = 0; i < num_grid; i++) {
            // Filtering some grids with too little points (the farther points)
            if (grid[i].PointsNumber >= min_grid_num) {
                int ground_downsample_rate_first = grid[i].PointsNumber / grid_ground_num_first;
                int ground_downsample_rate_second = grid[i].PointsNumber / grid_ground_num_second;
                if (grid[i].min_z - grid[i].NeighborMin_z < neighbor_height_diff) {   
                    for (int j = 0; j < grid[i].point_id.size(); j++) {
                        // Add to ground points and random downsample
                        if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference) {
                            // for example 5 - Downsample
                            if (j % ground_downsample_rate_first == 0) {
                #if 0
                                // Rough Estimate ground point normal
                                cloud_in->points[grid[i].point_id[j]].nx = 0.0;
                                cloud_in->points[grid[i].point_id[j]].ny = 0.0;
                                cloud_in->points[grid[i].point_id[j]].nz = 1.0;
                #endif
                                ground_idx.push_back(grid[i].point_id[j]); // Add to ground points

                                // Then downsample again for more flat ground points
                                if (j % ground_downsample_rate_second == 0) {
                                    ground_down_idx.push_back(grid[i].point_id[j]); // Add to ground_down points
                                }
                            }
                        }
                        // Add to nonground points (you could also do some random downsampling here)
                        else { 
                            //if (!grid[i].is_road && j % nonground_random_downsample_rate == 0)
                            if (j % nonground_random_downsample_rate == 0)
                                unground_idx.push_back(grid[i].point_id[j]);
                        }
                    }
                }
                // Add to nonground points (you could also do some random downsampling here) [This is the nonground grid]
                else {     
                    for (int j = 0; j < grid[i].point_id.size(); j++) {
                        //if (!grid[i].is_road && j % (nonground_random_downsample_rate + 1) == 0) //Downsample more points.
                        if (j % (nonground_random_downsample_rate + 1) == 0)
                            unground_idx.push_back(grid[i].point_id[j]);
                    }
                }
            }
        }

#if 0   // For displaying the result of curb detection
        std::shared_ptr<point_cloud_t<PointT>> curb_cloud_ptr(new point_cloud_t<PointT>);
        std::shared_ptr<point_cloud_t<PointT>> road_cloud_ptr(new point_cloud_t<PointT>);
        std::shared_ptr<point_cloud_t<PointT>> other_cloud_ptr(new point_cloud_t<PointT>);

        for (int i = 0; i < num_grid; i++)
        {
            //if (grid[i].is_curb)
            if (grid[i].is_road)
            {
                for (int j = 0; j < grid[i].point_id.size(); j++)
                {
                    road_cloud_ptr->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                }
            }
            // if (grid[i].is_curb)
            // {
            //     for (int j = 0; j < grid[i].point_id.size(); j++)
            //     {
            //         curb_cloud_ptr->points.push_back(cloud_in->points[grid[i].point_id[j]]);
            //     }
            // }
            else
            {
                for (int j = 0; j < grid[i].point_id.size(); j++)
                {
                    other_cloud_ptr->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                }
            }
        }
        static int kkk = 0;
        MapViewer<PointType> viewer;
        if (kkk % 20 == 0)
            viewer.Dispaly2Clouds(road_cloud_ptr, other_cloud_ptr, "Curb Detection", 1);

        kkk++;
#endif

        delete[] grid;

        t1 = clock();

#if 1
        //Use pcl normal estimator to estimate ground points' normal instead of directly using (0,0,1)
        PrincipleComponentAnalysis<pcl::PointNormal> pca_estimator;
        pca_estimator.CalculateNormalVector_KNN(cloud_in, ground_idx, 10);
#endif

        t2 = clock();

        LOG(INFO) << "Ground [" << ground_down_idx.size() << " | " << ground_idx.size() << "] UnGround [" << unground_idx.size() << "]";
        LOG(INFO) << "Ground Filter done in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";
        LOG(INFO) << "Ground Normal Estimation done in " << (float(t2 - t1) / CLOCKS_PER_SEC * 1000) << "ms";

        return 1;
    }

    // Classify unground points in a frame into Edge, Facade Planar and Sharp Sphere points using point neighbor PCA feature
    // Estimate unground points normal at the same time
    // TODO: Speed up
    bool RoughClassify(std::shared_ptr<point_cloud_t<PointT>> &cloud_in,
                       std::vector<unsigned int> &edge_source_idx, std::vector<unsigned int> &edge_target_idx,
                       std::vector<unsigned int> &planar_source_idx, std::vector<unsigned int> &planar_target_idx,
                       std::vector<unsigned int> &sphere_source_idx, std::vector<unsigned int> &sphere_target_idx,
                       const std::vector<unsigned int> &unground_idx,
                       const int neighbor_K_PCA,
                       const float neighbor_linear_thre_target, const float neighbor_planar_thre_target, const float neighbor_curvature_thre_target,
                       const float neighbor_linear_thre_source, const float neighbor_planar_thre_source, const float neighbor_curvature_thre_source,
                       const float linear_vertical_cosine_min, const float planar_horizontal_cosine_max,
                       const int edge_point_source_appro_num, const int planar_point_source_appro_num, const int sphere_point_source_appro_num)
    {
        clock_t t0, t1;
        t0 = clock();

        //Estimate Normal Multi-thread
        //Fix it (Remove PCL dependence)
        PrincipleComponentAnalysis<pcl::PointNormal> pca_estimator;
        std::vector<pcaFeature> unground_features;

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_unground_ptr(new pcl::PointCloud<pcl::PointNormal>);

        //Type Transformation (From Ours to pcl)
        for (int i = 0; i < unground_idx.size(); i++)
        {
            pcl::PointNormal pt;
            pt.x = cloud_in->points[unground_idx[i]].x;
            pt.y = cloud_in->points[unground_idx[i]].y;
            pt.z = cloud_in->points[unground_idx[i]].z;
            cloud_unground_ptr->points.push_back(pt);
        }

        //Get PCA feature
        pca_estimator.CalculatePcaFeaturesOfPointCloud(cloud_unground_ptr, unground_features, neighbor_K_PCA);

        //Free memory
        cloud_unground_ptr = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

        //Assign Normal Value for all the unground points
        for (int i = 0; i < unground_idx.size(); i++)
        {
            cloud_in->points[unground_idx[i]].nx = unground_features[i].vectors.normalDirection.x();
            cloud_in->points[unground_idx[i]].ny = unground_features[i].vectors.normalDirection.y();
            cloud_in->points[unground_idx[i]].nz = unground_features[i].vectors.normalDirection.z();
        }

        std::vector<std::pair<float, int>> points_linear;
        std::vector<std::pair<float, int>> points_planar;
        std::vector<std::pair<float, int>> points_sphere;
        

        for (int i = 0; i < unground_features.size(); i++)
        {
            if (unground_features[i].linear_2 > neighbor_linear_thre_target && std::abs(unground_features[i].vectors.principalDirection.z()) > linear_vertical_cosine_min) //Vertical Line
            {
                std::pair<float, int> point_linear;
                point_linear.first = unground_features[i].linear_2;
                point_linear.second = unground_idx[i];
                points_linear.push_back(point_linear);
            }
            else if (unground_features[i].planar_2 > neighbor_planar_thre_target && std::abs(unground_features[i].vectors.normalDirection.z()) < planar_horizontal_cosine_max) //Not the horizontal plane
            {
                std::pair<float, int> point_planar;
                point_planar.first = unground_features[i].planar_2;
                point_planar.second = unground_idx[i];
                points_planar.push_back(point_planar);
            }
            else if (unground_features[i].curvature > neighbor_curvature_thre_target)
            {
                bool local_maximum = true;
                for (int j = 0; j < unground_features[i].neighbor_indices.size(); j++)
                {
                    if (unground_features[i].curvature < unground_features[unground_features[i].neighbor_indices[j]].curvature)
                    {
                        local_maximum = false;
                        //LOG(INFO)<<"Not Local Maximia";
                        break;
                    }
                }
                if (local_maximum)
                {
                    std::pair<float, int> point_sphere;
                    point_sphere.first = unground_features[i].curvature;
                    point_sphere.second = unground_idx[i];
                    points_sphere.push_back(point_sphere);
                }
            }

            // else if (unground_features[i].spherical_2 > sphere_thre)
            // {
            //     sphere_idx.push_back(unground_idx[i]);
            //     if (unground_features[i].spherical_2 > sphere_thre_down)
            //     {
            //         sphere_down_idx.push_back(unground_idx[i]);
            //     }
            // }
            //}
        }

        // Sort them (we select the first k points as our feature points)
        sort(points_linear.begin(), points_linear.end(), compare_pair_first);
        sort(points_planar.begin(), points_planar.end(), compare_pair_first);
        sort(points_sphere.begin(), points_sphere.end(), compare_pair_first);

        // Get the feature points' indices

        for (int i = 0; i < points_linear.size(); i++)
        {
            if (i < edge_point_source_appro_num || points_linear[i].first > neighbor_linear_thre_source)
            {
                edge_source_idx.push_back(points_linear[i].second);
            }

            edge_target_idx.push_back(points_linear[i].second);
        }

        for (int i = 0; i < points_planar.size(); i++)
        {
            if (i < planar_point_source_appro_num || points_planar[i].first > neighbor_planar_thre_source)
            {
                planar_source_idx.push_back(points_planar[i].second);
            }

            planar_target_idx.push_back(points_planar[i].second);
        }

        for (int i = 0; i < points_sphere.size(); i++)
        {
            if (i < sphere_point_source_appro_num || points_sphere[i].first > neighbor_curvature_thre_source)
            {
                sphere_source_idx.push_back(points_sphere[i].second);
            }

            sphere_target_idx.push_back(points_sphere[i].second);
        }

        //Free the memory
        std::vector<pcaFeature>().swap(unground_features);
        std::vector<std::pair<float, int>>().swap(points_linear);
        std::vector<std::pair<float, int>>().swap(points_planar);
        std::vector<std::pair<float, int>>().swap(points_sphere);

        t1 = clock();
        LOG(INFO) << "Edge [" << edge_source_idx.size() << " | " << edge_target_idx.size() << "] Planar [" << planar_source_idx.size() << " | "
                  << planar_target_idx.size() << "] Sphere [" << sphere_source_idx.size() << " | " << sphere_target_idx.size() << "]";
        LOG(INFO) << "Feature points extracted done in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";

        return 1;
    }

    static bool compare_pair_first(const std::pair<float, int> a, const std::pair<float, int> b) // sort from big to small
    {
        return a.first > b.first;
    }
    
    
    // Classify the landmarks (traffic signs)
    bool LandmarkClassify(std::shared_ptr<point_cloud_t<PointT>> &cloud_in,
                          std::vector<std::shared_ptr<point_cloud_t<PointT>>> &landmarks,
                          int intensity, double high_threshold, int min_cluster_size, int max_cluster_size, double cluster_tolerance) const
    {
        landmarks.clear();
        std::shared_ptr<point_cloud_t<PointT>> landmark_cld = std::make_shared<point_cloud_t<PointT>>();
        for (int i = 0; i < cloud_in->points.size(); ++i)
        {
            if (cloud_in->points[i].intensity > intensity && cloud_in->points[i].z > high_threshold)
            {
                landmark_cld->points.push_back(cloud_in->points[i]);
            }
        }
        LOG(WARNING) << "Landmark Cloud " << landmark_cld->points.size() << " points";
        pcl::PointCloud<pcl::PointXYZI>::Ptr lamdmark_pcl = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        lamdmark_pcl->points.resize(landmark_cld->points.size());
        for (int i = 0; i < landmark_cld->points.size(); ++i)
        {
            lamdmark_pcl->points[i].x = landmark_cld->points[i].x;
            lamdmark_pcl->points[i].y = landmark_cld->points[i].y;
            lamdmark_pcl->points[i].z = landmark_cld->points[i].z;
            lamdmark_pcl->points[i].intensity = landmark_cld->points[i].intensity;
        }
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(lamdmark_pcl);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(lamdmark_pcl);
        ec.extract(cluster_indices);
        LOG(WARNING) << "Landmark Cluster Num " << cluster_indices.size();

        for (int i = 0; i < cluster_indices.size(); ++i)
        {
            std::shared_ptr<point_cloud_t<PointT>> extract_cld = std::make_shared<point_cloud_t<PointT>>();
            for (int j = 0; j < cluster_indices[i].indices.size(); ++j)
            {
                extract_cld->points.push_back(landmark_cld->points[cluster_indices[i].indices[j]]);
            }
            landmarks.push_back(extract_cld);
        }
        LOG(WARNING) << "Extract " << landmarks.size() << " landmarks";
#if 0
        // Visualization
        char ch[256];
        std::string str;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> pg_viewer(new pcl::visualization::PCLVisualizer("Landmark"));
        // Create two vertically separated viewports
//        int v1 (0);
//        int v2 (1);
//        pg_viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//        pg_viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//        // Set camera position and orientation
//        pg_viewer->setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//        pg_viewer->setSize (1280, 1024);  // Visualiser window size
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int i = 0; i < cloud_in->points.size(); ++i) {
            pcl::PointXYZRGB point_temp;
            point_temp.x = cloud_in->points[i].x;
            point_temp.y = cloud_in->points[i].y;
            point_temp.z = cloud_in->points[i].z;
            point_temp.r = 0;
            point_temp.g = cloud_in->points[i].intensity;
            point_temp.b = 0;
            pointcloud->points.push_back(point_temp);
        }
//        pg_viewer->addPointCloud(pointcloud, "full point cloud");
        pointcloud->points.clear();
        for (int i = 0; i < landmark_cld->points.size(); ++i) {
            pcl::PointXYZRGB point_temp;
            point_temp.x = landmark_cld->points[i].x;
            point_temp.y = landmark_cld->points[i].y;
            point_temp.z = landmark_cld->points[i].z;
            point_temp.r = 0;
            point_temp.g = landmark_cld->points[i].intensity;
            point_temp.b = landmark_cld->points[i].intensity;
            pointcloud->points.push_back(point_temp);
        }
        pg_viewer->addPointCloud(pointcloud, "high intensity point cloud");
        for (int i = 0; i < landmarks.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (int j = 0; j < landmarks[i]->points.size(); ++j) {
                pcl::PointXYZRGB point_temp;
                point_temp.x = landmarks[i]->points[j].x;
                point_temp.y = landmarks[i]->points[j].y;
                point_temp.z = landmarks[i]->points[j].z;
                point_temp.r = landmarks[i]->points[j].intensity;
                point_temp.g = 0;
                point_temp.b = 0;
                pointcloud->points.push_back(point_temp);
            }
            sprintf(ch, "landmark_%d", i);
            str = ch;
            pg_viewer->addPointCloud(pointcloud, str);
        }
        while (!pg_viewer->wasStopped())
        {
            pg_viewer->spinOnce(1);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
#endif
        return true;
    }

    // TODO
    bool Cluster(std::shared_ptr<point_cloud_t<PointT>> &cloud_in,
                 std::vector<std::shared_ptr<point_cloud_t<PointT>>> &clusters)
    {
        clusters.clear();
        std::deque<PointT> queue;
        std::set<PointT> checked_points;
        for (int i = 0; i < cloud_in->points.size(); ++i)
        {
        }
    }


    //Get Bound and Center of a Point Cloud
    void getBoundAndCenter(const point_cloud_t<PointT> &cloud,
                           bounds_t &bound, center_point_t &centerPoint) const
    {
        getCloudCenterPoint(cloud, centerPoint);
        getCloudBound(cloud, bound);
    }

    //Get Center of a Point Cloud
    void getCloudCenterPoint(const point_cloud_t<PointT> &cloud, center_point_t &centerPoint) const
    {
        double cx = 0, cy = 0, cz = 0;

        for (int i = 0; i < cloud.points.size(); i++)
        {
            cx += cloud.points[i].x;
            cy += cloud.points[i].y;
            cz += cloud.points[i].z;
        }
        cx /= cloud.points.size();
        cy /= cloud.points.size();
        cz /= cloud.points.size();

        centerPoint.x = cx;
        centerPoint.y = cy;
        centerPoint.z = cz;
    }

    //Get Bound of a Point Cloud
    void getCloudBound(const point_cloud_t<PointT> &cloud, bounds_t &bound) const
    {
        double min_x = cloud.points[0].x;
        double min_y = cloud.points[0].y;
        double min_z = cloud.points[0].z;
        double max_x = cloud.points[0].x;
        double max_y = cloud.points[0].y;
        double max_z = cloud.points[0].z;

        for (int i = 0; i < cloud.points.size(); i++)
        {
            if (min_x > cloud.points[i].x)
                min_x = cloud.points[i].x;
            if (min_y > cloud.points[i].y)
                min_y = cloud.points[i].y;
            if (min_z > cloud.points[i].z)
                min_z = cloud.points[i].z;
            if (max_x < cloud.points[i].x)
                max_x = cloud.points[i].x;
            if (max_y < cloud.points[i].y)
                max_y = cloud.points[i].y;
            if (max_z < cloud.points[i].z)
                max_z = cloud.points[i].z;
        }
        bound.min_x = min_x;
        bound.max_x = max_x;
        bound.min_y = min_y;
        bound.max_y = max_y;
        bound.min_z = min_z;
        bound.max_z = max_z;
    }

private:
    //Get Bound of Subsets of a Point Cloud
    void GetSubsetBoundary(std::shared_ptr<point_cloud_t<PointT>> cloud, std::vector<int> &index, bounds_t &bound)
    {
        std::shared_ptr<point_cloud_t<PointT>> temp_cloud(new point_cloud_t<PointT>);
        for (int i = 0; i < index.size(); i++)
        {
            temp_cloud->push_back(cloud->points[index[i]]);
        }
        getCloudBound(*temp_cloud, bound);
    }

};

} // namespace lls_loam
#endif //_INCLUDE_COMMON_FILTER_H_
