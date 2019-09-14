
#ifndef _INCLUDE_MAP_VIEWER_H
#define _INCLUDE_MAP_VIEWER_H

#include <string>
#include <fstream>
#include <vector>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <glog/logging.h>

#include "types.h"

namespace map_pose {

typedef pcl::PointXYZINormal PclPointType;

// Visualization color scale
enum color_type {
    INTENSITY,
    HEIGHT,
    SINGLE,
    FRAME
};

template <typename PointT>
class MapViewer {
public:
    //Constructor
    MapViewer(){};
    ~MapViewer(){};
    
    void DispalyPoses(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                       const std::vector<Pose3d, Eigen::aligned_allocator<Pose3d>> &poses,
                       std::string prefix, float red = 1.0, float green = 0.0, float blue = 0.0) const {
        float sphere_size = 1.5;
        // float line_width = 1.0;
        float red_v = red;
        float green_v = green;
        float blue_v = blue;
        char str_node[128];
        char str_direct[128];
        std::string node_name;

        for (int i = 0; i < poses.size(); ++i)
        {
            sprintf(str_node, "NODE_%03u", i);
            sprintf(str_direct, "DIRCT_%03u", i);
            pcl::PointXYZ point_center(poses[i].trans[0],
                                       poses[i].trans[1],
                                       poses[i].trans[2]);
            Eigen::Vector3d vehicle_head;
            vehicle_head << 0, 3, 0;
            vehicle_head = poses[i].quat * vehicle_head + poses[i].trans;
            pcl::PointXYZ point_direct(vehicle_head[0],
                                       vehicle_head[1],
                                       vehicle_head[2]);
            if (i == 0)
            {
                // Color the first Submap of each Transaction in White
                viewer->addSphere(point_center, sphere_size, 1, 1, 1, prefix + str_node);
            }
            else if (i == poses.size()-1)
            {
                // Color the last Submap of each Transaction in Red
                viewer->addSphere(point_center, sphere_size, 1, 0, 0, prefix + str_node);
            }
            else
            {
                viewer->addSphere(point_center, sphere_size, red_v, green_v, blue_v, prefix + str_node);
            }
            viewer->addLine(point_center, point_direct, red_v, green_v, blue_v, prefix + str_direct);
        }
    }

    void DispalySubmaps(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                         const MapOfSubMaps &map_of_submaps, std::string prefix,
                         float red = 1.0, float green = 0.0, float blue = 0.0) const
    {

        float sphere_size = 1.5;
        // float line_width = 1.0;
        float red_v = red;
        float green_v = green;
        float blue_v = blue;
        char str_node[128];
        char str_direct[128];
        std::string node_name;

        for (auto itr = map_of_submaps.cbegin(); itr != map_of_submaps.end(); ++itr)
        {
            sprintf(str_node, "NODE_%03u_%04u", itr->first.transaction_id, itr->first.submap_id);
            sprintf(str_direct, "DIRCT_%03u_%04u", itr->first.transaction_id, itr->first.submap_id);
            itr->second.pose.trans;
            itr->second.pose.quat;

            pcl::PointXYZ point_center(itr->second.pose.trans[0],
                                       itr->second.pose.trans[1],
                                       itr->second.pose.trans[2]);
            // y-axis = vehicle-direction (MENTION: not speed direction)
            Eigen::Vector3d vehicle_head;
            vehicle_head << 0, 3, 0;
            vehicle_head = itr->second.pose.quat * vehicle_head + itr->second.pose.trans;
            pcl::PointXYZ point_direct(vehicle_head[0],
                                       vehicle_head[1],
                                       vehicle_head[2]);
            if (itr->first.submap_id == 0)
            {
                // Color the first Submap of each Transaction in White, Because it is fixed point!
                viewer->addSphere(point_center, sphere_size, 1, 1, 1, prefix + str_node);
            }
            else
            {
                viewer->addSphere(point_center, sphere_size, red_v, green_v, blue_v, prefix + str_node);
            }
            viewer->addLine(point_center, point_direct, red_v, green_v, blue_v, prefix + str_direct);
            //            viewer->addArrow(point_center, point_direct, red_v, green_v, blue_v, false, prefix + str_direct);
        }
    }

    void DispalyEdges(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                       const MapOfSubMaps &map_of_submaps, const VectorOfEdges &vector_of_edges,
                       std::string prefix, float red = 1.0, float green = 0.0, float blue = 0.0) const
    {

        float red_v = red;
        float green_v = green;
        float blue_v = blue;
        char str_edge[128];

        for (auto itr = vector_of_edges.cbegin(); itr != vector_of_edges.end(); ++itr)
        {
            //            if (itr->edge_type != Edge::Adjacent) {
            //                continue;
            //            }
            sprintf(str_edge, "EDGE_%03u_%04u_%03u_%04u",
                    itr->submap_idx.first_submap.transaction_id, itr->submap_idx.first_submap.submap_id,
                    itr->submap_idx.second_submap.transaction_id, itr->submap_idx.second_submap.submap_id);

            auto first_submap_ptr = map_of_submaps.find(itr->submap_idx.first_submap);
            CHECK(first_submap_ptr != map_of_submaps.end()) << "Can Not Find First Submap!";
            auto second_submap_ptr = map_of_submaps.find(itr->submap_idx.second_submap);
            CHECK(second_submap_ptr != map_of_submaps.end()) << "Can Not Find Second Submap!";

            pcl::PointXYZ point_first(first_submap_ptr->second.pose.trans[0],
                                      first_submap_ptr->second.pose.trans[1],
                                      first_submap_ptr->second.pose.trans[2]);
            pcl::PointXYZ point_second(second_submap_ptr->second.pose.trans[0],
                                       second_submap_ptr->second.pose.trans[1],
                                       second_submap_ptr->second.pose.trans[2]);
            viewer->addLine(point_first, point_second, red_v, green_v, blue_v, prefix + str_edge);
        }
    }

    void Dispaly2CloudPcl(const pcl::PointCloud<PclPointType>::Ptr &Cloud1, const pcl::PointCloud<PclPointType>::Ptr &Cloud2, std::string displayname, int display_downsample_ratio)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        std::string s;
        int n = 0;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < Cloud1->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud1->points[i].x;
                pt.y = Cloud1->points[i].y;
                pt.z = Cloud1->points[i].z;
                pt.r = 255;
                pt.g = 215;
                pt.b = 0;
                pointcloud1->points.push_back(pt);
            }
        } // Golden

        viewer->addPointCloud(pointcloud1, "pointcloudT");

        for (size_t i = 0; i < Cloud2->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud2->points[i].x;
                pt.y = Cloud2->points[i].y;
                pt.z = Cloud2->points[i].z;
                pt.r = 233;
                pt.g = 233;
                pt.b = 216;
                pointcloud2->points.push_back(pt);
            }
        } // Silver

        viewer->addPointCloud(pointcloud2, "pointcloudS");

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    void Dispaly2Clouds(const std::shared_ptr<point_cloud_t<PointType>> Cloud1, const std::shared_ptr<point_cloud_t<PointType>> Cloud2, std::string displayname, int display_downsample_ratio)
    {

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        std::string s;
        int n = 0;

#if 1
        float line_size, sphere_size;
        line_size = 2.5;
        sphere_size = 0.4;

        pcl::PointXYZ ptc(0, 0, 0);
        sprintf(t, "%d", n);
        s = t;
        viewer->addSphere(ptc, sphere_size, 1.0, 1.0, 1.0, s);
        n++;

        pcl::PointXYZ point_x_1(line_size, 0, 0);
        pcl::PointXYZ point_y_1(0, line_size, 0);
        pcl::PointXYZ point_z_1(0, 0, line_size);

        sprintf(t, "line1_%d", n);
        s = t;
        viewer->addLine(ptc, point_x_1, 1.0, 0.0, 0.0, s);
        sprintf(t, "line2_%d", n);
        s = t;
        viewer->addLine(ptc, point_y_1, 0.0, 1.0, 0.0, s);
        sprintf(t, "line3_%d", n);
        s = t;
        viewer->addLine(ptc, point_z_1, 0.0, 0.0, 1.0, s);
        n++;
#endif

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < Cloud1->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud1->points[i].x;
                pt.y = Cloud1->points[i].y;
                pt.z = Cloud1->points[i].z;
                float intensity_ratio = 1.0 * Cloud1->points[i].intensity / 255;
                pt.r = 255 * intensity_ratio;
                pt.g = 215 * intensity_ratio;
                pt.b = 0 * intensity_ratio;
                pointcloud1->points.push_back(pt);
            }
        } // Golden

        viewer->addPointCloud(pointcloud1, "pointcloudT");

        for (size_t i = 0; i < Cloud2->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud2->points[i].x;
                pt.y = Cloud2->points[i].y;
                pt.z = Cloud2->points[i].z;
                float intensity_ratio = 1.0 * Cloud2->points[i].intensity / 255;
                pt.r = 233 * intensity_ratio;
                pt.g = 233 * intensity_ratio;
                pt.b = 216 * intensity_ratio;
                pointcloud2->points.push_back(pt);
            }
        } // Silver

        viewer->addPointCloud(pointcloud2, "pointcloudS");

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    void Dispaly2CloudsCompare(const std::shared_ptr<point_cloud_t<PointType>> Cloud1_left, const std::shared_ptr<point_cloud_t<PointType>> Cloud2_left,
                                const std::shared_ptr<point_cloud_t<PointType>> Cloud1_right, const std::shared_ptr<point_cloud_t<PointType>> Cloud2_right,
                                std::string displayname, int display_downsample_ratio)
    {

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        std::string s;
        int n = 0;

        //Create two vertically separated viewports
        int v1(0);
        int v2(1);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        // Set camera position and orientation

        // float x_position = clouds1[0]->points[0].x;
        // float y_position = clouds1[0]->points[0].y;
        // float z_position = clouds1[0]->points[0].z;

        // viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 0);
        // viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 1);
        viewer->setSize(1600, 900); // Visualiser window size

#if 1
        float line_size, sphere_size;
        line_size = 2.5;
        sphere_size = 0.4;

        pcl::PointXYZ ptc(0, 0, 0);
        sprintf(t, "%d", n);
        s = t;
        viewer->addSphere(ptc, sphere_size, 1.0, 1.0, 1.0, s);
        n++;

        pcl::PointXYZ point_x_1(line_size, 0, 0);
        pcl::PointXYZ point_y_1(0, line_size, 0);
        pcl::PointXYZ point_z_1(0, 0, line_size);

        sprintf(t, "line1_%d", n);
        s = t;
        viewer->addLine(ptc, point_x_1, 1.0, 0.0, 0.0, s);
        sprintf(t, "line2_%d", n);
        s = t;
        viewer->addLine(ptc, point_y_1, 0.0, 1.0, 0.0, s);
        sprintf(t, "line3_%d", n);
        s = t;
        viewer->addLine(ptc, point_z_1, 0.0, 0.0, 1.0, s);
        n++;
#endif

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1l(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2l(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < Cloud1_left->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud1_left->points[i].x;
                pt.y = Cloud1_left->points[i].y;
                pt.z = Cloud1_left->points[i].z;
                float intensity_ratio = 1.0 * Cloud1_left->points[i].intensity / 255;
                //float intensity_ratio_kitti = 1.0 * Cloud1_left->points[i].intensity;
                pt.r = 255 * intensity_ratio;
                pt.g = 215 * intensity_ratio;
                pt.b = 0 * intensity_ratio;
                pointcloud1l->points.push_back(pt);
            }
        } // Golden

        viewer->addPointCloud(pointcloud1l, "pointcloudT_left", v1);

        for (size_t i = 0; i < Cloud2_left->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud2_left->points[i].x;
                pt.y = Cloud2_left->points[i].y;
                pt.z = Cloud2_left->points[i].z;
                float intensity_ratio = 1.0 * Cloud2_left->points[i].intensity / 255;
                //float intensity_ratio_kitti = 1.0 * Cloud2_left->points[i].intensity;
                pt.r = 233 * intensity_ratio;
                pt.g = 233 * intensity_ratio;
                pt.b = 216 * intensity_ratio;
                pointcloud2l->points.push_back(pt);
            }
        } // Silver

        viewer->addPointCloud(pointcloud2l, "pointcloudS_left", v1);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1r(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2r(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < Cloud1_right->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud1_right->points[i].x;
                pt.y = Cloud1_right->points[i].y;
                pt.z = Cloud1_right->points[i].z;
                float intensity_ratio = 1.0 * Cloud1_right->points[i].intensity / 255;
                //float intensity_ratio_kitti = 1.0 * Cloud1_left->points[i].intensity;
                pt.r = 255 * intensity_ratio;
                pt.g = 215 * intensity_ratio;
                pt.b = 0 * intensity_ratio;
                pointcloud1r->points.push_back(pt);
            }
        } // Golden

        viewer->addPointCloud(pointcloud1r, "pointcloudT_right", v2);

        for (size_t i = 0; i < Cloud2_right->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud2_right->points[i].x;
                pt.y = Cloud2_right->points[i].y;
                pt.z = Cloud2_right->points[i].z;
                float intensity_ratio = 1.0 * Cloud2_right->points[i].intensity / 255;
                //float intensity_ratio_kitti = 1.0 * Cloud2_left->points[i].intensity;
                pt.r = 233 * intensity_ratio;
                pt.g = 233 * intensity_ratio;
                pt.b = 216 * intensity_ratio;
                pointcloud2r->points.push_back(pt);
            }
        } // Silver

        viewer->addPointCloud(pointcloud2r, "pointcloudS_right", v2);

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    void DisplayNClouds(const std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds1, 
                        const std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds2,
                        const std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds3, 
                        std::string displayname, color_type color_mode, int display_downsample_ratio) {
        
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);

        //Create two vertically separated viewports
        int v1(0);
        int v2(1);
        int v3(2);
        viewer->createViewPort(0.0, 0.0, 0.33, 1.0, v1);
        viewer->createViewPort(0.33, 0.0, 0.66, 1.0, v2);
        viewer->createViewPort(0.66, 0.0, 1.0, 1.0, v3);
        // Set camera position and orientation

        float x_position = clouds1[0]->points[0].x;
        float y_position = clouds1[0]->points[0].y;
        float z_position = clouds1[0]->points[0].z;

        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, v1);
        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, v2);
        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, v3);
        viewer->setSize(1800, 900); // Visualiser window size

        _DisplayNClouds(clouds1, viewer, "1-", color_mode, display_downsample_ratio, v1);
        _DisplayNClouds(clouds2, viewer, "2-", color_mode, display_downsample_ratio, v2);
        _DisplayNClouds(clouds3, viewer, "3-", color_mode, display_downsample_ratio, v3);

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }


    void DisplayNClouds(const std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds1, const std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds2,
                                std::string displayname, color_type color_mode, int display_downsample_ratio) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        std::string s;

        //Create two vertically separated viewports
        int v1(0);
        int v2(1);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

        // Set camera position and orientation
        float x_position = clouds1[0]->points[0].x;
        float y_position = clouds1[0]->points[0].y;
        float z_position = clouds1[0]->points[0].z;

        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 0);
        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 1);
        viewer->setSize(1600, 900); // Visualiser window size

        _DisplayNClouds(clouds1, viewer, "1-", color_mode, display_downsample_ratio, v1);
        _DisplayNClouds(clouds2, viewer, "2-", color_mode, display_downsample_ratio, v2);

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    void DisplayNClouds(const std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds, std::string displayname, color_type color_mode, int display_downsample_ratio) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        //Create two vertically separated viewports
        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        // Set camera position and orientation
        float x_position = clouds[0]->points[0].x;
        float y_position = clouds[0]->points[0].y;
        float z_position = clouds[0]->points[0].z;

        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 0);

        _DisplayNClouds(clouds, viewer, "prefix", color_mode, display_downsample_ratio, 0);

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    void DispalyFeatureFrame(std::shared_ptr<point_cloud_t<PointType>> &cloud_in_ptr,
                             std::vector<unsigned int> &ground_index,
                             std::vector<unsigned int> &edge_index,
                             std::vector<unsigned int> &planar_index,
                             std::vector<unsigned int> &sphere_index,
                             std::string displayname) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        std::string s;
        int n = 0;

        float line_size, sphere_size;
        line_size = 2.5;
        sphere_size = 0.6;

        pcl::PointXYZ ptc(0, 0, 0);
        sprintf(t, "%d", n);
        s = t;
        viewer->addSphere(ptc, sphere_size, 1.0, 1.0, 1.0, s);
        n++;

        Eigen::Matrix3d vehicle_attitude;
        vehicle_attitude = Eigen::Matrix3d::Identity();

        pcl::PointXYZ point_x_1(line_size, 0, 0);
        pcl::PointXYZ point_y_1(0, line_size, 0);
        pcl::PointXYZ point_z_1(0, 0, line_size);

        sprintf(t, "line1_%d", n);
        s = t;
        viewer->addLine(ptc, point_x_1, 1.0, 0.0, 0.0, s);
        sprintf(t, "line2_%d", n);
        s = t;
        viewer->addLine(ptc, point_y_1, 0.0, 1.0, 0.0, s);
        sprintf(t, "line3_%d", n);
        s = t;
        viewer->addLine(ptc, point_z_1, 0.0, 0.0, 1.0, s);
        n++;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_frame(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < ground_index.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = cloud_in_ptr->points[ground_index[i]].x;
            pt.y = cloud_in_ptr->points[ground_index[i]].y;
            pt.z = cloud_in_ptr->points[ground_index[i]].z;
            pt.r = 233;
            pt.g = 233;
            pt.b = 216;
            //Ground - Silver
            feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < edge_index.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = cloud_in_ptr->points[edge_index[i]].x;
            pt.y = cloud_in_ptr->points[edge_index[i]].y;
            pt.z = cloud_in_ptr->points[edge_index[i]].z;
            pt.r = 0;
            pt.g = 255;
            pt.b = 0;
            //Edge - Green
            feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < planar_index.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = cloud_in_ptr->points[planar_index[i]].x;
            pt.y = cloud_in_ptr->points[planar_index[i]].y;
            pt.z = cloud_in_ptr->points[planar_index[i]].z;
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            //Planar - Blue
            feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < sphere_index.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = cloud_in_ptr->points[sphere_index[i]].x;
            pt.y = cloud_in_ptr->points[sphere_index[i]].y;
            pt.z = cloud_in_ptr->points[sphere_index[i]].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            //Sphere - Red
            feature_frame->points.push_back(pt);
        }
        sprintf(t, "%d", n);
        s = t;
        viewer->addPointCloud(feature_frame, s);
        n++;

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    void DispalyFeatureFrameRegistration   (std::shared_ptr<point_cloud_t<PointType>> &s_cloud_ptr,
                                            std::vector<unsigned int> &s_ground_index,
                                            std::vector<unsigned int> &s_edge_index,
                                            std::vector<unsigned int> &s_planar_index,
                                            std::vector<unsigned int> &s_sphere_index,
                                            std::shared_ptr<point_cloud_t<PointType>> &t_cloud_ptr,
                                            std::vector<unsigned int> &t_ground_index,
                                            std::vector<unsigned int> &t_edge_index,
                                            std::vector<unsigned int> &t_planar_index,
                                            std::vector<unsigned int> &t_sphere_index,
                                            std::string displayname) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        std::string s;
        int n = 0;

        float line_size, sphere_size, sphere_size2;
        line_size = 2.5;
        sphere_size = 0.4;
        sphere_size2 = 0.01;

        pcl::PointXYZ ptc(0, 0, 0);
        sprintf(t, "%d", n);
        s = t;
        viewer->addSphere(ptc, sphere_size, 1.0, 1.0, 1.0, s);
        n++;

        pcl::PointXYZ point_x_1(line_size, 0, 0);
        pcl::PointXYZ point_y_1(0, line_size, 0);
        pcl::PointXYZ point_z_1(0, 0, line_size);

        sprintf(t, "line1_%d", n);
        s = t;
        viewer->addLine(ptc, point_x_1, 1.0, 0.0, 0.0, s);
        sprintf(t, "line2_%d", n);
        s = t;
        viewer->addLine(ptc, point_y_1, 0.0, 1.0, 0.0, s);
        sprintf(t, "line3_%d", n);
        s = t;
        viewer->addLine(ptc, point_z_1, 0.0, 0.0, 1.0, s);
        n++;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_feature_frame(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < s_ground_index.size(); ++i) {
            pcl::PointXYZRGB pt;
            pt.x = s_cloud_ptr->points[s_ground_index[i]].x;
            pt.y = s_cloud_ptr->points[s_ground_index[i]].y;
            pt.z = s_cloud_ptr->points[s_ground_index[i]].z;
            pt.r = 255;
            pt.g = 215;
            pt.b = 0;
            //Ground - Golden
            s_feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < s_edge_index.size(); ++i) {
            pcl::PointXYZRGB pt;
            pt.x = s_cloud_ptr->points[s_edge_index[i]].x;
            pt.y = s_cloud_ptr->points[s_edge_index[i]].y;
            pt.z = s_cloud_ptr->points[s_edge_index[i]].z;
            pt.r = 0;
            pt.g = 255;
            pt.b = 255;
            //Edge - Cyan
            s_feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < s_planar_index.size(); ++i) {
            pcl::PointXYZRGB pt;
            pt.x = s_cloud_ptr->points[s_planar_index[i]].x;
            pt.y = s_cloud_ptr->points[s_planar_index[i]].y;
            pt.z = s_cloud_ptr->points[s_planar_index[i]].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 255;
            //Planar - Purple
            s_feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < s_sphere_index.size(); ++i) { 
            pcl::PointXYZRGB pt;
            pt.x = s_cloud_ptr->points[s_sphere_index[i]].x;
            pt.y = s_cloud_ptr->points[s_sphere_index[i]].y;
            pt.z = s_cloud_ptr->points[s_sphere_index[i]].z;
            pt.r = 255;
            pt.g = 255;
            pt.b = 0;
            //Sphere - Yellow
            s_feature_frame->points.push_back(pt);
        }
        sprintf(t, "%d", n);
        s = t;
        viewer->addPointCloud(s_feature_frame, s);
        n++;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr t_feature_frame(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < t_ground_index.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = t_cloud_ptr->points[t_ground_index[i]].x;
            pt.y = t_cloud_ptr->points[t_ground_index[i]].y;
            pt.z = t_cloud_ptr->points[t_ground_index[i]].z;
            pt.r = 233;
            pt.g = 233;
            pt.b = 216;
            //Ground - Silver
            t_feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < t_edge_index.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = t_cloud_ptr->points[t_edge_index[i]].x;
            pt.y = t_cloud_ptr->points[t_edge_index[i]].y;
            pt.z = t_cloud_ptr->points[t_edge_index[i]].z;
            pt.r = 0;
            pt.g = 255;
            pt.b = 0;
            //Edge - Green
            t_feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < t_planar_index.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = t_cloud_ptr->points[t_planar_index[i]].x;
            pt.y = t_cloud_ptr->points[t_planar_index[i]].y;
            pt.z = t_cloud_ptr->points[t_planar_index[i]].z;
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            //Planar - Blue
            t_feature_frame->points.push_back(pt);
        }
        for (int i = 0; i < t_sphere_index.size(); ++i)
        {
            pcl::PointXYZRGB pt;
            pt.x = t_cloud_ptr->points[t_sphere_index[i]].x;
            pt.y = t_cloud_ptr->points[t_sphere_index[i]].y;
            pt.z = t_cloud_ptr->points[t_sphere_index[i]].z;
            pt.r = 255;
            pt.g = 0;
            pt.b = 0;
            //Sphere - Red
            t_feature_frame->points.push_back(pt);
        }

        sprintf(t, "%d", n);
        s = t;
        viewer->addPointCloud(t_feature_frame, s);
        n++;

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    // void initialize_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
    // {
    //     viewer->setBackgroundColor(0, 0, 0);
    //     viewer->addCoordinateSystem(1.0);

    //     viewer->registerKeyboardCallback(keyboardEventOccurred, (void *)viewer.get());
    // }

    // void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
    //                            void *viewer_void)
    // {
    //     pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    //     if (event.getKeySym() == "s" && event.keyDown())
    //     {
    //         std::cout << "s was pressed => stop here" << std::endl;

    //         while (!viewer->wasStopped())
    //         {
    //             viewer->spinOnce(100);
    //             boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //         }
    //     }
    // }
    // void initial_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
    // {
    //     int v1 = 0;
    //     viewer->createViewPort(0.0, 0.0, 0.75, 1.0, v1);
    //     viewer->setBackgroundColor(0, 0, 0, v1);

    //     int v2 = 1;
    //     viewer->createViewPort(0.75, 0.0, 1.0, 1.0, v2);
    //     viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    // }

    void DisplayLoopClosure(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                              const Submap &submap_first, const Submap &submap_second, const Edge &edge, int display_time_ms)
    {

        float sphere_size = 1.0;
        //        float line_width = 1.0;

        char str_edge[128];

        sprintf(str_edge, "EDGE_%03u_%04u_%03u_%04u",
                submap_first.submap_id.transaction_id, submap_first.submap_id.submap_id,
                submap_second.submap_id.transaction_id, submap_second.submap_id.submap_id);

        pcl::PointXYZ point_first(submap_first.pose.trans[0], submap_first.pose.trans[1], submap_first.pose.trans[2]);

        pcl::PointXYZ point_second(submap_second.pose.trans[0], submap_second.pose.trans[1], submap_second.pose.trans[2]);

        if (edge.information_matrix == Eigen::Matrix<double, 6, 6>::Zero(6, 6))
        {
            viewer->addLine(point_first, point_second, 1, 0, 0, str_edge);
        }
        else
        {
            switch (edge.edge_type)
            {
            case Edge::Intra:
                viewer->addLine(point_first, point_second, 0, 1, 1, str_edge);
                break;
            case Edge::Inter:
                viewer->addLine(point_first, point_second, 1, 0, 1, str_edge);
                break;
            case Edge::Adjacent:
                viewer->addLine(point_first, point_second, 0, 0, 1, str_edge);
                break;
            default:
                break;
            }
        }

        viewer->spinOnce(display_time_ms);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

    void UpdatePoseGraph(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, 
                         const std::vector<Pose3d, Eigen::aligned_allocator<Pose3d>> &poses,
                         int i, bool is_wrong_edge_candidate, int display_time_ms) {
        std::string prefix = "pose";
        float sphere_size = 1.5;
        //        float line_width = 1.0;

        char str_node[128];
        char str_direct[128];
        char str_adjacent[128];

        sprintf(str_node, "NODE_%03u", i);
        sprintf(str_direct, "DIRCT_%03u", i);
        sprintf(str_adjacent, "ADJACENT_%03u", i);

        viewer->removeShape(prefix + str_node);
        viewer->removeShape(prefix + str_direct);

        pcl::PointXYZ point_center(poses[i].trans[0],
                                   poses[i].trans[1],
                                   poses[i].trans[2]);
        Eigen::Vector3d vehicle_head;
        vehicle_head << 0, 3, 0;
        vehicle_head = poses[i].quat * vehicle_head + poses[i].trans;
        pcl::PointXYZ point_direct(vehicle_head[0],
                                   vehicle_head[1],
                                   vehicle_head[2]);

        viewer->addSphere(point_center, sphere_size, 0, 1, 0, prefix + str_node);

        viewer->addLine(point_center, point_direct, 0, 1, 0, prefix + str_direct);

        if (i > 0)
        {
            pcl::PointXYZ last_point_center(poses[i - 1].trans[0],
                                            poses[i - 1].trans[1],
                                            poses[i - 1].trans[2]);
            if (is_wrong_edge_candidate)
                viewer->addLine(point_center, last_point_center, 1, 0, 0, prefix + str_adjacent);
            else
                viewer->addLine(point_center, last_point_center, 0, 0, 1, prefix + str_adjacent);
        }

        viewer->spinOnce(display_time_ms);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    void UpdateSubmapClouds(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, Submap &submap, color_type color_mode, int display_downsample_ratio, int i, int display_time_ms)
    {
        char str_submap[128];

        sprintf(str_submap, "SUBMAP_%03u", i);
        viewer->removePointCloud(str_submap);

        //Add cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t i = 0; i < submap.cld_lidar_ptr->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0) //Downsample for display
            {
                pcl::PointXYZRGB pt;
                pt.x = submap.cld_lidar_ptr->points[i].x;
                pt.y = submap.cld_lidar_ptr->points[i].y;
                pt.z = submap.cld_lidar_ptr->points[i].z;
                pt.r = min_(1.1 * submap.cld_lidar_ptr->points[i].intensity, 255);
                pt.g = pt.r;
                pt.b = pt.r;
                rgbcloud->points.push_back(pt);
            }
        }
        viewer->addPointCloud(rgbcloud, str_submap);

        //Display
        viewer->spinOnce(display_time_ms);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

    void DisplaySubmapClouds(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, Submap &submap, color_type color_mode, int display_downsample_ratio, int i, int display_time_ms)
    {
        int submap_cloud_max = 30;
        int submap_pose_max = 100;

        float sphere_size, line_size, line_size_origin;

        sphere_size = 0.15;
        line_size = 0.4;
        line_size_origin = 2.0;

        float maxz, minz, maxz2, minz2, c_value, maxi, mini;
        float red_v = 1.0;
        float green_v = 1.0;
        float blue_v = 0.0;

        char str_submap[128];
        char str_origin[128];
        char str_origin_x_axis[128];
        char str_origin_y_axis[128];
        char str_origin_z_axis[128];

        if (i >= submap_cloud_max)
        {

            sprintf(str_submap, "SUBMAP_%03u", i - submap_cloud_max);
            viewer->removePointCloud(str_submap);
        }

        if (i >= submap_pose_max)
        {
            sprintf(str_origin, "Origin_%03u", i - submap_pose_max);
            sprintf(str_origin_x_axis, "Originx_%03u", i - submap_pose_max);
            sprintf(str_origin_y_axis, "Originy_%03u", i - submap_pose_max);
            sprintf(str_origin_z_axis, "Originz_%03u", i - submap_pose_max);
            viewer->removeShape(str_origin);
            viewer->removeShape(str_origin_x_axis);
            viewer->removeShape(str_origin_y_axis);
            viewer->removeShape(str_origin_z_axis);
            for (int j = 0; j < 100; j++)
            {
                char str_frame_LO[128];
                char str_frame_OXTS[128];
                sprintf(str_frame_OXTS, "OXTS_%03u_%03u", i - submap_pose_max, j);
                sprintf(str_frame_LO, "LO_%03u_%03u", i - submap_pose_max, j);
                viewer->removeShape(str_frame_OXTS);
                viewer->removeShape(str_frame_LO);
            }
        }

        std::string s;
        int n = 0;

        // //Get bounding box data
        // maxz = submap.bbox.max_z;
        // minz = submap.bbox.min_z;
        // if ((maxz - minz) > 8)
        // {
        //     maxz2 = maxz - 6;
        //     minz2 = minz + 2;
        // } //Set Color Ramp

        mini = 0;
        maxi = 255.0;

        float frame_color_r, frame_color_g, frame_color_b;

        //float lidar_height = 1.85;
        sprintf(str_origin, "Origin_%03u", i);
        sprintf(str_origin_x_axis, "Originx_%03u", i);
        sprintf(str_origin_y_axis, "Originy_%03u", i);
        sprintf(str_origin_z_axis, "Originz_%03u", i);
        pcl::PointXYZ pt0(submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(0, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(1, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(2, 3));

        viewer->addSphere(pt0, sphere_size, 1.0, 1.0, 1.0, str_origin);

        pcl::PointXYZ point_x_0(submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(0, 3) + line_size_origin, submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(1, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(2, 3));
        pcl::PointXYZ point_y_0(submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(0, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(1, 3) + line_size_origin, submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(2, 3));
        pcl::PointXYZ point_z_0(submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(0, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(1, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(2, 3) + line_size_origin);

        viewer->addLine(pt0, point_x_0, 1.0, 0.0, 0.0, str_origin_x_axis);
        viewer->addLine(pt0, point_y_0, 0.0, 1.0, 0.0, str_origin_y_axis);
        viewer->addLine(pt0, point_z_0, 0.0, 0.0, 1.0, str_origin_z_axis);

        //add cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t i = 0; i < submap.cld_lidar_ptr->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0) //Downsample for display
            {
                pcl::PointXYZRGB pt;
                pt.x = submap.cld_lidar_ptr->points[i].x;
                pt.y = submap.cld_lidar_ptr->points[i].y;
                pt.z = submap.cld_lidar_ptr->points[i].z;
                pt.r = min_(1.1*submap.cld_lidar_ptr->points[i].intensity,255);
                pt.g = pt.r;
                pt.b = pt.r;
                rgbcloud->points.push_back(pt);
                //LOG(INFO)<<"x,y,z,i: "<<pt.x<<" , "<<pt.y<<" , "<<pt.z<< " , "<< submap.cld_lidar_ptr->points[i].intensity;
            }
        }
        sprintf(str_submap, "SUBMAP_%03u", i);
        viewer->addPointCloud(rgbcloud, str_submap);

        for (int j = 0; j < submap.frame_number; j++)
        {
            char str_frame_pose[256];
            pcl::PointXYZ ptc1;
            ptc1.x = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(0, 3);
            ptc1.y = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(1, 3);
            ptc1.z = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(2, 3);
            sprintf(str_frame_pose, "LO_%03u_%03u", i, j);
            viewer->addSphere(ptc1, sphere_size, 1.0, 1.0, 0.0, str_frame_pose); //Lidar Odometry : Yellow

            pcl::PointXYZ ptc2;
            ptc2.x = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(0, 3);
            ptc2.y = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(1, 3);
            ptc2.z = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(2, 3);
            sprintf(str_frame_pose, "OXTS_%03u_%03u", i, j);
            viewer->addSphere(ptc2, sphere_size, 1.0, 0.0, 1.0, str_frame_pose); //Lidar(OXTS) INS Trajectory: Purple
        }

        // for (int j = 0; j < submap.frame_number; j++)
        // {
        //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        //     //Get random color for the frame
        //     frame_color_r = 255 * (rand() / (1.0 + RAND_MAX));
        //     frame_color_g = 255 * (rand() / (1.0 + RAND_MAX));
        //     frame_color_b = 255 * (rand() / (1.0 + RAND_MAX));

        //     for (size_t i = 0; i < submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points.size(); ++i)
        //     {
        //         if (i % display_downsample_ratio == 0) //Downsample for display
        //         {
        //             pcl::PointXYZRGB pt;
        //             pt.x = submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].x;
        //             pt.y = submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].y;
        //             pt.z = submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].z;

        //             switch (color_mode)
        //             {
        //             case SINGLE: //Single Color for all the points: Golden
        //             {
        //                 pt.r = 255;
        //                 pt.g = 215;
        //                 pt.b = 0;
        //                 break;
        //             }
        //             case HEIGHT: //Height ramp color scalar
        //             {
        //                 c_value = min_(max_(pt.z - minz2, 0) / (maxz2 - minz2), 1);
        //                 pt.r = 255 * c_value;
        //                 pt.g = 255 * (1.0 - c_value);
        //                 pt.b = 50 + 150 * c_value;
        //                 break;
        //             }
        //             case FRAME: //Random color for each frame
        //             {
        //                 pt.r = frame_color_r;
        //                 pt.g = frame_color_g;
        //                 pt.b = frame_color_b;
        //                 break;
        //             }
        //             case INTENSITY: //Fix it later
        //             {
        //                 //float color_intensity= 255.0 * (submap.frames[j].pointcloud_odom_down->points[i].intensity - mini)/(maxi-mini);
        //                 pt.r = min_(1.1 * submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].intensity, 255);
        //                 pt.g = pt.r;
        //                 pt.b = pt.r;
        //                 break;
        //             }
        //             default: //RED
        //             {
        //                 pt.r = 255;
        //                 pt.g = 0;
        //                 pt.b = 0;
        //                 break;
        //             }
        //             }

        //             rgbcloud->points.push_back(pt);
        //         }
        //     }

        //     sprintf(t, "%d", n);
        //     s = t;
        //     viewer->addPointCloud(rgbcloud, prefix+s);
        //     n++;

        //     pcl::PointXYZ ptc1;
        //     ptc1.x = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(0, 3);
        //     ptc1.y = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(1, 3);
        //     ptc1.z = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(2, 3);
        //     sprintf(t, "%d", n);
        //     s = t;
        //     if (j == 0)
        //         viewer->addSphere(ptc1, sphere_size, 0.0, 1.0, 0.0, prefix + s); //Lidar Odometry Start Point: Green
        //     else
        //         viewer->addSphere(ptc1, sphere_size, 1.0, 1.0, 0.0, prefix + s); //Lidar Odometry Trajectory: Yellow
        //     n++;

        //     Eigen::Matrix3d vehicle_attitude;
        //     vehicle_attitude = Eigen::Matrix3d::Identity();
        //     vehicle_attitude.block<3, 1>(0, 0) = line_size * submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 0) + submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 3);
        //     vehicle_attitude.block<3, 1>(0, 1) = line_size * submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 1) + submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 3);
        //     vehicle_attitude.block<3, 1>(0, 2) = line_size * submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 2) + submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 3);

        //     pcl::PointXYZ point_x_1(vehicle_attitude(0, 0),
        //                             vehicle_attitude(1, 0),
        //                             vehicle_attitude(2, 0));
        //     pcl::PointXYZ point_y_1(vehicle_attitude(0, 1),
        //                             vehicle_attitude(1, 1),
        //                             vehicle_attitude(2, 1));
        //     pcl::PointXYZ point_z_1(vehicle_attitude(0, 2),
        //                             vehicle_attitude(1, 2),
        //                             vehicle_attitude(2, 2));
        //     sprintf(t, "line1_%d", n);
        //     s = t;
        //     viewer->addLine(ptc1, point_x_1, 1.0, 0.0, 0.0, prefix + s);
        //     sprintf(t, "line2_%d", n);
        //     s = t;
        //     viewer->addLine(ptc1, point_y_1, 0.0, 1.0, 0.0, prefix + s);
        //     sprintf(t, "line3_%d", n);
        //     s = t;
        //     viewer->addLine(ptc1, point_z_1, 0.0, 0.0, 1.0, prefix + s);
        //     n++;

        //     pcl::PointXYZ ptc2;
        //     ptc2.x = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(0, 3);
        //     ptc2.y = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(1, 3);
        //     ptc2.z = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(2, 3);
        //     sprintf(t, "%d", n);
        //     s = t;
        //     if (j == 0)
        //         viewer->addSphere(ptc2, sphere_size, 0.0, 1.0, 0.0, prefix + s); //Lidar(OXTS) Start Point: Green
        //     else
        //         viewer->addSphere(ptc2, sphere_size, 1.0, 0.0, 1.0, prefix + s); //Lidar(OXTS) INS Trajectory: Purple
        //     n++;

        //     vehicle_attitude.block<3, 1>(0, 0) = line_size * submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 0) + submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 3);
        //     vehicle_attitude.block<3, 1>(0, 1) = line_size * submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 1) + submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 3);
        //     vehicle_attitude.block<3, 1>(0, 2) = line_size * submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 2) + submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 3);

        //     pcl::PointXYZ point_x_2(vehicle_attitude(0, 0),
        //                             vehicle_attitude(1, 0),
        //                             vehicle_attitude(2, 0));
        //     pcl::PointXYZ point_y_2(vehicle_attitude(0, 1),
        //                             vehicle_attitude(1, 1),
        //                             vehicle_attitude(2, 1));
        //     pcl::PointXYZ point_z_2(vehicle_attitude(0, 2),
        //                             vehicle_attitude(1, 2),
        //                             vehicle_attitude(2, 2));
        //     sprintf(t, "line1_%d", n);
        //     s = t;
        //     viewer->addLine(ptc2, point_x_2, 1.0, 0.0, 0.0, prefix + s);
        //     sprintf(t, "line2_%d", n);
        //     s = t;
        //     viewer->addLine(ptc2, point_y_2, 0.0, 1.0, 0.0, prefix + s);
        //     sprintf(t, "line3_%d", n);
        //     s = t;
        //     viewer->addLine(ptc2, point_z_2, 0.0, 0.0, 1.0, prefix + s);
        //     n++;
        // }

        // std::vector<Camera> cameras;
        // viewer->getCameras (cameras);
        // LOG(INFO)<<

        //while (!viewer->wasStopped())
        //{
        viewer->spinOnce(display_time_ms);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        //}
    }

    void DisplaySubmapClouds(Submap &submap, std::string displayname, color_type color_mode, int display_downsample_ratio)
    {

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        std::string s;
        int n = 0;

        float sphere_size, line_size, line_size_origin;

        sphere_size = 0.15;
        line_size = 0.5;
        line_size_origin = 2.0;

        float maxz, minz, maxz2, minz2, c_value, maxi, mini;
        float red_v = 1.0;
        float green_v = 1.0;
        float blue_v = 0.0;

        //Get bounding box data
        maxz = submap.bbox.max_z;
        minz = submap.bbox.min_z;
        if ((maxz - minz) > 8)
        {
            maxz2 = maxz - 6;
            minz2 = minz + 2;
        } //Set Color Ramp

        mini = 0;
        maxi = 255.0;

        float frame_color_r, frame_color_g, frame_color_b;

        float lidar_height = 1.85;
        pcl::PointXYZ pt0(submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(0, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(1, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(2, 3) - lidar_height);
        sprintf(t, "%d", n);
        s = t;
        viewer->addSphere(pt0, sphere_size, 1.0, 1.0, 1.0, s);
        n++;

        pcl::PointXYZ point_x_0(submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(0, 3) + line_size_origin, submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(1, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(2, 3) - lidar_height);
        pcl::PointXYZ point_y_0(submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(0, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(1, 3) + line_size_origin, submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(2, 3) - lidar_height);
        pcl::PointXYZ point_z_0(submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(0, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(1, 3), submap.raw_data_group[0].raw_gnss.pose.GetMatrix()(2, 3) + line_size_origin - lidar_height);

        sprintf(t, "line1_%d", n);
        s = t;
        viewer->addLine(pt0, point_x_0, 1.0, 0.0, 0.0, s);
        sprintf(t, "line2_%d", n);
        s = t;
        viewer->addLine(pt0, point_y_0, 0.0, 1.0, 0.0, s);
        sprintf(t, "line3_%d", n);
        s = t;
        viewer->addLine(pt0, point_z_0, 0.0, 0.0, 1.0, s);
        n++;

        for (int j = 0; j < submap.frame_number; j++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            //Get random color for the frame
            frame_color_r = 255 * (rand() / (1.0 + RAND_MAX));
            frame_color_g = 255 * (rand() / (1.0 + RAND_MAX));
            frame_color_b = 255 * (rand() / (1.0 + RAND_MAX));

            for (size_t i = 0; i < submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points.size(); ++i)
            {
                if (i % display_downsample_ratio == 0) //Downsample for display
                {
                    pcl::PointXYZRGB pt;
                    pt.x = submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].x;
                    pt.y = submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].y;
                    pt.z = submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].z;

                    switch (color_mode)
                    {
                    case SINGLE: //Single Color for all the points: Golden
                    {
                        pt.r = 255;
                        pt.g = 215;
                        pt.b = 0;
                        break;
                    }
                    case HEIGHT: //Height ramp color scalar
                    {
                        c_value = min_(max_(pt.z - minz2, 0) / (maxz2 - minz2), 1);
                        pt.r = 255 * c_value;
                        pt.g = 255 * (1.0 - c_value);
                        pt.b = 50 + 150 * c_value;
                        break;
                    }
                    case FRAME: //Random color for each frame
                    {
                        pt.r = frame_color_r;
                        pt.g = frame_color_g;
                        pt.b = frame_color_b;
                        break;
                    }
                    case INTENSITY: 
                    {
                        //float color_intensity= 255.0 * (submap.frames[j].pointcloud_odom_down->points[i].intensity - mini)/(maxi-mini);
                        pt.r = min_(1.1 * submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].intensity, 255);
                        pt.g = pt.r;
                        pt.b = pt.r;

                        //for kitti (intensity 0~1)
                        // pt.r = min_(1.1*submap.raw_data_group[j].raw_frame.cld_lidar_ptr->points[i].intensity*255.0, 255) ;
                        // pt.g = pt.r;
                        // pt.b = pt.r;

                        break;
                    }
                    default: //RED
                    {
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;
                        break;
                    }
                    }

                    rgbcloud->points.push_back(pt);
                }
            }

            sprintf(t, "%d", n);
            s = t;
            viewer->addPointCloud(rgbcloud, s);
            n++;

            pcl::PointXYZ ptc1;
            ptc1.x = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(0, 3);
            ptc1.y = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(1, 3);
            ptc1.z = submap.raw_data_group[j].raw_frame.pose.GetMatrix()(2, 3);
            sprintf(t, "%d", n);
            s = t;
            if (j == 0)
                viewer->addSphere(ptc1, sphere_size, 0.0, 1.0, 0.0, s); //Lidar Odometry Start Point: Green
            else
                viewer->addSphere(ptc1, sphere_size, 1.0, 1.0, 0.0, s); //Lidar Odometry Trajectory: Yellow
            n++;

            // Eigen::Matrix3d vehicle_attitude;
            // vehicle_attitude = Eigen::Matrix3d::Identity();
            // vehicle_attitude.block<3, 1>(0, 0) = line_size * submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 0) + submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 3);
            // vehicle_attitude.block<3, 1>(0, 1) = line_size * submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 1) + submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 3);
            // vehicle_attitude.block<3, 1>(0, 2) = line_size * submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 2) + submap.raw_data_group[j].raw_frame.pose.GetMatrix().block<3, 1>(0, 3);

            // pcl::PointXYZ point_x_1(vehicle_attitude(0, 0),
            //                         vehicle_attitude(1, 0),
            //                         vehicle_attitude(2, 0));
            // pcl::PointXYZ point_y_1(vehicle_attitude(0, 1),
            //                         vehicle_attitude(1, 1),
            //                         vehicle_attitude(2, 1));
            // pcl::PointXYZ point_z_1(vehicle_attitude(0, 2),
            //                         vehicle_attitude(1, 2),
            //                         vehicle_attitude(2, 2));
            // sprintf(t, "line1_%d", n);
            // s = t;
            // viewer->addLine(ptc1, point_x_1, 1.0, 0.0, 0.0, s);
            // sprintf(t, "line2_%d", n);
            // s = t;
            // viewer->addLine(ptc1, point_y_1, 0.0, 1.0, 0.0, s);
            // sprintf(t, "line3_%d", n);
            // s = t;
            // viewer->addLine(ptc1, point_z_1, 0.0, 0.0, 1.0, s);
            // n++;

            pcl::PointXYZ ptc2;
            ptc2.x = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(0, 3);
            ptc2.y = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(1, 3);
            ptc2.z = submap.raw_data_group[j].raw_gnss.pose.GetMatrix()(2, 3);
            sprintf(t, "%d", n);
            s = t;
            if (j == 0)
                viewer->addSphere(ptc2, sphere_size, 0.0, 1.0, 0.0, s); //Lidar(OXTS) Start Point: Green
            else
                viewer->addSphere(ptc2, sphere_size, 1.0, 0.0, 1.0, s); //Lidar(OXTS) INS Trajectory: Purple
            n++;

            // vehicle_attitude.block<3, 1>(0, 0) = line_size * submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 0) + submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 3);
            // vehicle_attitude.block<3, 1>(0, 1) = line_size * submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 1) + submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 3);
            // vehicle_attitude.block<3, 1>(0, 2) = line_size * submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 2) + submap.raw_data_group[j].raw_gnss.pose.GetMatrix().block<3, 1>(0, 3);

            // pcl::PointXYZ point_x_2(vehicle_attitude(0, 0),
            //                         vehicle_attitude(1, 0),
            //                         vehicle_attitude(2, 0));
            // pcl::PointXYZ point_y_2(vehicle_attitude(0, 1),
            //                         vehicle_attitude(1, 1),
            //                         vehicle_attitude(2, 1));
            // pcl::PointXYZ point_z_2(vehicle_attitude(0, 2),
            //                         vehicle_attitude(1, 2),
            //                         vehicle_attitude(2, 2));
            // sprintf(t, "line1_%d", n);
            // s = t;
            // viewer->addLine(ptc2, point_x_2, 1.0, 0.0, 0.0, s);
            // sprintf(t, "line2_%d", n);
            // s = t;
            // viewer->addLine(ptc2, point_y_2, 0.0, 1.0, 0.0, s);
            // sprintf(t, "line3_%d", n);
            // s = t;
            // viewer->addLine(ptc2, point_z_2, 0.0, 0.0, 1.0, s);
            // n++;

            // pcl::PointXYZ ptc3;
            // ptc3.x = submap.frames[j].oxts_noise_pose(0, 3);
            // ptc3.y = submap.frames[j].oxts_noise_pose(1, 3);
            // ptc3.z = submap.frames[j].oxts_noise_pose(2, 3);
            // sprintf(t, "%d", n);
            // s = t;
            // if (j == 0)
            //     viewer->addSphere(ptc3, sphere_size, 0.0, 1.0, 0.0, s); //GNSS INS Start Point: Green
            // else
            //     viewer->addSphere(ptc3, sphere_size, 1.0, 1.0, 0.0, s); //GNSS INS Trajectory: Yellow
            // n++;
#if 0
            pcl::PointXYZ ptc4;
            ptc4.x = submap.raw_data_group[j].body_gnss.pose.GetMatrix()(0, 3);
            ptc4.y = submap.raw_data_group[j].body_gnss.pose.GetMatrix()(1, 3);
            ptc4.z = submap.raw_data_group[j].body_gnss.pose.GetMatrix()(2, 3);
            sprintf(t, "%d", n);
            s = t;
            if (j == 0)
                viewer->addSphere(ptc4, sphere_size, 0.0, 1.0, 0.0, s); //Body(OXTS) Start Point: Green
            else
                viewer->addSphere(ptc4, sphere_size, 0.0, 1.0, 1.0, s); //Body(OXTS) Trajectory: Cyan
            n++;

            vehicle_attitude.block<3, 1>(0, 0) = line_size * submap.raw_data_group[j].body_gnss.pose.GetMatrix().block<3, 1>(0, 0) + submap.raw_data_group[j].body_gnss.pose.GetMatrix().block<3, 1>(0, 3);
            vehicle_attitude.block<3, 1>(0, 1) = line_size * submap.raw_data_group[j].body_gnss.pose.GetMatrix().block<3, 1>(0, 1) + submap.raw_data_group[j].body_gnss.pose.GetMatrix().block<3, 1>(0, 3);
            vehicle_attitude.block<3, 1>(0, 2) = line_size * submap.raw_data_group[j].body_gnss.pose.GetMatrix().block<3, 1>(0, 2) + submap.raw_data_group[j].body_gnss.pose.GetMatrix().block<3, 1>(0, 3);

            pcl::PointXYZ point_x_4(vehicle_attitude(0, 0),
                                    vehicle_attitude(1, 0),
                                    vehicle_attitude(2, 0));
            pcl::PointXYZ point_y_4(vehicle_attitude(0, 1),
                                    vehicle_attitude(1, 1),
                                    vehicle_attitude(2, 1));
            pcl::PointXYZ point_z_4(vehicle_attitude(0, 2),
                                    vehicle_attitude(1, 2),
                                    vehicle_attitude(2, 2));
            sprintf(t, "line1_%d", n);
            s = t;
            viewer->addLine(ptc4, point_x_4, 1.0, 0.0, 0.0, s);
            sprintf(t, "line2_%d", n);
            s = t;
            viewer->addLine(ptc4, point_y_4, 0.0, 1.0, 0.0, s);
            sprintf(t, "line3_%d", n);
            s = t;
            viewer->addLine(ptc4, point_z_4, 0.0, 0.0, 1.0, s);
            n++;

            sprintf(t, "connect_line%d", n);
            s = t;
            viewer->addLine(ptc2, ptc4, 1.0, 1.0, 1.0, s);
            n++;
#endif
        }

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

#if 0
    void display_trajectory(map_pose::Transaction &transaction, std::string displayname)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        string s;
        int n = 0;

        float sphere_size, sphere_size2, line_width;
        float f_red = 1.0, f_green = 1.0, f_blue = 1.0;
        sphere_size = 0.15;
        sphere_size2 = 0.1;
        line_width = 0.2;

        for (int i = 0; i < transaction.submap_number_; i++)
        {
            pcl::PointXYZ ptc1;
            ptc1.x = transaction.sub_maps_[i].pose.GetMatrix()(0, 3);
            ptc1.y = transaction.sub_maps_[i].pose.GetMatrix()(1, 3);
            ptc1.z = transaction.sub_maps_[i].pose.GetMatrix()(2, 3);
            sprintf(t, "%d", n);
            s = t;
            if (i == 0)
                viewer->addSphere(ptc1, sphere_size, 0.0, 1.0, 0.0, s); //Lidar Odometry Start Point: Green
            else
                viewer->addSphere(ptc1, sphere_size, 1.0, 0.0, 0.0, s); //Lidar Odometry Trajectory: Red
            n++;

            pcl::PointXYZ pt1;
            pt1.x = transaction.sub_maps_[i].bbox.min_x;
            pt1.y = transaction.sub_maps_[i].bbox.min_y;
            pt1.z = transaction.sub_maps_[i].bbox.min_z;
            sprintf(t, "%d", n);
            s = t;
            viewer->addSphere(pt1, sphere_size2, f_red, f_green, f_blue, s);
            n++;

            pcl::PointXYZ pt2;
            pt2.x = transaction.sub_maps_[i].bbox.min_x;
            pt2.y = transaction.sub_maps_[i].bbox.max_y;
            pt2.z = transaction.sub_maps_[i].bbox.min_z;
            sprintf(t, "%d", n);
            s = t;
            viewer->addSphere(pt2, sphere_size2, f_red, f_green, f_blue, s);
            n++;

            pcl::PointXYZ pt3;
            pt3.x = transaction.sub_maps_[i].bbox.max_x;
            pt3.y = transaction.sub_maps_[i].bbox.max_y;
            pt3.z = transaction.sub_maps_[i].bbox.min_z;
            sprintf(t, "%d", n);
            s = t;
            viewer->addSphere(pt3, sphere_size2, f_red, f_green, f_blue, s);
            n++;

            pcl::PointXYZ pt4;
            pt4.x = transaction.sub_maps_[i].bbox.max_x;
            pt4.y = transaction.sub_maps_[i].bbox.min_y;
            pt4.z = transaction.sub_maps_[i].bbox.min_z;
            sprintf(t, "%d", n);
            s = t;
            viewer->addSphere(pt4, sphere_size2, f_red, f_green, f_blue, s);
            n++;

            pcl::PointXYZ pt5;
            pt5.x = transaction.sub_maps_[i].bbox.min_x;
            pt5.y = transaction.sub_maps_[i].bbox.min_y;
            pt5.z = transaction.sub_maps_[i].bbox.max_z;
            sprintf(t, "%d", n);
            s = t;
            viewer->addSphere(pt5, sphere_size2, f_red, f_green, f_blue, s);
            n++;

            pcl::PointXYZ pt6;
            pt6.x = transaction.sub_maps_[i].bbox.min_x;
            pt6.y = transaction.sub_maps_[i].bbox.max_y;
            pt6.z = transaction.sub_maps_[i].bbox.max_z;
            sprintf(t, "%d", n);
            s = t;
            viewer->addSphere(pt6, sphere_size2, f_red, f_green, f_blue, s);
            n++;

            pcl::PointXYZ pt7;
            pt7.x = transaction.sub_maps_[i].bbox.max_x;
            pt7.y = transaction.sub_maps_[i].bbox.max_y;
            pt7.z = transaction.sub_maps_[i].bbox.max_z;
            sprintf(t, "%d", n);
            s = t;
            viewer->addSphere(pt7, sphere_size2, f_red, f_green, f_blue, s);
            n++;

            pcl::PointXYZ pt8;
            pt8.x = transaction.sub_maps_[i].bbox.max_x;
            pt8.y = transaction.sub_maps_[i].bbox.min_y;
            pt8.z = transaction.sub_maps_[i].bbox.max_z;
            sprintf(t, "%d", n);
            s = t;
            viewer->addSphere(pt8, sphere_size2, f_red, f_green, f_blue, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt1, pt2, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt2, pt3, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt3, pt4, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt4, pt1, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt5, pt6, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt6, pt7, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt7, pt8, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt8, pt5, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt1, pt5, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt2, pt6, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt3, pt7, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;

            sprintf(t, "%d", n);
            s = t;
            viewer->addLine(pt4, pt8, f_red, f_green, f_blue, s);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, s);
            n++;
        }

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
#endif
private:
    void _DisplayNClouds(const std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                         std::string prefix, color_type color_mode, int display_downsample_ratio, int viewport) {
        char ch_t[256];
        std::string str;

        float maxz, minz, maxz2, minz2, c_value;
        maxz2 = 20.0;
        minz2 = -20.0;
        float frame_color_r, frame_color_g, frame_color_b;

        for (int j = 0; j < clouds.size(); j++) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            switch (j) {
                case 0: {
                    frame_color_r = 255;
                    frame_color_g = 0;
                    frame_color_b = 0;
                    break;
                }
                case 1: {
                    frame_color_r = 0;
                    frame_color_g = 255;
                    frame_color_b = 0;
                    break;
                }
                case 2: {
                    frame_color_r = 0;
                    frame_color_g = 0;
                    frame_color_b = 255;
                    break;
                }
                case 3: {
                    frame_color_r = 255;
                    frame_color_g = 255;
                    frame_color_b = 0;
                    break;
                }
                default: {
                    frame_color_r = 255 * (rand() / (1.0 + RAND_MAX));
                    frame_color_g = 255 * (rand() / (1.0 + RAND_MAX));
                    frame_color_b = 255 * (rand() / (1.0 + RAND_MAX));
                    break;
                }
            }

            for (size_t i = 0; i < clouds[j]->points.size(); ++i) {
                if (i % display_downsample_ratio == 0) //Downsample for display
                {
                    pcl::PointXYZRGB pt;
                    pt.x = clouds[j]->points[i].x;
                    pt.y = clouds[j]->points[i].y;
                    pt.z = clouds[j]->points[i].z;

                    switch (color_mode)
                    {
                        case SINGLE: //Single Color for all the points: Golden
                        {
                            pt.r = 255;
                            pt.g = 215;
                            pt.b = 0;
                            break;
                        }
                        case HEIGHT: //Height ramp color scalar
                        {
                            c_value = min_(max_(pt.z - minz2, 0) / (maxz2 - minz2), 1);
                            pt.r = 255 * c_value;
                            pt.g = 255 * (1.0 - c_value);
                            pt.b = 50 + 150 * c_value;
                            break;
                        }
                        case FRAME: //Random color for each frame
                        {
                            pt.r = frame_color_r * clouds[j]->points[i].intensity / 255;
                            pt.g = frame_color_g * clouds[j]->points[i].intensity / 255;
                            pt.b = frame_color_b * clouds[j]->points[i].intensity / 255;
                            break;
                        }
                        case INTENSITY: //Fix it later
                        {
                            //float color_intensity= 255.0 * (submap.frames[j].pointcloud_odom_down->points[i].intensity - mini)/(maxi-mini);
                            pt.r = min_(1.1 * clouds[j]->points[i].intensity, 255);
                            pt.g = pt.r;
                            pt.b = pt.r;
                            break;
                        }
                        default: //RED
                        {
                            pt.r = 255;
                            pt.g = 0;
                            pt.b = 0;
                            break;
                        }
                    }

                    rgbcloud->points.push_back(pt);
                }
            }
            sprintf(ch_t, "%d", j);
            str = prefix + ch_t;
            viewer->addPointCloud(rgbcloud, str, viewport);
        }
    }
    //    vector<double> global_shift;
}; // namespace map_pose
} // namespace map_pose

#endif //_INCLUDE_MAP_VIEWER_H