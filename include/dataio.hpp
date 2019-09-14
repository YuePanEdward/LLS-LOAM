
#ifndef _INCLUDE_DATA_IO_H_
#define _INCLUDE_DATA_IO_H_

#include <pcl/io/pcd_io.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <memory>

#include <glog/logging.h>

#include "types.h"
//#include "transaction.h"

namespace map_pose
{

typedef pcl::PointXYZINormal PclPointType;

template <typename PointT>
class DataIo
{
public:
    bool checkDir(const std::string &dir) const
    {
        if (!boost::filesystem::exists(dir.c_str()))
        {
            if (boost::filesystem::create_directory(dir.c_str()))
                return true;
            else
                return false;
        }
        return true;
    }

    // submap odometry save submap
    bool saveSubmapToFile(std::string submap_full_cld_save_path,
                          std::string submap_feature_cld_save_path,
                          std::string submap_feature_idx_save_path,
                          Submap &submap)
    {
        clock_t t0, t1;
        t0 = clock();
        // Save PointCloud
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld_out(new pcl::PointCloud<pcl::PointXYZINormal>);
        OurPoint2PclPoint(submap.cld_lidar_ptr, cld_out);
        writePcdFile(submap_full_cld_save_path, cld_out);
        OurPoint2PclPoint(submap.cld_feature_ptr, cld_out);
        writePcdFile(submap_feature_cld_save_path, cld_out);
        // Save Feature Index
        // 1 ground; 2 edge; 3 planar; 4 sphere
        std::ofstream outfile(submap_feature_idx_save_path, std::ios::app); // add after the file
        //Then you should create a new file for a new project
        //std::ofstream outfile(submap_feature_idx_save_path); //default write

        if (outfile)
        {
            outfile << submap << "\n";
        }
        outfile.close();

        t1 = clock();
        LOG(INFO) << "Save Submap data done in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";
    }

    // loop detecion : read submaps of different transactions
    bool loadSubmapFromFile(std::string submap_feature_idx_save_path, VectorOfSubmaps &submaps) {
        Submap submap;
        // Load Feature Index
        std::ifstream infile(submap_feature_idx_save_path);
        if (!infile) {
            LOG(ERROR) << "Open File Failed! File is " << submap_feature_idx_save_path;
            return false;
        }

        //int ground_size, edge_size, planar_size, sphere_size;
        //        while (!infile.eof())
        while (infile.peek() != EOF) {
            infile >> submap;
            submap.index_type = Submap::Continuous;
            if (infile.fail())
                break;
            //            infile >> std::ws;

            submaps.push_back(submap);
        }
        infile.close();

        return true;
    }

    // loop detecion read adjacent edges of different transactions 
    bool loadEdgesFromFile(const std::string &save_folder, const std::string &save_filename, VectorOfEdges &edges) {
        std::string edge_save_path;
        edge_save_path = save_folder + "/" + save_filename;

        Edge edge;
        // Load Feature Index
        std::ifstream infile(edge_save_path);
        if (!infile) {
            LOG(ERROR) << "Open File Failed! File is " << edge_save_path;
            return false;
        }

        std::string data_type;
        std::string edge_type;

        while (!infile.eof()) {
            infile >> data_type;
            if (data_type == "EDGE") {         
                infile >> edge.submap_idx.first_submap >> edge.submap_idx.second_submap >> edge_type >> edge.pose >> edge.information_matrix;
            }
            else {
                continue;
            }
            if (infile.fail()) {
                break;
            }
            edges.push_back(edge);
        }
        infile.close();

        return true;
    }

    // loop detecion : read submap point cloud
    bool loadSubmapCldFromFile(std::string submap_feature_cld_root_path, Submap &submap) {
        // Recover all *_index, if *_index is empty and index_type is Continuous
        if (submap.ground_index.empty()) {
            if (submap.index_type == Submap::Continuous) {
                submap.ground_index.resize(submap.ground_size);
                submap.edge_index.resize(submap.edge_size);
                submap.planar_index.resize(submap.planar_size);
                submap.sphere_index.resize(submap.sphere_size);
                for (int i = 0; i < submap.ground_size; ++i) {  
                    submap.ground_index[i] = i;
                }
                for (int i = 0; i < submap.edge_size; ++i) {
                    submap.edge_index[i] = i + submap.ground_size;
                }
                for (int i = 0; i < submap.planar_size; ++i) {
                    submap.planar_index[i] = i + submap.ground_size + submap.edge_size;
                }
                for (int i = 0; i < submap.sphere_size; ++i) {
                    submap.sphere_index[i] = i + submap.ground_size + submap.edge_size + submap.planar_size;
                }
            }
            else {
                LOG(ERROR) << "Can Not Recover cld index, cld isn't Continuous!";
            }
        }
        // Load Feature PointCloud
        std::string submap_file = submap_feature_cld_root_path + "/" + submap.feature_cld_filename;
        readPcdFile(submap_file, submap.cld_feature_ptr, 2);

        return true;
    }

    // pcd IO
    bool readPcdFile(const std::string &fileName, std::shared_ptr<point_cloud_t<PointT>> &point_cloud_in, const int tag) const
    {
        clock_t t0, t1;

        t0 = clock();

        point_cloud_in = std::make_shared<PointCloud>();
        if (tag == 1)
        {
            //Read in XYZI Points
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new typename pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, *pointCloud) == -1)
            {
                PCL_ERROR("Couldn't read file\n");
                return false;
            }
            for (size_t i = 0; i < pointCloud->points.size(); ++i)
            {
                PointT point;
                point.x = pointCloud->points[i].x;
                point.y = pointCloud->points[i].y;
                point.z = pointCloud->points[i].z;
                point.intensity = pointCloud->points[i].intensity;
                point_cloud_in->points.push_back(point);
            }
        }
        else if (tag == 2)
        {
            //Read in XYZINormal points
            pcl::PointCloud<PclPointType>::Ptr pointCloud(new typename pcl::PointCloud<PclPointType>);
            if (pcl::io::loadPCDFile<PclPointType>(fileName, *pointCloud) == -1)
            {
                PCL_ERROR("Couldn't read file\n");
                return false;
            }
            for (size_t i = 0; i < pointCloud->points.size(); ++i)
            {
                PointT point;
                point.x = pointCloud->points[i].x;
                point.y = pointCloud->points[i].y;
                point.z = pointCloud->points[i].z;
                point.intensity = pointCloud->points[i].intensity;
                point.nx = pointCloud->points[i].normal_x;
                point.ny = pointCloud->points[i].normal_y;
                point.nz = pointCloud->points[i].normal_z;
                point_cloud_in->points.push_back(point);
            }
        }
        else
        {
            LOG(WARNING) << "NOT DEFINED TAG TYPE";
        }

        t1 = clock();

        LOG(INFO) << "read PCD: [" << fileName << "]\n done. " << point_cloud_in->points.size() << " points are loaded in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";

        return true;
    }

    //pcdIO -> to pcl point type
    //For temp usage
    bool readPcdFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
    {
        clock_t t0, t1;
        t0 = clock();
        if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't read file\n");
            return false;
        }
        t1 = clock();
        LOG(INFO) << "read PCD: [" << fileName << "]\n done. " << pointCloud->points.size() << " points are loaded in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";
        return true;
    }

    //Write pcd point cloud
    //the input cloud should be pcl point type
    bool writePcdFile(const std::string &fileName,
                      const pcl::PointCloud<PclPointType>::Ptr &pointCloud) const
    {
        clock_t t0, t1;
        t0 = clock();
        if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1)
        {
            PCL_ERROR("Couldn't write file\n");
            return false;
        }
        t1 = clock();
        LOG(INFO) << "write PCD: [" << fileName << "]\n done." << pointCloud->points.size() << " points are written out in " << (float(t1 - t0) / CLOCKS_PER_SEC * 1000) << "ms";
        return true;
    }

    //For batch point cloud reading from folder
#if 1
    bool batchReadFileNamesInFolders(const std::string &folderName, const std::string &extension, std::vector<std::string> &fileNames)
    {
        if (!boost::filesystem::exists(folderName))
        {
            return 0;
        }
        else
        {
            boost::filesystem::directory_iterator end_iter;
            for (boost::filesystem::directory_iterator iter(folderName); iter != end_iter; ++iter)
            {
                if (is_regular_file(iter->status()))
                {
                    std::string fileName;
                    fileName = iter->path().string();

                    boost::filesystem::path dir(fileName);

                    if (!dir.extension().string().empty())
                    {
                        if (!fileName.substr(fileName.rfind('.')).compare(extension))
                        {
                            fileNames.push_back(fileName);
                        }
                    }
                }
            }
        }
        LOG(INFO) << "There are " << fileNames.size() << " " << extension << " files found in the folder";
        return 1;
    }
#endif

    //Convert our point type to pcl point type
    bool OurPoint2PclPoint(const std::shared_ptr<PointCloud> cld_in_ptr,
                           typename pcl::PointCloud<PclPointType>::Ptr cld_out_ptr) const {
        cld_out_ptr->points.clear();
        PclPointType pt;
        for (unsigned int i = 0; i < cld_in_ptr->points.size(); i++)
        {
            pt.x = cld_in_ptr->points[i].x;
            pt.y = cld_in_ptr->points[i].y;
            pt.z = cld_in_ptr->points[i].z;
            pt.intensity = cld_in_ptr->points[i].intensity;
            pt.normal_x = cld_in_ptr->points[i].nx;
            pt.normal_y = cld_in_ptr->points[i].ny;
            pt.normal_z = cld_in_ptr->points[i].nz;
            cld_out_ptr->points.push_back(pt);
        }
    }

    //Convert pcl point type to our point type
    bool PclPoint2OurPoint(typename pcl::PointCloud<PclPointType>::Ptr cld_in_ptr,
                           const std::shared_ptr<PointCloud> cld_out_ptr)
    {
        cld_out_ptr->points.clear();
        PointT pt;
        for (unsigned int i = 0; i < cld_in_ptr->points.size(); i++)
        {
            pt.x = cld_in_ptr->points[i].x;
            pt.y = cld_in_ptr->points[i].y;
            pt.z = cld_in_ptr->points[i].z;
            pt.intensity = cld_in_ptr->points[i].intensity;
            pt.nx = cld_in_ptr->points[i].normal_x;
            pt.ny = cld_in_ptr->points[i].normal_y;
            pt.nz = cld_in_ptr->points[i].normal_z;
            cld_out_ptr->points.push_back(pt);
        }
    }

    // pose/imu IO
    bool ReadPoses(const std::string &fileName, std::vector<Eigen::Matrix4d> &poses)
    {
        std::ifstream in(fileName.c_str(), std::ios::in);
        if (!in)
        {
            return 0;
        }

        Eigen::Matrix4d pose_;
        std::string file;
        int i = 0;

        while (!in.eof())
        {
            in >> file;
            in >> pose_(0, 0) >> pose_(0, 1) >> pose_(0, 2) >> pose_(0, 3);
            in >> pose_(1, 0) >> pose_(1, 1) >> pose_(1, 2) >> pose_(1, 3);
            in >> pose_(2, 0) >> pose_(2, 1) >> pose_(2, 2) >> pose_(2, 3);

            if (pose_(0, 3) > 1000000 || pose_(1, 3) > 1000000)
                printf("Translation is too large. Do global shift to avoid precison loss\n");
            if (in.fail())
            {
                break;
            }
            pose_(3, 0) = 0;
            pose_(3, 1) = 0;
            pose_(3, 2) = 0;
            pose_(3, 3) = 1;
            poses.push_back(pose_);

            ++i;
        }
        in.close();

        return true;
    }
    
    // For KITTI Odometry Ground Truth pose 
    bool GetPoseFromFile(std::string &file_string, std::vector<Eigen::Matrix4d> &pose_vec) {
        std::ifstream tmp_file(file_string);
        pose_vec.clear();
        std::string line;
        int cnt = 0;
        if (tmp_file.is_open()) {
            while (getline(tmp_file, line)) {
                Eigen::Matrix4d tmp_mat;
                double tmp;
                std::istringstream in(line);
                for (int id = 0; id < 12; id++) {
                   in >> tmp;
                   int idy = id % 4;
                   int idx = id / 4;
                   tmp_mat(idx, idy) = tmp;
                }
                tmp_mat.block<1, 4>(3, 0) << 0., 0., 0., 1.;
                pose_vec.push_back(tmp_mat);
            }
        }
        else {
             std::cout << "Unable to open input file" << std::endl;
             return 0;
        }
        tmp_file.close();
        return 1;
    }


    bool ReadIMUData(const std::string &fileName, std::vector<std::vector<imu_info_t>> &imu_datas)
    {
        std::ifstream in(fileName.c_str(), std::ios::in);
        if (!in)
        {
            return false;
        }

        imu_info_t temp_imu_info;
        std::string file;
        while (!in.eof())
        {
            std::vector<imu_info_t> temp_imu_infos;
            temp_imu_infos.resize(imu_infos_t::frequncy);
            for (size_t j = 0; j < imu_infos_t::frequncy; ++j)
            {
                in >> file;
                in >> temp_imu_info.ax >> temp_imu_info.ay >> temp_imu_info.az >> temp_imu_info.wx >> temp_imu_info.wy >> temp_imu_info.wz;
                temp_imu_infos[j] = temp_imu_info;
            }
            imu_datas.push_back(temp_imu_infos);
        }
        in.close();

        return true;
    }

    bool PoseMatToPose3d(pose_mat_t &poseMat, pose_qua_t &poseQua)
    {
        /*
                 *  Eigen Store Matrix first in col, then in row.
                 *  So it looks like [0, 1, 2, 3, 4, 5, 6, 7, 8] --> | 0  3  6 |
                 *                                                   | 1  4  7 |
                 *                                                   | 2  5  8 |
                 *  be careful when using !!! Eigen::Map !!!
                 * */
        double rot_matrix[9] = {poseMat.transform[0], poseMat.transform[4], poseMat.transform[8],
                                poseMat.transform[1], poseMat.transform[5], poseMat.transform[9],
                                poseMat.transform[2], poseMat.transform[6], poseMat.transform[10]};
        poseQua.quat = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(rot_matrix);
        poseQua.trans << poseMat.transform[3], poseMat.transform[7], poseMat.transform[11];

        //        printf("rot_matrix:\n"
        //               "%lf %lf %lf %lf\n"
        //               "%lf %lf %lf %lf\n"
        //               "%lf %lf %lf %lf\n", poseMat.transform[0], poseMat.transform[1], poseMat.transform[2],poseMat.transform[3],
        //               poseMat.transform[4], poseMat.transform[5], poseMat.transform[6], poseMat.transform[7],
        //               poseMat.transform[8], poseMat.transform[9], poseMat.transform[10], poseMat.transform[11]);
        //        std::cout << "qua:" << std::endl;
        //        std::cout << poseQua.quat.toRotationMatrix() << std::endl;
        //        std::cout << poseQua.trans << std::endl;

        return true;
    }

    bool PoseMatToPose3d(pose_mat_t &poseMat, pose_se3_t &poseSe3)
    {
        /*
                 *  Matrix is stored first in row, then in col.
                 *  So it looks like [0, 1, 2, 3, 4, 5, 6, 7, 8] --> | 0  1  2 |
                 *                                                   | 3  4  5 |
                 *                                                   | 6  7  8 |
                 * */
        double rot_matrix[9] = {poseMat.transform[0], poseMat.transform[1], poseMat.transform[2],
                                poseMat.transform[4], poseMat.transform[5], poseMat.transform[6],
                                poseMat.transform[8], poseMat.transform[9], poseMat.transform[10]};
        ceres::RotationMatrixToAngleAxis(rot_matrix, poseSe3.se3);
        poseSe3.se3[3] = poseMat.transform[3];
        poseSe3.se3[4] = poseMat.transform[7];
        poseSe3.se3[5] = poseMat.transform[11];

        return true;
    }

    /*
     * Reads Vertexes and Edges from pose graph file
     * SUBMAP NODE transaction_id submap_id r11 r12 r13 t1 r21 r22 r23 t2 r31 r32 r33 t3
     * EDGE 1_transaction_id 1_submap_id 2_transaction_id 2_submap_id TYPE: ADJACENT r11 r12 r13 t1 r21 r22 r23 t2 r31 r32 r33 t3
     * */
    bool LoadPoseGraphFromFile(const std::string &filename, MapOfSubMaps &submaps,
                               VectorOfEdges &edges) const
    {
        submaps.clear();
        edges.clear();

        //    LOG(ERROR) << "Try Open File: " << filename;
        std::ifstream infile(filename.c_str());
        if (!infile)
        {
            LOG(ERROR) << "Open file Failed!" << filename;
            return false;
        }

        std::string data_type;
        while (infile.good())
        {
            // Read whether the type is a node or a constraint.
            infile >> data_type;
            if (data_type == "SUBMAP")
            {
                if (!LoadSubmapFromStream(infile, submaps))
                {
                    LOG(ERROR) << "Read Submaps Failed!";
                    return false;
                }
            }
            else if (data_type == "EDGE")
            {
                //                if (!ReadConstraint(&infile, edges))
                if (!LoadEdgeFromStream(infile, edges))
                {
                    LOG(ERROR) << "Read Edges Failed!";
                    return false;
                }
            }
            else
            {
                LOG(ERROR) << "Unknown data type: " << data_type;
                return false;
            }
            // Clear any trailing whitespace from the line.
            infile >> std::ws;
        }

        return true;
    }

    bool LoadSubmapFromStream(std::ifstream &infile, MapOfSubMaps &sub_maps) const
    {
        CHECK(infile) << "ifstream is closed!";
        submap_id_t submapId;
        Submap submap;
        infile >> submapId >> submap;
        submap.index_type = Submap::Continuous;
        submap.submap_id = submapId;

        // Ensure we don't have duplicate poses.
        if (sub_maps.find(submapId) != sub_maps.end())
        {
            LOG(ERROR) << "Duplicate vertex with ID: (" << submapId.transaction_id << ", " << submapId.submap_id << ")";
            return false;
        }
        auto result = sub_maps.insert(MapOfSubMaps::value_type(submapId, submap));
        CHECK(result.second) << "Load Submap " << result.first->first << " failed!";

        return true;
    }

    bool LoadSubmapFromFile(const std::string &filename, MapOfSubMaps &sub_maps) const
    {
        sub_maps.clear();

        std::ifstream infile(filename.c_str());
        if (!infile)
        {
            LOG(ERROR) << "Open file Failed!" << filename;
            return false;
        }

        std::string data_type;
        while (infile.good())
        {
            // Read whether the type is a node or a constraint.
            infile >> data_type;
            CHECK(data_type == "SUBMAP") << "Content is not SUBMAP, file is " << filename;
            if (!LoadSubmapFromStream(infile, sub_maps))
            {
                LOG(ERROR) << "Read Submaps Failed!";
                return false;
            }
            // Clear any trailing whitespace from the line.
            infile >> std::ws;
        }

        return true;
    }

    bool LoadEdgeFromStream(std::ifstream &infile, VectorOfEdges &edges) const
    {
        CHECK(infile) << "ifstream is closed!";
        submap_id_t submapId_b;
        submap_id_t submapId_e;
        std::string edge_type;
        Pose3d pose3D;
        Eigen::Matrix<double, 6, 6> information_matrix;

        infile >> submapId_b >> submapId_e >> edge_type >> pose3D >> information_matrix;

        Edge edge;
        edge.pose = pose3D;
        edge.submap_idx.first_submap = submapId_b;
        edge.submap_idx.second_submap = submapId_e;
        edge.information_matrix = information_matrix;
        if (edge_type == "ADJACENT")
            edge.edge_type = Edge::Adjacent;
        else if (edge_type == "INTRA")
            edge.edge_type = Edge::Intra;
        else if (edge_type == "INTER")
            edge.edge_type = Edge::Inter;
        else
            LOG(ERROR) << "Wrong Edge Type!";

        edges.push_back(edge);

        return true;
    }

    bool LoadEdgeFromFile(const std::string &filename, VectorOfEdges &edges) const
    {
        edges.clear();

        std::ifstream infile(filename.c_str());
        if (!infile)
        {
            LOG(ERROR) << "Open file Failed!" << filename;
            return false;
        }

        std::string data_type;
        while (infile.good())
        {
            // Read whether the type is a node or a constraint.
            infile >> data_type;
            CHECK(data_type == "EDGE") << "Content is not EDGE, file is " << filename;

            if (!LoadEdgeFromStream(infile, edges))
            {
                LOG(ERROR) << "Read Edges Failed!";
                return false;
            }
            // Clear any trailing whitespace from the line.
            infile >> std::ws;
        }

        return true;
    }

#if 0 //Fix it later
    bool writeOdomPose(const std::string &output_folder, Transaction &transaction)
    {
        std::string output_filename;
        output_filename = output_folder + "/Submap_Frame_Odom_Pose.txt";

        FILE *fp = fopen(output_filename.c_str(), "w");
        for (int i = 0; i < transaction.submap_number; i++)
        {
            for (int j = 0; j < transaction.submaps[i].frame_number; j++)
            {
                //FRAME #TRAN #SUBMAP #FRAME POSE3*4
                fprintf(fp, "FRAME\t%d\t%d\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", transaction.unique_id, transaction.submaps[i].id_in_transaction, transaction.submaps[i].frames[j].id_in_transaction,
                        transaction.submaps[i].frames[j].odom_pose(0, 0), transaction.submaps[i].frames[j].odom_pose(0, 1), transaction.submaps[i].frames[j].odom_pose(0, 2), transaction.submaps[i].frames[j].odom_pose(0, 3),
                        transaction.submaps[i].frames[j].odom_pose(1, 0), transaction.submaps[i].frames[j].odom_pose(1, 1), transaction.submaps[i].frames[j].odom_pose(1, 2), transaction.submaps[i].frames[j].odom_pose(1, 3),
                        transaction.submaps[i].frames[j].odom_pose(2, 0), transaction.submaps[i].frames[j].odom_pose(2, 1), transaction.submaps[i].frames[j].odom_pose(2, 2), transaction.submaps[i].frames[j].odom_pose(2, 3));
            }
        }
        fclose(fp);

        LOG(INFO) << "Output finished ... ...";
        return 1;
    }
#endif

    bool writeDeltaCalib(const std::string &output_folder, const std::string &output_filename, const Pose3d &calib_delta) const {
        checkDir(output_folder);
        std::string output_path;
        output_path = output_folder + "/" + output_filename;

        LOG(INFO) << "Output the pose graph delta_calib";
        LOG(INFO) << "File: " << output_path;

        std::ofstream outfile(output_path);
        if (outfile) {
            outfile << calib_delta << "\t";
        }
        outfile.close();

        LOG(INFO) << "Output finished ... ...";
        return 1;
    }

    bool loadDeltaCalib(const std::string &filename, Pose3d &calib_delta) const {
        std::ifstream infile(filename.c_str());
        if (!infile) {
            LOG(ERROR) << "Open file Failed!" << filename;
            return false;
        }

        while (infile.good())
        {
            // Read whether the type is a node or a constraint.
            infile >> calib_delta;
            infile >> std::ws;
            LOG(INFO) << "calib delta is " << calib_delta;
        }

        return true;
    }

    bool writePoseGraph(const std::string &output_folder, const std::string &output_filename, VectorOfSubmaps &sub_maps, VectorOfEdges &hdmap_edges)
    {
        checkDir(output_folder);
        std::string output_path;
        output_path = output_folder + "/" + output_filename;

        LOG(INFO) << "Output the pose graph edges and nodes";
        LOG(INFO) << "File: " << output_path;

        std::ofstream outfile(output_path);

        if (outfile)
        {
            CHECK(writeSubmapsIntoStream(outfile, sub_maps)) << "write submaps into stream failed!";
            CHECK(writeEdgesIntoStream(outfile, hdmap_edges)) << "write edges into stream failed!";
        }
        outfile.close();

        LOG(INFO) << "Output finished ... ...";
        return 1;
    }

    bool writeSubmaps(const std::string &output_folder, const std::string &output_filename, VectorOfSubmaps &sub_maps)
    {
        checkDir(output_folder);
        std::string output_path;
        output_path = output_folder + "/" + output_filename;

        LOG(INFO) << "Output the pose graph Submaps";
        LOG(INFO) << "File: " << output_path;

        std::ofstream outfile(output_path);

        if (outfile)
        {
            CHECK(writeSubmapsIntoStream(outfile, sub_maps)) << "write submaps into stream failed!";
        }
        outfile.close();

        LOG(INFO) << "Output finished ... ...";
        return 1;
    }

    bool writeSubmapsIntoStream(std::ofstream &outfile, VectorOfSubmaps &sub_maps)
    {
        CHECK(outfile) << "ofstream is closed!";

        for (int i = 0; i < sub_maps.size(); i++) //Node
        {                                         //SUBMAP #TRAN #SUBMAP POSE3*4
            outfile << "SUBMAP\t" << sub_maps[i].submap_id << "\t" << sub_maps[i] << "\n";
            //                outfile << "SUBMAP\t" << sub_maps[i].submap_id << sub_maps[i].pose << "\n";
        }
        return true;
    }

    bool writeEdgesIntoStream(std::ofstream &outfile, VectorOfEdges &hdmap_edges)
    {
        CHECK(outfile) << "ofstream is closed!";

        for (int i = 0; i < hdmap_edges.size(); i++) //Edges (Transformation from Node 2 to Node 1)
        {                                            //EDGE #TRAN1 #SUBMAP1 #TRAN2 #SUBMAP2 EDGETYPE POSE3*4 INFO-MATRIX 6*6
            outfile << "EDGE\t" << hdmap_edges[i].submap_idx.first_submap << hdmap_edges[i].submap_idx.second_submap;
            switch (hdmap_edges[i].edge_type)
            {
            case Edge::Adjacent:
                outfile << "ADJACENT\t";
                break;
            case Edge::Intra:
                outfile << "INTRA\t";
                break;
            case Edge::Inter:
                outfile << "INTER\t";
                break;
            default:
                outfile << "UNKNOWN\t";
                break;
            }
            outfile << hdmap_edges[i].pose << hdmap_edges[i].information_matrix << "\n";
        }
        return true;
    }

    bool writeEdges(const std::string &output_folder, const std::string &output_filename, VectorOfEdges &hdmap_edges)
    {
        checkDir(output_folder);
        std::string output_path;
        output_path = output_folder + "/" + output_filename;

        LOG(INFO) << "Output the pose graph edges";
        LOG(INFO) << "File: " << output_path;

        std::ofstream outfile(output_path);

        if (outfile)
        {
            CHECK(writeEdgesIntoStream(outfile, hdmap_edges)) << "write edges into stream failed!";
        }
        outfile.close();

        LOG(INFO) << "Output finished ... ...";
        return 1;
    }

    bool WritePGOResultOut(const std::string &output_folder, const std::string &output_filename, const std::vector<submapid_error_t, Eigen::aligned_allocator<submapid_error_t> > err_submaps,
        const std::vector<edgeid_error_t, Eigen::aligned_allocator<edgeid_error_t> > err_edges) const {
        checkDir(output_folder);
        std::string output_path;
        output_path = output_folder + "/" + output_filename;

        LOG(INFO) << "Output PGO Result, written to " << output_path;

        std::ofstream outfile(output_path);

        if (outfile) {
            for (int i = 0; i < err_submaps.size(); ++i) {
                outfile << "SUBMAP_ERROR" << "\t" << err_submaps[i].submap_id << "\t" << err_submaps[i].theta << "\t" << err_submaps[i].pose_error << "\n";
            }
            for (int i = 0; i < err_edges.size(); ++i) {
                outfile << "EDGE_ERROR" << "\t" << err_edges[i].edge.edge_type << "\t" << err_edges[i].edge.submap_idx << "\t" << err_edges[i].theta << "\t" << err_edges[i].pose_error << "\n";
            }

        }
        outfile.close();

        LOG(INFO) << "Output PGO Result finished ... ...";

        return true;
    }

private:
};

} // namespace map_pose

#endif // _INCLUDE_DATA_IO_H_
