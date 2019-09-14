#ifndef _INCLUDE_LLSLO_H_
#define _INCLUDE_LLSLO_H_

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H

#include <deque>
#include <fstream>
#include <limits>
#include "types.h"
#include "dataio.hpp"
#include "error_compute.h"
#include "filter.hpp"
#include "common_reg.hpp"
#include "sensors/gnssins.h"
#include "back_end_optimization.h"

// proj4
#include <proj_api.h>


namespace map_pose
{

class Transaction {
public:
    Transaction();

    Transaction(transaction_param_t &config);

    ~Transaction();
    
    // Set configure parameters, Fix it Later
    bool SetConfig(unsigned max_frames, double rotation_accumulate, double translation_accumulate);
    
    // Load IMU and GNSS raw datat from PC
    bool LoadPcImuGnss(int begin_frame, int end_frame);
                       
    // Divide the transaction into several submaps according to multiple rules 
    // Rules: consecutive frame number, accumulated translation (using), accumilated heading angle (using) ...
    // Notice, in this fuction, some frames with too little movement would be depreacted and not add to the front-end processing   
    void DivideSubmap();

    // Front-end Entrance (Processing the transaction)
    void FrontEnd();
    
    // Submap Odometry (Processing each submap in the transaction) 
    // Input: a given submap
    bool SubmapOdom(Submap &submap);
    
    // Merge all the frames in the submap to submap's reference coordinate system (submap's first frame)
    // Input: a given submap
    bool MergeFrames(Submap &submap);

    // Build adjacent edge by applying registration between adjacent submaps along the transaction  
    // Input: a given submap_id in the transcation
    bool BuildAdjacentEdge(int submap_id);
    // Batch do all the adjacent registration in the transaction
    bool BuildAdjacentEdges(); 
    
    // Release the memory of all submaps of the transaction
    bool ReleaseAllSubmaps();
    
    // Load KITTI Datasets' data for LO test
    void LoadKITTIData(int begin_frame, int end_frame);

    // Pure Lidar Odometry Entrance (For Lidar Odometry Test)
    void PureLidarOdometry();
    
    // Pure Lidar Odometry Implement, invoked by PureLidarOdometry
    bool RunPureLidarOdometry(Submap &submap);

private:
    
    // Use the submap's frames' bboxs and center points to calculate the submap's bounds and center 
    bool MergeFrameBounds(Submap &submap); 
 
    // Add noise to raw gnss 's pose of the LiDAR coordinate system , Fix it Later
    // Input: a given RawData (1 frame's data), translation noise, rotation noise
    bool AddNoise(RawData &raw_data_group, float noise_std_t, float noise_std_r);
    
    // Get the heading angle change within one frame (10 imu data) using Trapezoidal integral of imu heading angular velocity
    // Input: vector of imu_datas within a frame and the sample time of imu in second
    // Output: the heading changing
    float GetHeading(std::vector<imu_info_t> &imu_datas, float delta_time_second);
    
    // Get the distance between two adjacent frames using their gnss position
    // Input: two position (3d vector) of the adjacent frames provided by gnss
    // Output: the distance between them
    float GetDistance(Eigen::Vector3d &position1, Eigen::Vector3d &position2);
    
    // Get the Mean Absolute Error (MAE) between Lidar Odometry position and GNSS position of a submap
    // Input: a given submap
    // Output: the MAE (unit:m)
    float GetMae(Submap &submap);
    
    // Release a given submap's point cloud (for memory management) 
    bool ReleaseSubmapCld(int submap_id);

// members
public:
    
    //Config Parameters: include calibration data, transaction path, front-end key parameters 
    transaction_param_t config_; 

    // Submaps in Transaction, submap id equals its index in vector
    VectorOfSubmaps sub_maps_;
    
    // the the Mean Absolute Error (MAE) between Lidar Odometry position and GNSS position of the transaction (unit:m)
    double transaction_mae_;

    // transaction unique id
    unsigned int transaction_id_;

    // total frame number of the transaction
    unsigned int frame_number_;

    // total sumap number of the transaction
    unsigned int submap_number_;

    // transaction data root path  
    std::string transaction_root_path_;

    // lidar (pcd) file root path
    std::string lidar_root_path_;

    // lidar (pcd) file list path
    std::string pcd_list_path_;
    
    // gnssins (oxts) file root path
    std::string oxts_root_path_;

    // gnssins (oxts) file list path
    std::string oxts_list_path_;
    
    // Submap's full point cloud (for further display) saving path: HDL64_SubmapFull
    std::string submap_full_root_path_;  

    // Submap's feature point cloud (for loop closure registration) saving path: HDL64_SubmapFeature
    std::string submap_feature_root_path_; 

    // Transaction's pose graph (submap node and adjacent edges) saving path 
    std::string pose_graph_output_path_;

    // Adjacent edges in the transaction (no Loop Edge)
    VectorOfEdges adjacent_edges_;
 
    // RawData is group of useful raw data(PointCloud, GNSS, IMU) [1 raw data per frame]
    // but it may take up too much memory, it should be released after submaps are constructed 
    VectorOfRawDatas raw_datas_;

    // Calibration Matrix (Fix it, put it in the config file later)
    Pose3d calib_ldr_to_oxts_; // Tbl (b: body frame, l: lidar frame)
    Pose3d calib_cam_to_oxts_; // Tbc (b: body frame, c: camera frame)

    // Position Datum Origin (x,y,z unit:m)
    // This is the origin point of the map, which means the projected coordinate of each frame is 
    // shifted by minus the origin's projection coordinate
    // The aim is to avoid the precison loss when processing large figure (projected coordinate)
    double origin_datum_position_[3];

    // id of first sub map
    std::deque<int> active_submaps_;
        
    // Load Data API 
    DataIo<PointType> data_loader_;
    // Point Cloud Filter API
    CFilter<PointType> filter_;
    // Point Cloud Registration API
    CRegistration<PointType> reg_;
    // Visualization API
    MapViewer<PointType> viewer_;

    MapOfSubMaps map_of_submaps_;

    const unsigned int CAPACITY_ = 2;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace map_pose

#endif // _INCLUDE_TRANSACTION_H_
