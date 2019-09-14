
#ifndef _INCLUDE_TYPES_H_
#define _INCLUDE_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <map>
#include <memory>
#include <ceres/rotation.h>
#include <glog/logging.h>

//Max and Min
#define max_(a, b) (((a) > (b)) ? (a) : (b))
#define min_(a, b) (((a) < (b)) ? (a) : (b))

namespace map_pose
{

struct optimization_param_t
{
    std::string trust_region_strategy = "levenberg_marquardt";
    std::string linear_solver = "dense_schur";
    std::string sparse_linear_algebra_library = "suite_sparse";
    std::string dense_linear_algebra_library = "eigen";

    std::string ordering; // marginalization ..

    bool robustify = true; // loss function
    std::string loss_function;

    // double eta;
    int num_threads = 1; // default = 1
    int num_iterations = 10;

    std::string pose_graph_input_file = "";
    std::string pose_graph_output_folder = "";
    std::string pose_graph_output_file = "";
    std::string pose_graph_output_calib_file = "";
    std::vector<std::string> transaction_lidar_root_path;

    double submap_upper_bound_x = 0.2;
    double submap_upper_bound_y = 0.2;
    double submap_upper_bound_z = 5;
    double submap_lower_bound_x = 0.2;
    double submap_lower_bound_y = 0.2;
    double submap_lower_bound_z = 5;
    double calib_upper_bound_x = 0.1;
    double calib_upper_bound_y = 0.1;
    double calib_upper_bound_z = 0.1;
    double calib_lower_bound_x = 0.1;
    double calib_lower_bound_y = 0.1;
    double calib_lower_bound_z = 0.1;
};

struct loop_detection_param_t
{
    double distance;    // distance between submap center points
    double cross_ratio; // cross parts between two point cloud boundingboxes (IoU)
    unsigned int revisit_min_id_interval;
    unsigned int revisit_max_edge_per_submap;
};

struct center_point_t
{
    double x, y, z;
    center_point_t(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

struct bounds_t
{
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    bounds_t()
    {
        min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
    }
};

struct submap_id_t
{
    unsigned int transaction_id;
    unsigned int submap_id;
    unsigned int hdmap_unique_idx;

public:
    // store in STL sorted container (std::map std::set)
    bool operator<(const submap_id_t &smid) const
    {
        if (this->transaction_id < smid.transaction_id)
            return true;
        else if (this->transaction_id > smid.transaction_id)
            return false;
        else
        {
            if (this->submap_id < smid.submap_id)
                return true;
            else
                return false;
        }
    }
    bool operator>(const submap_id_t &smid) const
    {
        if (this->transaction_id > smid.transaction_id)
            return true;
        else if (this->transaction_id < smid.transaction_id)
            return false;
        else
        {
            if (this->submap_id > smid.submap_id)
                return true;
            else
                return false;
        }
    }
    bool operator==(const submap_id_t &smid) const {
        return ((this->transaction_id == smid.transaction_id) && (this->submap_id == smid.submap_id));
    }
};

struct pose_se3_t
{
    // 0, 1, 2      so3 | rotation vector
    // 3, 4, 5      translation
    double se3[6];

    Eigen::Matrix4d GetMatirx()
    {
        double rotation_matrix[9];
        ceres::AngleAxisToRotationMatrix(se3, rotation_matrix);
        Eigen::Matrix4d transformation_matrix;
        transformation_matrix << rotation_matrix[0], rotation_matrix[1], rotation_matrix[2], se3[3],
            rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], se3[4],
            rotation_matrix[6], rotation_matrix[7], rotation_matrix[8], se3[5],
            0, 0, 0, 1;
        return transformation_matrix;
    }
};

struct pose_qua_t
{
    Eigen::Vector3d trans;
    Eigen::Quaterniond quat;

    pose_qua_t()
    {
        trans << 0, 0, 0;
        quat = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
        quat.normalize();
    }
    pose_qua_t(const pose_qua_t &pose)
    {
        this->copyFrom(pose);
    }
    pose_qua_t(Eigen::Quaterniond quat,
               Eigen::Vector3d trans) : trans(trans), quat(quat)
    {
        quat.normalize();
    }
    pose_qua_t operator*(const pose_qua_t &pose) const
    {
        return pose_qua_t(this->quat.normalized() * pose.quat.normalized(), this->quat.normalized() * pose.trans + this->trans);
    }
    bool operator==(const pose_qua_t &pose) const
    {
        if (this->quat.x() == pose.quat.x() &&
            this->quat.y() == pose.quat.y() &&
            this->quat.z() == pose.quat.z() &&
            this->quat.w() == pose.quat.w() &&
            this->trans == pose.trans)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    Eigen::Matrix4d GetMatrix() const
    {
        //CHECK(quat.norm() == 1) << "NO EQUAL";
        Eigen::Matrix4d transformation_matrix;
        transformation_matrix.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix(); //You need to gurantee the quat is normalized
        transformation_matrix.block<3, 1>(0, 3) = trans;
        transformation_matrix.block<1, 4>(3, 0) << 0, 0, 0, 1;
        return transformation_matrix;
    }
    void SetPose(Eigen::Matrix4d transformation)
    {
        quat = Eigen::Quaterniond(transformation.block<3, 3>(0, 0)).normalized();
        trans << transformation(0, 3), transformation(1, 3), transformation(2, 3);
    }
    void copyFrom(const pose_qua_t &pose)
    {
        trans = pose.trans;
        //  trans << pose.trans[0], pose.trans[1], pose.trans[2];
        quat = Eigen::Quaterniond(pose.quat);
    }
    // inverse and return
    pose_qua_t inverse()
    {
        Eigen::Matrix4d transformation_matrix = GetMatrix();
        SetPose(transformation_matrix.inverse());
        return *this;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// used when reading pose from file
struct pose_mat_t
{
    double transform[12];
};

struct submap_pair_t
{
    submap_id_t first_submap;
    submap_id_t second_submap;

    bool operator==(const submap_pair_t &idx) const {
        return (this->first_submap == idx.first_submap && this->second_submap == idx.second_submap);
    }
};

template <typename Pose3d>
struct edge_t
{
    enum EdgeType
    {
        Adjacent,
        Inter,
        Intra
    };
    EdgeType edge_type = Adjacent;
    submap_pair_t submap_idx;
    Pose3d pose;
    Eigen::Matrix<double, 6, 6> information_matrix =
        Eigen::Matrix<double, 6, 6>::Identity();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct imu_info_t
{
    double ax;
    double ay;
    double az;
    double wx; //unit:degree
    double wy; //unit:degree
    double wz; //unit:degree

    timeval time_stamp;

    bool SetImuInfo(double ax_i, double ay_i, double az_i,
                    double wx_i, double wy_i, double wz_i,
                    timeval time_stamp_i)
    {
        ax = ax_i;
        ay = ay_i;
        az = az_i;
        wx = wx_i;
        wy = wy_i;
        wz = wz_i;
        time_stamp = time_stamp_i;
    }
};

struct imu_infos_t
{
    static const int frequncy = 10;
    //    std::array<imu_info_t, frequncy> imu_infos;     // one ImuInfos includes 10 ImuInfo
    std::vector<imu_info_t> imu_infos;
    timeval time_stamp; // time_stamp is the sensor time of the last ImuInfo
};

template <typename Pose3d>
struct gnss_t
{
    gnss_t() {}
    gnss_t(double origin_x, double origin_y, double latitude,
           double longitude, double height, double row,
           double pitch, double yaw){}; // decode LLT into (x, y) in constructor

    Pose3d pose;
    Eigen::Matrix<double, 6, 6> information_matrix =
        Eigen::Matrix<double, 6, 6>::Identity() * 1e3;
    timeval time_stamp;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct pointXYZ_t
{
    double x;
    double y;
    double z;
};

struct pointXYZI_t
{
    double x;
    double y;
    double z;
    float intensity;
};

struct pointXYRGB_t
{
    double x;
    double y;
    double z;
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

struct pointXYZRGBI_t
{
    double x;
    double y;
    double z;
    unsigned char r;
    unsigned char g;
    unsigned char b;
    float intensity;
};

struct pointXYZINormal_t
{
    double x;
    double y;
    double z;
    float intensity;
    double nx;
    double ny;
    double nz;

    pointXYZINormal_t() : x(0), y(0), z(0),
                          intensity(0), nx(0), ny(0), nz(0) {}
};

template <typename PointType>
struct point_cloud_t
{
    std::vector<PointType> points;
    double timestamp;
};

template <typename PointType, typename Pose3d>
struct frame_t
{
    //    unsigned int unique_id;
    //    unsigned int transaction_id;
    //    unsigned int id_in_transaction;
    //    unsigned int submap_id;
    unsigned int id_in_submap;
    std::string pcd_file_name; // e.g. HDL_64_*.pcd
    bounds_t bbox;
    center_point_t center_point;
    Pose3d pose;                                             // pose in submap
    Pose3d last_transform;                                   // transformation between this frame and the last frame
    std::shared_ptr<point_cloud_t<PointType>> cld_lidar_ptr; // downsampled point cloud in Submap
    std::vector<unsigned int> ground_index;
    std::vector<unsigned int> ground_down_index;
    std::vector<unsigned int> edge_index;
    std::vector<unsigned int> edge_down_index;
    std::vector<unsigned int> planar_index;
    std::vector<unsigned int> planar_down_index;
    std::vector<unsigned int> sphere_index;
    std::vector<unsigned int> sphere_down_index;

    timeval time_stamp;

    frame_t() { init(); }
    frame_t(std::string frame_name) : pcd_file_name(frame_name) {}
    //    ~frame_t() {
    //        deep_init();
    //    }
    void init()
    {
        bbox = bounds_t();
        center_point = center_point_t();
        pose = Pose3d();
        cld_lidar_ptr = std::make_shared<point_cloud_t<PointType>>();
    };
    void deep_init()
    {
        release_cld_lidar();
        release_feature_indices();
    }
    void release_cld_lidar()
    {
        std::vector<PointType>().swap(cld_lidar_ptr->points);
        cld_lidar_ptr = std::make_shared<point_cloud_t<PointType>>();
    }
    void release_feature_indices()
    {
        std::vector<unsigned int>().swap(ground_index);
        std::vector<unsigned int>().swap(ground_down_index);
        std::vector<unsigned int>().swap(edge_index);
        std::vector<unsigned int>().swap(edge_down_index);
        std::vector<unsigned int>().swap(planar_index);
        std::vector<unsigned int>().swap(planar_down_index);
        std::vector<unsigned int>().swap(sphere_index);
        std::vector<unsigned int>().swap(sphere_down_index);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointType, typename Pose3d>
struct raw_data_group_t
{
    // A raw data group consists of:
    // 1 -- frame
    // 10 -- imu
    // 1 -- gnss
    enum FLAG
    {
        DEAL,
        UNDEAL
    };
    FLAG flag;
    frame_t<PointType, Pose3d> raw_frame;
    gnss_t<Pose3d> raw_gnss;
    imu_infos_t raw_imu;

    //for test
    gnss_t<Pose3d> noise_gnss;
    gnss_t<Pose3d> body_gnss;
};

// Use Macro instead of typedef
// POSE_QUATERNION -- pose_qua_t
// Default -- pose_se3_t
//#define POSE_QUATERNION
typedef pointXYZINormal_t PointType;
typedef point_cloud_t<PointType> PointCloud;
typedef pose_qua_t Pose3d;
typedef frame_t<PointType, Pose3d> Frame;
typedef gnss_t<Pose3d> Gnss;
typedef std::map<submap_id_t, Gnss, std::less<submap_id_t>, Eigen::aligned_allocator<std::pair<const submap_id_t, Gnss>>> MapOfGnsses;
typedef edge_t<Pose3d> Edge;
typedef std::vector<Edge, Eigen::aligned_allocator<Edge>> VectorOfEdges;
typedef raw_data_group_t<PointType, Pose3d> RawData;
typedef std::vector<RawData, Eigen::aligned_allocator<RawData>> VectorOfRawDatas;

template <typename PointType, typename Pose3d>
struct submap_t
{
    enum IndexType
    {
        Continuous,
        UnContinuous
    };
    std::string full_cld_filename;
    std::string feature_cld_filename;
    std::string featrure_idx_filename;
    Pose3d pose;
    Pose3d last_frame_transform;
    unsigned int frame_number;
    bounds_t bbox;
    center_point_t center_point;
    double submap_mae;
    VectorOfRawDatas raw_data_group;        
    std::shared_ptr<point_cloud_t<PointType>> cld_lidar_ptr;   // downsampled point cloud in Submap
    std::shared_ptr<point_cloud_t<PointType>> cld_feature_ptr; // feature pointcloud
    std::vector<unsigned int> ground_index; 
    std::vector<unsigned int> edge_index;   
    std::vector<unsigned int> planar_index; 
    std::vector<unsigned int> sphere_index; 
    //we don't need downsample feature points up-to-now
    std::vector<unsigned int> ground_down_index;    
    std::vector<unsigned int> edge_down_index;      
    std::vector<unsigned int> planar_down_index;    
    std::vector<unsigned int> sphere_down_index;    
    unsigned int ground_size, edge_size, planar_size, sphere_size;
    IndexType index_type = IndexType::UnContinuous;

    submap_id_t submap_id;

    submap_t() { init(); };
    void init()
    {
        frame_number = 0;
        raw_data_group.clear();
        bbox = bounds_t();
        center_point = center_point_t();
        pose = Pose3d();
    }
    void releaseRawData()
    {
        VectorOfRawDatas().swap(raw_data_group);
    }
    void releaseIndex()
    {
        std::vector<unsigned int>().swap(ground_index);
        std::vector<unsigned int>().swap(edge_index);
        std::vector<unsigned int>().swap(planar_index);
        std::vector<unsigned int>().swap(sphere_index);
        std::vector<unsigned int>().swap(ground_down_index);
        std::vector<unsigned int>().swap(edge_down_index);
        std::vector<unsigned int>().swap(planar_down_index);
        std::vector<unsigned int>().swap(sphere_down_index);
    }
    void release_cld_lidar()
    {
        cld_lidar_ptr = std::make_shared<point_cloud_t<PointType>>();
        cld_feature_ptr = std::make_shared<point_cloud_t<PointType>>();
    }
    // release everything
    void releaseAll()
    {
        //init();
        release_cld_lidar();
        releaseIndex();
        releaseRawData();
    }
};

//For evaluation
struct road_pixel_t
{
    //std::vector<std::string> frame_names;
    float min_z_pgo;
    float max_z_pgo;
    float min_z_gnss;
    float max_z_gnss;
    float intensity;
    bool is_covered_pgo;
    bool is_covered_gnss;

    road_pixel_t()
    {
        min_z_pgo = 9999;
        max_z_pgo = -9999;
        min_z_gnss = 9999;
        max_z_gnss = -9999;
        is_covered_pgo = false;
        is_covered_gnss = false;
    }
};

// For evaluation
struct submapid_error_t {
    // pgo_pose ^ -1 * gnss_pose
    Pose3d pose_error;
    double theta;
    submap_id_t submap_id;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
struct edgeid_error_t {
    // pgo_edge_pose ^ -1 * reg_edge_pose
    Pose3d pose_error;
    double theta;
    Edge edge;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct transaction_param_t
{
    unsigned int transaction_id;
    unsigned int max_frames;
    double rotation_accumulate;
    double translation_accumulate;
    double leaf_size;
    std::string lidar_file_list;
    std::string transaction_root_path;
    std::string lidar_root_path;
    std::string gnss_root_path;
    std::string submap_full_root_path;
    std::string submap_feature_root_path;
    std::string pose_graph_output_path;
    pose_qua_t calib_ldr_to_oxts;
    pose_qua_t calib_cam_to_oxts;
};

typedef submap_t<PointType, Pose3d> Submap;
typedef std::map<submap_id_t, Submap, std::less<submap_id_t>, Eigen::aligned_allocator<std::pair<const submap_id_t, Submap>>> MapOfSubMaps;
typedef std::vector<Submap, Eigen::aligned_allocator<Submap>> VectorOfSubmaps;

inline std::istream &operator>>(std::istream &input, submap_id_t &submapId)
{
    input >> submapId.transaction_id >> submapId.submap_id;

    return input;
}

inline std::ostream &operator<<(std::ostream &output, const submap_id_t &submapId)
{
    //output << "TransactionID: " << submapId.transaction_id << " SubMapID: " << submapId.submap_id << " ";
    output << submapId.transaction_id << "\t" << submapId.submap_id << "\t";
    return output;
}

inline std::ostream &operator<<(std::ostream &output, const submap_pair_t &submap_pair)
{
    output << "first: " << submap_pair.first_submap << "\t"
           << "second: " << submap_pair.second_submap << "\t";
    return output;
}

inline std::istream &operator>>(std::istream &input, std::array<double, 36> &information_matrix)
{
    for (int i = 0; i < 36; ++i)
        input >> information_matrix[i];

    return input;
}

inline std::istream &operator>>(std::istream &input, Eigen::Matrix<double, 6, 6> &information_matrix)
{
    for (int i = 0; i < 36; ++i)
    {
        double num = 0;
        input >> num;
        information_matrix(i / 6, i % 6) = num;
    }

    return input;
}

inline std::ostream &operator<<(std::ostream &output, const Eigen::Matrix<double, 6, 6> &information_matrix)
{
    for (int i = 0; i < 36; ++i)
    {
        double num = information_matrix(i / 6, i % 6);
        output << num << "\t";
    }

    return output;
}

inline std::istream &operator>>(std::istream &input, pose_qua_t &pose)
{
    input >> pose.trans.x() >> pose.trans.y() >> pose.trans.z() >> pose.quat.x() >>
        pose.quat.y() >> pose.quat.z() >> pose.quat.w();
    // Normalize the quaternion to account for precision loss due to
    // serialization.
    pose.quat.normalize();
    return input;
}

inline std::ostream &operator<<(std::ostream &output, const pose_qua_t &pose)
{
    output << pose.trans.x() << "\t" << pose.trans.y() << "\t" << pose.trans.z() << "\t"
           << pose.quat.x() << "\t" << pose.quat.y() << "\t" << pose.quat.z() << "\t" << pose.quat.w() << "\t";
    return output;
}

inline std::ostream &operator<<(std::ostream &output, const bounds_t &bbox)
{
    output << bbox.min_x << "\t" << bbox.min_y << "\t" << bbox.min_z << "\t"
           << bbox.max_x << "\t" << bbox.max_y << "\t" << bbox.max_z << "\t";
    return output;
}

inline std::ostream &operator<<(std::ostream &output, const submap_t<pointXYZINormal_t, pose_qua_t> &submap)
{
    output << submap.feature_cld_filename << "\t" << submap.ground_size << "\t" << submap.edge_size << "\t"
           << submap.planar_size << "\t" << submap.sphere_size << "\t" << submap.pose << "\t" << submap.bbox
           << "\t" << submap.frame_number << "\t";
    for (int i = 0; i < submap.frame_number; ++i)
    {
        output << submap.raw_data_group[i].raw_frame.pcd_file_name << "\t" << submap.raw_data_group[i].raw_frame.pose << "\t";
    }
    for (int i = 0; i < submap.frame_number; ++i)
    {
        output << submap.raw_data_group[i].raw_gnss.pose << "\t";
    }
    output << submap.raw_data_group[0].raw_gnss.information_matrix;
    return output;
}

inline std::istream &operator>>(std::istream &input, submap_t<pointXYZINormal_t, pose_qua_t> &submap)
{
    input >> submap.feature_cld_filename >> submap.ground_size >> submap.edge_size >> submap.planar_size >> submap.sphere_size >>
        submap.pose.trans.x() >> submap.pose.trans.y() >> submap.pose.trans.z() >> submap.pose.quat.x() >> submap.pose.quat.y() >> submap.pose.quat.z() >> submap.pose.quat.w() >>
        submap.bbox.min_x >> submap.bbox.min_y >> submap.bbox.min_z >> submap.bbox.max_x >> submap.bbox.max_y >> submap.bbox.max_z >> submap.frame_number;
    submap.raw_data_group.resize(submap.frame_number);
    for (int i = 0; i < submap.frame_number; ++i)
    {
        input >> submap.raw_data_group[i].raw_frame.pcd_file_name >> submap.raw_data_group[i].raw_frame.pose;
    }
    for (int i = 0; i < submap.frame_number; ++i)
    {
        input >> submap.raw_data_group[i].raw_gnss.pose;
    }
    input >> submap.raw_data_group[0].raw_gnss.information_matrix;
    return input;
}

inline std::istream &operator>>(std::istream &input, pose_mat_t &pose)
{
    input >> pose.transform[0] >> pose.transform[1] >> pose.transform[2] >> pose.transform[3] >> pose.transform[4] >> pose.transform[5] >> pose.transform[6] >> pose.transform[7] >> pose.transform[8] >> pose.transform[9] >> pose.transform[10] >> pose.transform[11];
    return input;
}

inline std::istream &operator>>(std::istream &input, bounds_t bbox)
{
    input >> bbox.min_x >> bbox.min_y >> bbox.min_z >> bbox.max_x >> bbox.max_y >> bbox.max_z;
    return input;
}

} // namespace map_pose

#endif // _INCLUDE_TYPES_H_
