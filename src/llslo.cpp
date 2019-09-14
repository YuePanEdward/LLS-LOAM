
#include "llslo.h"

namespace map_pose {

// Get the 6DOF pose from OXTS data
// Here, we use proj4 library to do the UTM (Universe Tranverse Mecator) Projection
// proj4 library: https://github.com/OSGeo/PROJ (Refer the website for download methods and 'readme')
// Instead of the KITTI's projection calculation method
// According to experiment, there are up to 5 meter difference between the two projection method in a local map (1km^2)
// So we choose to use the standard UTM projection
Eigen::Matrix4d GetTransform(sensor::gnssins_info_t &gnss_info) {
    
    const static double global_scale = 0.863828;
    const static double earth_radius = 6378137.0; // unit m

    //double lat = gnss_info.lat * M_PI / 180;
    //double lon = gnss_info.lon * M_PI / 180; //unit: rad
    double lat = gnss_info.lat; //unit:degree
    double lon = gnss_info.lon; //unit:degree
    double alt = gnss_info.alt;
    double roll = gnss_info.roll * M_PI / 180;
    double pitch = gnss_info.pitch * M_PI / 180;
    double yaw = gnss_info.yaw * M_PI / 180;

    // Calculate Rotation from roll, pitch, yaw
    Eigen::Matrix3d Rx(Eigen::Matrix3d::Identity());
    Rx(1, 1) = Rx(2, 2) = cos(roll);
    Rx(2, 1) = sin(roll);
    Rx(1, 2) = -Rx(2, 1);

    Eigen::Matrix3d Ry(Eigen::Matrix3d::Identity());
    Ry(0, 0) = Ry(2, 2) = cos(pitch);
    Ry(0, 2) = sin(pitch);
    Ry(2, 0) = -Ry(0, 2);

    Eigen::Matrix3d Rz(Eigen::Matrix3d::Identity());
    Rz(0, 0) = Rz(1, 1) = cos(yaw);
    Rz(1, 0) = sin(yaw);
    Rz(0, 1) = -Rz(1, 0);

    // A reference: Compare of the same single points' coordinate calculated by UTM and KITTI's projection
    // UTM 51 Zone Proj X:396595.067945 , Y:3414994.320534
    // KITTI Proj X:11723782.924684 , Y:3122787.514189
    
    // Use proj4 to do the UTM projection
    projPJ pj_merc, pj_latlong;
    
    // Notice the UTM projection zone
    // Shanghai/Hangzhou UTM-WGS84 Zone 51 N
    // Tokyo 54 N
    // Beijing/Shenzhen/HongKong 50 N
    
    // From WGS84 geodesy (latlon) system to WGS84 UTM map system (XYZ)  
    if (!(pj_merc = pj_init_plus("+proj=utm +zone=51 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs"))) 
        exit(1);
    if (!(pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs"))) //WGS84
        exit(1);
    double x_utm = lon * DEG_TO_RAD;
    double y_utm = lat * DEG_TO_RAD;

    int p = pj_transform(pj_latlong, pj_merc, 1, 1, &x_utm, &y_utm, NULL);

    //Free Memory
    pj_free(pj_merc);
    pj_free(pj_latlong);

    Eigen::Vector3d trans;
    trans[0] = x_utm;
    trans[1] = y_utm;
    trans[2] = alt;

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = Rx * Ry * Rz;
    pose.block<3, 1>(0, 3) << trans[0], trans[1], trans[2];

    return pose;
}

Transaction::Transaction() {}

// Set the config parameters
Transaction::Transaction(transaction_param_t &config) {

    transaction_id_ = config.transaction_id; // transaction id
    frame_number_ = 0;
    submap_number_ = 0;
    transaction_root_path_ = config.transaction_root_path;
    lidar_root_path_ = config.lidar_root_path; // root path saved Transaction Data
    submap_full_root_path_ = config.submap_full_root_path;
    submap_feature_root_path_ = config.submap_feature_root_path;
    pose_graph_output_path_ = config.pose_graph_output_path;
    oxts_root_path_ = config.gnss_root_path;
    pcd_list_path_ = config.lidar_file_list;
    oxts_list_path_ = "";
    sub_maps_.clear(); // all sub maps
    VectorOfSubmaps().swap(sub_maps_);
    VectorOfEdges().swap(adjacent_edges_);
    calib_ldr_to_oxts_.copyFrom(config.calib_ldr_to_oxts);
    calib_cam_to_oxts_.copyFrom(config.calib_cam_to_oxts);
    std::deque<int>().swap(active_submaps_); // id of first sub map
    // data_loader_;
    SetConfig(config.max_frames, config.rotation_accumulate, config.translation_accumulate);
}

Transaction::~Transaction() {}

// Set configure parameters, Fix it Later
bool Transaction::SetConfig(unsigned max_frames, double rotation_accumulate,
                            double translation_accumulate) {
    config_.max_frames = max_frames;
    config_.rotation_accumulate = rotation_accumulate;
    config_.translation_accumulate = translation_accumulate;
    return true;
}

// Load IMU and GNSS raw datat from PC
bool Transaction::LoadPcImuGnss(int begin_frame, int end_frame) {
    if (!raw_datas_.empty()) {
        LOG(WARNING) << "Transaction " << transaction_id_ << " has loaded some Data, those Data will be deleted!";
        raw_datas_.clear(); // Notice: memory is not released here
    }

    sensor::gnssins_params_t gnssins_params(sensor::GNSS_OXTS, 0, oxts_root_path_, "OXTS");
    sensor::GNSSINS gnssins(gnssins_params);
    std::ifstream lidar_file_list(pcd_list_path_);
    if (!lidar_file_list.is_open()) {
        LOG(ERROR) << "open lidar_file_list failed, file is " << pcd_list_path_;
        return false;
    }

    // Define transformation

    // Frame install transform matrix
    Eigen::Matrix3d R_trans;
    R_trans << 0, 1, 0,
               1, 0, 0,
               0, 0,-1;

    Eigen::Matrix4d T_l2b; // Calibration transformation from Lidar to Body

    // Fix it later (add to the config file and read in the calibration matrixs)
    Eigen::Matrix4d T_18, T_19, T_new;

    T_19 << 0.999941, 0.000890298, 0.0107934, -0.00455165,
        -0.000615842, 0.999677, -0.0254047, -1.1316,
        -0.0108126, 0.0253966, 0.999619, -1.03544,
        0, 0, 0, 1; // Body to Lidar 

    T_18 << 0.999804594428700, 0.0166008465068046, -0.0107324207204138, -0.00734124,
        -0.0165612202380415, 0.999855743880570, 0.00377059852786576, 1.15753529,
        0.0107934676304485, -0.00359211974866819, 0.999935296772857, 1.00634,
        0, 0, 0, 1; // Lidar to Body

    T_new << 0.99999, 0.00094809, -0.0031162, -0.0046,
        -0.00099999, 0.99986, -0.016696, -1.1316,
        0.0031, 0.016699, 0.99986, -1.0354,
        0, 0, 0, 1; 
   
    // Choose from them
    // T_l2b = T_new.inverse();
    // T_l2b = T_19.inverse();
    T_l2b = T_18;

    // Set initial calibration matrix
    calib_ldr_to_oxts_.SetPose(T_l2b);

    // Define origin drift (unit:m)
    // The aim is to avoid the precison loss when processing large figure (projected coordinate)
    // Old [FOR KITTI projection]
    // origin_datum_position_[0] = 11723000;
    // origin_datum_position_[1] = 3123000;
    // origin_datum_position_[2] = 0;

    //New [FOR UTM 51 Zone projection]
    origin_datum_position_[0] = 400000;
    origin_datum_position_[1] = 3415000;
    origin_datum_position_[2] = 0;
    
    static int frame_id = 0;
    static Eigen::Matrix4d gnss_pose0_inv;
    std::vector<std::string> lidar_file_names;
    std::vector<std::string> lidar_full_file_names;
    std::vector<timeval> time_stampes;
    std::vector<Eigen::Matrix4d> source_gnssins_poses;

    clock_t t0, t1;

    t0 = clock();
    std::string lidar_full_path;

    // Load sensors' data
    while (lidar_file_list.peek() != EOF){
        ++frame_id;
        std::string cur_file;
        lidar_file_list >> cur_file;
        if (frame_id < begin_frame){
            continue;
        }
        if (frame_id > end_frame){
            break;
        }
        
        Frame raw_frame;
        Gnss raw_gnss;
        imu_infos_t raw_imu_infos;

        lidar_full_path = lidar_root_path_ + "/" + cur_file;
        sensor::TimeStamp lidar_ts;
        if (!sensor::getTimeStampFromFileName(lidar_full_path, &lidar_ts)){
            LOG(WARNING) << "read lidar timestamp from file failed, file: " << lidar_full_path;
        }

        // Initialize raw frame pcd file name and timestamp
        raw_frame.pcd_file_name = cur_file;
        raw_frame.time_stamp = lidar_ts.getTime();

        std::vector<sensor::gnssins_info_t> gnssins_infos;

        // Initialize gnss information matrix
        // GNSS information matrix is Diagonal Matrix, diagonal elements are north(m), east(m), down(m), roll(rad), pith(rad), yaw(rad)
        static std::array<double, 6> information_diag = {0, 0, 0, 0, 0, 0};
        CHECK(gnssins.readGnssImuFromLdrTS(lidar_ts, gnssins_infos)) << "Read IMU Failed! lidar timestamp is: sec "
                                                                     << lidar_ts.getTime().tv_sec << " usec " << lidar_ts.getTime().tv_usec;

        if (!gnssins_infos.empty()){
#if 0       //output trajectory wgs84 file toghther with the pcd file
            if (frame_id % 10 == 0){ //Output frequency: 1Hz
                std::ofstream outfile("trajectory-wgs84-with-pcd-name.txt", std::ios::app);
                if (outfile){
                    outfile << setprecision(10) << gnssins_infos[0].lon << "\t" << gnssins_infos[0].lat << "\t" << cur_file << "\n";
                }
                outfile.close();
            }
#endif
            for (int i = 0; i < gnssins_infos.size(); ++i){
                Eigen::Matrix4d pose = GetTransform(gnssins_infos[0]); // OXTS pose of Body : Twb

                // Do the initial calibration transform
                pose.block<3, 3>(0, 0) = R_trans * pose.block<3, 3>(0, 0) * R_trans.inverse(); // Twb' = A*Twb*inv(A)

#if 1   //Relatve to origin datum (apply origin drift) [Body Frame] \
        //Twb" = Twb' - Two
                pose(0, 3) -= origin_datum_position_[0];
                pose(1, 3) -= origin_datum_position_[1];
                pose(2, 3) -= origin_datum_position_[2];
#endif //To Do: Add it when write the point cloud in world coordinate system

                // Get OXTS pose of Lidar: Twl
                pose = pose * T_l2b; // Twl = Twb" * Tbl    [Note that Tbl is just T_l2b]

                static bool is_first = true;

#if 0 //Relative to the first frame (OXTS pose of Lidar)
                if (is_first)
                {
                    LOG(WARNING) << "Inital position: [" << pose(0, 3) << " , " << pose(1, 3) << " , " << pose(2, 3) << "]";
                    //gnss_pose0_inv = pose.inverse();
                    gnss_pose0_inv = pose.inverse();
                    is_first = false;
                }
                pose = gnss_pose0_inv * pose;
#endif

                if (i == 0)
                {
                    // gnssins_infos[0] is corresponding to lidar_ts
                    raw_frame.pose.SetPose(pose);
                    raw_gnss.pose.SetPose(pose);
                    raw_gnss.time_stamp = gnssins_infos[0].time_stamp;
                    raw_imu_infos.time_stamp = gnssins_infos[0].time_stamp;
                    raw_imu_infos.imu_infos.resize(gnssins_infos.size());
                    // update information matrix status
                    if (!gnssins.initConfidenceStatus(lidar_ts, information_diag)) {
                        LOG(WARNING)
                            << "INIT GNSS Confidence Status(corres to 1st LidarFrame) Failed!, Information Matix is init as diag(0,0,0,0,0,0)";
                        raw_gnss.information_matrix = Eigen::Matrix<double, 6, 6>::Zero();
                    }
                    else {
                        raw_gnss.information_matrix = Eigen::Matrix<double, 6, 6>::Identity();
                        raw_gnss.information_matrix(0, 0) = information_diag[0];
                        raw_gnss.information_matrix(1, 1) = information_diag[1];
                        raw_gnss.information_matrix(2, 2) = information_diag[2];
                        raw_gnss.information_matrix(3, 3) = information_diag[3];
                        raw_gnss.information_matrix(4, 4) = information_diag[4];
                        raw_gnss.information_matrix(5, 5) = information_diag[5];
                    }
                    // import IMU
                    imu_info_t raw_imu;
                    // LOG(ERROR) << "wx is " << gnssins_infos[i].wx << "wy is " << gnssins_infos[i].wy << "wz is " << gnssins_infos[i].wz;
                    raw_imu.SetImuInfo(gnssins_infos[i].ax, gnssins_infos[i].ay, gnssins_infos[i].az,
                                       gnssins_infos[i].wx, gnssins_infos[i].wy, gnssins_infos[i].wz,
                                       gnssins_infos[i].time_stamp);
                    raw_imu_infos.imu_infos[i] = raw_imu;
                }
                else {
                    // import IMU
                    imu_info_t raw_imu;
                    // LOG(ERROR) << "wx is " << gnssins_infos[i].wx << "wy is " << gnssins_infos[i].wy << "wz is " << gnssins_infos[i].wz;
                    raw_imu.SetImuInfo(gnssins_infos[i].ax, gnssins_infos[i].ay, gnssins_infos[i].az,
                                       gnssins_infos[i].wx, gnssins_infos[i].wy, gnssins_infos[i].wz,
                                       gnssins_infos[i].time_stamp);
                    raw_imu_infos.imu_infos[i] = raw_imu;
                }
            }
        }
        else {   
            LOG(WARNING) << "Load GNSS-IMU Failed! Lidar File is " << lidar_full_path;
        }

        // Goup together
        RawData raw_data;
        raw_data.flag = RawData::UNDEAL;
        raw_data.raw_frame = raw_frame;
        raw_data.raw_gnss = raw_gnss;
        raw_data.raw_imu = raw_imu_infos;
        raw_datas_.push_back(raw_data);
    }
    t1 = clock();
    frame_number_ = raw_datas_.size(); 
    LOG(WARNING) << "Load HDMap's " << transaction_id_ << " transaction data done in " << (float(t1 - t0) / CLOCKS_PER_SEC) << " s"
                 << " , there are " << raw_datas_.size() << " frames.";

    return true;
}

// Divide the transaction into several submaps according to multiple rules 
// Rules: consecutive frame number, accumulated translation (using), accumilated heading angle (using) ...
// Notice, in this fuction, some frames with too little movement would be depreacted and not add to the front-end processing 
void Transaction::DivideSubmap() {
    // Timing
    clock_t t0, t1;
    t0 = clock();

    if (submap_number_ != 0) {
        LOG(WARNING) << "Transaction " << transaction_id_ << " has been divided! Divide it one more time!";
        submap_number_ = 0;
        sub_maps_.clear(); // Memory not released
    }

    // parameters setting
    int max_frame_num = config_.max_frames;
    float max_tran_dis = config_.translation_accumulate;
    float max_heading_angle = config_.rotation_accumulate;
    float tran_dis_accu_thre = 0.15;
    // accumulated distance of translation for discarding some frames when the car is not moving

    // initalization
    int temp_frame_num_in_frame = 0;
    float temp_tran_dis = 0;
    float temp_heading_angle = 0;
    float temp_move_dis_frame = tran_dis_accu_thre;

    LOG(INFO) << "Submap division criterion is: \n"
              << "1. Frame Number <= " << max_frame_num
              << " , 2. Translation <= " << max_tran_dis
              << "m , 3. Rotation <= " << max_heading_angle << " degree.";

    Submap temp_submap;
    temp_submap.init();

    for (size_t i = 0; i < frame_number_ - 1; i++) {
        // Do the accumulation
        temp_frame_num_in_frame++;
        temp_tran_dis += (raw_datas_[i].raw_gnss.pose.trans - raw_datas_[i + 1].raw_gnss.pose.trans).norm();
        temp_heading_angle += GetHeading(raw_datas_[i + 1].raw_imu.imu_infos, 0.001 * imu_infos_t::frequncy);
        
        // Don't discard the frame
        if (temp_move_dis_frame >= tran_dis_accu_thre) {
            // push i frame -> submap
            temp_submap.raw_data_group.push_back(raw_datas_[i]);
            temp_submap.frame_number++;
            temp_move_dis_frame = (raw_datas_[i].raw_gnss.pose.trans - raw_datas_[i + 1].raw_gnss.pose.trans).norm();

            // Three Submap Terminal Condition -> create a new submap
            if (temp_frame_num_in_frame >= max_frame_num ||
                temp_tran_dis >= max_tran_dis ||
                temp_heading_angle >= max_heading_angle) {  
                temp_submap.pose.copyFrom(temp_submap.raw_data_group[0].raw_gnss.pose);
                temp_submap.submap_id.transaction_id = transaction_id_;
                temp_submap.submap_id.submap_id = submap_number_;

                sub_maps_.push_back(temp_submap); // submap id is submap_number

                LOG(INFO) << "Submap [" << submap_number_ << "] raw_frame_number: " << temp_frame_num_in_frame
                          << " distance: " << temp_tran_dis << " heading: " << temp_heading_angle << " left_frame_number: " << temp_submap.frame_number;
                // print the condition when a submap is determined

                submap_number_++;

                temp_tran_dis = 0;
                temp_heading_angle = 0;
                temp_frame_num_in_frame = 0;
                temp_submap.init();

                temp_move_dis_frame = tran_dis_accu_thre;
            }
        }
        // discard the frame
        else {
            //LOG(INFO) << "Discard frame " << i;
            temp_move_dis_frame += (raw_datas_[i].raw_gnss.pose.trans - raw_datas_[i + 1].raw_gnss.pose.trans).norm();
        }
    }

    // add the last frame into the last submap
    temp_submap.raw_data_group.push_back(raw_datas_[frame_number_ - 1]);
    temp_submap.frame_number++;
    temp_submap.pose.copyFrom(temp_submap.raw_data_group[0].raw_gnss.pose);

    sub_maps_.push_back(temp_submap);
    if (frame_number_ > 1) {
        temp_tran_dis += (raw_datas_[frame_number_ - 2].raw_gnss.pose.trans - raw_datas_[frame_number_ - 1].raw_gnss.pose.trans).norm();
        temp_heading_angle += GetHeading(raw_datas_[frame_number_ - 1].raw_imu.imu_infos,
                                         0.001 * imu_infos_t::frequncy);
    }

    temp_frame_num_in_frame++;

    LOG(INFO) << "Submap [" << submap_number_ << "] raw_frame_number: " << temp_frame_num_in_frame
              << " distance: " << temp_tran_dis << " heading: " << temp_heading_angle << " left_frame_number: " << temp_submap.frame_number;

    submap_number_++;

    LOG(INFO) << "The total submap number is " << submap_number_;
    t1 = clock();

    LOG(WARNING) << "Divide submap done in " << (float(t1 - t0) / CLOCKS_PER_SEC) << " s";
}

// Get the heading angle change within one frame (10 imu data) using Trapezoidal integral of imu heading angular velocity
// Input: vector of imu_datas within a frame and the sample time of imu in second
// Output: the heading changing
float Transaction::GetHeading(std::vector<imu_info_t> &imu_datas, float delta_time_second) {

    float result = 0;

    for (int i = 0; i < imu_datas.size() - 1; i++)
    {
        //result += (180.0 / 3.14159) * 0.5 * (imu_datas[i].wz + imu_datas[i + 1].wz) * delta_time_second;
        result += 0.5 * (imu_datas[i].wz + imu_datas[i + 1].wz) * delta_time_second;
    }
    return fabs(result);
}

// Front-end Entrance (Processing the transaction)
void Transaction::FrontEnd() {

    clock_t t0, t1;
    std::string display_name;

    ErrorCompute ec;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > oxts_pose;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > odom_pose;
    transaction_mae_ = 0;
    int candidate_wrong_edge_num = 0;
    
    std::ostringstream oss;
    oss << transaction_id_;

    std::string front_end_viewer_name = "Front-end [ Transaction " + oss.str() + " ]";
    std::string pose_graph_viewer_name = "Pose graph [ Transaction " + oss.str() + " ]";    

    boost::shared_ptr<pcl::visualization::PCLVisualizer> frontend_viewer(new pcl::visualization::PCLVisualizer(front_end_viewer_name));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pose_viewer(new pcl::visualization::PCLVisualizer(pose_graph_viewer_name));

    std::vector<Pose3d, Eigen::aligned_allocator<Pose3d>> transaction_poses;
    for (size_t i = 0; i < submap_number_; i++)
    {
        transaction_poses.push_back(sub_maps_[i].pose);
    }
    viewer_.DispalyPoses(pose_viewer, transaction_poses, "pose", 1.0, 0.0, 0.0);

    for (size_t i = 0; i < submap_number_; i++)
    {
        t0 = clock();

        //Lidar Odom Main Entrance
        LOG(INFO) << "---------Submap [" << i << "] starts---------";
        LOG(INFO) << "Current rate of progress: "<< i << " / " << submap_number_ << " [ " << 100.0 * i / submap_number_ << " % ]";
        //Get prior transformation for uniform motion model initial guess
        if (i > 0)
            sub_maps_[i].raw_data_group[0].raw_frame.last_transform = sub_maps_[i - 1].last_frame_transform;
        SubmapOdom(sub_maps_[i]);

#if 1 //for submap display
        viewer_.DisplaySubmapClouds(frontend_viewer, sub_maps_[i], INTENSITY, 1, i, 10);
#endif

        //For Error Evaluation
        for (unsigned int j = 0; j < sub_maps_[i].frame_number; j++)
        {
            oxts_pose.push_back(sub_maps_[i].raw_data_group[j].raw_gnss.pose.GetMatrix());
            odom_pose.push_back(sub_maps_[i].raw_data_group[j].raw_frame.pose.GetMatrix());
        }
        transaction_mae_ += sub_maps_[i].submap_mae;

        //Save processed data
        std::ostringstream oss;
        oss << transaction_id_ << "_" << i;
        sub_maps_[i].full_cld_filename = "submap_" + oss.str() + ".pcd";
        sub_maps_[i].feature_cld_filename = "submap" + oss.str() + "_feature.pcd";
        sub_maps_[i].featrure_idx_filename = "submap" + oss.str() + "_feature_idx.txt";
        data_loader_.checkDir(submap_full_root_path_);
        data_loader_.checkDir(submap_feature_root_path_);
        data_loader_.saveSubmapToFile(submap_full_root_path_ + "/" + sub_maps_[i].full_cld_filename,
                                      submap_feature_root_path_ + "/" + sub_maps_[i].feature_cld_filename,
                                      submap_feature_root_path_ + "/feature_cld_idx.txt",
                                      sub_maps_[i]);

        t1 = clock();
        LOG(INFO) << "---------Submap [" << i << "] ends---------";
        LOG(INFO) << "Consuming time for the submap is " << (float(t1 - t0) / CLOCKS_PER_SEC) << " s";
        LOG(INFO) << "Mean consuming time for each frame is " << ((float(t1 - t0) / CLOCKS_PER_SEC) / sub_maps_[i].frame_number * 1000) << " ms";
        //Time for visualization is also involved

        if (i != 0)
        {
            BuildAdjacentEdge(i); //Adjacent Submap Registration

            bool is_candidate_wrong_edge = false;
            if (adjacent_edges_[i - 1].information_matrix == Eigen::Matrix<double, 6, 6>::Zero(6, 6))
            {
                candidate_wrong_edge_num++;
                is_candidate_wrong_edge = true;
            }

#if 1 //for display

            std::shared_ptr<PointCloud> submap_pointcloud_temp = std::make_shared<PointCloud>();
            Pose3d pose2to1_reg;
            pose2to1_reg.SetPose(sub_maps_[i - 1].pose.GetMatrix() * adjacent_edges_[i - 1].pose.GetMatrix() * sub_maps_[i].pose.GetMatrix().inverse());
            reg_.transformPointCloud(sub_maps_[i].cld_lidar_ptr, submap_pointcloud_temp, pose2to1_reg);

            submap_pointcloud_temp->points.swap(sub_maps_[i].cld_lidar_ptr->points);
            LOG(INFO) << "Submap display point number is " << sub_maps_[i].cld_lidar_ptr->points.size();

            viewer_.UpdatePoseGraph(pose_viewer, transaction_poses, i, is_candidate_wrong_edge, 10); //Apply Submao Adjacent Registration Correction
            viewer_.UpdateSubmapClouds(frontend_viewer, sub_maps_[i], INTENSITY, 1, i, 10);
#endif

            //transaction_information_matrix = transaction_information_matrix + adjacent_edges_[i - 1].information_matrix;
            ReleaseSubmapCld(i - 1); // Free Last Submap
        }
    }
    //Writeout the transaction's front-end edge result
    data_loader_.writeEdges(pose_graph_output_path_, "Transaction_adjacent_edges.txt", adjacent_edges_);

    ec.compute(oxts_pose, odom_pose);
    ec.print_error();
    transaction_mae_ /= submap_number_;
    LOG(INFO) << "The transaction's Mean Abosulte Error is " << transaction_mae_;
    LOG(INFO) << "There are " << candidate_wrong_edge_num << " candidate wrong adjacent edges in transaction #" << transaction_id_;
    LOG(INFO) << "End of the frontend of transaction #" << transaction_id_;

    // Free Memory
    ReleaseAllSubmaps();
}

// Submap Odometry (Processing each submap in the transaction) 
// Input: a given submap
bool Transaction::SubmapOdom(Submap &submap)
{
    // Timing
    clock_t t0, t1;

    // Parameters List (add it to config later)

    // Distance Filter
    float xy_max = 40.0; // xy_max is the filter radius, points outside the circle would be deleted (unit:m)
    float z_min = -2.8;  // z_min is used to filter some noise underground points (unit:m)
    float z_max = 30.0;  // z_max is used to filter some noise unground points (unit:m)

    // Ground Filter (Segment Ground and Unground points)
    // gf_min_grid_num is the min point number in a grid. those grid whose point number < gf_min_grid_num would be ignored  
    int gf_min_grid_num = 10;
    // gf_grid_resolution is the size of a grid (unit:m)               
    float gf_grid_resolution = 0.75;
    // points whose [(z - min_z of the grid) > gf_max_grid_height_diff] would be regarded as unground points (unit:m)
    float gf_max_grid_height_diff = 0.15;
    // grids whose [(z_min - z_min of the grid's 8 neighbor grids) > gf_neighbor_height_diff would be regarded as unground grids (unit:m)
    float gf_neighbor_height_diff = 1.0;
    // points whose z is larger than gf_max_ground_height would be regarded as unground points whatever (unit:m)  
    float gf_max_ground_height = 1.0;
    // gf_downsample_rate_nonground is the random downsample ratio of detected unground points [the downsample is for efficiency concerning]
    int gf_downsample_rate_nonground = 2;
    // only gf_downsample_grid_number_first points would be randomly selected as the ground points in a grid
    // This group of ground points are used for registration [target ground points (more point number)] 
    int gf_downsample_grid_number_first = 2;  
    // only gf_downsample_grid_number_second points would be randomly selected as the ground points in a grid
    // This group of ground points are used for registration [source ground points (less point number)] 
    int gf_downsample_grid_number_second = 1; 

    // Feature Points Detection
    // Search neighbor_search_K nearest neighbor for doing neighbor PCA 
    // For neighbor PCA result, we define e1,e2,e3 (e1>e2>e3) are three eigen value and v1,v2,v3 are the correponding eigen vector
    // We call v1 the primary vector and v3 the normal vector
    // We define point linearty, planarity and curvature
    // linearty a_1d = (e1-e2)/e1 , planarity a_2d = (e2-e3)/e1 , curvature = e3/(e1+e2+e3)
    int neighbor_search_K = 8;
    // Those candidate edge points whose primary direction's z component < linear_vertical_cosine_min would be rejected
    float linear_vertical_cosine_min = 0.75;
    // Those candidate planar points whose normal direction's z component > planar_horizontal_cosine_max would be rejected
    float planar_horizontal_cosine_max = 0.4;
    // linearty threshold of edge feature points for registration [target edge points (more point number)] 
    float neighbor_linear_thre_target = 0.6;
    // planarty threshold of planar feature points for registration [target planar points (more point number)] 
    float neighbor_planar_thre_target = 0.5;
    // curvature threshold of sphere feature points for registration [target sphere points (more point number)] 
    float neighbor_curvature_thre_target = 0.2;
    // linearty threshold of edge feature points for registration [source edge points (less point number)] 
    float neighbor_linear_thre_source = 0.75;
    // planarty threshold of planar feature points for registration [source planar points (less point number)] 
    float neighbor_planar_thre_source = 0.7;
    // curvature threshold of sphere feature points for registration [source sphere points (less point number)] 
    float neighbor_curvature_thre_source = 0.25;
    // edge_point_source_appro_num points with larger linearity would be regarded as source edge points (less point number)
    int edge_point_source_appro_num = 300;   
    // planar_point_source_appro_num points with larger planarity would be regarded as source planar points (less point number)
    int planar_point_source_appro_num = 300; 
    // sphere_point_source_appro_num points with larger curvature would be regarded as source sphere points (less point number)
    int sphere_point_source_appro_num = 50;  
    
    // Tune these parameters for the trade-off between time efficiency and LO accuracy

    // Noise
    float noise_t = 0.0, noise_r = 0.0; // Now add no noise to gnss raw data 

    // Begin
    LOG(INFO) << "Submap's frame number is " << submap.frame_number;

    // Preprocessing
    LOG(INFO) << "Preprocessing & Registration for each frame";
    std::shared_ptr<PointCloud> pointcloud_lidar_filtered(new PointCloud);
    std::vector<unsigned int> cld_unground_index;

    // INIT frame pose as gnss pose
    submap.raw_data_group[0].raw_frame.pose.copyFrom(submap.raw_data_group[0].raw_gnss.pose);
    t0 = clock();
    for (int i = 0; i < submap.frame_number; i++) {
        while (xy_max <= 60 && gf_min_grid_num >= 6) {
            LOG(INFO) << "---------------Frame [" << i << "] in Submap ["
                      << submap.submap_id.transaction_id << " - " << submap.submap_id.submap_id << "]--------------";
            LOG(INFO) << "*** *** FILTER *** ***";
            pointcloud_lidar_filtered = std::make_shared<PointCloud>();
            cld_unground_index.clear();

            submap.raw_data_group[i].raw_frame.ground_index.clear();
            submap.raw_data_group[i].raw_frame.ground_down_index.clear();
            submap.raw_data_group[i].raw_frame.edge_index.clear();
            submap.raw_data_group[i].raw_frame.edge_down_index.clear();
            submap.raw_data_group[i].raw_frame.planar_index.clear();
            submap.raw_data_group[i].raw_frame.planar_down_index.clear();
            submap.raw_data_group[i].raw_frame.sphere_index.clear();
            submap.raw_data_group[i].raw_frame.sphere_down_index.clear();
            submap.raw_data_group[i].raw_frame.cld_lidar_ptr = std::make_shared<PointCloud>(); // origin pcd

            //Add noise
            if (i == 0) {
                AddNoise(submap.raw_data_group[i], 0, 0);
            } 
            else {
                AddNoise(submap.raw_data_group[i], 2 * noise_t, 2 * noise_r);
            }

            //Import cloud
            data_loader_.readPcdFile(lidar_root_path_ + "/" + submap.raw_data_group[i].raw_frame.pcd_file_name,
                                     submap.raw_data_group[i].raw_frame.cld_lidar_ptr, 1);  //tag 1 for Point XYZI type

            //TODO Active Objects Filtering (@ Jingwei Li)

            // std::vector<std::shared_ptr<PointCloud>> cloud_show_ptr;
            // cloud_show_ptr.push_back(submap.raw_data_group[i].raw_frame.cld_lidar_ptr);
            // viewer_.DispalyNClouds(cloud_show_ptr, "Raw data", INTENSITY, 1);

            //Dis Filter, ROI filter
            filter_.DisFilter(submap.raw_data_group[i].raw_frame.cld_lidar_ptr,
                              pointcloud_lidar_filtered, xy_max, z_min, z_max);

            //viewer_.Dispaly2Clouds(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, pointcloud_lidar_filtered, "Compare", 1);

            submap.raw_data_group[i].raw_frame.cld_lidar_ptr->points.swap(pointcloud_lidar_filtered->points);

            // Ground Filter
            // 1. Points are divided into [Ground and Unground] Points, by grid two threshold ground filter
            // 2. (Optional) Detect the curb grids and determine the road grids, those unground points in road grids
            // would be regarded as active objects (car, pedestrian, etc..)
            // Notice, this active object filter is based on rules and is not robust (switch to learning based methods later)
            filter_.FastGroundFilter(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, submap.raw_data_group[i].raw_frame.ground_index,
                                     submap.raw_data_group[i].raw_frame.ground_down_index, cld_unground_index,
                                     gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff, gf_max_ground_height,
                                     gf_downsample_grid_number_first, gf_downsample_grid_number_second, gf_downsample_rate_nonground);

            // Detected Feature Points 
            // 1. Unground points are divided into [Edge, Planar, Sharp Sphere] Points, by PCA and rules 
            // 2. Calculate unground points normal, by PCA 
            // Notice, all the features (Ground, Edge, Planar, Sphere) are detected by two different downsample rate 
            // The group with more feature points would be used as target cloud (not transformed in the process) for registration
            // while the other group with less points would be regarded as source cloud
            // The aim is to improve time efficiency
            filter_.RoughClassify(submap.raw_data_group[i].raw_frame.cld_lidar_ptr,
                                  submap.raw_data_group[i].raw_frame.edge_down_index, submap.raw_data_group[i].raw_frame.edge_index,
                                  submap.raw_data_group[i].raw_frame.planar_down_index, submap.raw_data_group[i].raw_frame.planar_index,
                                  submap.raw_data_group[i].raw_frame.sphere_down_index, submap.raw_data_group[i].raw_frame.sphere_index,
                                  cld_unground_index, neighbor_search_K,
                                  neighbor_linear_thre_target, neighbor_planar_thre_target, neighbor_curvature_thre_target,
                                  neighbor_linear_thre_source, neighbor_planar_thre_source, neighbor_curvature_thre_source,
                                  linear_vertical_cosine_min, planar_horizontal_cosine_max,
                                  edge_point_source_appro_num, planar_point_source_appro_num, sphere_point_source_appro_num);

#if 0 //Display the feature points in a frame
            if (i % 20 == 0)
            {
                viewer_.DispalyFeatureFrame(submap.raw_data_group[i].raw_frame.cld_lidar_ptr,
                                              submap.raw_data_group[i].raw_frame.ground_index,
                                              submap.raw_data_group[i].raw_frame.edge_index,
                                              submap.raw_data_group[i].raw_frame.planar_index,
                                              submap.raw_data_group[i].raw_frame.sphere_index,
                                              "Frame_feature  Ground (Silver) ; Edge (Green) ; Planar (Blue) ; Sphere (Red)");
                viewer_.DispalyFeatureFrame(submap.raw_data_group[i].raw_frame.cld_lidar_ptr,
                                              submap.raw_data_group[i].raw_frame.ground_down_index,
                                              submap.raw_data_group[i].raw_frame.edge_down_index,
                                              submap.raw_data_group[i].raw_frame.planar_down_index,
                                              submap.raw_data_group[i].raw_frame.sphere_down_index,
                                              "Frame_feature [Double Downsample] Ground (Silver) ; Edge (Green) ; Planar (Blue) ; Sphere (Red)");
            }
#endif
            
            LOG(INFO) << "Frame's position: " << submap.raw_data_group[i].raw_gnss.pose.trans.transpose();

            //Lidar Odometry Registration
            if (i == 0) {
                break;
            } else {
                LOG(INFO) << "*** *** REGISTRATION *** ***";
                Pose3d pose2to1;
                int code = reg_.PairwiseReg(submap.raw_data_group[i-1], submap.raw_data_group[i], pose2to1);
                if (code == 1) {
                    submap.raw_data_group[i].raw_frame.last_transform.copyFrom(pose2to1);
                    //Get frame's lidar odometry pose
                    submap.raw_data_group[i].raw_frame.pose.SetPose(
                            submap.raw_data_group[i - 1].raw_frame.pose.GetMatrix() * pose2to1.GetMatrix());
                    break;
                } else if (code == -1 || code == -2 || code == -3) {
                    // Expand Dis Filter
                    gf_min_grid_num -= 2;
                    xy_max += 10;
                    continue;
                } else {
                    // code == -4  Registration Failed, Nothing we can do.
                    LOG(FATAL) << "Fix Registration Method Between Frames";
                    break;
                }
#if 0
                //Scan to scan gnss merge
                reg_.pairwise_initial_guess(submap.raw_data_group[i],
                                            submap.raw_data_group[i + 1]);
#endif
                
            }
        }
    }

    //Free memory
    std::vector<PointType>().swap(pointcloud_lidar_filtered->points);
    pointcloud_lidar_filtered = std::make_shared<PointCloud>();
    std::vector<unsigned int>().swap(cld_unground_index);



    submap.last_frame_transform.copyFrom(submap.raw_data_group[submap.frame_number - 1].raw_frame.last_transform);
    t1 = clock();

    LOG(INFO) << "Mean LO consuming time for each frame is " << ((float(t1 - t0) / CLOCKS_PER_SEC) / submap.frame_number * 1000) << " ms";

    // INIT submap parameters
    submap.pose.copyFrom(submap.raw_data_group[0].raw_gnss.pose);

    //Merge the frames to the submap
    MergeFrames(submap);

    //Use the submap's cloud in world frame to calculate bounding box
    filter_.getCloudBound(*submap.cld_lidar_ptr, submap.bbox);

    //Evaluate the accuracy
    submap.submap_mae = GetMae(submap);

    if (submap.submap_mae < 0.1)
    {
        LOG(INFO) << "The submap's Mean Absolute Error is " << submap.submap_mae;
    }

    else
    {
        if (submap.submap_mae < 0.4)
        {
            LOG(WARNING) << "The submap's Mean Absolute Error is " << submap.submap_mae;
        }
        else
        {
            LOG(ERROR) << "The submap's Mean Absolute Error is " << submap.submap_mae;
        }

#if 1
        double dis_diff_lidar, dis_diff_oxts;

        dis_diff_lidar = (submap.raw_data_group[submap.frame_number - 1].raw_frame.pose.trans - submap.raw_data_group[0].raw_frame.pose.trans).norm();
        dis_diff_oxts = (submap.raw_data_group[submap.frame_number - 1].body_gnss.pose.trans - submap.raw_data_group[0].body_gnss.pose.trans).norm();
        LOG(WARNING) << "Submap: "
                     << " LO diff: " << dis_diff_lidar << " OXTS diff: " << dis_diff_oxts << "  Difference: " << dis_diff_lidar - dis_diff_oxts;
        std::ostringstream oss;
        oss << "Submap_" << submap.submap_id.transaction_id << "_" << submap.submap_id.submap_id;
        //viewer_.DisplaySubmapClouds(submap, oss.str() + "\t LO Lidar Frame (Yellow) ; OXTS Lidar Frame (Purple) ; OXTS Body Frame (Cyan)", INTENSITY, 2);
#endif
    }

    return true;
}

// mergeFrames can be defined as a member function of submap, before you do that,
// you should change submap from structure into class in a new file, otherwise "types.h" and "common_reg.hpp" will conflict (Circular reference)
bool Transaction::MergeFrames(Submap &submap)
{
    clock_t t0, t1;

    t0 = clock();

    // init
    submap.cld_lidar_ptr = std::make_shared<point_cloud_t<PointType>>();
    submap.cld_feature_ptr = std::make_shared<point_cloud_t<PointType>>();
    submap.ground_index.clear();
    submap.ground_down_index.clear();
    submap.edge_index.clear();
    submap.edge_down_index.clear();
    submap.planar_index.clear();
    submap.planar_down_index.clear();
    submap.sphere_index.clear();
    submap.sphere_down_index.clear();
    std::shared_ptr<point_cloud_t<PointType>> ground_cld_ptr(new point_cloud_t<PointType>);
    std::shared_ptr<point_cloud_t<PointType>> edge_cld_ptr(new point_cloud_t<PointType>);
    std::shared_ptr<point_cloud_t<PointType>> planar_cld_ptr(new point_cloud_t<PointType>);
    std::shared_ptr<point_cloud_t<PointType>> sphere_cld_ptr(new point_cloud_t<PointType>);
    std::shared_ptr<point_cloud_t<PointType>> feature_cld_ptr(new point_cloud_t<PointType>);
    std::shared_ptr<point_cloud_t<PointType>> transform_cld_ptr(new point_cloud_t<PointType>);
    std::shared_ptr<point_cloud_t<PointType>> downsample_cld_ptr(new point_cloud_t<PointType>);
    // for accelerate
    //    ground_cld_ptr->points.resize(100000);
    //    edge_cld_ptr->points.resize(100000);
    //    planar_cld_ptr->points.resize(100000);
    //    sphere_cld_ptr->points.resize(100000);
    CHECK(submap.raw_data_group.size() == submap.frame_number) << "CHECK frame number";

    //Merge frame parameter : Skip some frames for merging
    int merge_frame_sample_rate = submap.frame_number / 15 + 1;

    //double merge_tran_dis_threshold = 0.1;

    int random_downsample_step_low_intensity = 30;
    int random_downsample_step_high_intensity = 10;
    int intensity_thre = 180;

    // merge feature point cloud
    CHECK(submap.cld_feature_ptr->points.empty()) << "cld_feature_ptr isn't empty, please check";

    //double frame_tran = 0;
    //int merge_frame_number = 0;

    for (size_t i = 0; i < submap.frame_number; ++i)
    {
        if (i % merge_frame_sample_rate == 0)
        {
            Pose3d frame_pose_ref_submap;
            frame_pose_ref_submap.SetPose(submap.pose.GetMatrix().inverse() *
                                          submap.raw_data_group[i].raw_frame.pose.GetMatrix());
            // get ground points
            filter_.FilterCldWithIdx(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, feature_cld_ptr,
                                     submap.raw_data_group[i].raw_frame.ground_down_index);
            // transform ground points
            reg_.transformPointCloud(feature_cld_ptr, transform_cld_ptr, frame_pose_ref_submap);

            // accumulate ground points
            ground_cld_ptr->points.insert(ground_cld_ptr->points.end(), transform_cld_ptr->points.begin(), transform_cld_ptr->points.end());

            // edge points
            filter_.FilterCldWithIdx(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, feature_cld_ptr,
                                     submap.raw_data_group[i].raw_frame.edge_down_index);
            reg_.transformPointCloud(feature_cld_ptr, transform_cld_ptr, frame_pose_ref_submap);
            edge_cld_ptr->points.insert(edge_cld_ptr->points.end(), transform_cld_ptr->points.begin(), transform_cld_ptr->points.end());

            // planar points
            filter_.FilterCldWithIdx(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, feature_cld_ptr,
                                     submap.raw_data_group[i].raw_frame.planar_down_index);
            reg_.transformPointCloud(feature_cld_ptr, transform_cld_ptr, frame_pose_ref_submap);
            planar_cld_ptr->points.insert(planar_cld_ptr->points.end(), transform_cld_ptr->points.begin(), transform_cld_ptr->points.end());

            // sphere points
            filter_.FilterCldWithIdx(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, feature_cld_ptr,
                                     submap.raw_data_group[i].raw_frame.sphere_down_index);
            reg_.transformPointCloud(feature_cld_ptr, transform_cld_ptr, frame_pose_ref_submap);
            sphere_cld_ptr->points.insert(sphere_cld_ptr->points.end(), transform_cld_ptr->points.begin(), transform_cld_ptr->points.end());

            //Do downsampling for every frame
            filter_.RandomDownsample(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, downsample_cld_ptr,
                                     random_downsample_step_low_intensity, random_downsample_step_high_intensity, intensity_thre);
            Pose3d frame_pose;

#if 0 //Pose refer to the submap's first frame
            frame_pose.SetPose(submap.pose.GetMatrix().inverse() *
                               submap.raw_data_group[i].raw_frame.pose.GetMatrix());
#endif

#if 1 //Pose (Lidar Odometry) refer to the world system
            frame_pose = submap.raw_data_group[i].raw_frame.pose;
#endif

#if 0 //Pose (Lidar OXTS) refer to the world system 
            frame_pose = submap.raw_data_group[i].raw_gnss.pose;
#endif
            //Transform the frame's downsampled cloud to its target frame
            reg_.transformPointCloud(downsample_cld_ptr, transform_cld_ptr, frame_pose);

            //Merge the frames
            submap.cld_lidar_ptr->points.insert(submap.cld_lidar_ptr->points.end(), transform_cld_ptr->points.begin(),
                                                transform_cld_ptr->points.end());
        }
    }
    LOG(INFO) << "Transform feature and downsampled points done.";

    //init submap cld
    submap.cld_feature_ptr->points.insert(submap.cld_feature_ptr->points.end(), ground_cld_ptr->points.begin(), ground_cld_ptr->points.end());
    submap.cld_feature_ptr->points.insert(submap.cld_feature_ptr->points.end(), edge_cld_ptr->points.begin(), edge_cld_ptr->points.end());
    submap.cld_feature_ptr->points.insert(submap.cld_feature_ptr->points.end(), planar_cld_ptr->points.begin(), planar_cld_ptr->points.end());
    submap.cld_feature_ptr->points.insert(submap.cld_feature_ptr->points.end(), sphere_cld_ptr->points.begin(), sphere_cld_ptr->points.end());
    size_t ground_length = ground_cld_ptr->points.size();
    size_t edge_length = edge_cld_ptr->points.size();
    size_t planar_length = planar_cld_ptr->points.size();
    size_t sphere_length = sphere_cld_ptr->points.size();
    submap.ground_index.resize(ground_length);
    submap.edge_index.resize(edge_length);
    submap.planar_index.resize(planar_length);
    submap.sphere_index.resize(sphere_length);
    for (int i = 0; i < ground_length; ++i)
        submap.ground_index[i] = i;
    for (int i = 0; i < edge_length; ++i)
        submap.edge_index[i] = ground_length + i;
    for (int i = 0; i < planar_length; ++i)
        submap.planar_index[i] = i + ground_length + edge_length;
    for (int i = 0; i < sphere_length; ++i)
        submap.sphere_index[i] = i + ground_length + edge_length + planar_length;
    submap.ground_size = submap.ground_index.size();
    submap.edge_size = submap.edge_index.size();
    submap.planar_size = submap.planar_index.size();
    submap.sphere_size = submap.sphere_index.size();

    LOG(INFO) << "Merge feature and downsampled points done.";

    LOG(INFO) << "Submap's points number after downsampling: " << submap.cld_lidar_ptr->points.size();

    t1 = clock();
    LOG(INFO) << "Submap merging done in " << (float(t1 - t0) / CLOCKS_PER_SEC) * 1000.0 << " ms";

    return true;
}

// Build adjacent edge by applying registration between adjacent submaps along the transaction  
// Batch do all the adjacent registration in the transaction
bool Transaction::BuildAdjacentEdges()
{
    // build adjacent edge
    adjacent_edges_.resize(sub_maps_.size() - 1);
    adjacent_edges_.clear();
    for (size_t i = 0; i < sub_maps_.size() - 1; ++i)
    {
        adjacent_edges_[i].submap_idx.first_submap.transaction_id = transaction_id_;
        adjacent_edges_[i].submap_idx.first_submap.submap_id = i;
        adjacent_edges_[i].submap_idx.second_submap.transaction_id = transaction_id_;
        adjacent_edges_[i].submap_idx.second_submap.submap_id = i + 1;
        reg_.PairwiseReg(adjacent_edges_[i], sub_maps_[i], sub_maps_[i + 1]);
    }
}

// Build adjacent edge by applying registration between adjacent submaps along the transaction 
// Input: a given submap_id in the transcation
bool Transaction::BuildAdjacentEdge(int submap_id)
{
    // calc adjacent edge between submap_id and (submap_id-1)
    LOG(INFO) << "Adjacent registration between submap " << submap_id - 1 << " and submap " << submap_id;
    Edge new_edge;
    new_edge.submap_idx.first_submap.transaction_id = transaction_id_;
    new_edge.submap_idx.first_submap.submap_id = submap_id - 1;
    new_edge.submap_idx.second_submap.transaction_id = transaction_id_;
    new_edge.submap_idx.second_submap.submap_id = submap_id;
    reg_.PairwiseReg(new_edge, sub_maps_[submap_id - 1], sub_maps_[submap_id]);
    adjacent_edges_.push_back(new_edge);
}

bool Transaction::MergeFrameBounds(Submap &submap)
{
    submap.bbox = submap.raw_data_group[0].raw_frame.bbox;
    for (int i = 1; i < submap.frame_number; i++)
    {
        if (submap.raw_data_group[i].raw_frame.bbox.max_x > submap.bbox.max_x)
            submap.bbox.max_x = submap.raw_data_group[i].raw_frame.bbox.max_x;
        if (submap.raw_data_group[i].raw_frame.bbox.max_y > submap.bbox.max_y)
            submap.bbox.max_y = submap.raw_data_group[i].raw_frame.bbox.max_y;
        if (submap.raw_data_group[i].raw_frame.bbox.max_z > submap.bbox.max_z)
            submap.bbox.max_z = submap.raw_data_group[i].raw_frame.bbox.max_z;
        if (submap.raw_data_group[i].raw_frame.bbox.min_x < submap.bbox.min_x)
            submap.bbox.min_x = submap.raw_data_group[i].raw_frame.bbox.min_x;
        if (submap.raw_data_group[i].raw_frame.bbox.min_y < submap.bbox.min_y)
            submap.bbox.min_y = submap.raw_data_group[i].raw_frame.bbox.min_y;
        if (submap.raw_data_group[i].raw_frame.bbox.min_z < submap.bbox.min_z)
            submap.bbox.min_z = submap.raw_data_group[i].raw_frame.bbox.min_z;
    }
    submap.center_point.x = 0.5 * (submap.bbox.max_x + submap.bbox.min_x);
    submap.center_point.y = 0.5 * (submap.bbox.max_y + submap.bbox.min_y);
    submap.center_point.z = 0.5 * (submap.bbox.max_z + submap.bbox.min_z);
    LOG(INFO) << "Submap bounding box: x [" << submap.bbox.max_x << " - " << submap.bbox.min_x << "] y [" << submap.bbox.max_y << " - " << submap.bbox.min_y << "] z [" << submap.bbox.max_z << " - " << submap.bbox.min_z << "]\n";
}

float Transaction::GetMae(Submap &submap)
{
    double dis_dif = 0;
    for (int i = 0; i < submap.frame_number; i++)
    {
        Eigen::Vector3d position_dif =
            submap.raw_data_group[i].raw_gnss.pose.trans -
            submap.raw_data_group[i].raw_frame.pose.trans;
        dis_dif += position_dif.norm();
    }
    return (dis_dif / submap.frame_number);
}

bool Transaction::ReleaseSubmapCld(int submap_id){
    if (submap_id < 0 || submap_id >= sub_maps_.size()){
        LOG(WARNING) << "release submap_id wrong!";
        return false;
    }
    sub_maps_[submap_id].release_cld_lidar();
    sub_maps_[submap_id].releaseIndex();
    sub_maps_[submap_id].releaseRawData();
    return true;
}

bool Transaction::ReleaseAllSubmaps(){
    for (size_t i = 0; i < sub_maps_.size(); ++i){
        sub_maps_[i].release_cld_lidar();
        sub_maps_[i].releaseIndex();
        sub_maps_[i].releaseRawData();
    }
    return true;
}

// Add noise to raw gnss 's pose of the LiDAR coordinate system , Fix it Later
// Input: a given RawData (1 frame's data), translation noise, rotation noise
bool Transaction::AddNoise(RawData &raw_data_group, float noise_std_t, float noise_std_r){
    //const float mean_t=0.0, mean_r=0.0;
    //const float mean_tx=0.0, mean_ty=0.0, mean_tz=0.0;

    //Normal Distribution: Some Problem
    //std::default_random_engine noise_generator;
    //std::normal_distribution<float> dist (mean_t, noise_std_t);
    //std::normal_distribution<float> ang (mean_r, noise_std_r);

    // frame.oxts_noise_pose(0,3)=frame.oxts_pose(0,3)+ dist(noise_generator);
    // frame.oxts_noise_pose(1,3)=frame.oxts_pose(1,3)+ dist(noise_generator);
    // frame.oxts_noise_pose(2,3)=frame.oxts_pose(2,3)+ dist(noise_generator);

    Eigen::Matrix4d oxts_noise_pose;
    oxts_noise_pose = raw_data_group.raw_gnss.pose.GetMatrix();

    oxts_noise_pose(0, 3) = oxts_noise_pose(0, 3) + 0.8 * (0.5 * noise_std_t - noise_std_t * ((double)rand() / RAND_MAX));
    //Not too much horizontal error compared with vertical one
    oxts_noise_pose(1, 3) = oxts_noise_pose(1, 3) + 0.8 * (0.5 * noise_std_t - noise_std_t * ((double)rand() / RAND_MAX));

    oxts_noise_pose(2, 3) = oxts_noise_pose(2, 3) + 0.5 * noise_std_t - noise_std_t * ((double)rand() / RAND_MAX);

    float noise_roll, noise_pitch, noise_yaw;
    // noise_roll = ang(noise_generator)/180.0*M_PI;
    // noise_pitch = ang(noise_generator)/180.0*M_PI;
    // noise_yaw = ang(noise_generator)/180.0*M_PI;

    noise_roll = (0.5 * noise_std_r - noise_std_r * ((double)rand() / RAND_MAX)) / 180.0 * M_PI;
    noise_pitch = (0.5 * noise_std_r - noise_std_r * ((double)rand() / RAND_MAX)) / 180.0 * M_PI;
    noise_yaw = (0.5 * noise_std_r - noise_std_r * ((double)rand() / RAND_MAX)) / 180.0 * M_PI;

    Eigen::Matrix3d Rx, Ry, Rz;
    Rx << 1, 0, 0,
        0, cos(noise_roll), -sin(noise_roll),
        0, sin(noise_roll), cos(noise_roll);

    Ry << cos(noise_pitch), 0, sin(noise_pitch),
        0, 1, 0,
        -sin(noise_pitch), 0, cos(noise_pitch);

    Rz << cos(noise_yaw), -sin(noise_yaw), 0,
        sin(noise_yaw), cos(noise_yaw), 0,
        0, 0, 1;

    Eigen::Matrix3d rot_noise_Matrix = Rz * Ry * Rx;

    oxts_noise_pose.block<3, 3>(0, 0) = rot_noise_Matrix * oxts_noise_pose.block<3, 3>(0, 0);
    oxts_noise_pose.block<1, 4>(3, 0) = oxts_noise_pose.block<1, 4>(3, 0);

    raw_data_group.noise_gnss.pose.SetPose(oxts_noise_pose);

    return 1;
}

// Load KITTI Datasets' data for LO test
void Transaction::LoadKITTIData(int begin_frame, int end_frame) {
    
    frame_number_= end_frame-begin_frame+1;  
    
    Eigen::Matrix3d trans_mat;
    trans_mat << 0, -1, 0,
			     0, 0, -1, 
			     1, 0, 0;

    // Import pose
    std::vector<Eigen::Matrix4d> pose_vec;
    
    std::ostringstream oss;
    oss.setf(ios::right);
    oss.fill('0');
    oss.width(2);
    oss << transaction_id_<< ".txt";
   
    std::string pose_file=oxts_root_path_+"/" + oss.str();
    data_loader_.GetPoseFromFile(pose_file, pose_vec);
    
    // Import pcd point cloud
    std::vector<std::string> pcd_filenames;

    std::ifstream lidar_file_list(pcd_list_path_);
    if (!lidar_file_list.is_open()) {
        LOG(ERROR) << "open lidar_file_list failed, file is " << pcd_list_path_;
    }
   
    while (lidar_file_list.peek() != EOF){
        std::string cur_file;
        lidar_file_list >> cur_file;
        pcd_filenames.push_back(cur_file);   
    }
    
    raw_datas_.resize(frame_number_);

    // Assign Raw data
    for (int i=begin_frame; i<= end_frame; i++)
    {
        pose_vec[i].block<3, 1>(0, 3) = trans_mat.transpose() * pose_vec[i].block<3, 1>(0, 3);
        pose_vec[i].block<3, 3>(0, 0) = trans_mat.transpose() * pose_vec[i].block<3, 3>(0, 0) * trans_mat;
        
        raw_datas_[i-begin_frame].raw_gnss.pose.SetPose(pose_vec[i]); 
        raw_datas_[i-begin_frame].raw_frame.pcd_file_name=pcd_filenames[i];
    } 
    
    Submap kitti_submap;
    kitti_submap.pose=raw_datas_[begin_frame].raw_gnss.pose;
    kitti_submap.frame_number = frame_number_;
    kitti_submap.raw_data_group.insert(kitti_submap.raw_data_group.end(),raw_datas_.begin(),raw_datas_.end());
    sub_maps_.push_back(kitti_submap);
}

void Transaction::PureLidarOdometry(){
    clock_t t0, t1;

    ErrorCompute ec;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > oxts_pose;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > odom_pose;

    t0 = clock();

    //Lidar Odom Main Entrance
    LOG(INFO) << "---------Pure LO test starts---------";
    RunPureLidarOdometry(sub_maps_[0]);

    t1 = clock();

    //For Error Evaluation
    for (unsigned int j = 0; j < sub_maps_[0].frame_number; j++)
    {
        oxts_pose.push_back(sub_maps_[0].raw_data_group[j].raw_gnss.pose.GetMatrix());
        odom_pose.push_back(sub_maps_[0].raw_data_group[j].raw_frame.pose.GetMatrix());
    }
    ec.compute(oxts_pose, odom_pose);
    ec.print_error();
    LOG(INFO) << "---------Pure LO test ends---------";
#if 0
    //Save processed data
    std::ostringstream oss;
    oss << transaction_id_ << "_" << 0;
    sub_maps_[0].full_cld_filename = "submap_" + oss.str() + ".pcd";
    sub_maps_[0].feature_cld_filename = "submap" + oss.str() + "_feature.pcd";
    sub_maps_[0].featrure_idx_filename = "submap" + oss.str() + "_feature_idx.txt";
    data_loader_.checkDir(submap_full_root_path_);
    data_loader_.checkDir(submap_feature_root_path_);
    data_loader_.saveSubmapToFile(submap_full_root_path_ + "/" + sub_maps_[0].full_cld_filename,
                                  submap_feature_root_path_ + "/" + sub_maps_[0].feature_cld_filename,
                                  submap_feature_root_path_ + "/feature_cld_idx.txt",
                                  sub_maps_[0]);
#endif
}

bool Transaction::RunPureLidarOdometry(Submap &submap)
{
    // Timing
    clock_t t0, t1;

    // Parameters List (add it to config later)

    // Distance Filter
    float xy_max = 40.0; // xy_max is the filter radius, points outside the circle would be deleted (unit:m)
    float z_min = -2.8;  // z_min is used to filter some noise underground points (unit:m)
    float z_max = 30.0;  // z_max is used to filter some noise unground points (unit:m)

    // Ground Filter (Segment Ground and Unground points)
    // gf_min_grid_num is the min point number in a grid. those grid whose point number < gf_min_grid_num would be ignored  
    int gf_min_grid_num = 9;
    // gf_grid_resolution is the size of a grid (unit:m)               
    float gf_grid_resolution = 0.75;
    // points whose [(z - min_z of the grid) > gf_max_grid_height_diff] would be regarded as unground points (unit:m)
    float gf_max_grid_height_diff = 0.15;
    // grids whose [(z_min - z_min of the grid's 8 neighbor grids) > gf_neighbor_height_diff would be regarded as unground grids (unit:m)
    float gf_neighbor_height_diff = 1.0;
    // points whose z is larger than gf_max_ground_height would be regarded as unground points whatever (unit:m)  
    float gf_max_ground_height = 1.0;
    // gf_downsample_rate_nonground is the random downsample ratio of detected unground points [the downsample is for efficiency concerning]
    int gf_downsample_rate_nonground = 4;
    // only gf_downsample_grid_number_first points would be randomly selected as the ground points in a grid
    // This group of ground points are used for registration [target ground points (more point number)] 
    int gf_downsample_grid_number_first = 4;  
    // only gf_downsample_grid_number_second points would be randomly selected as the ground points in a grid
    // This group of ground points are used for registration [source ground points (less point number)] 
    int gf_downsample_grid_number_second = 2; 

    // Feature Points Detection
    // Search neighbor_search_K nearest neighbor for doing neighbor PCA 
    // For neighbor PCA result, we define e1,e2,e3 (e1>e2>e3) are three eigen value and v1,v2,v3 are the correponding eigen vector
    // We call v1 the primary vector and v3 the normal vector
    // We define point linearty, planarity and curvature
    // linearty a_1d = (e1-e2)/e1 , planarity a_2d = (e2-e3)/e1 , curvature = e3/(e1+e2+e3)
    int neighbor_search_K = 9;
    // Those candidate edge points whose primary direction's z component < linear_vertical_cosine_min would be rejected
    float linear_vertical_cosine_min = 0.75;
    // Those candidate planar points whose normal direction's z component > planar_horizontal_cosine_max would be rejected
    float planar_horizontal_cosine_max = 0.4;
    // linearty threshold of edge feature points for registration [target edge points (more point number)] 
    float neighbor_linear_thre_target = 0.6;
    // planarty threshold of planar feature points for registration [target planar points (more point number)] 
    float neighbor_planar_thre_target = 0.55;
    // curvature threshold of sphere feature points for registration [target sphere points (more point number)] 
    float neighbor_curvature_thre_target = 0.2;
    // linearty threshold of edge feature points for registration [source edge points (less point number)] 
    float neighbor_linear_thre_source = 0.75;
    // planarty threshold of planar feature points for registration [source planar points (less point number)] 
    float neighbor_planar_thre_source = 0.75;
    // curvature threshold of sphere feature points for registration [source sphere points (less point number)] 
    float neighbor_curvature_thre_source = 0.25;
    // edge_point_source_appro_num points with larger linearity would be regarded as source edge points (less point number)
    int edge_point_source_appro_num = 400;   
    // planar_point_source_appro_num points with larger planarity would be regarded as source planar points (less point number)
    int planar_point_source_appro_num = 400; 
    // sphere_point_source_appro_num points with larger curvature would be regarded as source sphere points (less point number)
    int sphere_point_source_appro_num = 50;  
    
    // Tune these parameters for the trade-off between time efficiency and LO accuracy

    // Noise
    float noise_t = 0.0, noise_r = 0.0; // Now add no noise to gnss raw data 

    // Begin
    LOG(INFO) << "---------Pure LO test starts---------";
    LOG(INFO) << "Total frame number is " << submap.frame_number;

    // Preprocessing
    LOG(INFO) << "Preprocessing for each frame";
    std::shared_ptr<PointCloud> pointcloud_lidar_filtered(new PointCloud);
    std::vector<unsigned int> cld_unground_index;
    
    //submap.raw_data_group[0].raw_frame.pose.copyFrom(submap.pose);
    submap.raw_data_group[0].raw_frame.pose.SetPose(Eigen::Matrix4d::Identity(4,4));

    t0 = clock();
    for (int i = 0; i < submap.frame_number; i++)
    {
        LOG(INFO) << "---------------Frame [" << i << "] --------------";
        pointcloud_lidar_filtered = std::make_shared<PointCloud>();
        std::vector<unsigned int>().swap(cld_unground_index);

        submap.raw_data_group[i].raw_frame.ground_index.clear();
        submap.raw_data_group[i].raw_frame.ground_down_index.clear();
        submap.raw_data_group[i].raw_frame.edge_index.clear();
        submap.raw_data_group[i].raw_frame.edge_down_index.clear();
        submap.raw_data_group[i].raw_frame.planar_index.clear();
        submap.raw_data_group[i].raw_frame.planar_down_index.clear();
        submap.raw_data_group[i].raw_frame.sphere_index.clear();
        submap.raw_data_group[i].raw_frame.sphere_down_index.clear();
        submap.raw_data_group[i].raw_frame.cld_lidar_ptr = std::make_shared<PointCloud>(); // origin pcd
        
        // Add noise  //fix it later
        if (i == 0)
        {
            AddNoise(submap.raw_data_group[i], 0, 0);
        }
        else
        {
            AddNoise(submap.raw_data_group[i], 2 * noise_t, 2 * noise_r);
        }
        
        LOG(INFO)<<"Begin preprocessing";
        // Import cloud
        submap.raw_data_group[i].raw_frame.init();
        data_loader_.readPcdFile(lidar_root_path_ + "/" + submap.raw_data_group[i].raw_frame.pcd_file_name,
                                 submap.raw_data_group[i].raw_frame.cld_lidar_ptr, 1); //tag 1 for Point XYZI type

        // TODO Active Objects Filtering (@ Jingwei Li)

        // std::vector<std::shared_ptr<PointCloud>> cloud_show_ptr;
        // cloud_show_ptr.push_back(submap.raw_data_group[i].raw_frame.cld_lidar_ptr);
        // viewer_.DispalyNClouds(cloud_show_ptr, "Raw data", INTENSITY, 1);

        //Dis Filter, ROI filter
        filter_.DisFilter(submap.raw_data_group[i].raw_frame.cld_lidar_ptr,
                          pointcloud_lidar_filtered, xy_max, z_min, z_max);

        //viewer_.Dispaly2Clouds(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, pointcloud_lidar_filtered, "Compare", 1);

        submap.raw_data_group[i].raw_frame.cld_lidar_ptr->points.swap(pointcloud_lidar_filtered->points);
        pointcloud_lidar_filtered = std::make_shared<PointCloud>();

        // Ground Filter
        // 1. Points are divided into [Ground and Unground] Points, by grid two threshold ground filter
        // 2. (Optional) Detect the curb grids and determine the road grids, those unground points in road grids
        // would be regarded as active objects (car, pedestrian, etc..)
        // Notice, this active object filter is based on rules and is not robust (switch to learning based methods later)
        filter_.FastGroundFilter(submap.raw_data_group[i].raw_frame.cld_lidar_ptr, submap.raw_data_group[i].raw_frame.ground_index,
                                 submap.raw_data_group[i].raw_frame.ground_down_index, cld_unground_index,
                                 gf_min_grid_num, gf_grid_resolution, gf_max_grid_height_diff, gf_neighbor_height_diff, gf_max_ground_height,
                                 gf_downsample_grid_number_first, gf_downsample_grid_number_second, gf_downsample_rate_nonground);

        // Detected Feature Points 
        // 1. Unground points are divided into [Edge, Planar, Sharp Sphere] Points, by PCA and rules 
        // 2. Calculate unground points normal, by PCA 
        // Notice, all the features (Ground, Edge, Planar, Sphere) are detected by two different downsample rate 
        // The group with more feature points would be used as target cloud (not transformed in the process) for registration
        // while the other group with less points would be regarded as source cloud
        // The aim is to improve time efficiency
        filter_.RoughClassify(submap.raw_data_group[i].raw_frame.cld_lidar_ptr,
                              submap.raw_data_group[i].raw_frame.edge_down_index, submap.raw_data_group[i].raw_frame.edge_index,
                              submap.raw_data_group[i].raw_frame.planar_down_index, submap.raw_data_group[i].raw_frame.planar_index,
                              submap.raw_data_group[i].raw_frame.sphere_down_index, submap.raw_data_group[i].raw_frame.sphere_index,
                              cld_unground_index,
                              neighbor_search_K,
                              neighbor_linear_thre_target, neighbor_planar_thre_target, neighbor_curvature_thre_target,
                              neighbor_linear_thre_source, neighbor_planar_thre_source, neighbor_curvature_thre_source,
                              linear_vertical_cosine_min, planar_horizontal_cosine_max,
                              edge_point_source_appro_num, planar_point_source_appro_num, sphere_point_source_appro_num);


#if 0 //Display the feature points in a frame
        if (i < 1)
        {
            viewer_.DispalyFeatureFrame(submap.raw_data_group[i].raw_frame.cld_lidar_ptr,
                                          submap.raw_data_group[i].raw_frame.ground_index,
                                          submap.raw_data_group[i].raw_frame.edge_index,
                                          submap.raw_data_group[i].raw_frame.planar_index,
                                          submap.raw_data_group[i].raw_frame.sphere_index,
                                          "Frame_feature  Ground (Silver) ; Edge (Green) ; Planar (Blue) ; Sphere (Red)");
        }
#endif
        if (i > 0) {  
//Scan to scan registration
#if 1
            Pose3d pose2to1;
            int code = reg_.PairwiseReg(submap.raw_data_group[i - 1],
                             submap.raw_data_group[i], pose2to1);
            //if (code == 1) {
                submap.raw_data_group[i].raw_frame.last_transform.copyFrom(pose2to1);
                //Get frame's lidar odometry pose
                submap.raw_data_group[i].raw_frame.pose.SetPose(
                        submap.raw_data_group[i - 1].raw_frame.pose.GetMatrix() * pose2to1.GetMatrix());
                //LOG(INFO) << "Frame's position (OXTS): " << submap.raw_data_group[i - 1].raw_gnss.pose.trans.transpose();
                //LOG(INFO) << "Frame's position (LO): " << submap.raw_data_group[i - 1].raw_frame.pose.trans.transpose();

                //Free the memory
                submap.raw_data_group[i - 1].raw_frame.release_feature_indices();

                //Downsampling for memory management
                filter_.RandomDownsample(submap.raw_data_group[i - 1].raw_frame.cld_lidar_ptr, pointcloud_lidar_filtered, 100, 20, 180);
                pointcloud_lidar_filtered->points.swap(submap.raw_data_group[i - 1].raw_frame.cld_lidar_ptr->points);
                pointcloud_lidar_filtered = std::make_shared<PointCloud>();
                //break;
            //} else if (code == -1 || code == -2 || code == -3) {
                // Expand Dis Filter
                //gf_min_grid_num -= 3;
                //xy_max += 10;
                //continue;
            //} else {
                // code == -4  Registration Failed, Nothing we can do.
                //LOG(ERROR) << "Fix Registration Method Between Frames";
                //break;
            //}
#endif
#if 0
            reg_.pairwise_initial_guess(submap.raw_data_group[i - 1],
                                        submap.raw_data_group[i]);
#endif
           
        }
    }

    t1 = clock();
    //Free memory
    std::vector<unsigned int>().swap(cld_unground_index);

    filter_.RandomDownsample(submap.raw_data_group[submap.frame_number - 1].raw_frame.cld_lidar_ptr, pointcloud_lidar_filtered, 100, 20, 180);
    pointcloud_lidar_filtered->points.swap(submap.raw_data_group[submap.frame_number - 1].raw_frame.cld_lidar_ptr->points);

    LOG(INFO) << "Mean LO consuming time for each frame is " << ((float(t1 - t0) / CLOCKS_PER_SEC) / submap.frame_number * 1000) << " ms";

    //Merge the frames to the submap
    //MergeFrames(submap);

    //Use the submap's cloud in world frame to calculate bounding box
    //filter_.getCloudBound(*submap.cld_lidar_ptr, submap.bbox);

    //Evaluate the accuracy
    submap.submap_mae = GetMae(submap);

    pointcloud_lidar_filtered = std::make_shared<PointCloud>();
    
    ErrorCompute ec;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > oxts_pose;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > odom_pose;

    double dis_diff_lidar, dis_diff_oxts;
    std::shared_ptr<PointCloud> frame_pointcloud_temp = std::make_shared<PointCloud>();
    for (int i = 0; i < submap.frame_number; i++) {
        
        oxts_pose.push_back(submap.raw_data_group[i].raw_gnss.pose.GetMatrix());
        odom_pose.push_back(submap.raw_data_group[i].raw_frame.pose.GetMatrix());
   
        reg_.transformPointCloud(submap.raw_data_group[i].raw_frame.cld_lidar_ptr,
                                 frame_pointcloud_temp,
                                 submap.raw_data_group[i].raw_frame.pose);
        frame_pointcloud_temp->points.swap(submap.raw_data_group[i].raw_frame.cld_lidar_ptr->points);
        frame_pointcloud_temp = std::make_shared<PointCloud>();
    }
    
    frame_pointcloud_temp = std::make_shared<PointCloud>();

    dis_diff_lidar = (submap.raw_data_group[submap.frame_number - 1].raw_frame.pose.trans - submap.raw_data_group[0].raw_frame.pose.trans).norm();
    dis_diff_oxts = (submap.raw_data_group[submap.frame_number - 1].raw_gnss.pose.trans - submap.raw_data_group[0].raw_gnss.pose.trans).norm();
    LOG(WARNING) << " LO diff: " << dis_diff_lidar << " OXTS diff: " << dis_diff_oxts << "  Difference: " << dis_diff_lidar - dis_diff_oxts;
    
    ec.compute(oxts_pose, odom_pose);
    ec.print_error();

#if 1 //Display submap \
    
    //For display submap (convert submap's frame cloud to world coordinate system according to lidar odom pose)
    viewer_.DisplaySubmapClouds(submap, "LO Lidar Frame (Yellow) ; OXTS Lidar Frame (Purple) ; OXTS Body Frame (Cyan)", INTENSITY, 1);
    
    //Free the memory
    
#endif

    return true;
}


#if 0 // fix calib 1

      Eigen::Matrix3d transInforMat = Eigen::Matrix3d::Identity();
      transInforMat(0, 0) = gnssins_infos[0].pos_east_accuracy;
      transInforMat(1, 1) = gnssins_infos[0].pos_nrth_accuracy;
      transInforMat(2, 2) = gnssins_infos[0].pos_alti_accuracy;                       
      Eigen::Matrix3d gain = T.block<3,3>(0,0);
      transInforMat = gain * transInforMat * gain.transpose();
      transInforMat = transInforMat.inverse().eval();
      Eigen::Matrix3d rotInforMat = Eigen::Matrix3d::Ones() * 1e6;                       
                             
      double exp_sigma2_div4 = exp(gnssins_infos[0].rot_yaw_accuracy * gnssins_infos[0].rot_yaw_accuracy / 4);
      rotInforMat(2, 2) = 1 / (0.5 * (1 - exp_sigma2_div4) * (1 + exp_sigma2_div4 * cos(gnssins_infos[0].rot_yaw_accuracy) ) );
      raw_gnss.information_matrix.block<3,3>(0, 0) = transInforMat; // TODO
      raw_gnss.information_matrix.block<3,3>(3, 3) = rotInforMat;
      LOG(ERROR) << "Gnss Information Matrix is \n" << raw_gnss.information_matrix;

#endif

#if 0 // fix calib 2
    ceres::Solver::Options options_;
    options_.minimizer_progress_to_stdout;
    options_.function_tolerance = 1e-16;
    options_.parameter_tolerance = 1e-16;
    options_.gradient_tolerance = 1e-16;
    options_.max_num_iterations = 10000;
    options_.minimizer_progress_to_stdout = true;
    options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options_.num_threads = 1;
    options_.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options_.dense_linear_algebra_library_type = ceres::EIGEN;
    options_.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Problem problem;
    Eigen::Quaterniond calib(Eigen::Matrix3d::Identity());
    Eigen::Matrix<double, 3, 3> InformationMatrix = Eigen::Matrix<double, 3, 3>::Identity();
    ceres::LocalParameterization *q_p = new ceres::EigenQuaternionParameterization;
    for (int i = 0; i < submap_number_; i++)
    {
        for (int j = 0; j < sub_maps_[i].frame_number; j++)
        {
            ceres::CostFunction *cost_function = FixCalibErrorTerm::Create(sub_maps_[i].raw_data_group[j].raw_gnss.pose,
                                                                           sub_maps_[i].raw_data_group[j].raw_frame.pose,
                                                                           InformationMatrix);
            ceres::LossFunction *huber_loss = new ceres::HuberLoss(1.0);
            problem.AddResidualBlock(cost_function, huber_loss, calib.coeffs().data());
            problem.SetParameterization(calib.coeffs().data(), q_p);
        }
    }
    ceres::Solver::Summary summary;
    ceres::Solve(options_, &problem, &summary);
    // std::cout << summary.FullReport() << std::endl;
    LOG(WARNING) << "calib is " << calib.toRotationMatrix();

#endif

#if 0 //Fix UTM projection. use actual UTM projection instead of KITTI's
    //Calculate Location from latitude, longitude, altitude [KITTI's projection equation]
    Eigen::Vector2d mercator;
    mercator[0] = global_scale * lon * earth_radius;
    mercator[1] = global_scale * earth_radius * log(tan(0.25 * M_PI + lat / 2));

    double kitti_x = mercator[0] - kitti_origin_x;
    double kitti_y = mercator[1] - kitti_origin_y;

    printf("KITTI Proj X:%lf , Y:%lf\n", kitti_x, kitti_y);

    printf("UTM 51 Zone Proj X:%lf , Y:%lf\n", utm51_x, utm51_y);

    if (kkk % 100 == 0)
    {
        std::ofstream outfile("trajectory-wgs84.txt", std::ios::app);
        if (outfile)
        {
            outfile << setprecision(10) << lon << "\t" << lat << "\n";
        }
        outfile.close();
    }

    if (kkk % 10 == 0)
    {
        std::ofstream outfile("trajectory-utm-notran.txt", std::ios::app);
        if (outfile)
        {
            outfile << setprecision(10)<< x_utm << "\t" << y_utm << "\t" << alt << "\n";
        }
        outfile.close();

        std::ofstream outfile1("trajectory-utm51.txt", std::ios::app); // add after the file
        std::ofstream outfile2("trajectory-kitti.txt", std::ios::app);
        if (outfile1)
        {
            outfile1 << utm51_x << "\t" << utm51_y << "\t" << alt << "\n";
        }
        outfile1.close();
        if (outfile2)
        {
            outfile2 << kitti_x << "\t" << kitti_y << "\t" << alt << "\n";
        }
        outfile2.close();
    }

    printf("UTM 51 - KITTI : dX:%lf , dY:%lf ,unit(mm)\n", (utm51_x - kitti_x) * 1000.0, (utm51_x - kitti_x) * 1000.0);
    LOG(WARNING) << "Old - X:" << mercator[0] << ",Y:" << mercator[1];

    utm_kitti_x += (utm51_x - kitti_x) * 1000.0;
    utm_kitti_y += (utm51_y - kitti_y) * 1000.0;

#endif

} // namespace map_pose