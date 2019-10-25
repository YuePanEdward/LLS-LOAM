#include <iostream>
#include <fstream>
#include <string>
#include <map>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "types.h"
#include "llslo.h"
#include "map_viewer.hpp"
#include "dataio.hpp"

DEFINE_int32(transaction_id, -1, "transaction id.");
DEFINE_string(transaction_folder, "", "transaction folder.");
DEFINE_string(pcd_filelist, "", "pcd file list.");
DEFINE_string(config_file, "", "config file.");
DEFINE_string(output_foler, "", "output folder.");
DEFINE_int32(begin_frame_id, -1, "begin frame id.");
DEFINE_int32(end_frame_id, -1, "end frame id.");
DEFINE_string(exp_num, "1", "id of experiment");


int main(int argc, char **argv)
{
    //google::SetLogDestination(google::GLOG_INFO, "./log");
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging("lls_loam"); // init google logging must be carried out after parse command line flags is carried out.
    LOG(INFO) << "Logging is written to " << FLAGS_log_dir;
    CHECK(FLAGS_transaction_folder != "") << "Need to specify the pcd folder to read.";
    CHECK(FLAGS_pcd_filelist != "") << "Need to specify the pcd file list to read.";
    CHECK(FLAGS_begin_frame_id != -1) << "Need to specify the begin frame id.";
    CHECK(FLAGS_end_frame_id != -1) << "Need to specify the end frame id.";

    lls_loam::transaction_param_t trsct_prm;
    trsct_prm.transaction_id = FLAGS_transaction_id;
    trsct_prm.transaction_root_path = FLAGS_transaction_folder;
    trsct_prm.lidar_root_path = FLAGS_transaction_folder + "/HDL64";
    trsct_prm.gnss_root_path = FLAGS_transaction_folder + "/OXTS";
    trsct_prm.lidar_file_list = FLAGS_pcd_filelist;
    trsct_prm.submap_full_root_path = FLAGS_transaction_folder + "/SubmapFull_" + FLAGS_exp_num;
    trsct_prm.submap_feature_root_path = FLAGS_transaction_folder + "/SubmapFeature_" + FLAGS_exp_num;
    //Key parameter to set
    trsct_prm.local_map_max_size=10; //Local map's max frame number 

    lls_loam::Transaction transaction(trsct_prm);

    // Import Data
    transaction.LoadKITTIData(FLAGS_begin_frame_id,FLAGS_end_frame_id);

    // Lidar Odometery
    transaction.RunPureLidarOdometry(transaction.sub_maps_[0]);
}