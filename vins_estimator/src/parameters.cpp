#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double      BIAS_ACC_THRESHOLD;
double      BIAS_GYR_THRESHOLD;
double      SOLVER_TIME;
int         NUM_ITERATIONS;
int         ESTIMATE_EXTRINSIC;
int         ESTIMATE_TD;
int         ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double      ROW, COL;
double      TD, TR;

std::string TimestampPrint(const uint64_t nsec)
{
    uint64_t sec  = static_cast<uint64_t>(std::floor(nsec / 1000000000));
    uint64_t msec = static_cast<uint64_t>(nsec % 1000000000);

    return std::to_string(sec) + ", " + std::to_string(msec);
}

void readParameters(ros::NodeHandle& n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        LOG(INFO) << "readParameters --- ERROR: Wrong path to settings";
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME    = fsSettings["max_solver_time"];     // 单次优化最大求解时间
    NUM_ITERATIONS = fsSettings["max_num_iterations"];  // 单词优化最大迭代次数
    MIN_PARALLAX   = fsSettings["keyframe_parallax"];   // 根据视差确定关键帧
    MIN_PARALLAX   = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    LOG(INFO) << "readParameters --- result path " << VINS_RESULT_PATH;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    // imu、图像相关参数
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW   = fsSettings["image_height"];
    COL   = fsSettings["image_width"];
    LOG(INFO) << "readParameters --- ROW: " << ROW << ", COL:  " << COL;

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        LOG(WARNING) << "readParameters --- have no prior about extrinsic param, calibrate extrinsic param";
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    }
    else
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            LOG(WARNING) << "readParameters ---  Optimize extrinsic param around initial guess!";
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }

        if (ESTIMATE_EXTRINSIC == 0)
        {
            LOG(WARNING) << "readParameters --- fix extrinsic param ";
        }

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        LOG(INFO) << "readParameters --- Extrinsic_R : " << std::endl << RIC[0];
        LOG(INFO) << "readParameters --- Extrinsic_T : " << std::endl << TIC[0].transpose();
    }

    INIT_DEPTH         = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    // 传感器时间延时相关
    TD          = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        LOG(INFO) << "readParameters --- Unsynchronized sensors, online estimate time offset, initial td: " << TD;
    else
        LOG(INFO) << "readParameters --- Synchronized sensors, fix time offset: " << TD;

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        LOG(INFO) << "readParameters --- rolling shutter camera, read out time per line: " << TR;
    }
    else
    {
        TR = 0;
    }

    fsSettings.release();
}
