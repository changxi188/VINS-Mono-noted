#include "parameters.h"

std::string              IMAGE_TOPIC;
std::string              IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string              FISHEYE_MASK;
int                      MAX_CNT;
int                      MIN_DIST;
int                      WINDOW_SIZE;
int                      FREQ;
double                   F_THRESHOLD;
int                      SHOW_TRACK;
int                      STEREO_TRACK;
int                      EQUALIZE;
int                      ROW;
int                      COL;
int                      FOCAL_LENGTH;
int                      FISHEYE;
bool                     PUB_THIS_FRAME;

// 读配置参数，通过roslaunch文件的参数服务器获得
void readParameters(ros::NodeHandle& n)
{
    std::string config_file;
    // 首先获得配置文件的路径
    config_file = readParam<std::string>(n, "config_file");
    // 使用opencv的yaml文件接口来读取文件
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        LOG(ERROR) << "ERROR: Wrong path to settings";
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT     = fsSettings["max_cnt"];
    MIN_DIST    = fsSettings["min_dist"];
    ROW         = fsSettings["image_height"];
    COL         = fsSettings["image_width"];
    FREQ        = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK  = fsSettings["show_track"];
    EQUALIZE    = fsSettings["equalize"];  // 是否做均衡化处理
    FISHEYE     = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE    = 20;
    STEREO_TRACK   = false;
    FOCAL_LENGTH   = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
}
