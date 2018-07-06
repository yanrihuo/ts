//
// Created by huangyewei on 2017/4/9.
//

#ifndef PARKINGLOT_SLAMHELPER_H
#define PARKINGLOT_SLAMHELPER_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <thread>
#include <mutex>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>


//pylon
#include <pylon/PylonIncludes.h>
//lcm
#include <lcm/lcm-cpp.hpp>
//#include "recon.hpp"
#include "reckonlcm.hpp"
#include <lcm/lcm.h>
#include <lcm/eventlog.h>
#include <lcm/lcm_coretypes.h>

#include "PacketTagSlam.hpp"
#include "PacketLot_with_time.hpp"

#include "tag.h"
#include "parameterReader.h"


using namespace std;
using namespace cv;
using namespace Pylon;

static parameterReader *pd = parameterReader::GetInstance();
const int imageWidth = atoi( pd->getData("image_width").c_str() );
const int imageHeight = atoi( pd->getData("image_height").c_str() );
const double m_tagSize = atof( pd->getData("tag_size").c_str() );
const double m_fx = atof( pd->getData("camera_fx").c_str() );
const double m_fy = atof( pd->getData("camera_fy").c_str() );
const double m_cx = atof( pd->getData("camera_cx").c_str() );
const double m_cy = atof( pd->getData("camera_cy").c_str() );
const double distort1 = atof( pd->getData("camera_distort1").c_str() );
const double distort2 = atof( pd->getData("camera_distort2").c_str() );
const double distort3 = atof( pd->getData("camera_distort3").c_str() );
const double distort4 = atof( pd->getData("camera_distort4").c_str() );

const string run_type = pd->getData("mapping_or_localization");
const string ini_path = pd->getData("ekf_param_path");
const string map_path = pd->getData("tag_path");
const string output_path = pd->getData("map_result_path");

//distortion
const cv::Mat m_D = (cv::Mat_<double>(1,8)<< distort1, distort2, distort3, distort4, 0,0,0,0);
//left camera matrix
const cv::Mat m_K = (cv::Mat_<double>(3,3)<<m_fx, 0, m_cx,
        0, m_fy, m_cy,
        0, 0, 1 );


class SLAMHelper{
public:
    bool start_offline, save_online;

    int tag_count;

    double x_pre, y_pre;
    Mat map1, map2;
    tag *tag_dealer;

    string save_data_dir;

public:
    SLAMHelper();

    //static void tag_thread_online(SLAMHelper *helper);
    //static void park_thread_online(SLAMHelper *helper);

    void tag_thread_offline(string TagDataPath, double x_now, double y_now, float a_now);
    void park_thread_offline(vector <Point2f> park_4pts, double x_now, double y_now, float a_now);

    void save();
public:
    int CreatDir(string dir);
    int returnPositonInString(string str, int time_start, string match);
    void splitString(string str, long& time, float &dx, float &dy, float &angle);
    void getROI(float x[], float y[], Mat &img, Mat &roi);

};




#endif //PARKINGLOT_SLAMHELPER_H
