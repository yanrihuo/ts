//
// Created by huangyewei on 2017/4/9.
//

#ifndef PARKINGLOT_PARAM_H
#define PARKINGLOT_PARAM_H
#include "opencv2/opencv.hpp"

#define TEST_NUM 5000
#define TEST_INI "../data/TagJiaoTong.ini"
#define TEST_COORDINATE "../data/TagJiaoTong.txt"
#define TEST_TXT "../data/New.txt"

const double m_tagSize = 0.488; // April tag side  length in meters of square black frame
const double m_fx =1100.8;
const double m_fy = 1105.3;// camera focal length in pixels
const double m_px = 662.2400;
const double m_py = 505.6058;// camera principal point
const int width = 1280;
const int height = 1024;

Mat m_K = (cv::Mat_<double>(3,3)<<m_fx, 0, m_px,
        0, m_fy, m_py,
        0, 0, 1 );

Mat m_D = (cv::Mat_<double>(1,8)<< -0.1727, 0.1784, 0.00, 0.00000, 0, 0,0,0);

#endif //PARKINGLOT_PARAM_H
