//
// Created by huangyewei on 16/6/27.
//

#ifndef PARKINGLOT_TAG_H
#define PARKINGLOT_TAG_H
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <stdlib.h>
#include "kalman.h"
#include "taginfo.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Apriltag/tag36artoolkit.h"

using namespace std;
using namespace cv;

class tag {
private:
    const float pi = 3.1415926535898;
    float scale;
    Point2f origin;
    Point2f edge;

private://apriltag
    float tagSize;
    float fx;
    float fy;
    float px;
    float py;
    apriltag_family_t *tf;
    apriltag_detector_t *td;

private://FastSLAM
    kalman *Kalman;

    int SLAM_Mood;

    int time_count=0;
    Point2f pre_IMU_pt, pre_optimize_pt;
    vector<taginfo> lm_list;

    map<int, string> type_map;
    map<int, int> id_map;
    bool start = 0;

public://for IMU restriction
    bool bInIMURestriction;
    
    //Added by John for IMU restricted zones
    vector<Point2f> ptIMUrestrict;
    float fIMUrestrictRadius;

public:
    tag(float m_tagSize,float m_fx ,float m_fy ,float m_px ,float m_py);
    ~tag();
    //detect tag
    int AprilTagDetect(Mat &img, float v_direction, float max_dist, vector<taginfo>& TagInfo );
    int LotDetect(vector<Point2f> Park_4pt, Point2f _v_trans, float v_angle,
                  vector<taginfo>& ParkInfo, vector<Point2f> &mLot);
    //get pose

    bool getCarPose(Point2f &vehiclePt, Point2f &optimizePt,
                            float v_, float G_, float &_v_direction, vector<taginfo> Info, float cost);
    bool getCarPoseWithIMU(Point2f &IMUPos, Point2f _v_trans);

    void circleParkOnImage(Mat &img, float x[], float y[]);

    //read info
    void readParam(string mood, string path_INI, string path_LM);
    void writeLandMarkPose(string path);

    //draw Map
    void drawLandMark(Mat &img);
    void drawPark(Mat &img, taginfo lm, char color);
    void drawCar(Point2f CarPos, Mat&img, int r, int g, int b);

private:
    void readMap(string path_LM);

    bool img_2_meter(vector<Point2f> pts, float angle, taginfo &info);
    void sort(float x[], float y[]);
    int searchLandMark(taginfo &info);


    void drawTag(Mat &img, taginfo lm);
    void circleTagOnImage(Mat &img, apriltag_detection_t det);

};


#endif //PARKINGLOT_TAG_H
