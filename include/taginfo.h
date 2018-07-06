//
// Created by huangyewei on 16/9/6.
//

#ifndef PARKINGLOT_TAGINFO_H
#define PARKINGLOT_TAGINFO_H

#include <fstream>
#include <iostream>
#include "Apriltag/apriltag.h"
#include "Apriltag/tag36h11.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>


using namespace std;
using namespace cv;

class parkinfo{
public:
    float width;
	float height;
	float direction;
};

class taginfo{
private:
	float pi = 3.1415926535898;
	float tagSize;
	float fx;
	float fy;
    float px;
	float py;

public:
    int id;
    string type;
    Point2f car_pos;
    Point2f tag_pos;

    float angle;
	float trans;
    //boost::shared_ptr<parkinfo> park=NULL;
	parkinfo park;
    taginfo(float m_tagSize=0 ,float m_fx=0 ,float m_fy=0 ,float m_px=0 ,float m_py=0);
	void getCoordinateOneTag(Point2f _tagPosCenter, apriltag_detection_t * detection, float car_direction);
	bool compareTagPos(taginfo t2);
};



#endif //PARKINGLOT_TAGINFO_H
