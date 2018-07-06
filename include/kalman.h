//
// Created by huangyewei on 16/9/6.
//

#ifndef PARKINGLOT_KALMAN_H
#define PARKINGLOT_KALMAN_H
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Dense>
#include "fastslam/fastslam_core.h"
#include "fastslam/utils.h"
#include "taginfo.h"

using namespace std;
using namespace cv;

class kalman {
private:
    const float pi = 3.14159265;

    int SLAM_Mood;
    SLAM_Conf   *slam_conf;//params

    //initial settings
    MatrixXf Q;
    MatrixXf R;
    float sigmaPhi;// radians, heading uncertainty

    //wp
    VectorXf x;
    MatrixXf P;

    float dt;
    float direction_pre;

    vector<int> ftag; //identifier for each landmark
    vector<int> da_table;//data ssociation table

public:
    kalman();
    ~kalman();
    void setParam(string path, vector<taginfo> lmpt, string mood);
    void setParamMode(vector<taginfo> lmpt, string mood);
    void initialize(Point2f pt, float direction_now);
    bool KalmanUpdate(bool updatelm, float G_,float& direction_now, bool bHeadIgnore, float trans,
                      vector<taginfo> &TagInfo, Point2f &vehicle, float dt_);

    bool Nearest_Neighbor_Match(VectorXf z, int &id);

    //added for retreiving and reseting P matrix
    MatrixXf getP();
    bool setKalman(MatrixXf p_, VectorXf x_, vector<int> ftag_, vector<int> da_);
private:
    int find_idx(int id_ftag);
};








#endif //PARKINGLOT_KALMAN_H
