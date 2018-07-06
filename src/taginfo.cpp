//
// Created by huangyewei on 16/9/6.
//

#include "taginfo.h"

Eigen::Matrix4d getRelativeTransform(float tag_size, float fx, float fy, float px, float py, apriltag_detection_t * detection) {
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;
    float s = tag_size/2;
    objPts.push_back(cv::Point3f(-s, s, 0));
    objPts.push_back(cv::Point3f( s, s, 0));
    objPts.push_back(cv::Point3f( s,-s, 0));
    objPts.push_back(cv::Point3f(-s,-s, 0));

	std::pair<float, float> p1 = std::make_pair(detection->p[0][0], detection->p[0][1]);
	std::pair<float, float> p2 = std::make_pair(detection->p[1][0], detection->p[1][1]);
	std::pair<float, float> p3 = std::make_pair(detection->p[2][0], detection->p[2][1]);
	std::pair<float, float> p4 = std::make_pair(detection->p[3][0], detection->p[3][1]);
    imgPts.push_back(cv::Point2f(p1.first, p1.second));
    imgPts.push_back(cv::Point2f(p2.first, p2.second));
    imgPts.push_back(cv::Point2f(p3.first, p3.second));
    imgPts.push_back(cv::Point2f(p4.first, p4.second));

    cv::Mat rvec, tvec;
    cv::Matx33f cameraMatrix(
            fx, 0, px,
            0, fy, py,
            0,  0,  1);
    cv::Vec4f distParam(0,0,0,0); // all 0?
    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
    cv::Matx33d r;

    Eigen::Vector3d rr;
    rr<<rvec.at<double>(0),rvec.at<double>(1),rvec.at<double>(2);

    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d wRo;
    wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0,0,0,1;

    return T;
}

taginfo::taginfo(float m_tagSize ,float m_fx ,float m_fy ,float m_px ,float m_py):
        tagSize(m_tagSize), fx(m_fx), fy(m_fy), px(m_px), py(m_py){

}

void taginfo::getCoordinateOneTag(cv::Point2f _tagPosCenter, apriltag_detection_t * detection, float _car_direction){
    if(tagSize == 0){
        cout<<"An error occured in taginfo.cpp getCoordinateOneTag!"<<endl;
        return;
    }
    type = "tag";
    Eigen::Matrix4d T, invT;
    Eigen::Vector4d X;
    Eigen::Vector2d X_tag_car;

    float dx, car_tag_angle_north;

    T=getRelativeTransform(tagSize, fx, fy, px, py, detection);
    invT=T.inverse();
    X<< 0,0,0,1;
    X=invT*X;
    X_tag_car<<X(2)/X(3),X(0)/X(3);

    id = detection->id;
	trans = sqrt(X_tag_car(0)*X_tag_car(0)+X_tag_car(1)*X_tag_car(1));

    dx=(detection->p[0][0]+detection->p[1][0]+detection->p[2][0]+detection->p[3][0])/4-px;
    angle=atan(dx/fx);

//    if(car_tag_angle_car<0)
//        angle=car_tag_angle_car+2*pi;
//    else
//        angle=car_tag_angle_car;

    tag_pos.x = _tagPosCenter.x;
    tag_pos.y = _tagPosCenter.y;

    car_tag_angle_north = _car_direction + angle;

    car_pos.x = _tagPosCenter.x - sin(car_tag_angle_north)*trans;
    car_pos.y = _tagPosCenter.y - cos(car_tag_angle_north)*trans;

}

bool taginfo::compareTagPos(taginfo t2){
    if(t2.type!=type)
        return false;
    double dx,dy,dw,dh;
    dx = abs(tag_pos.x - t2.tag_pos.x);
    dy = abs(tag_pos.y - t2.tag_pos.y);
    if(type == "park"){
        dw = abs(park.width - t2.park.width);
        dh = abs(park.height - t2.park.height);
    }
    else{
        dw = 0;
        dh = 0;
    }
    if( dx<1.5 && dy<1.5 && dw<0.5 && dh<0.5)
        return true;
    return false;
}