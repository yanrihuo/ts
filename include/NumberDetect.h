//
// Created by huangyewei on 2017/4/8.
//

#ifndef PARKINGLOT_NUMBERDETECT_H
#define PARKINGLOT_NUMBERDETECT_H

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;

class numberdetect{
private:
    char* data = NULL;
    char* names = NULL;
    char* cfg = NULL;
    char* weights = NULL;
public:
    numberdetect(string _data = "/Users/Yewei/ClionProjects/parkinglot_multi/data/cfg/voc.data",
                 string _names = "/Users/Yewei/ClionProjects/parkinglot_multi/data/cfg/voc.names",
                 string _cfg = "/Users/Yewei/ClionProjects/parkinglot_multi/data/cfg/yolo-voc.cfg",
                 string _weights = "/Users/Yewei/ClionProjects/parkinglot_multi/data/cfg/yolo-voc_400.weights");
    ~numberdetect(){
        if(data!=NULL)
            delete data;
        if(names!=NULL)
            delete names;
        if(cfg!=NULL)
            delete cfg;
        if(weights!=NULL)
            delete weights;
    };
    int detect(Mat &img);
};





#endif //PARKINGLOT_NUMBERDETECT_H
