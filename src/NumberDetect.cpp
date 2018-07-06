//
// Created by huangyewei on 2017/4/9.
//
#include "NumberDetect.h"

bool LessSort(rectInformation a, rectInformation b){
    return (a.x<b.x);
}

numberdetect::numberdetect(string _data, string _names, string _cfg, string _weights){
    const char *tmp;
    tmp = _data.c_str();
    data = new char[strlen(tmp)+1];
    strcpy(data, tmp);
    tmp =  _names.c_str();
    names = new char[strlen(tmp)+1];
    strcpy(names, tmp);
    tmp = _cfg.c_str();
    cfg = new char[strlen(tmp)+1];
    strcpy(cfg, tmp);
    tmp = _weights.c_str();
    weights = new char[strlen(tmp)+1];
    strcpy(weights, tmp);
}

int numberdetect::detect(Mat &img){
    IplImage tmp = IplImage(img);
    IplImage *imgIpl = &tmp;
    vector<rectInformation> detectors;
    int id = 0;
    //detectResultStruct result =NumberDetect(data, names, cfg, weights, imgIpl);
    //cvReleaseImage(tmp);
    //if(result.size == 0)
    if(result.size !=2)
        return -1;
    for(int j=0; j<result.size; j++){
        //detectors.push_back(*result.rect);
        result.rect++;
    }
    sort(detectors.begin(),detectors.end(),LessSort);
    for(int j=0; j<result.size; j++){
        id = id*10 + detectors[j].classID;
    }
    id = id+100;
    return id;
}

