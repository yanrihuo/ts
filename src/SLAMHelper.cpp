//
// Created by huangyewei on 2017/4/9.
//
#include"SLAMHelper.h"

SLAMHelper::SLAMHelper()
{
    x_pre=0; y_pre=0;
    start_offline = 0;
    tag_count = 0;
   
    //
    tag_dealer = new tag(m_tagSize, m_fx , m_fy , m_cx , m_cy);
    tag_dealer->readParam(run_type, ini_path, map_path);

    Mat_<double>I =  Mat_<double>::eye(3, 3);
    Mat new_K= m_K;
    initUndistortRectifyMap(m_K, m_D, I, new_K, Size(imageWidth, imageHeight), 0, map1, map2);

    int saveDataFlag = atoi( pd->getData("saveOnlineData").c_str() );
    if(saveDataFlag != 1)
        save_online = false;
    else {
        save_online = true;
        //get the time and creat pathName
        time_t now;
        struct tm timenow;
        time(&now);
        timenow = *localtime(&now);

        //save tag img
        stringstream saveDataDirStream;
        saveDataDirStream << "../data/img" << timenow.tm_year + 1900 << timenow.tm_mon + 1 << timenow.tm_mday <<
                          timenow.tm_hour << timenow.tm_min << timenow.tm_sec;
        save_data_dir = saveDataDirStream.str();

        ofstream TagReckonOut (save_data_dir + "TagReckon.txt");
        ofstream ParkReckonOut(save_data_dir + "ParkReckon.txt");
        ofstream ParkDetectOut(save_data_dir + "ParkDetect.txt");
        TagReckonOut.close();
        ParkReckonOut.close();
        ParkDetectOut.close();
    }
}

///***********************************  offline test  **********************************//

void SLAMHelper::tag_thread_offline(string TagDataPath, double x_now, double y_now, float a_now){
    struct timeval tpstart,tpend;
    double timeuse;
    gettimeofday(&tpstart,NULL);

    cout<<"tag mode:"<<endl;

    vector <taginfo> Info;
    Mat input, detect, ground;
    double dx, dy;
    Point2f vehiclePos, KFPos, IMUPos;

    stringstream ss;
    tag_count++;
    ss<<TagDataPath<<"l_"<<tag_count<<".jpg";
    input = imread(ss.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    remap(input, input, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
    cvtColor(input, detect, CV_GRAY2RGB);
    if(tag_count == 1){
        x_pre = x_now;
        y_pre = y_now;
    }
    dx = x_now - x_pre;
    dy = y_now - y_pre;

    x_pre = x_now;
    y_pre = y_now;

    cout<<"IMU:"<<dx<<" "<<dy<<endl;
    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
    cout << "Time cost initialize:" << timeuse << endl;
    tpstart = tpend;

    int count = tag_dealer->AprilTagDetect(input, a_now, 30, Info);
    if(count==0 && !start_offline)
        return;
    if(!start_offline)
        start_offline = true;

    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
    cout << "Time cost ApriltagDetect:" << timeuse << endl;
    tpstart = tpend;

    tag_dealer->getCarPose(vehiclePos, KFPos, sqrt(dx*dx + dy*dy), 0, a_now, Info, 0);
    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
    cout << "Time cost getCarPose:" << timeuse << endl;
    tpstart = tpend;

    tag_dealer->getCarPoseWithIMU(IMUPos, Point2f(dx, dy));

    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
    cout << "Time cost getPosewithIMU:" << timeuse << endl;
    tpstart = tpend;

    ground = Mat::zeros(500, 800, 16);
    tag_dealer->drawLandMark(ground);

    if(count!=0)
        tag_dealer->drawCar(vehiclePos, ground, 176, 201, 90);
    tag_dealer->drawCar(IMUPos, ground, 49, 181, 214);
    tag_dealer->drawCar(KFPos, ground, 247, 82, 165);

    imshow("result", ground);
    waitKey(1);

    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
    cout << "Time cost drawing:" << timeuse << endl;


}

void SLAMHelper::park_thread_offline(vector <Point2f> park_4pts, double x_now, double y_now, float a_now){
    if(!start_offline)
        return;
    struct timeval tpstart,tpend;
    double timeuse;
    gettimeofday(&tpstart,NULL);

    cout<<"park mode:"<<endl;

    vector <taginfo> Info;
    vector<Point2f> mLot;
    stringstream ss;
    int count;
    double dx, dy;
    Mat ground;
    Point2f vehiclePos, KFPos, IMUPos;

    dx = x_now - x_pre;
    dy = y_now - y_pre;

    cout<<"IMU:"<<dx<<" "<<dy<<endl;
    x_pre = x_now;
    y_pre = y_now;

    count = tag_dealer->LotDetect(park_4pts, Point2f(dx, dy), a_now, Info, mLot);
    tag_dealer->getCarPose(vehiclePos, KFPos, sqrt(dx*dx + dy*dy), 0, a_now, Info, 0);
    tag_dealer->getCarPoseWithIMU(IMUPos, Point2f(dx, dy));

    ground = Mat::zeros(500, 800, 16);
    tag_dealer->drawLandMark(ground);

    if(count!=0){
        tag_dealer->drawCar(vehiclePos, ground, 176, 201, 90);
        tag_dealer->drawPark(ground, Info[count-1], 'R');
    }
    tag_dealer->drawCar(IMUPos, ground, 49, 181, 214);
    tag_dealer->drawCar(KFPos, ground, 247, 82, 165);

    imshow("result", ground);
    waitKey(1);

    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
    cout << "Time cost park:" << timeuse << endl;

}

//***********************************  offline test  **********************************//


//寻找分隔符
int SLAMHelper::returnPositonInString(string str, int time_start, string match){
    int len = str.length();
    for(int i=time_start; i<len; i++){
        string t = str.substr(i, 1);
        if(t == match){
            return i;
        }
    }
    return -1;
}

//读取时间戳
void SLAMHelper::splitString(string str, long& time, float &dx, float &dy, float &angle){
    int time_start, time_end, time1, time2, time3, time4;
    time_start = returnPositonInString(str, 0, " ") + 1;
    time_end = returnPositonInString(str, time_start, "-");

    time1 = atoi(str.substr(time_start, time_end).c_str());

    time_start = time_end+1;
    time_end = returnPositonInString(str, time_start, "-");

    time2 = atoi(str.substr(time_start, time_end).c_str());

    time_start = time_end+1;
    time_end = returnPositonInString(str, time_start, "-");

    time3 = atoi(str.substr(time_start, time_end).c_str());

    time_start = time_end+1;
    time_end = returnPositonInString(str, time_start, " ");

    time4 = atoi(str.substr(time_start, time_end).c_str());

    time = (time1*3600+time2*60+time3)*1000+time4;

    time_start = time_end+1;
    time_end = returnPositonInString(str, time_start, " ");
    dy = atof(str.substr(time_start, time_end).c_str());

    time_start = time_end+1;
    time_end = returnPositonInString(str, time_start, " ");
    dx = atof(str.substr(time_start, time_end).c_str());

    time_start = time_end+1;
    angle = atof(str.substr(time_start, str.length()).c_str());

}

//获取字符检测区域
void SLAMHelper::getROI(float x[], float y[], Mat &img, Mat &roi){
    Mat processed;
    copyMakeBorder(img, processed, 20, 20, 20, 20, BORDER_REPLICATE);

    roi = processed(Rect(x[0]/2+x[1]/2, y[0]/2+y[1]/2, 40, 40));

    if (x[3] < x[0]){
        transpose(roi, roi);
        flip(roi, roi, 1);
    }
    else{
        transpose(roi, roi);
        flip(roi, roi, 0);
    }
}

int SLAMHelper::CreatDir(string dir)
{
    const int dir_err = mkdir(dir.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == dir_err)
    {
        printf("Error creating directory!n");
        return 0;
    }
    return 1;
}

void SLAMHelper::save(){
    tag_dealer->writeLandMarkPose(output_path);
}