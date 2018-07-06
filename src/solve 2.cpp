//
// Created by huangyewei on 16/6/25.
//
#include <fstream>
#include <opencv/highgui.h>
#include <sys/time.h>
#include <thread>
#include <mutex>

//定位及画图函数
#include "tag.h"
//坐标转换等预处理函数
#include "SLAMHelper.h"
//参数文件
#include "Param.h"

using namespace cv;
using namespace std;

bool detect_park=0;
double x_pre=0, y_pre=0;
tag m_tag(m_tagSize, m_fx , m_fy , m_px , m_py);
mutex mtx;



//lcm class
class Handler
{
public:
    ~Handler() {}

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const exlcm::recon* msg)
    {
        x = msg->y;
        y = msg->x;
        angle = msg->angle;
       //cout << "x = " << x << "; y = " << y << ";angle = "<<angle<<endl;
    }

    void handleParkinglotMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const PacketLot* msg)
    {
        mtx.lock();
        detect_park = 1;
        mtx.unlock();
        polys.clear();
        polys.push_back(Point2f(msg->p0.x, msg->p0.y));
        polys.push_back(Point2f(msg->p1.x, msg->p1.y));
        polys.push_back(Point2f(msg->p2.x, msg->p2.y));
        polys.push_back(Point2f(msg->p3.x, msg->p3.y));

//        park_thread(polys, angle, x, y);

    }
    //int handle_type;
    double x,y,angle;
    vector <Point2f> polys;
};

Handler handlerObject;

void tag_thread_func(Mat map1, Mat map2){

    //
    PylonInitialize();

    CTlFactory& TlFactory = CTlFactory::GetInstance();
    DeviceInfoList_t lstDevices;
    TlFactory.EnumerateDevices(lstDevices);
    if (!lstDevices.empty()) {
        DeviceInfoList_t::const_iterator it;
        for (it = lstDevices.begin(); it != lstDevices.end(); ++it)
            cout << it->GetFullName() << endl;;
    }
    else
    {
        cerr << "No devices found!" << endl;
    }

    // Create an instant camera object with the camera device found first.
    CInstantCamera camera(TlFactory.CreateDevice(lstDevices[0]));

    camera.Open();
    if (!camera.IsOpen())
        cerr << "can not open camera!" << endl;

    // The parameter MaxNumBuffer can be used to control the count of buffers
    // allocated for grabbing. The default value of this parameter is 10.
    //        camera.MaxNumBuffer = 5;

    CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = PixelType_BGR8packed;
    CPylonImage polynImg;

    // Start the grabbing of c_countOfImagesToGrab images.
    // The camera device is parameterized with a default configuration which
    // sets up free-running continuous acquisition.
    camera.StartGrabbing( GrabStrategy_LatestImages);

    // This smart pointer will receive the grab result data.
    CGrabResultPtr ptrGrabResult;


    while(true){
        camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
        if (!ptrGrabResult->GrabSucceeded())
            continue;
        formatConverter.Convert(polynImg, ptrGrabResult);
        Mat  cvImg = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)polynImg.GetBuffer());

        struct timeval tpstart, tpend;
        double timeuse;
        gettimeofday(&tpstart, NULL);
        cout<<"Tag mode:"<<endl;
        Point2f vehiclePos, KFPos, IMUPos;

        mtx.lock();
        double x_now = handlerObject.x;
        double y_now = handlerObject.y;
        double angle = handlerObject.angle;
        mtx.unlock();

        double dx = x_now - x_pre;
        double dy = y_now - y_pre;
        cout <<"dx:"<<dx<<endl;
        cout <<"dy:"<<dy<<endl;

        mtx.lock();
        x_pre = x_now;y_pre = y_now;
        mtx.unlock();

        if(abs(dx) >1 || abs(dy)>1)
            continue;
        Mat input, ground;
        cvtColor(cvImg, input, CV_RGB2GRAY);
        remap(input, input, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

        vector<taginfo> lm_Info;

        m_tag.AprilTagDetect(input, angle, 15, lm_Info);
        mtx.lock();
        m_tag.getCarPose(vehiclePos, KFPos, Point2f(dx, dy), angle, lm_Info);
        mtx.unlock();
        m_tag.getCarPoseWithIMU(IMUPos, Point2f(dx, dy), angle);
        ground = Mat::zeros(500, 500, cvImg.type());
        m_tag.drawLandMark(ground);
        m_tag.drawCar(KFPos, ground, 247, 82, 165);
        m_tag.drawCar(vehiclePos, ground, 176, 201, 90);
        m_tag.drawCar(IMUPos, ground, 49, 181, 214);
        imshow("map",ground);
        waitKey(1);
        gettimeofday(&tpend,NULL);
        timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
        cout << "Time cost:" << timeuse << endl;
        usleep(100*1000);

    }
}

void park_thread_func(){
    while (true){
        mtx.lock();
        bool check = detect_park;
        detect_park = 0;
        double angle = handlerObject.angle;
        double x_now = handlerObject.x;
        double y_now = handlerObject.y;
        mtx.unlock();
        if(check != 1){
            continue;
        }
        struct timeval tpstart, tpend;
        double timeuse;
        gettimeofday(&tpstart, NULL);
        cout<<"Park mode:"<<endl;
        Point2f vehiclePos, KFPos, IMUPos;

        double dx = x_now - x_pre;
        double dy = y_now - y_pre;

        cout <<"dx:"<<dx<<endl;
        cout <<"dy:"<<dy<<endl;

        mtx.unlock();
        x_pre = x_now;y_pre = y_now;
        mtx.unlock();

        if(abs(dx) >1 || abs(dy)>1)
            continue;
        vector<taginfo> lm_Info;
        VectorXd lm_temp;
        vector <VectorXd> lms;
        double x[4], y[4];

        mtx.lock();
        for (int i=0; i<handlerObject.polys.size()/4; i++){
            x[0] = handlerObject.polys[i*4 + 0].x;y[0] = handlerObject.polys[i*4 + 0].y;
            x[1] = handlerObject.polys[i*4 + 1].x;y[1] = handlerObject.polys[i*4 + 1].y;
            x[2] = handlerObject.polys[i*4 + 2].x;y[2] = handlerObject.polys[i*4 + 2].y;
            x[3] = handlerObject.polys[i*4 + 3].x;y[3] = handlerObject.polys[i*4 + 3].y;

            sort(x,y);
            if(!img_2_meter(x, y, angle, lm_temp))
                continue;

            lms.push_back(lm_temp);
        }
        mtx.unlock();

        vector<Point2f> mLot;
        m_tag.LotDetect(lms,Point2f(dx, dy),angle,lm_Info, mLot);
        mylcm.publish("Lot", &mLot);

        mtx.lock();
        m_tag.getCarPose(vehiclePos, KFPos, Point2f(dx, dy), angle, lm_Info);
        mylcm.publish("Pos", &KFPos);

        mtx.unlock();
        m_tag.getCarPoseWithIMU(IMUPos, Point2f(dx, dy), angle);

        gettimeofday(&tpend,NULL);
        timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
        cout << "Time cost:" << timeuse << endl;
        usleep(50*1000);
    }

}

void getrecon(lcm::LCM &lcm)
{
    while(0 == lcm.handle());
}

void getparkinglot(lcm::LCM &lcm)
{
    while(0 == lcm.handle());
}

int main ()
{
    lcm::LCM lcmer("udpm://239.255.76.67:7667?ttl=1");

    if(!lcmer.good())
        return -1;
    lcmer.subscribe("reconPos", &Handler::handleMessage, &handlerObject);
    lcmer.subscribe("Lot", &Handler::handleParkinglotMessage, &handlerObject);

    //制造前视图像纠正模版
    Mat map1, map2;
    Mat_<double> I = Mat_<double>::eye(3, 3);
    Mat new_K = m_K;
    initUndistortRectifyMap(m_K, m_D, I, new_K, Size(width, height), 0, map1, map2);

    m_tag.readParam(TEST_INI, TEST_COORDINATE);

    //    reckon thread
    thread listenreckon(getrecon, ref(lcmer));
    thread listenparkinglot(getparkinglot, ref(lcmer));


    //ofstream reckoncheck("../data/reckonresult.txt");

    thread tag_thread(tag_thread_func, map1, map2);
    thread park_thread(park_thread_func);

    //reckoncheck.close();

    m_tag.writeLandMarkPose(TEST_TXT);

    listenreckon.join();
    listenparkinglot.join();
    tag_thread.join();
    park_thread.join();

//    while(1){}
//    system("pause");
    return 0;
}
