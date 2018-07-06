//
// Created by huangyewei on 16/6/25.
//
#include <fstream>
#include <opencv/highgui.h>
//定位及画图函数
#include "tag.h"
//坐标转换等预处理函数
#include "SLAMHelper.h"

using namespace cv;
using namespace std;

//***********************************  online test  **********************************//


mutex mtx;

bool detect_park = 0, control_map = 0;
int posSequence = 0;

lcm::LCM lcmer = lcm::LCM("udpm://239.255.76.67:7667?ttl=1");


//lcm class
class Handler
{
public:
    ~Handler() {}

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const exlcm::reckon* msg)
    {
        x = msg->filtery;
        y = msg->filterx;
        v = msg->v;
        mtx.lock();
        dangle += msg->dSteerAngle;
        mtx.unlock();
        angle = pi_to_pi(msg->angle);
        filteredangle =pi_to_pi(msg->filterangle);
        filteredangle = angle;
        //filteredangle = -pi;
//        if(angle > pi )
//            angle = angle - 2*pi;
//        cout << "x = " << x << "; y = " << y << ";angle = "<<angle<<endl;
//        cout<<"v="<<v<<"; dangle = "<<dangle<<endl;
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
    float x, y, v, dangle=0, angle, filteredangle;
    vector <Point2f> polys;
};

Handler handlerObject;

void getrecon(lcm::LCM &lcm)
{
    while(0 == lcm.handle());
}

void getparkinglot(lcm::LCM &lcm)
{
    while(0 == lcm.handle());
}

void tag_thread_online(SLAMHelper *helper){

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

    float time_cost;
    Mat  cvImg;
    camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
    while (!ptrGrabResult->GrabSucceeded()){

    }
    formatConverter.Convert(polynImg, ptrGrabResult);
    cvImg = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)polynImg.GetBuffer());


    while(true){
        struct timeval tpstart, tpend;
        float timeuse;
        gettimeofday(&tpstart, NULL);

        camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
        if (!ptrGrabResult->GrabSucceeded())
            continue;
        formatConverter.Convert(polynImg, ptrGrabResult);
        cvImg.data = (uchar*)(polynImg.GetBuffer());
        //= Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)
        cout<<"Tag mode:"<<endl;
        Point2f vehiclePos, KFPos, IMUPos;

        float x_now = handlerObject.x;
        float y_now = handlerObject.y;
        float v_now = handlerObject.v;
        float dangle = handlerObject.dangle;
        float filteredangle= handlerObject.filteredangle;
        mtx.lock();
        handlerObject.dangle = 0;
        mtx.unlock();
        float angle = handlerObject.angle;

        float dx = x_now - helper->x_pre;
        float dy = y_now - helper->y_pre;
        cout <<"dx:"<<dx<<endl;
        cout <<"dy:"<<dy<<endl;
        float angle_out=angle;
        if(angle_out<0)
            angle_out+=2*pi;
        cout<<"dangle:"<<dangle<<endl;
        cout<<"angle:"<<angle_out<<endl;

        mtx.lock();
        helper->x_pre = x_now;helper->y_pre = y_now;
        mtx.unlock();

        if(abs(dx) >1 || abs(dy)>1)
            continue;
        Mat input, ground;
        cvtColor(cvImg, input, CV_RGB2GRAY);
        remap(input, input, helper->map1, helper->map2, INTER_LINEAR, BORDER_CONSTANT);

        vector<taginfo> lm_Info;

        helper->tag_dealer->AprilTagDetect(input, filteredangle, 20, lm_Info);

        mtx.lock();
        helper->tag_dealer->getCarPose(vehiclePos, KFPos, v_now, dangle,angle, lm_Info, time_cost);
        mtx.unlock();

        //CHK
        if( angle < 0 )
        {
            angle += 2*pi;
        }

        angle = pi/2 - angle;

        if( angle <0 )
        {
            angle += 2*pi;
        }

        //此处增加发送位置部分
        PacketTagSlam lcm_pos;
        lcm_pos.x = KFPos.x;
        lcm_pos.y = KFPos.y;
        lcm_pos.angle = angle;

        cout<<"tag::---------------------------angle:"<<angle<<endl;

        lcmer.publish("TagSlamPos", &lcm_pos);

        
        char key = waitKey(1);
        if (key == 27) {
            control_map = 1;
            posSequence = 0;
        }
        //added by John for adding restricted zone
        if (key =='r')
        {
            /* read current pos and make the neighbourhood area as IMU forbidden zone */
            helper->tag_dealer->ptIMUrestrict.push_back(KFPos);
            helper->tag_dealer->fIMUrestrictRadius = 8;
            cout<<"ACUTION: IMU denied areas, number added "<< helper->tag_dealer->ptIMUrestrict.size()<<endl;
        }
        if (key =='q')
        {
            /* reset the center of neighbourhood area as IMU forbidden zone */
            helper->tag_dealer->ptIMUrestrict.clear();
            cout<<"ACUTION: IMU denied areas, reset"<< helper->tag_dealer->ptIMUrestrict.size()<<endl;
        }

        //added by John for checking and undo IMU restriction
        int idx = -1;
        for (int i = 0; i < helper->tag_dealer->ptIMUrestrict.size(); ++i)
        {
            if (norm((KFPos - helper->tag_dealer->ptIMUrestrict[i])) < helper->tag_dealer->fIMUrestrictRadius)
            {
                /* enter the IMU restricted zone */
                helper->tag_dealer->bInIMURestriction = true;
                cout<<"ACUTION: IMU denied area, centered at "<< helper->tag_dealer->ptIMUrestrict[i].x <<" " << helper->tag_dealer->ptIMUrestrict[i].y<<endl;
                idx = i;
                break;
            }
           
            helper->tag_dealer->bInIMURestriction = false;
        }
       
        /* undo current IMU forbidden zone */
        if (key =='u' && idx > -1)
        {
            
            //if (helper->tag_dealer->ptIMUrestrict.size() > idx + 1)
            {
                helper->tag_dealer->ptIMUrestrict.erase(helper->tag_dealer->ptIMUrestrict.begin() + idx);
                cout<<"ACUTION: IMU denied areas, number decreased "<< helper->tag_dealer->ptIMUrestrict.size()<<endl;
            }
        }

        if(control_map){

            ofstream mapcreatfile("tagmapfile.txt",ios::app);
            mapcreatfile << posSequence ++ << " "<< lcm_pos.x << " " <<lcm_pos.y << " " <<angle <<endl;
            std::cout<<"posSequence"<<posSequence<<std::endl;
            mapcreatfile.close();
        }

        helper->tag_dealer->getCarPoseWithIMU(IMUPos, Point2f(dx, dy));
        ground = Mat::zeros(500, 800, 16);
        helper->tag_dealer->drawLandMark(ground);
        helper->tag_dealer->drawCar(KFPos, ground, 247, 82, 165);
        helper->tag_dealer->drawCar(vehiclePos, ground, 176, 201, 90);
        helper->tag_dealer->drawCar(IMUPos, ground, 49, 181, 214);
        imshow("map",ground);
        waitKey(1);
        helper->tag_count++;

        if(helper->save_online){
            stringstream ss;
            ss<<helper->save_data_dir<<"/"<<helper->tag_count<<"jpg";
            //imwrite(ss.str(), cvImg);
            ofstream reckon(helper->save_data_dir+"/TagReckon.txt",ios::app);
            gettimeofday(&tpend,NULL);
            reckon<<tpend.tv_sec<<" "<<x_now<<" "<<y_now<<" "<<angle<<endl;
            reckon.close();
        }

        if(run_type == "MAP")
            helper->save();

        usleep(1);
        gettimeofday(&tpend,NULL);
        timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
        time_cost = timeuse;
        cout << "Time cost:" << timeuse << endl;
        cout << "fused angle:" <<  angle << endl;


    }
}

void park_thread_online(SLAMHelper *helper){
    float time_cost;
    while (true){
        if(detect_park != 1){
            usleep(50*1000);
            continue;
        }

        mtx.lock();
        detect_park = 0;
        mtx.unlock();


        struct timeval tpstart, tpend;
        float timeuse;
        gettimeofday(&tpstart, NULL);
        cout<<"Park mode:"<<endl;
        Point2f vehiclePos, KFPos, IMUPos;

        float x_now = handlerObject.x;
        float y_now = handlerObject.y;
        float v_now = handlerObject.v;
        float dangle = handlerObject.dangle;
        mtx.lock();
        handlerObject.dangle = 0;
        mtx.unlock();
        float angle = handlerObject.angle;

        float dx = x_now - helper->x_pre;
        float dy = y_now - helper->y_pre;
        cout <<"dx:"<<dx<<endl;
        cout <<"dy:"<<dy<<endl;
        float angle_out=angle;
        if(angle_out<0)
            angle_out+=2*pi;
        cout<<"dangle:"<<dangle<<endl;
        cout<<"angle:"<<angle_out<<endl;

        mtx.unlock();
        helper->x_pre = x_now;helper->y_pre = y_now;
        mtx.unlock();

        if(abs(dx) >1 || abs(dy)>1)
            continue;

        vector <taginfo> Info;
        vector <Point2f> mLot, polys;
        Mat ground;

        polys = handlerObject.polys;
        int lot_count = helper->tag_dealer->LotDetect(handlerObject.polys, Point2f(dx,dy), angle, Info, mLot);

        //此处增加lcmer发送mLot部分
        //此处增加发送位置部分
        PacketLot Lot_lcm;
        Lot_lcm.p0.x = mLot[0].x;
        Lot_lcm.p0.y = mLot[0].y;
        Lot_lcm.p1.x = mLot[1].x;
        Lot_lcm.p1.y = mLot[1].y;
        Lot_lcm.p2.x = mLot[2].x;
        Lot_lcm.p2.y = mLot[2].y;
        Lot_lcm.p3.x = mLot[3].x;
        Lot_lcm.p3.y = mLot[3].y;

        lcmer.publish("SLAMLOT", &Lot_lcm);

        mtx.lock();
        helper->tag_dealer->getCarPose(vehiclePos, KFPos, v_now, dangle,angle, Info, time_cost);
        mtx.unlock();

        PacketTagSlam lcm_pos;
        lcm_pos.x = KFPos.x;
        lcm_pos.y = KFPos.y;
        lcm_pos.angle = angle_out;

        cout<<"park::---------------------------angle:"<<angle_out<<endl;

        lcmer.publish("TagSlamPos", &lcm_pos);

        if(control_map){

            ofstream mapcreatfile("tagmapfile.txt",ios::app);
            mapcreatfile << posSequence ++ << " "<< lcm_pos.x << " " <<lcm_pos.y << " " <<angle <<endl;
            mapcreatfile.close();
        }

        helper->tag_dealer->getCarPoseWithIMU(IMUPos, Point2f(dx, dy));

        ground = Mat::zeros(500, 800, 16);
        helper->tag_dealer->drawLandMark(ground);
        helper->tag_dealer->drawCar(KFPos, ground, 247, 82, 165);
        helper->tag_dealer->drawCar(vehiclePos, ground, 176, 201, 90);
        helper->tag_dealer->drawCar(IMUPos, ground, 49, 181, 214);
        imshow("map",ground);

        if(helper->save_online){
            ofstream reckon(helper->save_data_dir+"ParkReckon.txt",ios::app);
            gettimeofday(&tpend,NULL);
            reckon<<tpend.tv_sec<<" "<<x_now<<" "<<y_now<<" "<<angle<<endl;
            reckon.close();
            ofstream detect(helper->save_data_dir+"ParkDetect.txt",ios::app);
            detect<<helper->tag_count<<"    "<<lot_count<<endl;
            for(int i=0; i<lot_count; i++){
                detect<<i+1<<endl;
                detect<<polys[i*4 + 0].x<<" "<<polys[i*4 + 0].y<<endl;
                detect<<polys[i*4 + 1].x<<" "<<polys[i*4 + 1].y<<endl;
                detect<<polys[i*4 + 2].x<<" "<<polys[i*4 + 2].y<<endl;
                detect<<polys[i*4 + 3].x<<" "<<polys[i*4 + 3].y<<endl;
            }
            detect.close();
        }

        gettimeofday(&tpend,NULL);
        timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
        time_cost = timeuse;
        cout << "Time cost:" << timeuse << endl;
        usleep(50*1000);
    }

}


//***********************************  online test  **********************************//



int main ()
{
    SLAMHelper m_SLAMHelper;
    //tag位置信息读入

    if(atoi( pd->getData("online").c_str() ) == 0)
    {
        ///***********************************  离线test  **********************************//

        string TagDataPath = pd->getData("tag_dataset");
        string TagReckonPath = pd->getData("tag_reckon");
        string ParkDetectPath = pd->getData("park_detect");
        string ParkReckonPath = pd->getData("park_reckon");

        ifstream TagReckonFile(TagReckonPath);
        ifstream ParkReckonFile(ParkReckonPath);
        ifstream ParkDetectFile(ParkDetectPath);
        if(!TagReckonFile.is_open() || !ParkReckonFile.is_open() || !ParkDetectFile.is_open()){
            cerr << "open local file failed!" << endl;
            return -1;
        }

        bool get;
        long time_tag, time_park;
        float dx_tag, dy_tag, angle_tag;
        float dx_park, dy_park, angle_park;
        string str_park, str_tag;

        str_park = "";
        str_tag = "";


        Point2f temp,vehiclePos, KFPos, IMUPos;
        int num, count, count_all = 0;

        vector <Point2f> park_4pts;

        //制造前视图像纠正模版

        vector<Point2f> mLot;

        while (true){
            count_all++;
            if(count_all == 2500)
                break;
            //读入txt
            if(str_park == ""){
                getline(ParkReckonFile, str_park);
                if(str_park == "")
                    break;
                m_SLAMHelper.splitString(str_park, time_park, dx_park, dy_park, angle_park);
            }

            if(str_tag == ""){
                getline(TagReckonFile, str_tag);
                if(str_park == "")
                    break;
                m_SLAMHelper.splitString(str_tag, time_tag, dx_tag, dy_tag, angle_tag);
            }

            //计时开始

            if(time_tag<time_park){
                m_SLAMHelper.tag_thread_offline(TagDataPath, dx_tag, dy_tag, angle_tag);
                str_tag = "";
            }
            else{
                ParkDetectFile>>num>>count;

                park_4pts.clear();
                for (int i=0; i<count; i++){
                    ParkDetectFile>>num;
                    for (int j=0; j<4; j++){
                        ParkDetectFile>>temp.x>>temp.y;
                        park_4pts.push_back(temp);
                    }//coordinate（x：左到右；y:下到上）
                }
                m_SLAMHelper.park_thread_offline(park_4pts, dx_park, dy_park, angle_park);
                str_park="";
            }
        }

        TagReckonFile.close();
        ParkReckonFile.close();
        ParkDetectFile.close();
        if(run_type == "MAP")
            m_SLAMHelper.save();
        ///***********************************  离线test  **********************************//
        ///
    }
    else
    {

        cout<<useOptimized()<<endl;
        if(!lcmer.good())
            return -1;
        lcmer.subscribe("reconPos", &Handler::handleMessage, &handlerObject);
        lcmer.subscribe("Lot", &Handler::handleParkinglotMessage, &handlerObject);

        ofstream mapcreatfile("tagmapfile.txt");
        mapcreatfile.close();

        thread listenreckon(getrecon, ref(lcmer));
        thread listenparkinglot(getparkinglot, ref(lcmer));

        thread tag_thread(tag_thread_online, &m_SLAMHelper);
        thread park_thread(park_thread_online, &m_SLAMHelper);


        listenreckon.join();
        listenparkinglot.join();
        tag_thread.join();
        park_thread.join();
//        interact_thread.join();
    }
}
