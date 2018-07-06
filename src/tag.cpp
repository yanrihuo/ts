//
// Created by huangyewei on 16/6/27.
//
#include"tag.h"
#include <Eigen/Dense>

tag::tag(float m_tagSize,float m_fx ,float m_fy ,float m_px ,float m_py)
{
    tagSize = m_tagSize;
    fx = m_fx;
    fy = m_fy;
    px = m_px;
    py = m_py;

    //apriltag
    tf = NULL;
    tf = tag36h11_create();
    tf->black_border = 1;

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;//original 1.0
    td->quad_sigma = 0.8;
    td->nthreads = 8;
    td->debug = 0;
    td->refine_edges = 1;
    td->refine_decode = 0;
    td->refine_pose = 0;

    td->qtp.max_line_fit_mse = 1.0;
    td->qtp.min_white_black_diff = 15;

    Kalman = new kalman;

 //
    ptIMUrestrict.clear();
    fIMUrestrictRadius = 0;//restricted area in meters
    bInIMURestriction = false;
}

tag::~tag()
{
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

void tag::readMap(string path_LM){
    //read the landmark file
    int type_amount, type_id, lm_amount, dimu_amount;
    int lm_id, lm_type;
    float lm_x, lm_y, lm_w, lm_h, lm_a;
    unsigned long pos, pos1, pos2;

    ifstream LMfile(path_LM.c_str());
    string str, check, type_name;

    getline(LMfile, str);
    pos = str.find("=");
    check = str.substr(0,pos);

    if(check!="type_amount")
    {
        cout<<"An error occurs in the Coordinate Path:"<<path_LM<<"!"<<endl;
        return;
    }

    type_amount = atoi(str.substr(pos+1, str.length()-pos-1).c_str());

    for (int i=0; i<type_amount; i++){
        getline(LMfile, str);
        pos1 = str.find(":");
        pos2 = str.find("=");

        type_name = str.substr(pos1+1, pos2-pos1-1);
        type_id = atoi(str.substr(pos2+1, str.length()-pos-1).c_str());

        type_map.insert(pair<int, string>(type_id, type_name));
    }

    getline(LMfile, str);
    pos = str.find("=");

    check = str.substr(0,pos);
    if(check!="landmark_amount")
    {
        cout<<"An error occurs in the Coordinate Path:"<<path_LM<<"!"<<endl;
        return;
     }


    lm_amount = atoi(str.substr(pos+1, str.length()-pos-1).c_str());

    //added by John for reseting kalman filer
    VectorXf x_;
    x_.resize(3,1);
    x_(0) = 0.0;
    x_(1) = 0.0;
    x_(2) = 0.0;
    vector<int> ftag_;
    vector<int> da_;

    //read landmarks
    lm_list.clear();
    for(int i=0; i<lm_amount; i++){
        LMfile>>lm_id>>lm_type>>lm_x>>lm_y;
        taginfo m_taginfo;
        m_taginfo.id = lm_id;
        m_taginfo.type = type_map[lm_type];
        m_taginfo.tag_pos.x = lm_x;
        m_taginfo.tag_pos.y = lm_y;
        if(type_map[lm_type] == "park"){
            LMfile>>lm_w>>lm_h>>lm_a;
            m_taginfo.park.width = lm_w;
            m_taginfo.park.height = lm_h;
            m_taginfo.park.direction = lm_a;
        }
        lm_list.push_back(m_taginfo);
        id_map.insert(pair<int, int>(lm_id, i));

        //added by John for reseting kalman filer
        //x
        int len = x_.size();
        x_.conservativeResize(len+2);
        x_(len)   = lm_x;
        x_(len+1) = lm_y;
        //ftag
        ftag_.push_back(lm_id);
        //da_table
        da_.push_back(i);
    }
    string str1;
    LMfile>>str1;
    //getline(LMfile, str);
    pos = str1.find("=");

    check = str1.substr(0,pos);
    if(check!="landmark_P")
    {
        cout<<"An error occurs in the Coordinate Path:"<<path_LM<<"!"<<endl;
        return;
    }

    //added by John for reading P
    MatrixXf readP;
    readP.resize(2*lm_amount + 3, 2*lm_amount + 3);
    for (int i = 0; i < (2*lm_amount + 3) * (2*lm_amount + 3); ++i)
    {
        float temp;
        LMfile>>temp;
        readP((int)(i/(2*lm_amount + 3)), i%(2*lm_amount + 3)) = temp;
    }
   
    //reset kalman
    Kalman->setKalman(readP, x_, ftag_, da_);

    //added by John for reading IMU denied areas
    LMfile>>str1;
    //getline(LMfile, str);
    pos = str1.find("=");

    check = str1.substr(0,pos);
    if(check!="IMUDeny")
    {
        cout<<"An error occurs in the Coordinate Path:"<<path_LM<<"!"<<endl;
    }
    else
    {
        LMfile>>fIMUrestrictRadius;
        dimu_amount = atoi(str1.substr(pos+1, str1.length()-pos-1).c_str());
        Point2f pt;
        for (int i = 0; i < dimu_amount; ++i)
        {
            LMfile>>pt.x>>pt.y;
            ptIMUrestrict.push_back(pt);
        }
    }
    //
    LMfile.close();
}

void tag::readParam(string mood, string path_INI, string path_LM)
{

    Kalman->setParam(path_INI, lm_list, mood);

    readMap(path_LM);

    if(mood == "MAP" && lm_list.size() == 0)
        SLAM_Mood = 1;
    if(mood == "MAP" && lm_list.size() != 0)
        SLAM_Mood = 2;
    if(mood == "LOC")
        SLAM_Mood = 3;
    //do it again for reset Slam_mood
    Kalman->setParamMode(lm_list, mood);
}


void tag::drawLandMark(Mat &img){
    float col_max, row_max, col_min, row_min;
        col_max = -1;
    row_max = -1;
    col_min = 10e6;
    row_min = 10e6;
    for (int i = 0; i < lm_list.size(); i++) {
        Point2f Pt = lm_list[i].tag_pos;
        if (Pt.x > col_max)
            col_max = Pt.x;
        else if (Pt.x < col_min)
            col_min = Pt.x;
        if (Pt.y > row_max)
            row_max = Pt.y;
        else if (Pt.y < row_min)
            row_min = Pt.y;
    }

    col_max = col_max + (col_max - col_min) * 0.1f;
    row_max = row_max + (row_max - row_min) * 0.2f;
    col_min = col_min - (col_max - col_min) * 0.1f;
    row_min = row_min - (row_max - row_min) * 0.1f;

//    col_min = -50;
//    row_min = -35;
//    col_max = 40;
//    row_max = 35;
    if(col_max-col_min<row_max-row_min)
        scale = img.cols*1.0 / (col_max-col_min);
    else
        scale = img.rows*1.0 / (row_max-row_min);

    origin.x = col_min;
    origin.y = row_min;
    edge.x = col_max;
    edge.y = row_max;

    for (int i = 0; i<lm_list.size(); i++){
        taginfo m_taginfo = lm_list[i];
        if(m_taginfo.type == "tag")
            drawTag(img, m_taginfo);
        else if(m_taginfo.type == "park")
            drawPark(img, m_taginfo, 'W');
    }
}

void tag::drawTag(Mat &img, taginfo lm){
    Point2f center;
    float length = scale * tagSize / 2;

    center.x = scale * (lm.tag_pos.x - origin.x);
    center.y = float(img.rows) - scale * (lm.tag_pos.y - origin.y);

    Point2f tl(center.x-length, center.y-length);
    Point2f br(center.x+length, center.y+length);
    rectangle(img, tl, br,CV_RGB(255, 255, 255));
    stringstream ss;
    ss << lm.id;
    putText(img, ss.str().c_str(), br, CV_FONT_HERSHEY_COMPLEX, length*0.2, CV_RGB(255, 255, 255));

}

void tag::drawPark(Mat &img, taginfo lm, char color){
    Point2f lm_c,lm_l_u,lm_l_d,lm_r_u,lm_r_d;
    lm_c.x = lm.tag_pos.x;
    lm_c.y = lm.tag_pos.y;

    float angle = -lm.park.direction;


    lm_l_u.x = 0-lm.park.width/2.0*cos(angle)-lm.park.height/2.0*sin(angle)+lm_c.x;
    lm_l_u.y = 0-lm.park.width/2.0*sin(angle)+lm.park.height/2.0*cos(angle)+lm_c.y;

    lm_l_d.x = 0-lm.park.width/2.0*cos(angle)+lm.park.height/2.0*sin(angle)+lm_c.x;
    lm_l_d.y = 0-lm.park.width/2.0*sin(angle)-lm.park.height/2.0*cos(angle)+lm_c.y;

    lm_r_u.x = lm.park.width/2.0*cos(angle)-lm.park.height/2.0*sin(angle)+lm_c.x;
    lm_r_u.y = lm.park.width/2.0*sin(angle)+lm.park.height/2.0*cos(angle)+lm_c.y;

    lm_r_d.x = lm.park.width/2.0*cos(angle)+lm.park.height/2.0*sin(angle)+lm_c.x;
    lm_r_d.y = lm.park.width/2.0*sin(angle)-lm.park.height/2.0*cos(angle)+lm_c.y;


    lm_c.x = scale*(lm_c.x-origin.x);
    lm_c.y = img.rows*1.0-scale*(lm_c.y-origin.y);

    lm_l_u.x = scale*(lm_l_u.x-origin.x);
    lm_l_u.y = img.rows*1.0-scale*(lm_l_u.y-origin.y);

    lm_l_d.x = scale*(lm_l_d.x-origin.x);
    lm_l_d.y = img.rows*1.0-scale*(lm_l_d.y-origin.y);

    lm_r_u.x = scale*(lm_r_u.x-origin.x);
    lm_r_u.y = img.rows*1.0-scale*(lm_r_u.y-origin.y);

    lm_r_d.x = scale*(lm_r_d.x-origin.x);
    lm_r_d.y = img.rows*1.0-scale*(lm_r_d.y-origin.y);

    if(color == 'W'){
        line(img, lm_l_u, lm_l_d, CV_RGB(255, 255, 255));
        line(img, lm_r_u, lm_r_d, CV_RGB(255, 255, 255));
        line(img, lm_l_u, lm_r_u, CV_RGB(255, 255, 255));
        line(img, lm_l_d, lm_r_d, CV_RGB(255, 255, 255));
    }

    if(color == 'R'){
        line(img, lm_l_u, lm_l_d, CV_RGB(255, 0, 0));
        line(img, lm_r_u, lm_r_d, CV_RGB(255, 0, 0));
        line(img, lm_l_u, lm_r_u, CV_RGB(255, 0, 0));
        line(img, lm_l_d, lm_r_d, CV_RGB(255, 0, 0));
    }
    stringstream ss;
    ss << lm.id;
    putText(img, ss.str().c_str(), lm_c, CV_FONT_HERSHEY_SIMPLEX, 0.01*scale, CV_RGB(255, 255, 255));
}

void tag::drawCar(Point2f CarPos, Mat&img, int r, int g, int b)
{
    Point2f pt(scale * (CarPos.x - origin.x), img.rows - scale * (CarPos.y - origin.y));
    circle(img, pt, 1, CV_RGB(r,g,b), 2);
}

void tag::circleTagOnImage(Mat &img, apriltag_detection_t det){
    //draw tag
    line(img, Point2f(det.p[0][0], det.p[0][1]),
         Point2f(det.p[1][0], det.p[1][1]),
         Scalar(0, 0xff, 0), 2);
    line(img, Point2f(det.p[0][0], det.p[0][1]),
         Point2f(det.p[3][0], det.p[3][1]),
         Scalar(0, 0, 0xff), 2);
    line(img, Point2f(det.p[1][0], det.p[1][1]),
         Point2f(det.p[2][0], det.p[2][1]),
         Scalar(0xff, 0, 0), 2);
    line(img, Point2f(det.p[2][0], det.p[2][1]),
         Point2f(det.p[3][0], det.p[3][1]),
         Scalar(0xff, 0, 0), 2);

    stringstream ss;
    ss << det.id;
    string text = ss.str();
    int baseline, fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
    float fontscale = 1.0;
    Size textsize = getTextSize(text, fontface, fontscale, 2,&baseline);
    putText(img, text, Point2f(det.c[0] - textsize.width / 2,
                             det.c[1] + textsize.height / 2),
            fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
}

void tag::circleParkOnImage(Mat &birdview, float x[], float y[]){
    Scalar parking_space_color = Scalar(255, 0, 0);
    int lineWidth = 2;
    line(birdview, Point2f(x[0], y[0]), Point2f(x[1], y[1]), parking_space_color, lineWidth);
    line(birdview, Point2f(x[1], y[1]), Point2f(x[2], y[2]), parking_space_color, lineWidth);
    line(birdview, Point2f(x[2], y[2]), Point2f(x[3], y[3]), parking_space_color, lineWidth);
    line(birdview, Point2f(x[3], y[3]), Point2f(x[0], y[0]), parking_space_color, lineWidth);
}

bool tag::getCarPoseWithIMU(Point2f &IMUPos, Point2f _v_trans){
    if(!start)
        return 0;
    else{
        IMUPos.x = pre_IMU_pt.x + _v_trans.x;
        IMUPos.y = pre_IMU_pt.y + _v_trans.y;
    }
    pre_IMU_pt = IMUPos;
    return 1;
}

int tag::AprilTagDetect(Mat &img, float v_direction, float max_dist, vector<taginfo>& TagInfo ){
    Mat out = img;
    image_u8_t im = { img.cols, img.rows, img.cols, img.data };
    zarray_t *detections = apriltag_detector_detect(td, &im);

    Point2f lm_pos;
    float m_dist;
    for(int i=0; i<zarray_size(detections); i++){

        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        taginfo m_taginfo(tagSize, fx, fy, px, py);
        map<int ,int >::iterator id_find = id_map.find(det->id);
        //if the tag haven't been detected yet, add the tag to the map



        if(id_find == id_map.end()){
            if(SLAM_Mood == 1)
                lm_pos = Point2f(0,0);
            else if(SLAM_Mood == 2 && start)
                lm_pos = Point2f(0,0);
            else
                continue;
        }
        else
            lm_pos = lm_list[id_map[det->id]].tag_pos;

        m_taginfo.getCoordinateOneTag(lm_pos, det, v_direction);
        m_dist = m_taginfo.trans;
        if(m_dist > max_dist){
            continue;
        }

        if(id_find == id_map.end() && SLAM_Mood != 3){
            id_map.insert(pair<int, int>(det->id, lm_list.size()));
            lm_list.push_back(m_taginfo);
        }
        circleTagOnImage(out, *det);
        TagInfo.push_back(m_taginfo);
    }
    pyrDown(out, out, Size(out.cols/2, out.rows/2));
    imshow("TagDetect",out);
    zarray_destroy(detections);

    return TagInfo.size();
}

void tag::sort(float x[], float y[]){
    Point2f lt, lb, rt, rb;
    float c=(x[0]+x[1]+x[2]+x[3])/4;
    float y1=0,y2=0;

    for (int i=0; i<4; i++){
        if(x[i]<c){
            y1+=y[i]/2.0;
        }
        else{
            y2+=y[i]/2.0;
        }
    }
    for (int i=0; i<4; i++){
        if(x[i]<c){
            if(y[i]<y1)
                lt = Point2f(x[i], y[i]);
            else
                lb = Point2f(x[i], y[i]);
        }
        else{
            if(y[i]<y2)
                rt = Point2f(x[i], y[i]);
            else
                rb = Point2f(x[i], y[i]);
        }
    }
    if (c>150) {
        x[0] = lt.x;y[0] = lt.y;
        x[1] = lb.x;y[1] = lb.y;
        x[2] = rb.x;y[2] = rb.y;
        x[3] = rt.x;y[3] = rt.y;
    } else{
        x[0] = rt.x;y[0] = rt.y;
        x[1] = rb.x;y[1] = rb.y;
        x[2] = lb.x;y[2] = lb.y;
        x[3] = lt.x;y[3] = lt.y;
    }
}

bool tag::img_2_meter(vector<Point2f> pts, float angle, taginfo &info){
    float x[4], y[4];
    float center_x = 0, center_y = 0;
    float p_width, p_height, park_angle;
    float tag_car_dist, tag_car_angle;

    for (int j=0; j<4; j++){
        x[j] = pts[j].x / 31.25 - 4.8 + 0.45;//0.1575;
        y[j] = -pts[j].y / 31.25 + 4.8 + 0.121;//1.1100;
        center_x+=x[j];center_y+=y[j];
    }
    sort(x, y);
    center_x/=4.0;
    center_y/=4.0;

    p_width =(sqrt(pow(x[0]-x[3],2)+pow(y[0]-y[3],2))+sqrt(pow(x[1]-x[2],2)+pow(y[1]-y[2],2)))/2;
    p_height=(sqrt(pow(x[0]-x[1],2)+pow(y[0]-y[1],2))+sqrt(pow(x[3]-x[2],2)+pow(y[3]-y[2],2)))/2;

    if (p_width>5 || p_width<3.5)
        return 0;

    park_angle = acos((y[0]-y[1])/sqrt(pow(x[0]-x[1],2)+pow(y[0]-y[1],2)));
    if((center_x<0 && x[0]>x[1]) || (center_x>0 && x[0]<x[1]))
        park_angle = pi - park_angle;
    if(x[0]<0)
        park_angle = - park_angle;
    park_angle += angle;

    if(park_angle>pi)
        park_angle = park_angle - 2*pi;
    if(park_angle<-pi)
        park_angle = park_angle + 2*pi;

    tag_car_dist = sqrt(center_x * center_x + center_y * center_y);
    tag_car_angle = asin(center_x/tag_car_dist);
    if(center_y<0)
        tag_car_angle = pi-tag_car_angle;

    if(tag_car_angle>pi)
        tag_car_angle = tag_car_angle - 2*pi;

    info.id = -1;
    info.type = "park";

    info.trans = tag_car_dist;
    info.angle = tag_car_angle;

    info.park.width = abs(p_width);
    info.park.height = abs(p_height);
    info.park.direction = park_angle;
    return 1;
}

int tag::LotDetect(vector<Point2f> Park_4pt, Point2f _v_trans, float v_angle,
                   vector<taginfo>& ParkInfo, vector<Point2f> &mLot){

    mLot.resize(4);
    if(!start)
        return false;
    long len = Park_4pt.size()/4;
    float tag_car_angle, tag_car_dist;
    Point2f estimate_pt;
    estimate_pt.x = pre_optimize_pt.x + _v_trans.x;
    estimate_pt.y = pre_optimize_pt.y + _v_trans.y;

    vector<Point2f> Park_one;
    taginfo m_parkinfo;
    Park_one.resize(4);


    for(int i=0; i<len; i++){
        Park_one[0] = Park_4pt[i*4+0];
        Park_one[1] = Park_4pt[i*4+1];
        Park_one[2] = Park_4pt[i*4+2];
        Park_one[3] = Park_4pt[i*4+3];

        if(!img_2_meter(Park_one, v_angle, m_parkinfo))
            return false;

        tag_car_angle = m_parkinfo.angle + v_angle;
        tag_car_dist = m_parkinfo.trans;

        m_parkinfo.tag_pos.x = estimate_pt.x+sin(tag_car_angle)*tag_car_dist;
        m_parkinfo.tag_pos.y = estimate_pt.y+cos(tag_car_angle)*tag_car_dist;

        searchLandMark(m_parkinfo);
        cout<<"park id:"<<m_parkinfo.id<<endl;

        m_parkinfo.car_pos.x = m_parkinfo.tag_pos.x - sin(tag_car_angle)*tag_car_dist;
        m_parkinfo.car_pos.y = m_parkinfo.tag_pos.y - cos(tag_car_angle)*tag_car_dist;

        ParkInfo.push_back(m_parkinfo);

        double angle_mLot = -m_parkinfo.park.direction;

        mLot[0].x = 0-m_parkinfo.park.width/2*cos(angle_mLot)-m_parkinfo.park.height/2*sin(angle_mLot)+m_parkinfo.tag_pos.x;
        mLot[0].y = 0-m_parkinfo.park.width/2*sin(angle_mLot)+m_parkinfo.park.height/2*cos(angle_mLot)+m_parkinfo.tag_pos.y;

        mLot[1].x = 0-m_parkinfo.park.width/2*cos(angle_mLot)+m_parkinfo.park.height/2*sin(angle_mLot)+m_parkinfo.tag_pos.x;
        mLot[1].y = 0-m_parkinfo.park.width/2*sin(angle_mLot)-m_parkinfo.park.height/2*cos(angle_mLot)+m_parkinfo.tag_pos.y;

        mLot[2].x = m_parkinfo.park.width/2*cos(angle_mLot)-m_parkinfo.park.height/2*sin(angle_mLot)+m_parkinfo.tag_pos.x;
        mLot[2].y = m_parkinfo.park.width/2*sin(angle_mLot)+m_parkinfo.park.height/2*cos(angle_mLot)+m_parkinfo.tag_pos.y;

        mLot[3].x = m_parkinfo.park.width/2*cos(angle_mLot)+m_parkinfo.park.height/2*sin(angle_mLot)+m_parkinfo.tag_pos.x;
        mLot[3].y = m_parkinfo.park.width/2*sin(angle_mLot)-m_parkinfo.park.height/2*cos(angle_mLot)+m_parkinfo.tag_pos.y;

    }
    return ParkInfo.size();
}

bool tag::getCarPose(Point2f &vehiclePt, Point2f &optimizePt,
                     float v_, float G_, float& _v_direction, vector<taginfo> Info, float cost){
    //x,y,w,h,a
    bool updatelm;

    int count = 0;
    Point2f b_pos;
    vector<Point2f> _v_lm_trans, _v_lm_pos;
    b_pos.x = 0;b_pos.y = 0;
    long len = Info.size();

    //calculate the observation car pose
    for (int i=0; i<len; i++){
        taginfo t_info = Info[i];
        if(t_info.id == -1)
            continue;
        b_pos = b_pos + t_info.car_pos;
        count++;
    }
    if(count!=0){
        vehiclePt.x = b_pos.x/count;
        vehiclePt.y = b_pos.y/count;
    }

    //Kalman
    if(len==0){
        if(!start)
            return 0;
        else
            updatelm = 0;
    }
    else if(!start){
        if(Info[0].type!="tag")
            return 0;
        pre_IMU_pt = vehiclePt;

    }
//    else if(SLAM_Mood == 3)//若不相信库位位置，这句可以注�?//        updatelm = 0;//
    else{
        updatelm = 1;
    }

    optimizePt = vehiclePt;
    if(!start){
        Kalman->initialize(optimizePt, _v_direction);
        start = 1;
    }
    else
        Kalman->KalmanUpdate(updatelm, G_, _v_direction, bInIMURestriction, v_,  Info,
                         optimizePt, cost);

    if(updatelm){
        for(int i=0; i<len; i++){
            int id_now = Info[i].id;
            if(id_now == -1)
                continue;

            map<int, int>::iterator id_find = id_map.find(id_now);
            if(id_find != id_map.end()){
                lm_list[id_find->second].tag_pos = Info[i].tag_pos;
                continue;
            }

            id_map.insert(pair<int, int>(id_now, lm_list.size()));
            lm_list.push_back(Info[i]);
        }
    }
    pre_optimize_pt = optimizePt;
    cout<<"car pose:"<<optimizePt<<endl;
    if(vehiclePt.x == 0 && vehiclePt.y == 0)
        return 0;
    else
        return 1;

}

int tag::searchLandMark(taginfo &info) {
    int id_now = info.id, id_return = -1;
    float angle = info.angle;
    float trans = info.trans;
    if (id_now != -1) {
        map<int, int>::iterator id_find = id_map.find(id_now);
        if (id_find == id_map.end()) {
            id_map.insert(pair<int, int>(id_now, lm_list.size()));
            lm_list.push_back(info);
        }
        info = lm_list[id_map[id_now]];
        info.angle = angle;
        info.trans = trans;
        return id_map[id_now];
    }

    for (int i = 0; i < lm_list.size(); i++) {
        if (lm_list[i].compareTagPos(info)){
            info = lm_list[i];
            info.angle = angle;
            info.trans = trans;
            return i;
        }
    }
    return -1;
}

void tag::writeLandMarkPose(string path){
    int m_type;
    ofstream outfile(path);
    if (outfile)
    {
        outfile<<"type_amount=2"<<endl;
        outfile<<"type:tag=1"<<endl;
        outfile<<"type:park=2"<<endl;
        outfile<<"landmark_amount="<<lm_list.size()<<endl;
        for (int i = 0; i < lm_list.size(); i++){
            if(lm_list[i].type == "tag")
                m_type = 1;
            else if(lm_list[i].type == "park")
                m_type = 2;
            outfile << lm_list[i].id<<" "<< m_type <<" "<< lm_list[i].tag_pos.x <<" "<< lm_list[i].tag_pos.y<<endl;
            if(lm_list[i].type == "park")
                outfile << lm_list[i].park.width<<" "<<lm_list[i].park.height<<" "<<lm_list[i].park.direction<<endl;

        }

        //added by John for writing P
        outfile<<"landmark_P="<<endl;
        outfile<<Kalman->getP()<<endl;

        //added by John for writing IMU denied area
        outfile<<"IMUDeny="<<ptIMUrestrict.size()<<endl;
        outfile<<fIMUrestrictRadius<<endl;
        for (int i = 0; i < ptIMUrestrict.size(); ++i)
        {
            outfile<<ptIMUrestrict[i].x<< " "<< ptIMUrestrict[i].y<<endl;
        }
    }
    else{
        cout << "write file error" << endl;
    }
    outfile.close();
}


