//
// Created by huangyewei on 16/9/6.
//

#include "kalman.h"

kalman::kalman(){
    slam_conf = new SLAM_Conf;
}

void kalman::setParam(string path, vector<taginfo> lmpt, string mood){
    long lm_rows, lm_cols;

    lm_rows = 2;
    lm_cols = lmpt.size();

    if(mood == "MAP" && lm_cols == 0)
        SLAM_Mood = 1;
    if(mood == "MAP" && lm_cols != 0)
        SLAM_Mood = 2;
    if(mood == "LOC")
        SLAM_Mood = 3;

    slam_conf->load(path);

    for(int i=0; i<lm_cols; i++){
        ftag.push_back(lmpt[i].id);
        da_table.push_back(-1);
    }

    //noises
    Q.resize(2,2);
    R.resize(2,2);
    Q << pow(slam_conf->sigmaV,2), 0,
            0 , pow(slam_conf->sigmaG,2);
    R << slam_conf->sigmaR*slam_conf->sigmaR, 0,
            0, slam_conf->sigmaB*slam_conf->sigmaB;
    sigmaPhi = slam_conf->sigmaT;

    //wp
    x.resize(3,1);
    P.resize(3,3);

    dt = slam_conf->DT_CONTROLS; //change in time btw predicts
}

//used after readmap, fix
void kalman::setParamMode(vector<taginfo> lmpt, string mood){
    long lm_rows, lm_cols;
    lm_cols = lmpt.size();

    if(mood == "MAP" && lm_cols == 0)
        SLAM_Mood = 1;
    if(mood == "MAP" && lm_cols != 0)
        SLAM_Mood = 2;
    if(mood == "LOC")
        SLAM_Mood = 3;
}

void kalman::initialize(Point2f pt, float direction_now){
    direction_now = pi_to_pi(pi/2 - direction_now);
//    if(direction_now>pi)
//        direction_now-=2*pi;
    if(P.cols() == 3) {
        P.setZero(3, 3);
    }
    x(0) = pt.x;
    x(1) = pt.y;
    x(2) = direction_now;

    direction_pre = direction_now;
}

 MatrixXf kalman::getP()
 {
    return P;
 }

 bool kalman::setKalman(MatrixXf p_, VectorXf x_, vector<int> ftag_, vector<int> da_)
 {
    P = p_;
    x = x_;
    ftag = ftag_;
    da_table = da_;
 }

bool kalman::KalmanUpdate(bool updatelm, float G_, float& direction_now, bool bHeadIgnore, float trans,
                            vector<taginfo> &TagInfo, Point2f &vehicle, float dt_) {
    float V;  // default velocity
    float G;      //initial steer angle

    vector<VectorXf> z;//range and bearings of visible landmarks
    vector<VectorXf> zf;
    vector<VectorXf> zn;

    vector<int> ftag_visible;
    vector<int> idf;

    direction_now = pi_to_pi(pi/2 - direction_now);
//    if(direction_now>pi)
//        direction_now-=2*pi;
    dt = dt_;
    cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<"dt:"<<dt<<endl;
    G = G_;
//    direction_pre = direction_now;
    //whether to ignore heading obsveration
    if (!bHeadIgnore)
    {   cout<<"ekf_observe_heading ekf_observe_heading ekf_observe_heading"<<endl;
        ekf_observe_heading(x, P,
                        direction_now,
                        slam_conf->SWITCH_HEADING_KNOWN,
                        sigmaPhi);
    }
	
    //speed this time
    V = trans;
    //if(trans.x*cos(direction_now)+trans.y*sin(direction_now)<0)
    //    V=-V;


    // predict position & heading
    ekf_predict(x, P, V, G, Q, slam_conf->WHEELBASE, dt);

    if(!updatelm){
        vehicle.x = x(0);vehicle.y = x(1);
        direction_now = x(2);//bug fixed, otherwize heading might be drifting
        return 1;
    }

    for (int i=0; i<TagInfo.size(); i++){
        VectorXf zvec(2);
        zvec<<TagInfo[i].trans, - TagInfo[i].angle;
        int id_now = TagInfo[i].id;

        if(id_now == -1){
            if(!Nearest_Neighbor_Match(zvec, id_now))
                continue;
            if(id_now == -1){
                switch (SLAM_Mood){
                        case 1:
                            id_now = 200+ftag.size();
                        break;
                        case 2:
                            id_now = 300+ftag.size();
                        break;
                        default:
                            continue;
                    }
            }
            TagInfo[i].id = id_now;
        }

        vector<int>::iterator id_find = find(ftag.begin(), ftag.end(), id_now);

        //if a new landmark is not stored in the landmark list
        if (id_find == ftag.end()){
            ftag.push_back(id_now);
            da_table.push_back(-1);
        }

        for(int j=0; j<ftag.size(); j++){
            if(ftag[j] == id_now){
                ftag_visible.push_back(j);
                break;
            }
        }

        z.push_back(zvec);
    }

    //get association
    if(z.size()!=0){

        ekf_data_associate_known(x, z, ftag_visible,
                                 zf, idf, zn,
                                 da_table);
        //update
        ekf_update(x, P, zf, R, idf, slam_conf->SWITCH_BATCH_UPDATE);
        //add new landmark zn into the state
        ekf_augment(x, P, zn, R);

    }

    if(SLAM_Mood != 3) {

        for (int i = 0; i < TagInfo.size(); i++) {
            int _x_idx = find_idx(TagInfo[i].id);
            if(_x_idx == -1)
                continue;

            TagInfo[i].tag_pos.x = x(_x_idx * 2 + 3);
            TagInfo[i].tag_pos.y = x(_x_idx * 2 + 4);
        }
    }
    else{

        for (int i=0; i<TagInfo.size(); i++){
            int _x_idx = find_idx(TagInfo[i].id);
            if(_x_idx == -1)
                continue;

            x(_x_idx*2+3) = TagInfo[i].tag_pos.x;
            x(_x_idx*2+4) = TagInfo[i].tag_pos.y;
        }
    }
    vehicle.x = x(0);vehicle.y = x(1);
    direction_now = x(2);
    cout<<"final final final final final final final final final final final final final final "<<x(2)<<endl;
    return 1;

}

kalman::~kalman(){
    delete slam_conf;

}

bool kalman::Nearest_Neighbor_Match(VectorXf z, int &id){
    float nis, nd;
    float gate1 = slam_conf->GATE_REJECT, gate2 = slam_conf->GATE_AUGMENT;
    float nbest = 1e60, outer = 1e60, jbest = -1;
    long Nxv = 3;
    long Nf = (x.size() - Nxv)/2;
    for (int j=0; j<Nf; j++) {
		if(ftag[j]<100)//前100位编号预留给tag
            continue;

        ekf_compute_association(x, P, z, R, j,
                                nis, nd);

        if (nis < gate1 && nd < nbest) {//若landmark的观测和经验值差距不太大则赋值
            nbest = nd;
            jbest = j;
        } else if (nis < outer) {//否则认为其是外点
            outer = nis;
        }
    }
    if(outer<gate2){
        id = -1;
        return 0;
    }
    if( jbest > -1 )
        id = ftag[jbest];
    else
        id =-1;
    return 1;

}

int kalman::find_idx(int id_ftag){
    if(id_ftag == -1)
        return -1;
    for (int j=0; j<ftag.size(); j++){
        if (ftag[j] == id_ftag)
            return da_table[j];
    }
    return -1;
}