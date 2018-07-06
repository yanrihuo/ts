
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <string>
#include <vector>
#include <algorithm>
#include <iterator>

#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "fastslam/fastslam_core.h"

using namespace std;
using namespace Eigen;

inline float sqr(float x)
{
    return x*x;
}

void add_control_noise(float V, float G, MatrixXf &Q, int addnoise, float *VnGn)
{
	if (addnoise ==1) {
		VectorXf A(2);
		A(0) = V;
		A(1) = G;
		VectorXf C(2);
		C = multivariate_gauss(A,Q,1);
		VnGn[0] = C(0);
		VnGn[1] = C(1);
	}
}

//prediction
void predict_true(VectorXf &xv, float V, float G, float WB, float dt)
{
    xv(0) = xv(0) + V*dt*cos(G+xv(2));//G是增量？
    xv(1) = xv(1) + V*dt*sin(G+xv(2));
    xv(2) = pi_to_pi(xv(2) + G);//V*dt*sin(G)/WB);
}

void compute_steering(VectorXf &x, MatrixXf &wp, int &iwp, float minD,
                      float &G, float rateG, float maxG, float dt)
{
    /*
        % INPUTS:
        %   x - true position ／／所有真实位置
        %   wp - waypoints 所有waypoint
        %   iwp - index to current waypoint 当前waypoint的index
        %   minD - minimum distance to current waypoint before switching to next   可接受的与真值误差范围
        %   rateG - max steering rate (rad/s) 最大航向变化速度？
        %   maxG - max steering angle (rad) 最大航向？
        %   dt - timestep 时间段
        % Output:
        %   G - current steering angle 当前航向
    */

    //determine if current waypoint reached
    Vector2d cwp;
    cwp[0] = wp(0, iwp);    //-1 since indexed from 0 取出当前需要优化的点
    cwp[1] = wp(1, iwp);

    float d2 = pow((cwp[0] - x[0]),2) + pow((cwp[1]-x[1]),2); //与真值的距离

    if (d2 < minD*minD) {
        iwp++; //switch to next 小于可接受范围则不进行后续步骤
        if (iwp >= wp.cols()) {
            iwp =-1;
            return;
        }

        cwp[0] = wp(0,iwp); //-1 since indexed from 0 重新取点
        cwp[1] = wp(1,iwp);
    }

    //compute change in G to point towards current waypoint 这是什么？
    float deltaG = atan2(cwp[1]-x[1], cwp[0]-x[0]) - x[2] - G;
    deltaG = pi_to_pi(deltaG);//将角归化到-pi到pi

    //limit rate
    float maxDelta = rateG*dt;
    if (abs(deltaG) > maxDelta) {
        int sign = (deltaG > 0) ? 1 : ((deltaG < 0) ? -1 : 0);
        deltaG = sign*maxDelta;
    }

    //limit angle
    G = G+deltaG;
    if (abs(G) > maxG) {
        int sign2 = (G > 0) ? 1: ((G < 0) ? -1 : 0);
        G = sign2*maxG;
    }
}







//z is range and bearing of visible landmarks
// find associations (zf) and new features (zn)
void data_associate_known(vector<VectorXf> &z, vector<int> &idz, VectorXf &table, int Nf,
                          vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn)
{
    idf.clear();
    vector<int> idn;

    unsigned long ii;

    for (unsigned long i =0; i< idz.size(); i++) {
        ii = idz[i];
        VectorXf z_i;
        if (table(ii) ==-1) { //new feature
            z_i = z[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        }
        else {
            z_i = z[i];
            zf.push_back(z_i);
            idf.push_back(table(ii));
        }
    }

    assert(idn.size() == zn.size());
    for (unsigned long  i=0; i<idn.size(); i++) {
        table(idn[i]) = Nf+i;
    }
}


//z is the list of measurements conditioned on the particle.
void feature_update(Particle &particle, vector<VectorXf> &z,
                    vector<int> &idf, MatrixXf &R)
{
    //Having selected a new pose from the proposal distribution,
    //  this pose is assumed perfect and each feature update maybe
    //  computed independently and without pose uncertainty
    vector<VectorXf> xf;    //updated EKF means
    vector<MatrixXf> Pf;    //updated EKF covariances

	for (unsigned long i=0; i<idf.size(); i++) {
		xf.push_back(particle.xf()[idf[i]]); //means
		Pf.push_back(particle.Pf()[idf[i]]); //covariances
	}	
	
	vector<VectorXf> zp;
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;
    
	compute_jacobians(particle,idf,R,zp,&Hv,&Hf,&Sf);

	vector<VectorXf> v; //difference btw two measurements (used to update mean)
	for (unsigned long i=0; i<z.size(); i++) {
		VectorXf measure_diff = z[i] - zp[i];
		measure_diff[1] = pi_to_pi(measure_diff[1]);
		v.push_back(measure_diff);
	}

    VectorXf vi; 
    MatrixXf Hfi;
    MatrixXf Pfi;
    VectorXf xfi; 

	for (unsigned long i=0; i<idf.size(); i++) {
		vi = v[i];
		Hfi = Hf[i];
		Pfi = Pf[i];
		xfi = xf[i];
        KF_cholesky_update(xfi,Pfi,vi,R,Hfi);
		xf[i] = xfi;
		Pf[i] = Pfi;
	}

	for (unsigned long i=0; i<idf.size(); i++) {
		particle.setXfi(idf[i],xf[i]);
		particle.setPfi(idf[i],Pf[i]);
	}
}

vector<VectorXf> get_observations(VectorXf &x, MatrixXf lm, vector<int> &idf, float rmax)
{
	get_visible_landmarks(x,lm,idf,rmax);
	return compute_range_bearing(x,lm);	
}

void get_visible_landmarks(VectorXf &x, MatrixXf &lm, vector<int> &idf, float rmax)
{
	//select set of landmarks that are visible within vehicle's 
	//semi-circular field of view
	vector<float> dx;
	vector<float> dy;

	for (int c=0; c<lm.cols(); c++) {
		dx.push_back(lm(0,c) - x(0));
		dy.push_back(lm(1,c) - x(1));
	}

	float phi = x(2);

	//distant points are eliminated
	vector<int> ii = find2(dx,dy,phi,rmax); 

	MatrixXf lm_new (lm.rows(), ii.size());
	unsigned j,k;
	for (j=0; j<lm.rows(); j++){
		for(k=0; k< ii.size(); k++){
			lm_new(j,k) = lm(j,ii[k]);
		}
	}
	lm = MatrixXf(lm_new); 

	vector<int> idf_backup(idf);
	idf.clear();
	for(unsigned long i=0; i<ii.size(); i++) {
		idf.push_back(idf_backup[ii[i]]);
	}
}

vector<VectorXf> compute_range_bearing(VectorXf &x, MatrixXf &lm)
{
	vector<float> dx; 
	vector<float> dy;

	for (int c=0; c<lm.cols(); c++) {
		dx.push_back(lm(0,c) - x(0));
		dy.push_back(lm(1,c) - x(1));
	}	

	assert(dx.size() == lm.cols());
	assert(dy.size() == lm.cols());

	float phi = x(2);
	vector<VectorXf> z;

	for (int i =0; i<lm.cols(); i++) {
		VectorXf zvec(2);
        zvec << sqrt(pow(dx[i],2) + pow(dy[i],2)), atan2(dy[i],dx[i]) - phi;
		z.push_back(zvec);
	}

	return z; 
}

vector<int> find2(vector<float> &dx, vector<float> &dy, float phi, float rmax)
{
	vector<int> index;
	//incremental tests for bounding semi-circle
	for (unsigned long j=0; j<dx.size(); j++) {
		if ((abs(dx[j]) < rmax) && (abs(dy[j]) < rmax)
				&& ((dx[j]* cos(phi) + dy[j]* sin(phi)) > 0.0)
				&& (((float)pow(dx[j],2) + (float)pow(dy[j],2)) < (float)pow(rmax,2))){
			index.push_back(j);			
		}
	}
	return index;				
}

void KF_cholesky_update(VectorXf &x, MatrixXf &P,VectorXf &v,MatrixXf &R,MatrixXf &H)
{
    MatrixXf PHt = P*H.transpose();
    MatrixXf S = H*PHt + R;

    // FIXME: why use conjugate()?
    S = (S+S.transpose()) * 0.5; //make symmetric
    MatrixXf SChol = S.llt().matrixU();
    //SChol.transpose();
    //SChol.conjugate();


    MatrixXf SCholInv = SChol.inverse(); //tri matrix
    MatrixXf W1 = PHt * SCholInv;

    MatrixXf W = W1 * SCholInv.transpose();

    x = x + W*v;
    P = P - W1*W1.transpose();
}


void KF_joseph_update(VectorXf &x, MatrixXf &P, float v, float R, MatrixXf &H)
{
    //x是x(k|k-1) P是经验的协方差 R是观测的协方差
    //H是观测的函数h(x(k|k-1)))的雅可比函数
    VectorXf PHt = P*H.transpose();
    MatrixXf S = H*PHt;
    MatrixXf _t = S;
    _t.setOnes();
    _t = _t*R;
    S = S + _t;
    //计算S(k)
    MatrixXf Si = S.inverse();
    //Si = make_symmetric(Si);
    make_symmetric(Si);
    MatrixXf PSD_check = Si.llt().matrixU(); //chol of scalar is sqrt
    //PSD_check是sqrt(Si)

    //PSD_check.transpose();
    //PSD_check.conjugate();

    //W是卡尔曼增益
    VectorXf W = PHt*Si;
    x = x+W*v;
    
    //Joseph-form covariance update
    MatrixXf eye(P.rows(), P.cols());
    eye.setIdentity();
    MatrixXf C = eye - W*H;
    P = C*P*C.transpose() + W*R*W.transpose();  
    //更新P(K|K)

    float eps = 2.2204*pow(10.0,-16); //numerical safety
    P = P+eye*eps;

    PSD_check = P.llt().matrixL();
    PSD_check.transpose();
    PSD_check.conjugate(); //for upper tri
}

MatrixXf make_symmetric(MatrixXf &P)
{
    return (P + P.transpose())*0.5;
}


MatrixXf line_plot_conversion(MatrixXf &lnes)
{
	int len = lnes.cols()*3 -1;			
	MatrixXf p(2,len);

	for (int j=0; j<len; j+=3) {
		int k = floor((j+1)/3); //reverse of len calculation
		p(0,j) = lnes(0,k);
		p(1,j) = lnes(1,k);
		p(0,j+1) = lnes(2,k);
		p(1,j+1) = lnes(3,k);
		if (j+2 <len) {
			p(0,j+2) = 0;
			p(1,j+2) = 0;
		}
	}
	return p;
}

//rb is measurements
//xv is robot pose
MatrixXf make_laser_lines(vector<VectorXf> &rb, VectorXf &xv)
{
    if ( rb.empty() ) {
        return MatrixXf(0,0);
    }

    unsigned long len = rb.size();
    MatrixXf lnes(4,len);

    MatrixXf globalMat(2,rb.size());
    int j;
    for (j=0; j<globalMat.cols();j++) {
        globalMat(0,j) = rb[j][0]*cos(rb[j][1]);
        globalMat(1,j) = rb[j][0]*sin(rb[j][1]);
    }

    TransformToGlobal(globalMat, xv);

    for (int c=0; c<lnes.cols();c++) {
        lnes(0,c) = xv(0);
        lnes(1,c) = xv(1);
        lnes(2,c) = globalMat(0,c);
        lnes(3,c) = globalMat(1,c);
    }

    //return line_plot_conversion(lnes);
    return lnes;
}


void make_covariance_ellipse(MatrixXf &x, MatrixXf &P, MatrixXf &lines)
{
    int         i, N;
    float       p_inc, p, cp, sp;
    float       s;
    MatrixXf    r;

    N       = 16;
    p_inc   = 2.0*M_PI/N;
    s       = 2.0;

    lines.setZero(2, N+1);

    r = P.sqrt();

    for(i=0; i<=N; i++) {
        p  = i*p_inc;
        cp = cos(p);
        sp = sin(p);

        lines(0, i) = x(0) + s*(r(0, 0)*cp + r(0, 1)*sp);
        lines(1, i) = x(1) + s*(r(1, 0)*cp + r(1, 1)*sp);
    }
}

//http://moby.ihme.washington.edu/bradbell/mat2cpp/randn.cpp.xml
MatrixXf nRandMat::randn(int m, int n)
{
    // use formula 30.3 of Statistical Distributions (3rd ed)
    // Merran Evans, Nicholas Hastings, and Brian Peacock
    int urows = m * n + 1;
    VectorXf u(urows);

    //u is a random matrix
#if 1
    for (int r=0; r<urows; r++) {
        // FIXME: better way?
        u(r) = std::rand() * 1.0/RAND_MAX;
    }
#else
    u = ( (VectorXf::Random(urows).array() + 1.0)/2.0 ).matrix();
#endif


    MatrixXf x(m,n);

    int     i, j, k;
    float   square, amp, angle;

    k = 0;
    for(i = 0; i < m; i++) {
        for(j = 0; j < n; j++) {
            if( k % 2 == 0 ) {
                square = - 2. * std::log( u(k) );
                if( square < 0. )
                    square = 0.;
                amp = sqrt(square);
                angle = 2. * M_PI * u(k+1);
                x(i, j) = amp * std::sin( angle );
            }
            else
                x(i, j) = amp * std::cos( angle );

            k++;
        }
    }

    return x;
}

MatrixXf nRandMat::rand(int m, int n)
{
    MatrixXf x(m,n);
    int i, j;
    float rand_max = float(RAND_MAX);

    for(i = 0; i < m; i++) {
        for(j = 0; j < n; j++)
            x(i, j) = float(std::rand()) / rand_max;
    }
    return x;
}

//add random measurement noise. We assume R is diagnoal matrix
void add_observation_noise(vector<VectorXf> &z, MatrixXf &R, int addnoise)
{
    if (addnoise == 1) {
        unsigned long len = z.size();
        if (len > 0) {
            MatrixXf randM1 = nRandMat::randn(1,len);
            MatrixXf randM2 = nRandMat::randn(1,len);

            for (unsigned long c=0; c<len; c++) {
                z[c][0] = z[c][0] + randM1(0,c)*sqrt(R(0,0));
                z[c][1] = z[c][1] + randM2(0,c)*sqrt(R(1,1));
            }
        }
    }
}


VectorXf multivariate_gauss(VectorXf &x, MatrixXf &P, int n)
{
    int len = x.size();

    //choleksy decomposition
    MatrixXf S = P.llt().matrixL();
    MatrixXf X = nRandMat::randn(len,n);

    return S*X + x;
}


void pi_to_pi(VectorXf &angle)
{
    int     i, n;

    for (i=0; i<angle.size(); i++) {
        if ((angle[i] < (-2*M_PI)) || (angle[i] > (2*M_PI))) {
            n = floor(angle[i]/(2*M_PI));
            angle[i] = angle[i]-n*(2*M_PI);
        }

        if (angle[i] > M_PI)
            angle[i] = angle[i] - (2*M_PI);

        if (angle[i] < -M_PI)
            angle[i] = angle[i] + (2*M_PI);
    }
}

float pi_to_pi(float ang) 
{
    int n;

    if ( (ang < -2*M_PI) || (ang > 2*M_PI) ) {
        n   = floor(ang/(2*M_PI));//floor：向下取整
        ang = ang - n*(2*M_PI);//将角度归化到-2pi到2pi
    }
    //将角度归化到-pi到pi
    if (ang > M_PI)
        ang = ang - (2*M_PI);

    if (ang < -M_PI)
        ang = ang + (2*M_PI);

    return ang;
}

float pi_to_pi2(float ang)
{
    if (ang > M_PI)
        ang = ang - (2*M_PI);

    if (ang < -M_PI)
        ang = ang + (2*M_PI);

    return ang;
}





//
// add new features
//
void add_feature(Particle &particle, vector<VectorXf> &z, MatrixXf &R)
{
    int lenz = z.size();
    vector<VectorXf> xf;
    vector<MatrixXf> Pf;
    VectorXf xv = particle.xv();

    float r,b,s,c;
    MatrixXf Gz(2,2);

    for (int i=0; i<lenz; i++) {
        r = z[i][0];
        b = z[i][1];
        s = sin(xv(2)+b);
        c = cos(xv(2)+b);

        VectorXf measurement(2);
        measurement(0) = xv(0) + r*c;
        measurement(1) = xv(1) + r*s;
        xf.push_back(measurement);
        Gz <<c,-r*s,s,r*c;

        Pf.push_back(Gz*R*Gz.transpose());
    }

    int lenx = particle.xf().size();

    for (int i=lenx; i<lenx+lenz; i++) {
        particle.setXfi(i, xf[(i-lenx)]);
        particle.setPfi(i, Pf[(i-lenx)]);
    }
}

void compute_jacobians(
        Particle &particle,
        vector<int> &idf,
        MatrixXf &R,
        vector<VectorXf> &zp,   // measurement (range, bearing)
        vector<MatrixXf> *Hv,   // jacobians of function h (deriv of h wrt pose)
        vector<MatrixXf> *Hf,   // jacobians of function h (deriv of h wrt mean)
        vector<MatrixXf> *Sf)   // measurement covariance
{
    VectorXf xv = particle.xv();

    vector<VectorXf> xf;
    vector<MatrixXf> Pf;

    unsigned int i;
    for (i=0; i<idf.size(); i++) {
        xf.push_back(particle.xf()[idf[i]]);
        Pf.push_back((particle.Pf())[idf[i]]); //particle.Pf is a array of matrices
    }

    float dx,dy,d2,d;
    MatrixXf HvMat(2,3);
    MatrixXf HfMat (2,2);

    for (i=0; i<idf.size(); i++) {
        dx = xf[i](0) - xv(0);
        dy = xf[i](1) - xv(1);
        d2 = pow(dx,2) + pow(dy, 2);
        d  = sqrt(d2);

        VectorXf zp_vec(2);

        //predicted observation
        zp_vec[0] = d;
        zp_vec[1] = pi_to_pi(atan2(dy,dx) - xv(2));
        zp.push_back(zp_vec);

        //Jacobian wrt vehicle states
        HvMat<< -dx/d, -dy/d,  0,
                dy/d2, -dx/d2,-1;

        //Jacobian wrt feature states
        HfMat<< dx/d,   dy/d,
                -dy/d2, dx/d2;

        Hv->push_back(HvMat);
        Hf->push_back(HfMat);

        //innovation covariance of feature observation given the vehicle'
        MatrixXf SfMat = HfMat*Pf[i]*HfMat.transpose() + R;
        Sf->push_back(SfMat);
    }
}


void resample_particles(vector<Particle> &particles, int Nmin, int doresample) 
{
    unsigned long   i;
    unsigned long   N = particles.size();
    VectorXf        w(N);

    for (i=0; i<N; i++) {
        w(i) = particles[i].w();
    }

    float ws = w.sum();
    for (i=0; i<N; i++) {
        particles[i].setW(w(i)/ws);
    }

    float       Neff=0;
    vector<int> keep;

    stratified_resample(w, keep, Neff);

    vector<Particle> old_particles = vector<Particle>(particles);
    particles.resize(keep.size());	

    if ((Neff < Nmin) && (doresample == 1)) {
        for(i=0; i< keep.size(); i++) {
            particles[i] = old_particles[keep[i]]; 	
        }

        for (i=0; i<N; i++) {
			float new_w = 1.0f/(float)N;
            particles[i].setW(new_w);
        }
    }
}

void stratified_random(unsigned long N, vector<float> &di)
{ 
    float k = 1.0/(float)N;
   
    //deterministic intervals
    float temp = k/2;
    while (temp < (1-k/2)) {
        di.push_back(temp);
        temp = temp+k;
    }
   
    // FIXME: when set NPARTICLES = 30, this whill failed
    assert(di.size() == N); 
    
    //dither within interval
    vector<float>::iterator diter; 
    for (diter = di.begin(); diter != di.end(); diter++) {
        *diter = (*diter) + unifRand() * k - (k/2);
    }
}

//
// Generate a random number between 0 and 1
// return a uniform number in [0,1].
double unifRand()
{
    return rand() / double(RAND_MAX);
}

// FIXME: input w will be modified?
void stratified_resample(VectorXf w, vector<int> &keep, float &Neff)
{
    VectorXf    wsqrd(w.size());
    double      w_sum = w.sum();

    for (int i=0; i<w.size(); i++) {
        // FIXME: matlab version is: w = w / sum(w)
        w(i) = w(i)/w_sum;
        wsqrd(i) = pow(w(i),2);
    }
    Neff = 1/wsqrd.sum();

    int len = w.size();
    keep.resize(len);
    for (int i=0; i<len; i++) {
        keep[i] = -1;
    }

    vector<float> select;
    stratified_random(len, select);
    cumsum(w);    

    int ctr=0;
    for (int i=0; i<len; i++) {
        while ((ctr<len) && (select[ctr]<w(i))) {
            keep[ctr] = i;
            ctr++;
        }
    }
}

//
//returns a cumulative sum array
//
void cumsum(VectorXf &w) 
{
    VectorXf csumVec(w.size());
    for (int i=0; i< w.size(); i++) {
        float sum =0;
        for (int j=0; j<=i; j++) {
            sum+=w(j);
        }			
        csumVec(i) = sum;
    }

    w = VectorXf(csumVec); //copy constructor. Double check
}


void TransformToGlobal(MatrixXf &p, VectorXf &b)
{
	//rotate
	MatrixXf rot(2,2);
    rot << cos(b(2)), -sin(b(2)), sin(b(2)), cos(b(2));
	
	MatrixXf p_resized;
	p_resized = MatrixXf(p);
	p_resized.conservativeResize(2,p_resized.cols());
	p_resized = rot*p_resized;		
	
	//translate
	int c;
	for (c=0;c<p_resized.cols();c++) {
		p(0,c) = p_resized(0,c)+b(0); 				
		p(1,c) = p_resized(1,c)+b(1); 				
	}

	float input;
	//if p is a pose and not a point
	if (p.rows() ==3){
		for (int k=0; k<p_resized.cols();k++) {
			input = p(2,k) +b(2);
   			p(2,k) = pi_to_pi(input);
		}		
	}	
}



void read_slam_input_file(const string s, MatrixXf *lm, MatrixXf *wp)
{
    if(access(s.c_str(), R_OK) == -1) {
        std::cerr << "Unable to read input file" << s << std::endl;
        exit(EXIT_FAILURE);
    }

    ifstream in(s.c_str());

    int lineno = 0;
    unsigned long lm_rows =0;
    int lm_cols =0;
    unsigned long  wp_rows =0;
    unsigned long  wp_cols =0;

    while(in) {
        lineno++;
        string str;
        getline(in,str);
        istringstream line(str);

        vector<string> tokens;
        copy(istream_iterator<string>(line),
             istream_iterator<string>(),
             back_inserter<vector<string> > (tokens));

        if(tokens.size() ==0) {
            continue;
        }
        else if (tokens[0][0] =='#') {
            continue;
        }
        else if (tokens[0] == "lm") {
            if(tokens.size() != 3) {
                std::cerr<<"Wrong args for lm!"<<std::endl;
                std::cerr<<"Error occuredon line"<<lineno<<std::endl;
                std::cerr<<"line:"<<str<<std::endl;
                exit(EXIT_FAILURE);
            }
            lm_rows = strtof(tokens[1].c_str(),NULL);
            lm_cols = strtof(tokens[2].c_str(),NULL);

            lm->resize(lm_rows,lm_cols);
            for (int c =0; c<lm_cols; c++) {
                lineno++;
                if (!in) {
                    std::cerr<<"EOF after reading" << std::endl;
                    exit(EXIT_FAILURE);
                }
                getline(in,str);
                istringstream line(str);
                vector<string> tokens;
                copy(istream_iterator<string>(line),
                     istream_iterator<string>(),
                     back_inserter<vector<string> > (tokens));
                if(tokens.size() < lm_rows) {
                    std::cerr<<"invalid line for lm coordinate!"<<std::endl;
                    std::cerr<<"Error occured on line "<<lineno<<std::endl;
                    std::cerr<<"line: "<<str<<std::endl;
                    exit(EXIT_FAILURE);
                }

                for (unsigned long r=0; r< lm_rows; r++) {
                    (*lm)(r,c) = strtof(tokens[r].c_str(),NULL);
                }
            }
        }
        else if (tokens[0] == "wp") {
            if(tokens.size() != 3) {
                std::cerr<<"Wrong args for wp!"<<std::endl;
                std::cerr<<"Error occured on line"<<lineno<<std::endl;
                std::cerr<<"line:"<<str<<std::endl;
                exit(EXIT_FAILURE);
            }
            wp_rows = strtof(tokens[1].c_str(),NULL);
            wp_cols = strtof(tokens[2].c_str(),NULL);
            wp->resize(wp_rows, wp_cols);
            for (unsigned long c =0; c<wp_cols; c++) {
                lineno++;
                if (!in) {
                    std::cerr<<"EOF after reading" << std::endl;
                    exit(EXIT_FAILURE);
                }
                getline(in,str);
                istringstream line(str);
                std::vector<string> tokens;
                copy(istream_iterator<string>(line),
                     istream_iterator<string>(),
                     back_inserter<std::vector<string> > (tokens));
                if(tokens.size() < wp_rows) {
                    std::cerr<<"invalid line for wp coordinate!"<<std::endl;
                    std::cerr<<"Error occured on line "<<lineno<<std::endl;
                    std::cerr<<"line: "<<str<<std::endl;
                    exit(EXIT_FAILURE);
                }

                for (unsigned long r=0; r< lm_rows; r++) {
                    (*wp)(r,c) = strtof(tokens[r].c_str(),NULL);
                }
            }
        }
        else {
            std::cerr<<"Unkwown command"<<tokens[0] <<std::endl;
            std::cerr<<"Error occured on line"<<lineno<<std::endl;
            std::cerr<<"line: "<<str<<std::endl;
            exit(EXIT_FAILURE);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
// particle class
////////////////////////////////////////////////////////////////////////////////
Particle::Particle() 
{
	_w = 1.0; 
	_xv = VectorXf(3);
    _xv.setZero(3);
    _Pv = MatrixXf(3, 3);
    _Pv.setZero(3, 3);
	_da = NULL;
}

Particle::Particle(float &w, VectorXf &xv, MatrixXf &Pv,
                   vector<VectorXf> &xf, vector<MatrixXf> &Pf, float* da)
{
    _w  = w;
	_xv = xv;
	_Pv = Pv;
	_xf = xf;
	_Pf = Pf;
	_da = da;		
}

Particle::~Particle()
{
}

//getters
float& Particle::w()
{
	return _w;	
}

VectorXf& Particle::xv()
{
	return _xv;	
}

MatrixXf& Particle::Pv()
{
	return _Pv;	
}

vector<VectorXf>& Particle::xf()
{
	return _xf;	
}

vector<MatrixXf>& Particle::Pf()
{
	return _Pf;	
}

float* Particle::da()
{
	return _da;	
}

//setters
void Particle::setW(float w)
{
	_w = w;
}

void Particle::setXv(VectorXf &xv)
{
	_xv = xv;
}

void Particle::setPv(MatrixXf &Pv)
{
	_Pv = Pv;
}

void Particle::setXf(vector<VectorXf> &xf)
{
	_xf = xf;
}

void Particle::setXfi(unsigned long i, VectorXf &vec) 
{
	if (i >= _xf.size()){
		_xf.resize(i+1);
	}
	_xf[i] = vec;
}

void Particle::setPf(vector<MatrixXf> &Pf)
{
	_Pf = Pf;
}

void Particle::setPfi(unsigned long i, MatrixXf &m) 
{
	if(i >= _Pf.size()) {
		_Pf.resize(i+1);
	}
	_Pf[i] = m;
}

void Particle::setDa(float* da)
{
	_da = da;
}

////////////////////////////////////////////////////////////////////////////////
// EKF-SLAM functions
////////////////////////////////////////////////////////////////////////////////

void ekf_predict(VectorXf &x, MatrixXf &P, float V, float G, MatrixXf &Q, float WB, float dt)
{
    //x(x_(x-1)),P(x_(x-1)的协方差),V(速度),G(航向变化),Q(噪声),WB(模式，=1，不知道是什么),dt(时间差))
    float       s, c;
    float       vts, vtc;
    MatrixXf    Gv(3, 3), Gu(3, 2);         // Jacobians

    s = sin(G+x(2));//航向是与y轴夹脚
    c = cos(G+x(2));
    vts = V*dt*s;//
    vtc = V*dt*c;//
    //Gv为f(xk-1|k-1,uk-1)
    Gv <<   1, 0, -vts,
            0, 1, vtc,
            0, 0, 1;
    //
    // Gu <<   dt*c, -vts,
    //         dt*s, vtc,
    //         0   , 1;
   Gu <<   dt*c, -vts,
           dt*s, vtc,
           0   , 1;

    // predict covariance
    //      P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
    //Gu是啥？
    //其中Gv为f(xk-1|k-1,uk-1),
    P.block(0, 0, 3, 3) = Gv*P.block(0, 0, 3, 3)*Gv.transpose() + Gu*Q*Gu.transpose();

    int m = P.rows();
    //为了不让其形状发生变化？
    if( m > 3 ) {
        P.block(0, 3, 3, m-3) = Gv * P.block(0, 3, 3, m-3);
        P.block(3, 0, m-3, 3) = P.block(0, 3, 3, m-3).transpose();
    }

    // predict state
    x(0) = x(0) + vtc;
    x(1) = x(1) + vts;
    x(2) = pi_to_pi(x(2) + G/*V*dt*sin(G)/WB*/);
}

//update
void ekf_observe_heading(VectorXf &x, MatrixXf &P,
                         float phi,
                         int useheading,
                         float sigmaPhi)
{
    //x是x(k|k-1) P是经验值的协方差 useheading是是否执行的标志
     //phi是惯性导航等其他传感器获得的数据 sigmaPhi是sqrt(R(k))
    if( useheading == 0 ) return;

    float       v;
    MatrixXf    H = MatrixXf(1, x.size());
    H.setZero(1, x.size());
    H(2) = 1;
    //H是观测的函数h(x(k|k-1)))的雅可比函数
    //float v1, v2, x2_2pi, phi_2pi;
    v = pi_to_pi(phi - x(2));
    //maybe a bug 0706
//    if(x(2)<0)
//        x2_2pi = x(2)+2*M_PI;
//
//    if(phi<0)
//        phi_2pi = phi+2*M_PI;
//
//    v2 = pi_to_pi(phi_2pi - x2_2pi);
//    v = min(v1, v2);
    //
    KF_joseph_update(x, P, v, sigmaPhi*sigmaPhi, H);
}

//landmark observation
void ekf_observe_model(VectorXf &x, int idf,
                       VectorXf &z, MatrixXf &H)
{
    //x是x(k|k) P是经验的协方差  idf是landmark在当前时刻
    //
    int     Nxv, fpos;
    float   dx, dy, d2, d, xd, yd, xd2, yd2;

    Nxv = 3;//车辆状态维数
    H.setZero(2, x.size());//与观测到的landmark矩阵大小一致
    z.setZero(2);

    // position of xf in state
    fpos = Nxv + idf*2;

    dx  = x(fpos)   - x(0);//landmark与车辆状态x方向距离
    dy  = x(fpos+1) - x(1);//landmark与车辆状态y方向距离
    d2  = sqr(dx) + sqr(dy);//landmark与车辆状态距离平方
    d   = sqrtf(d2);//landmark与车辆状态距离
    xd  = dx / d;//sin（航向）
    yd  = dy / d;//cos（航向）
    xd2 = dx / d2;//sin（航向）/d
    yd2 = dy / d2;//cos（航向）/d

    z(0) = d;//landmark与车辆距离
    z(1) = atan2(dy, dx) - x(2);//landmark与车辆连线与航向夹角
    //构建H矩阵
    H(0, 0) = -xd;  H(0, 1) = -yd;  H(0, 2) = 0;
    H(1, 0) = yd2;  H(1, 1) = -xd2; H(1, 2) = -1;

    H(0, fpos) = xd;        H(0, fpos+1) = yd;
    H(1, fpos) = -yd2;      H(1, fpos+1) = xd2;
}

void ekf_compute_association(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &R, int idf,
                             float &nis, float &nd)
{
    //x是x(k|k) P是经验的协方差
    //z观测到的当前landmark的坐标与当前位置的距离
    //R是观测噪声 idf是landmark在当前时刻
    //

    VectorXf        zp;
    MatrixXf        H;
    VectorXf        v(2);
    MatrixXf        S;

    ekf_observe_model(x, idf, zp, H);
    //zp是通过观测得到的landmark位置

    v = z - zp;//经验和观测的差值
    v(1) = pi_to_pi(v(1));

    S = H*P*H.transpose() + R;

    nis = v.transpose()*S.inverse()*v;//衡量观测和经验值的差距
    nd  = nis + log(S.determinant());//这是啥
}

void ekf_data_associate(VectorXf &x, MatrixXf &P, vector<VectorXf> &z, MatrixXf &R,
                        float gate1, float gate2,
                        vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn
                        , vector<int> &table, vector<int> &idz)
{
    //x(x(k|k)) P是经验的协方差 z观测到的tag的坐标与当前位置的距离 R是观测的噪声
    //gate1 gate2 是参数
    unsigned long Nxv, Nf;
    unsigned long i, j;
    long jbest;
    float   nis, nd;
    float   nbest, outer;

    zf.clear();
    zn.clear();
    idf.clear();

    Nxv = 3;                        // number of vehicle pose states 车辆状态数
    Nf  = (x.size() - Nxv)/2;       // number of features already in map 地图中landmark数（稍后推入）

    // linear search for nearest-neighbour, no clever tricks (like a quick
    // bounding-box threshold to remove distant features; or, better yet,
    // a balanced k-d tree lookup). TODO: implement clever tricks.
    for(i=0; i<z.size(); i++) {
        jbest = -1;
        nbest = 1e60;
        outer = 1e60;

        for(j=0; j<Nf; j++) {//若状态中有landmark，则搜索
            ekf_compute_association(x, P, z[i], R, j,
                                    nis, nd);

            if( nis < gate1 && nd < nbest ) {//若landmark的观测和经验值差距不太大则赋值
                nbest = nd;
                jbest = j;
            } else if ( nis < outer ) {//否则认为其是外点
                outer = nis;
            }
        }

        if( jbest > -1 ) {//如果比较了，则加入列表
            // add nearest-neighbour to association list
            zf.push_back(z[i]);//将观测推入观测列表
            idf.push_back(jbest);
        } else if ( outer > gate2 ) {//否则，加入新特征
            // z too far to associate, but far enough to be a new feature
            zn.push_back(z[i]);
            table[idz[i]] = Nf + i;
        }
    }
}


//z is range and bearing of visible landmarks
// find associations (zf) and new features (zn)
void ekf_data_associate_known(VectorXf &x, vector<VectorXf> &z, vector<int> &idz,
                          vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn, vector<int> &table)
{
    //x(x(k|k)) z观测到的tag的坐标与当前位置的距离 idz是观测到tag的id
    vector<int> idn;
    unsigned    i, ii;

    zf.clear();
    zn.clear();
    idf.clear();
    idn.clear();

    for (i=0; i<idz.size(); i++) {
        ii = idz[i];
        VectorXf z_i;

        if (table[ii] == -1) {
            // new feature若该landmark未出现过则加入zn与idn
            z_i = z[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        } else {
            // exist feature
            z_i = z[i];//若该landmark已出现过则加入zf与idf
            zf.push_back(z_i);
            idf.push_back(table[ii]);
        }
    }

    // add new feature IDs to lookup table
    int Nxv = 3;                        // number of vehicle pose states 车状态维数
    int Nf  = (x.size() - Nxv)/2;       // number of features already in map 地图中landmark数量

    // add new feature positions to lookup table
    //      table[idn]= Nf + (1:size(zn,2));
    for (i=0; i<idn.size(); i++) {
        table[idn[i]] = Nf+i;//在表中加入新观测到landmark首次出现的位置
    }
}



void ekf_batch_update(VectorXf &x, MatrixXf &P,
                      vector<VectorXf> &zf, MatrixXf &R,
                      vector<int> &idf)
{
    int         lenz, lenx;
    MatrixXf    H, RR;
    VectorXf    v;
    VectorXf    zp;
    MatrixXf    H_;

    int         i;

    lenz = zf.size();
    lenx = x.size();

    H.setZero(2*lenz, lenx);
    v.setZero(2*lenz);
    RR.setZero(2*lenz, 2*lenz);

    zp.setZero(2);


    for(i=0; i<lenz; i++) {
        //for each observation
        ekf_observe_model(x, idf[i], zp, H_);
        H.block(i*2, 0, 2, lenx) = H_;
        //before and after adding observation noise
        v(2*i)   = zf[i](0) - zp(0);
        v(2*i+1) = pi_to_pi(zf[i](1)-zp(1));

        RR.block(i*2, i*2, 2, 2) = R;
    }

    KF_cholesky_update(x, P, v, RR, H);
}

void ekf_update(VectorXf &x, MatrixXf &P, vector<VectorXf> &zf, MatrixXf &R,
                vector<int> &idf, int batch)
{
    if( batch == 1 )
        ekf_batch_update(x, P, zf, R, idf);
}



//add new landmark
void ekf_add_one_z(VectorXf &x, MatrixXf &P,
                 VectorXf &z, MatrixXf &Re)
{
    int     len;
    float   r, b;
    float   s, c;

    MatrixXf    Gv, Gz;

    len = x.size();

    //r=landmark x,b=landmark y,s、c=与航向连线得到的单位向量
    r = z(0); b = z(1);
    s = sin(x(2) + b);
    c = cos(x(2) + b);

    // augment x
    //      x = [x; x(1)+r*c; x(2)+r*s];
    x.conservativeResize(len+2);
    x(len)   = x(0) + r*c;//landmark observed based on the position
    x(len+1) = x(1) + r*s;//通过车辆状态位置计算landmark位置

    // Jacobians
    Gv.setZero(2, 3);
    Gz.setZero(2, 2);

    Gv <<   1, 0, -r*s,
            0, 1, r*c;
    Gz <<   c, -r*s,
            s, r*c;

    // augment P
    P.conservativeResize(len+2, len+2);

    // feature cov
    //      P(rng,rng)= Gv*P(1:3,1:3)*Gv' + Gz*R*Gz';

    P.block(len, len, 2, 2) =Gv*P.block(0,0,3,3)*Gv.transpose() +
                              Gz*Re*Gz.transpose();
    // vehicle to feature xcorr
    //      P(rng,1:3)= Gv*P(1:3,1:3);
    //      P(1:3,rng)= P(rng,1:3)';
    P.block(len, 0, 2, 3) = Gv*P.block(0,0,3,3);
    P.block(0, len, 3, 2) = P.block(len,0, 2, 3).transpose();


    if( len > 3 ) {
        // map to feature xcoor
        //   P(rng,rnm)= Gv*P(1:3,rnm);
        //若x中原来已有landmark，对原有landmark的协方差也进行更新
        P.block(len, 3, 2, len-3) = Gv*P.block(0,3,3,len-3);
        //   P(rnm,rng)= P(rng,rnm)';
        P.block(3, len, len-3, 2) = P.block(len, 3, 2, len-3).transpose();
    }
}

void ekf_augment(VectorXf &x, MatrixXf &P,
                 vector<VectorXf> &zn, MatrixXf &Re)
{
    for(unsigned long i=0; i<zn.size(); i++)
        ekf_add_one_z(x, P, zn[i], Re);
}

////////////////////////////////////////////////////////////////////////////////
// fastslam config Variables
//      Configuration file
//      Permits various adjustments to parameters of the FastSLAM algorithm.
//      See fastslam_sim.h for more information
////////////////////////////////////////////////////////////////////////////////


int SLAM_Conf::parse(void)
{
    ////////////////////////////////////////////////////////////////////////////
    /// set default values
    ////////////////////////////////////////////////////////////////////////////

    // control parameters
    V                   = 3.0;              // m/s
    MAXG                = 30*M_PI/180;      // radians, maximum steering angle (-MAXG < g < MAXG)
    RATEG               = 20*M_PI/180;      // rad/s, maximum rate of change in steer angle
    WHEELBASE           = 4;                // metres, vehicle wheel-base
    DT_CONTROLS         = 0.025;            // seconds, time interval between control signals

    // control noises
    sigmaV              = 0.3;              // m/s
    sigmaG              = (3.0*M_PI/180);   // radians


    // observation parameters
    MAX_RANGE           = 30.0;                     // metres
    DT_OBSERVE          = 8*DT_CONTROLS;            // seconds, time interval between observations

    // observation noises
    sigmaR              = 0.1;                      // metres
    sigmaB              = (1.0*M_PI/180);           // radians
    sigmaT              = (1.0*M_PI/180);           // IMU angular noise (radians)


    // data association innovation gates (Mahalanobis distances)
    GATE_REJECT         = 4.0;                      // maximum distance for association
    GATE_AUGMENT        = 25.0;                     // minimum distance for creation of new feature
    // For 2-D observation:
    //   - common gates are: 1-sigma (1.0), 2-sigma (4.0), 3-sigma (9.0), 4-sigma (16.0)
    //   - percent probability mass is: 1-sigma bounds 40%, 2-sigma 86%, 3-sigma 99%, 4-sigma 99.9%.


    // waypoint proximity
    AT_WAYPOINT         = 1.0;                      // metres, distance from current
                                                    // waypoint at which to switch to next waypoint
    NUMBER_LOOPS        = 2;                        // number of loops through the waypoint list

    // resampling
    NPARTICLES          = 100;
    NEFFECTIVE          = 0.75*NPARTICLES;          // minimum number of effective particles before resampling

    // switches
    SWITCH_CONTROL_NOISE    = 1;
    SWITCH_SENSOR_NOISE     = 1;
    SWITCH_INFLATE_NOISE    = 0;
    SWITCH_PREDICT_NOISE    = 0;    // sample noise from predict (usually 1 for fastslam1.0 and 0 for fastslam2.0)
    SWITCH_SAMPLE_PROPOSAL  = 1;    // sample from proposal (no effect on fastslam1.0 and usually 1 for fastslam2.0)
    SWITCH_HEADING_KNOWN    = 1;
    SWITCH_RESAMPLE         = 1;
    SWITCH_PROFILE          = 1;
    SWITCH_SEED_RANDOM      = 0;    // if not 0, seed the randn() with its value at beginning //
                                    //  of simulation (for repeatability)

    SWITCH_ASSOCIATION_KNOWN= 0;
    SWITCH_BATCH_UPDATE     = 1;
    SWITCH_USE_IEKF         = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// parse values
    ////////////////////////////////////////////////////////////////////////////
    f("V",                          V);
    f("MAXG",                       MAXG);
    f("RATEG",                      RATEG);
    f("WHEELBASE",                  WHEELBASE);
    f("DT_CONTROLS",                DT_CONTROLS);

    f("sigmaV",                     sigmaV);
    f("sigmaG",                     sigmaG);

    f("MAX_RANGE",                  MAX_RANGE);
    f("DT_OBSERVE",                 DT_OBSERVE);

    f("sigmaR",                     sigmaR);
    f("sigmaB",                     sigmaB);
    f("sigmaT",                     sigmaT);

    f("GATE_REJECT",                GATE_REJECT);
    f("GATE_AUGMENT",               GATE_AUGMENT);

    f("AT_WAYPOINT",                AT_WAYPOINT);
    i("NUMBER_LOOPS",               NUMBER_LOOPS);

    i("NPARTICLES",                 NPARTICLES);
    i("NEFFECTIVE",                 NEFFECTIVE);

    i("SWITCH_CONTROL_NOISE",       SWITCH_CONTROL_NOISE);
    i("SWITCH_SENSOR_NOISE",        SWITCH_SENSOR_NOISE);
    i("SWITCH_INFLATE_NOISE",       SWITCH_INFLATE_NOISE);
    i("SWITCH_PREDICT_NOISE",       SWITCH_PREDICT_NOISE);
    i("SWITCH_SAMPLE_PROPOSAL",     SWITCH_SAMPLE_PROPOSAL);
    i("SWITCH_HEADING_KNOWN",       SWITCH_HEADING_KNOWN);
    i("SWITCH_RESAMPLE",            SWITCH_RESAMPLE);
    i("SWITCH_PROFILE",             SWITCH_PROFILE);
    i("SWITCH_SEED_RANDOM",         SWITCH_SEED_RANDOM);

    i("SWITCH_ASSOCIATION_KNOWN",   SWITCH_ASSOCIATION_KNOWN);
    i("SWITCH_BATCH_UPDATE",        SWITCH_BATCH_UPDATE);
    i("SWITCH_USE_IEKF",            SWITCH_USE_IEKF);

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// fastslam2 function
//
////////////////////////////////////////////////////////////////////////////////

void FastSLAM2_predict(Particle &particle,float V,float G,MatrixXf &Q, float WB,float dt, int addrandom)
{
    VectorXf xv = particle.xv();
    MatrixXf Pv = particle.Pv();
#if 0
    cout<<"particle.xv() in predict"<<endl;
    cout<<particle.xv()<<endl;
    cout<<"particle.Pv() in predict"<<endl;
    cout<<particle.Pv()<<endl;
#endif
    //Jacobians
    float phi = xv(2);
    MatrixXf Gv(3,3);

    Gv <<   1,0,-V*dt*sin(G+phi),
            0,1,V*dt*cos(G+phi),
            0,0,1;
    MatrixXf Gu(3,2);
    Gu <<   dt*cos(G+phi), -V*dt*sin(G+phi),
            dt*sin(G+phi), V*dt*cos(G+phi),
            dt*sin(G)/WB, V*dt*cos(G)/WB;

    //predict covariance
    MatrixXf newPv;//(Pv.rows(),Pv.cols());

    //TODO: Pv here is somehow corrupted. Probably in sample_proposal
#if 0
    cout<<"Pv in predict"<<endl;
    cout<<Pv<<endl;
#endif

    newPv = Gv*Pv*Gv.transpose() + Gu*Q*Gu.transpose();
    particle.setPv(newPv);

    //optional: add random noise to predicted state
    if (addrandom ==1) {
        VectorXf A(2);
        A(0) = V;
        A(1) = G;
        VectorXf VG(2);
        VG = multivariate_gauss(A,Q,1);
        V = VG(0);
        G = VG(1);
    }

    //predict state
    VectorXf xv_temp(3);
    xv_temp << xv(0) + V*dt*cos(G+xv(2)),
            xv(1) + V*dt*sin(G+xv(2)),
            pi_to_pi2(xv(2) + V*dt*sin(G/WB));
    particle.setXv(xv_temp);
}


void FastSLAM2_observe_heading(Particle &particle, float phi, int useheading)
{
    if (useheading ==0){
        return;
    }

    float sigmaPhi = 0.01*M_PI/180.0;
    // cout<<"sigmaPhi "<<sigmaPhi<<endl;
    VectorXf xv = particle.xv();
    // cout<<"xv"<<endl;
    // cout<<xv<<endl;
    MatrixXf Pv = particle.Pv();
    // cout<<"Pv"<<endl;
    // cout<<Pv<<endl;

    MatrixXf H(1,3);
    H<<0,0,1;

    float v = pi_to_pi(phi-xv(2));
    // cout<<"v"<<endl;
    // cout<<v<<endl;
    KF_joseph_update(xv,Pv,v,pow(sigmaPhi,2),H);
    // cout<<"KF_Joseph = xv"<<endl;
    // cout<<xv<<endl;
    // cout<<"KF_Joseph = Pv"<<endl;
    // cout<<Pv<<endl;

    particle.setXv(xv);
    particle.setPv(Pv);
}

float gauss_evaluate(VectorXf &v, MatrixXf &S, int logflag)
{
    int D = v.size();
    MatrixXf Sc = S.llt().matrixL();

    //normalised innovation
    VectorXf nin = Sc.jacobiSvd(ComputeThinU | ComputeThinV).solve(v);

    int s;

    float E=0;
    for (s=0; s<nin.size(); s++) {
        nin(s) = pow(nin(s),2);
        E += nin(s);
    }
    E=-0.5*E;
    //Note: above is a fast way to compute sets of inner-products

    unsigned i;
    float C,w;
    unsigned  m = min(Sc.rows(), Sc.cols());

    if (logflag !=1) {
        float prod = 1;
        for (i=0; i<m; i++) {
            prod=prod*Sc(i,i); //multiply the diagonals
        }
        C = pow((2*M_PI),(D/2))*prod; //normalizing term(makes Gaussian hyper volume =1
        w = exp(E)/C; //likelihood
    } else {
        float sum=0;
        for (i=0; i<m; i++) {
            sum+=log(Sc(i,i));
        }
        C = 0.5*D*log(2*M_PI) + sum; //log of normalising term
        w = E-C; //log-likelihood
    }
    return w;
}


//
//compute particle weight for sampling
//

float likelihood_given_xv(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
{
    float w = 1;
    vector<int> idfi;

    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;

    vector<VectorXf> zp;
    VectorXf v(z[0].rows());

    unsigned i,k;

    for (i=0; i<idf.size(); i++){
        idfi.clear();
        idfi.push_back(idf[i]);
        zp.clear();

        compute_jacobians(particle,idfi,R,zp,&Hv,&Hf,&Sf);

        for (k=0; k<z[0].rows(); k++) {
            v(k) = z[i][k]-zp[0][k];
        }
        v(1) = pi_to_pi(v(1));

        w = w*gauss_evaluate(v,Sf[0],0);
    }
    return w;
}

VectorXf delta_xv(VectorXf &xv1, VectorXf &xv2)
{
    //compute innovation between two xv estimates, normalising the heading component
    VectorXf dx = xv1-xv2;
    dx(2) = pi_to_pi(dx(2));
    return dx;
}

float FastSLAM2_compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
{
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;
    vector<VectorXf> zp;

    //process each feature, incrementally refine proposal distribution
    compute_jacobians(particle,idf,R,zp,&Hv,&Hf,&Sf);

    vector<VectorXf> v;

    for (unsigned long j =0; j<z.size(); j++) {
        VectorXf v_j = z[j] - zp[j];
        v_j[1] = pi_to_pi(v_j[1]);
        v.push_back(v_j);
    }

    float w = 1.0;

    MatrixXf S;
    float den, num;
    for (unsigned long i=0; i<z.size(); i++) {
        S = Sf[i];
        den = 2*M_PI*sqrt(S.determinant());
        num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
        w = w*num/den;
    }
    return w;
}

//compute proposal distribution, then sample from it, and compute new particle weight
//void sample_proposal(Particle &particle, MatrixXf z, vector<int> idf, MatrixXf R)
void FastSLAM2_sample_proposal(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
{
    VectorXf xv(particle.xv()); //robot position
    MatrixXf Pv(particle.Pv()); //controls (motion command)

    VectorXf xv0(xv);
    MatrixXf Pv0(Pv);

    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;

    // FIXME: z is std vector, get first item's rows?
    vector<VectorXf>    zp;
    MatrixXf            Hvi;
    MatrixXf            Hfi;
    MatrixXf            Sfi;

    // FIXME: z is std vector, get first item's rows?
    VectorXf vi(z[0].rows());

    //process each feature, incrementally refine proposal distribution
    unsigned long i;

    for (i =0; i<idf.size(); i++) {
        vector<int>     j;
        j.push_back(idf[i]);
        zp.clear();

        compute_jacobians(particle, j, R, zp, &Hv, &Hf, &Sf);
        //assert(zp.size() == 1);

        Hvi = Hv[0];
        Hfi = Hf[0];
        Sfi = Sf[0].inverse();

        vi = z[i] - zp[0];
        vi[1] = pi_to_pi(vi[1]);

        //proposal covariance
        MatrixXf Pv_inv = Pv.llt().solve(MatrixXf::Identity(Pv.rows(), Pv.cols()));
        Pv = Hvi.transpose() * Sfi * Hvi + Pv_inv; //Pv.inverse();
        Pv = Pv.llt().solve(MatrixXf::Identity(Pv.rows(), Pv.cols()));//Pv.inverse();

        //proposal mean
        xv = xv + Pv * Hvi.transpose() * Sfi *vi;
        particle.setXv(xv);
        particle.setPv(Pv);
    }

    //sample from proposal distribution
    VectorXf xvs = multivariate_gauss(xv, Pv, 1);
    particle.setXv(xvs);
    MatrixXf zeros(3,3);
    zeros.setZero();
    particle.setPv(zeros);

    //compute sample weight: w = w* p(z|xk) p(xk|xk-1) / proposal
    float       like, prior, prop, w;
    VectorXf    v1, v2;

    v1 = delta_xv(xv0,xvs);
    v2 = delta_xv(xv, xvs);

    like = likelihood_given_xv(particle, z, idf, R);
    prior = gauss_evaluate(v1, Pv0, 0);
    prop = gauss_evaluate(v2, Pv, 0);
    w = particle.w() * like * prior / prop;

    particle.setW(w);
}

