//
// Created by huangyewei on 2017/3/8.
//

#ifndef PARKINGLOT_GRAPHOPTIMIZE_H
#define PARKINGLOT_GRAPHOPTIMIZE_H
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "Eigen/Dense"
#include "taginfo.h"

using namespace g2o;
using namespace cv;
using namespace Eigen;
using namespace std;

class GraphOptimize{
private:
    const double pi = 3.1415926535898;
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    SlamLinearSolver* linearSolver;
    SlamBlockSolver* blockSolver;

    OptimizationAlgorithmLevenberg* solver;
    //OptimizationAlgorithmGaussNewton* solver;

    g2o::SparseOptimizer globalOptimizer;
    int car_id;
    int fix_id;
    int edge_id=0;

    vector<EdgeSE2PointXY*> edges_tag;
    vector<EdgeSE2*> edges_trans;


public:
    GraphOptimize(int start_id);
    ~GraphOptimize();

    void AddCarVertex(double direction_pre, double direction_now,
                      Point2d trans_pre, Point2d vehicle, vector<taginfo> TagInfo);
    void AddTagVertex(int id, Point2d pose);
    void OptimizeAll();
    Point2d GetTagVertex(int id);
    Point3d GetNewestCarVertex();
    void ClearAll();
};





#endif //PARKINGLOT_GRAPHOPTIMIZE_H
