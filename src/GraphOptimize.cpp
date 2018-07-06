//
// Created by huangyewei on 2017/3/8.
//

#include "GraphOptimize.h"
GraphOptimize::GraphOptimize(int start_id){
    linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    blockSolver = new SlamBlockSolver(linearSolver);
    solver = new OptimizationAlgorithmLevenberg(blockSolver);

    //solver = new OptimizationAlgorithmGaussNewton(blockSolver);
    globalOptimizer.setAlgorithm( solver );
    // 输出调试信息
    globalOptimizer.setVerbose( true );


    car_id  = start_id;
    fix_id = start_id;

}

GraphOptimize::~GraphOptimize(){
    globalOptimizer.clear();
}

void GraphOptimize::AddTagVertex(int id, Point2d pose){
    Vector2D t(pose.x, pose.y);
    VertexPointXY* tag = new VertexPointXY;
    tag->setId(id);
    tag->setEstimate(t);
    globalOptimizer.addVertex(tag);
}

void GraphOptimize::AddCarVertex(double direction_pre, double direction_now, Point2d trans_pre,
                                 Point2d vehicle, vector<taginfo> TagInfo){
    const SE2 v(vehicle.x, vehicle.y, direction_now);

    VertexSE2* car = new VertexSE2;
    car->setId(car_id);
    car->setEstimate(v);

    globalOptimizer.addVertex(car);

    //add the corresponding edges with tag
    for (int i=0; i<TagInfo.size(); i++){
        taginfo tag_now = TagInfo[i];
        Vector2d o(tag_now.tag_pos.x-tag_now.car_pos.x, tag_now.tag_pos.y-tag_now.car_pos.y);

        Matrix2d info = Matrix< double, 2,2 >::Identity();
        info(0,0)=25;info(1,1)=25;//info(2,2)=400;

        EdgeSE2PointXY* observation =  new EdgeSE2PointXY;
        observation->vertices()[0] = globalOptimizer.vertex(car_id);
        observation->vertices()[1] = globalOptimizer.vertex(tag_now.id);
        observation->setMeasurement(o);
        observation->setInformation(info);
        observation->setParameterId(0,0);
        //edges_tag.push_back(observation);

        globalOptimizer.addEdge(observation);
    }
    //add the corresponding edges with the former car pose
    if(car_id != fix_id){
        const SE2 t(trans_pre.x, trans_pre.y, direction_now - direction_pre);

        Matrix3d info_imu = Matrix<double,3,3>::Identity();
        info_imu(0,0) = 100;info_imu(1,1) = 6.25;info_imu(2,2) = 100;

        EdgeSE2* reckon = new EdgeSE2;
        reckon->vertices()[0] = globalOptimizer.vertex(car_id-1);
        reckon->vertices()[1] = globalOptimizer.vertex(car_id  );
        reckon->setMeasurement(t);
        reckon->setInformation(info_imu);
        reckon->setParameterId(0,0);
        //edges_trans.push_back(reckon);

        globalOptimizer.addEdge(reckon);
    }
    else{
        VertexPointXY* first_tag = dynamic_cast<VertexPointXY*>(globalOptimizer.vertex(TagInfo[0].id));
        first_tag->setFixed(true);
    }

    car_id++;
}

void GraphOptimize::OptimizeAll(){
    //int inliers_trans=0, inliers_tag=0, outliers_trans=0, outliers_tag=0;
    //VertexSE2* first_car = dynamic_cast<VertexSE2*>(globalOptimizer.vertex(car_id-1));
    //first_car->setFixed(true);

    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(10); //可以指定优化步数

    /*cout<<"error:"<<endl;
    for ( int i=0; i<edges_tag.size();i++ ) {
        EdgeSE2PointXY *e = edges_tag[i];
        e->computeError();
             // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( e->chi2() > 1 ) {
               cout<<"error = "<<e->chi2()<<endl;
            outliers_tag++;
        }
        else {
            inliers_tag++;
         }
     }
    cout<<"tag inliers:"<<inliers_tag<<endl;
    cout<<"tag outliers:"<<outliers_tag<<endl;

    cout<<endl;
    for ( int i=0; i<edges_trans.size();i++  ) {
        EdgeSE2 *e = edges_trans[i];
        e->computeError();
        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( e->chi2() > 1 ) {
            cout<<"error = "<<e->chi2()<<endl;
            outliers_trans++;
        }
        else {
            inliers_trans++;
        }
    }
    cout<<"trans inliers:"<<inliers_trans<<endl;
    cout<<"trans outliers:"<<outliers_trans<<endl;*/

}

Point2d GraphOptimize::GetTagVertex(int id){
    VertexPointXY* tag = dynamic_cast<VertexPointXY*>(globalOptimizer.vertex(id));
    Vector2d pose = tag->estimate();
    return Point2d(pose(0), pose(1));
}

Point3d GraphOptimize::GetNewestCarVertex(){
    VertexSE2* car = dynamic_cast<VertexSE2*>(globalOptimizer.vertex(car_id-1));
    Vector3d pose = car->estimate().toVector();
    return Point3d(pose(0), pose(1), pose(2));
}

void GraphOptimize::ClearAll(){
    //edges_tag.clear();
    //edges_trans.clear();
    globalOptimizer.clear();
    car_id = fix_id;
}