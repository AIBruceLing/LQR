#ifndef LG_REFERENCE_PATH_H
#define LG_REFERENCE_PATH_H

#include<iostream>
#include<vector>
#include<cmath>
#include<algorithm>
#include<Eigen/Dense>


using namespace std;
using namespace Eigen;

constexpr double PI = 3.1415926;
// #define PI 3.1415926

struct parameters
{
    int L;
    int NX,NU,T;
    double dt;
};


class MyReference_path{
    public:
    MyReference_path();

    vector<double> calcTrackError( vector<double> robot_state);

    double normalizeAngle(double angle);

    public:
    //refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
    vector<vector<double>> refer_path;
    vector<double> refer_x,refer_y;
};
















#endif

