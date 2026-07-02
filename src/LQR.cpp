#include "LQR.h"
#include<cmath>


    LQRControl::LQRControl(int n):N(n){};

    //MatrixXd 是 Eigen 库中的一个模板类，用于创建和操作动态大小的矩阵。
    // 参数 A 和 B 分别代表系统的状态矩阵和控制矩阵。
    // 参数 Q 和 R 分别代表状态权重矩阵和控制权重矩阵。
    // Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
    // R矩阵元素变大意味着希望控制输入能够尽可能小,即消耗的能耗更小
    // 返回值是解得的Riccati矩阵 P
    MatrixXd LQRControl::calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R){
        MatrixXd Qf = Q;      // 终端状态权重矩阵
        MatrixXd P_old = Qf;
        MatrixXd P_new;

        for(int i = 0;i < N;i++){
         P_new = Q + A.transpose() * P_old * A - A.transpose() * P_old * B * (R + B.transpose() * P_old * B).inverse() * B.transpose() * P_old * A;
        //  if((P_new - P_old).cwiseAbs().maxCoeff()<EPS) break;
        if((P_new-P_old).maxCoeff()<EPS&&(P_old-P_new).maxCoeff()<EPS)break;
        // if ((P_new - P_old).cwiseAbs().maxCoeff() < EPS) break; //cwiseAbs 取矩阵中每个元素的绝对值

         P_old= P_new;
    }

    /*
    矩阵 P 被称为“Cost-to-go”（剩余代价矩阵 ， 它代表的物理意义是：从当前状态开始，一直到控制过程结束，系统未来总共还会产生多少代价
    */

    return P_new;
}


// ***************************************************************************//
/**
 * LQR控制器
 * @param robot_state
 * @param refer_path
 * @param s0
 * @param A
 * @param B
 * @param Q
 * @param R
 * @return
 */
    double LQRControl::lqrControl(vector<double>robot_state,vector<vector<double>>refer_path, double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R){
        MatrixXd X(3,1);
        X<< robot_state[0] - refer_path[s0][0],
            robot_state[1] - refer_path[s0][1],
            robot_state[2] - refer_path[s0][2];//  x是当前位置和预瞄点的偏差  x,y,yaw三个偏差

        MatrixXd P = calRicatti(A,B,Q,R);    //   3*3

        MatrixXd K = (R+B.transpose()*P*B).inverse()*B.transpose()*P*A;  // 反馈增益 2*3
        MatrixXd u = -K*X;   // 2*1
        return u(1,0);  // return delta_theta   航向角差值
    }