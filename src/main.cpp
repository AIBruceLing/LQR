#include "LQR.h"
#include "Reference_path.h"
#include "KinematicModel.h"
#include "matplotlibcpp.h"



namespace plt = matplotlibcpp;

int main(){
    double dt = 0.1;
    double L = 2.0;
    double  = 2.0;
    double x_v0 = 0.0;
    double y_0 = 1.0;
    double psi_0 = 0.0;

    int N = 500;
    MatrixXd Q(3,3);
    Q<<3,0,0,
       0,3,0,
       0,0,3;

    MatrixXd R(2,2);
    R<<2.0,0.0,
        0.0,2.0;
    

    // 保存机器人移动过程中的轨迹
    vector<double>x_, y_;
    MyReference_path referencePath;
    KinematicModel robot(x_0,y_0,psi_0,v,L,dt);

    LQRControl robot_motion_LQR(N);//求解Riccati矩阵 P 时 迭代N次

    vector<double> robot_state;


    for(int i = 0;i<700;i++){
        plt::clf();
        robot_state = robot.getState();// {x , y , psi , v};
        vector<double>one_trial = referencePath.calcTrackError(robot_state);
        double k = one_trial[1];
        double ref_yaw = one_trial[2];// 预瞄点曲率
        double s0 = one_trial[3];  // min_distance_index

        double ref_delta = atan2(L*k,1);  // 求出参考轨迹上的预瞄点的航向角
        vector<MatrixXd>state_space = robot.stateSpace(ref_delta,ref_yaw);   //{A,B} 矩阵


     // 传入机器人状态、参考轨迹、min_index, A , B , Q, R     求解得到前轮转角的增量
        double delta = robot_motion_LQR.lqrControl(robot_state, referencePath.refer_path,
                                                   s0, state_space[0], state_space[1], Q, R);// 前轮转角
        delta += ref_delta;

        robot.updateState(0,delta);   // 加速度设为0，恒速

        cout<<" speed :"<< robot.v<<" m/s "<<endl;

        x_.push_back(robot.x);
        y_.push_back(robot.y);
        
        // 参考轨迹
        plt::plot(referencePath.refer_x,referencePath.refer_y,"b");
        plt::grid(true);
        plt::ylim(-5,5);

        //机器人轨迹
        plt::plot(x_, y_,"r");
        plt::pause(0.01);
    }


     const char* filename = "./LQR.png";
     plt::save(filename);
     plt::show();
     return 0;
  
}