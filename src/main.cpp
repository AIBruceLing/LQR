#include "LQR.h"
#include "Reference_path.h"
#include "KinematicModel.h"
#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;

int main(){
    double dt = 0.1;     //前向欧拉离散时间步长
    double L = 2.0;      //车辆长度


    //车辆初始状态
    double x_0 = 0.0;
    double y_0 = 1.0;
    double v = 2.0;      //车辆速度
    double psi_0 = 0.0;

    int N = 500;         //迭代次数

    //代价函数的Q和R矩阵
    MatrixXd Q(3,3);
    Q<<3,0,0,
       0,3,0,
       0,0,3;

    MatrixXd R(2,2);
    R<<2.0,0.0,
        0.0,2.0;
    

    vector<double>x_, y_;           // 保存机器人移动过程中的轨迹
    
    MyReference_path referencePath; //自定义参考轨迹，包含x坐标、y坐标、偏航角和曲率

    KinematicModel robot(x_0,y_0,psi_0,v,L,dt);  //单车模型

    LQRControl robot_motion_LQR(N);//求解Riccati矩阵 P 时 迭代N次

    vector<double> robot_state;


    for(int i = 0;i<700;i++){
        plt::clf();
        robot_state = robot.getState();// {x , y , psi , v};
        vector<double>one_trial = referencePath.calcTrackError(robot_state); //{error , k , yaw , min_index}
        double k = one_trial[1];
        double ref_yaw = one_trial[2];// 预瞄点曲率
        double s0 = one_trial[3];  // min_distance_index

        double ref_delta = atan2(L*k,1);  // 求出参考轨迹上的预瞄点的前轮转角
        vector<MatrixXd>state_space = robot.stateSpace(ref_delta,ref_yaw);   //离散化的误差状态空间方程的雅可比 {A,B} 矩阵


        // 传入机器人状态、参考轨迹、min_index, A , B , Q, R     求解得到前轮转角的增量
        double delta = robot_motion_LQR.lqrControl(robot_state, referencePath.refer_path, s0, state_space[0], state_space[1], Q, R);// 前轮转角增量
        delta += ref_delta;

        robot.updateState(0, delta);   // 加速度设为0，恒速

        cout<<" 前轮转角增量 :"<< delta <<" rad "<<endl;

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