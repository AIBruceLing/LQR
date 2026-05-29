#include "Reference_path.h"



//************************ 构造函数 ， 求出参考轨迹点上的曲率等信息 ******************************************//
MyReference_path::MyReference_path(){
    refer_path = vector<vector<double>> (1000,vector<double>(4));// 1000个点 ， 每个点有x、y、偏航角yaw和曲率（curvature）

    //生成参考轨迹

    //（1）生成x坐标和y坐标， 以及保存x坐标和y坐标的refer_x和refer_y
    for(int i = 0;i<1000;i++){
        refer_path[i][0] = 0.1*i; // x坐标
        refer_path[i][1] = 2 * sin(refer_path[i][0] / 3.0) + 2.5 * cos(refer_path[i][0] / 2.0);  //y坐标
        refer_x.push_back(refer_path[i][0]); // 1000 * 1
        refer_y.push_back(refer_path[i][1]); // 1000 * 1
    }

    // （2）计算曲率（需要计算一阶导和二阶导）和偏航角
    double dx,dy,ddx,ddy;
    for(int i=0;i<refer_path.size();i++){
        if (i==0){
             dx = refer_path[i+1][0] - refer_path[i][0];
             dy = refer_path[i+1][1] - refer_path[i][1];
             ddx = refer_path[2][0] + refer_path[0][0] - 2*refer_path[1][0];
             ddy = refer_path[2][1] + refer_path[0][1] - 2*refer_path[1][1];    //在计算一阶和二阶导数时，对于路径的起点和终点，使用特殊的差分公式来处理边界条件。
        }else if(i==refer_path.size()-1){
             dx = refer_path[i][0] - refer_path[i-1][0];
             dy = refer_path[i][1] - refer_path[i-1][1];
             ddx = refer_path[i][0] + refer_path[i-2][0] - 2*refer_path[i-1][0];
             ddy = refer_path[i][1] + refer_path[i-2][1] - 2*refer_path[i-1][1];
        }else{
             dx = refer_path[i+1][0] - refer_path[i][0]; // 下一个点减当前点 dx
             dy = refer_path[i+1][1] - refer_path[i][1]; // 下一个点减当前点 dy
             ddx = refer_path[i+1][0] + refer_path[i-1][0] - 2*refer_path[i][0];    // 上一个点+下一个点 - 2*当前点    ddx
             ddy = refer_path[i+1][1] + refer_path[i-1][1] - 2*refer_path[i][1];    // 上一个点+下一个点 - 2*当前点    ddy
        }

        refer_path[i][2] = atan2(dy,dx);                                                            //偏航角 yaw
        //计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        //参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
        refer_path[i][3]= (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3.0 / 2) ;               // 曲率k计算
    }
}

// ********************************计算跟踪误差 **************************************** //
vector<double> MyReference_path::calcTrackError(vector<double>robot_state){   //robot_state : 1 *2   (x , y)
    double x = robot_state[0];
    double y = robot_state[1];

    vector<double>d_x(refer_path.size());
    vector<double>d_y(refer_path.size());
    vector<double>d(refer_path.size());

    for(int i = 0;i<refer_path.size();i++){
        d_x[i] = refer_path[i][0] - x;
        d_y[i] = refer_path[i][1] - y;
        d[i] = sqrt(d_x[i]*d_x[i]+d_y[i]*d_y[i]);
    }

    size_t min_index = min_element(d.begin(),d.end()) - d.begin();  // 使用减法可以得到最近点索引------> 车辆/机器人当前位置和参考轨迹距离最短的点 ： 预瞄点

    double yaw = refer_path[min_index][2];
    double k = refer_path[min_index][3];
    double angle = normalizeAngle(yaw - atan2(d_y[min_index] , d_x[min_index]));
    double error = d[min_index];
    if(angle<0) error *= -1;    //  需要判断 车辆/机器人 当前在参考轨迹的左边还是右边   error 有正负号  ------> 负代表在轨迹左边 ， 正表示在轨迹右边

    return {error , k , yaw , static_cast<double>(min_index)};

}


// ************************ 角度归一化（-PI ， PI） *******************************//
    double MyReference_path::normalizeAngle(double angle){
        while(angle>PI){
            angle -= 2*PI;
        }
        while(angle<-PI){
            angle += 2*PI;
        }

        return angle;
    }

