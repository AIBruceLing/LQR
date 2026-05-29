#include "pid_control.h"


    PID_controller::PID_controller(double Kp, double Ki, double Kd, double target, double upper, double lower):Kp(Kp),
                                                                                                             Ki(Ki),
                                                                                                             Kd(Kd),
                                                                                                             target(target),
                                                                                                             upper(upper),
                                                                                                             lower(lower) {}



    void PID_controller::setTarget(double target){
        this->target = target;

    }

    void PID_controller::setK(double Kp , double Ki , double Kd){
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
    }

    void PID_controller::setBound(double upper , double lower){
        this->upper = upper;
        this->lower = lower;
    }

    double PID_controller::calOutput(double state){   // 控制量加速度输出 ， 传入机器人当前速度 ， 输出加速度 （根据当前速度和目标速度的差值）
        this->error = target - state;
        double u = error * Kp + sum_error*Ki + (error - pre_error) * Kd;
        if(u<lower) u = lower;
        else if(u>upper) u = upper;
        this->pre_error = this->error;
        this->sum_error = sum_error + error;
        return u;
    }

    void PID_controller::reset(){
        error = 0;
        pre_error = 0;
        sum_error = 0;
    }

    void PID_controller::setSumError(double sum_error){
        this->sum_error = sum_error;
    }