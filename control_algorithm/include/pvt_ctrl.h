#pragma once

#include <fstream>
#include "json/json.h"
#include <string>
#include "lpf_fst.h"
#include <vector>
#include <cmath>
#include "data_bus.h"

class pvt_ctrl
{
private:
    std::vector<lpf_fst> tau_out_lpf;
    std::vector<int> PV_enable;
    double sign(double in);
    const std::vector<std::string> motorName={"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};

public:
    int jointNum;
    std::vector<double> motor_pos_cur; // 电机当前位置
    std::vector<double> motor_pos_des_old;
    std::vector<double> motor_vel;// 电机当前速度
    std::vector<double> motor_tor_out; // final tau output
    
    pvt_ctrl(double timeStepIn, const char * jsonPath);
    void calMotorsPVT();
    void calMotorsPVT(double deltaP_Lim);
    void calMotorsPVT(int init);
    void enablePV(); // enable PV control item
    void disablePV(); // disable PV control item
    void enablePV(int jtId); // enable PV control item
    void disablePV(int jtId); // disable PV control item
    void setJointPD(double kp, double kd, const char * jointName);
    void dataBusRead(DataBus &busIn);
    void dataBusWrite(DataBus &busIn);

    std::vector<double> motor_pos_des; // P des
    std::vector<double> motor_vel_des; // V des
    std::vector<double> motor_tor_des; // T des

    std::vector<double> pvt_Kp;
    std::vector<double> pvt_Kd;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;


};

 