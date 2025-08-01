#pragma once
#include <mujoco/mujoco.h>
#include "data_bus.h"
#include <string>
#include <vector>

class MJ_Interface {
public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tau;
    const std::vector<std::string> motorName = {"actuator1","actuator2","actuator3","actuator4","actuator5","actuator6","actuator7"};

    // iiwa 7dof
    const std::vector<std::string> JointName={"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};
    MJ_Interface(mjModel *mj_modelIn, mjData  *mj_dataIn);
    void updateSensorValues();
    void setMotorsTorque(std::vector<double> &tauIn);
    void dataBusWrite(DataBus &busIn);

private:
    mjModel *mj_model;
    mjData  *mj_data;
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;

    double timeStep{0.001}; // second
    bool isIni{false};
};