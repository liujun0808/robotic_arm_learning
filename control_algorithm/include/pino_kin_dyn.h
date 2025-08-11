#pragma once
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <string>
#include <json/json.h>
#include <vector>
#include "data_bus.h"

template <typename T>
const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

class Pin_KinDyn{
public:
    std::vector<bool> motorReachLimit;
    Eigen::VectorXd motorMaxTorque;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;
    Eigen::VectorXd tauJointOld;
    const std::vector<std::string> motorName = {"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};
    pinocchio::Model arm;
    int model_nv;
    pinocchio::JointIndex joint1,joint2,joint3,joint4,joint5,joint6,joint7, tool_joint;// 工具关节 用于绑定末端工具
    pinocchio::FrameIndex link1,link2,link3,link4,link5,link6,link7,ee_link;
    Eigen::VectorXd q,dq,ddq;
    Eigen::VectorXd tool0_pos_b; // 在base下位置
    Eigen::Matrix3d tool_R;// 在base的旋转矩阵
    Eigen::Quaternion<double> tool0_qua; //在base的四元数
    Eigen::Matrix<double,6,-1> J_tool0,dJ_tool0;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;

    struct IkRes
    {
        int status;
        int itr;
        Eigen::VectorXd err;
        Eigen::VectorXd jointPoseRes;
    };

    Pin_KinDyn(std::string urdf_path);
    void dataRead(const DataBus  &robotState);
    void dataWrite(DataBus &robotState);
    void computeJ_dJ();
    void computeDyn();
    void print_tool0_pos();
    Eigen::VectorXd computeForwordTau(Eigen::VectorXd &q_des,Eigen::VectorXd &dq_des,Eigen::VectorXd &ddq_des);   
    IkRes computeIK(const Eigen::Matrix3d &tool_R_res,const Eigen::Vector3d &tool0_pos_res);



private:
    pinocchio::Data data_arm;
};

