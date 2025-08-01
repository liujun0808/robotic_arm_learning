#pragma once

#include "Eigen/Dense"
#include <iostream>
#include <vector>
#include "iomanip"

struct DataBus{
    const int model_nv; // number of dq
    // motors, sensors and states feedback
    std::vector<double> motors_pos_cur;
    std::vector<double> motors_vel_cur;
    std::vector<double> motors_tor_cur;

    // PVT controls
    std::vector<double> motors_pos_des;
    std::vector<double> motors_vel_des;
    std::vector<double> motors_tor_des;
    std::vector<double> motors_tor_out;

    // states and key variables
    Eigen::VectorXd q, dq, ddq;
    Eigen::VectorXd qOld;
    Eigen::MatrixXd J_tool0;
    Eigen::MatrixXd dJ_tool0;

    Eigen::Vector3d tool0_pos_W; // in world frame
    Eigen::Matrix3d tool0_rot_W;
    Eigen::Vector3d tool0_pos_b; // in body frame
    Eigen::Matrix3d tool_R;
    Eigen::VectorXd qCmd, dqCmd;
    Eigen::VectorXd tauJointCmd;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_G, dyn_Non;

    Eigen::Vector3d slop;
    Eigen::Matrix<double,3,3>   inertia;

    // cmd value from the joystick interpreter
    Eigen::Vector3d     js_eul_des;
    Eigen::Vector3d     js_pos_des;
    Eigen::Vector3d     js_omega_des;
    Eigen::Vector3d     js_vel_des;

    // cmd values for MPC
    Eigen::VectorXd     Xd;
    Eigen::VectorXd     X_cur;
//    Eigen::Vector3d     mpc_eul_des;
//    Eigen::Vector3d     mpc_pos_des;
//    Eigen::Vector3d     mpc_omega_des;
//    Eigen::Vector3d     mpc_vel_des;
    Eigen::VectorXd     X_cal;
    Eigen::VectorXd     dX_cal;
    Eigen::VectorXd     fe_react_tau_cmd;

    int 	qp_nWSR_MPC;
    double 	qp_cpuTime_MPC;
    int 	qpStatus_MPC;

    // cmd values for WBC
    Eigen::Vector3d base_rpy_des;
    Eigen::Vector3d base_pos_des;
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q;
    Eigen::Vector3d swing_fe_pos_des_W;
    Eigen::Vector3d swing_fe_rpy_des_W;
    Eigen::Vector3d stance_fe_pos_cur_W;
    Eigen::Matrix3d stance_fe_rot_cur_W;
    Eigen::VectorXd wbc_delta_q_final, wbc_dq_final, wbc_ddq_final;
    Eigen::VectorXd wbc_tauJointRes;
    Eigen::VectorXd wbc_FrRes;
    Eigen::VectorXd Fr_ff;
    int qp_nWSR;
    double qp_cpuTime;
    int qp_status;

    

    double thetaZ_des{0};

    // for jump
    Eigen::Vector3d base_pos_stand;
    Eigen::Matrix<double,6,1> pfeW_stand, pfeW0;
    //Eigen::Vector3d mpc_eul_des, mpc_omega_des, mpc_vel_des, mpc_pos_des;

    DataBus(int model_nvIn): model_nv(model_nvIn){
        motors_pos_cur.assign(model_nv,0);
        motors_vel_cur.assign(model_nv,0);
        motors_tor_cur.assign(model_nv,0);
        motors_pos_des.assign(model_nv,0);
        motors_vel_des.assign(model_nv,0);
        motors_tor_des.assign(model_nv,0);
        motors_tor_out.assign(model_nv,0);
        q=Eigen::VectorXd::Zero(model_nv);
        qOld=Eigen::VectorXd::Zero(model_nv);
        dq=Eigen::VectorXd::Zero(model_nv);
        ddq=Eigen::VectorXd::Zero(model_nv);
        qCmd=Eigen::VectorXd::Zero(model_nv);
        dqCmd=Eigen::VectorXd::Zero(model_nv);
        tauJointCmd=Eigen::VectorXd::Zero(model_nv);


        Xd = Eigen::VectorXd::Zero(12*10);
        X_cur = Eigen::VectorXd::Zero(12);
        X_cal = Eigen::VectorXd::Zero(12);
        dX_cal = Eigen::VectorXd::Zero(12);
        fe_react_tau_cmd = Eigen::VectorXd::Zero(13*3);
        des_ddq = Eigen::VectorXd::Zero(model_nv);
        des_dq = Eigen::VectorXd::Zero(model_nv);
        des_delta_q = Eigen::VectorXd::Zero(model_nv);
    };

    // update q according to sensor values, must update sensor values before
    void updateQ(){
    }


    static void printdq(const Eigen::VectorXd &q){
        std::cout<<std::setprecision(5)<<q.block<6,1>(0,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<7,1>(6,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<7,1>(13,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<4,1>(20,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<6,1>(24,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<6,1>(30,0).transpose()<<std::endl;
    }

    static void printq(const Eigen::VectorXd &q){
        std::cout<<std::setprecision(5)<<q.block<7,1>(0,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<7,1>(7,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<7,1>(14,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<4,1>(21,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<6,1>(25,0).transpose()<<std::endl;
        std::cout<<std::setprecision(5)<<q.block<6,1>(31,0).transpose()<<std::endl;
    }

    Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw) {
        Eigen::Matrix<double,3,3> Rx,Ry,Rz;
        Rz<<cos(yaw),-sin(yaw),0,
                sin(yaw),cos(yaw),0,
                0,0,1;
        Ry<<cos(pitch),0,sin(pitch),
                0,1,0,
                -sin(pitch),0,cos(pitch);
        Rx<<1,0,0,
                0,cos(roll),-sin(roll),
                0,sin(roll),cos(roll);
        return Rz*Ry*Rx;
    }

    Eigen::Quaterniond eul2quat(double roll, double pitch, double yaw) {
        Eigen::Matrix3d R= eul2Rot(roll,pitch,yaw);
        Eigen::Quaternion<double> quatCur;
        quatCur = R; //rotation matrix converted to quaternion
        Eigen::Quaterniond resQuat;
        resQuat=quatCur;
        return resQuat;
    }

};