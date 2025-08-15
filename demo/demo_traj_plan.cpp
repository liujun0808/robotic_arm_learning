#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callback.h"
#include "data_bus.h"
#include "MJ_interface.h"
#include "pino_kin_dyn.h"
#include <string>
#include <iostream>
#include "pvt_ctrl.h"
#include "useful_math.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/Goal.h"
#include <ompl/config.h>
#include <Eigen/Dense>
#include "planner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

const double dt = 0.001;
char error[1000] = "Could  not load binary model";
// const char* MODEL_XML = "../model/iiwa_description/urdf/scene.xml"; 
const char* MODEL_XML = "/home/lj/project/force_control/model/iiwa_description/urdf/scene.xml"; 

void printWorldPose(const mjModel* model, const mjData* data,
                       const std::string& body)
{
    // int body_id = mj_name2id(model, mjOBJ_JOINT, body.c_str());
    int site_id = mj_name2id(model, mjOBJ_BODY, body.c_str());
    if (site_id == -1) {
        std::cerr << "Error: Body not found!" << std::endl;
        return;
    }

    const mjtNum* position = data->xpos + 3 * site_id;
     // 获取世界坐标系中的旋转矩阵（3x3）
    const mjtNum* rotation = data->xmat + 9 * site_id;

    // 打印位置信息
        printf("Position in World Frame:\n");
        printf("  X: %8.4f, Y: %8.4f, Z: %8.4f\n", 
            position[0], position[1], position[2]);
        
        // 打印旋转矩阵
        printf("Rotation Matrix (World Frame):\n");
        for (int row = 0; row < 3; row++) {
            printf("  [ ");
            for (int col = 0; col < 3; col++) {
                printf("% 8.5f ", rotation[3 * row + col]);
            }
            printf("]\n");
        }

}

void printWorldPose(const mjModel* model, const mjData* data,
                       const std::string& body,int site)
{
    // int body_id = mj_name2id(model, mjOBJ_JOINT, body.c_str());
    int body_id = mj_name2id(model, mjOBJ_SITE, body.c_str());
    if (body_id == -1) {
        std::cerr << "Error: Body not found!" << std::endl;
        return;
    }

    const mjtNum* position = data->site_xpos + 3 * body_id;
     // 获取世界坐标系中的旋转矩阵（3x3）
    const mjtNum* rotation = data->site_xmat + 9 * body_id;

    // 打印位置信息
        printf("tool0 Position in mujoco(base):");
        printf("%8.6f %8.6f %8.6f\n", 
            position[0], position[1], position[2]);
        
        // 打印旋转矩阵
        printf("Rotation Matrix in mujoco(base):\n");
        for (int row = 0; row < 3; row++) {
            printf("  [ ");
            for (int col = 0; col < 3; col++) {
                printf("% 8.5f ", rotation[3 * row + col]);
            }
            printf("]\n");
        }

}


int main(int argc, char **argv)
{   
    mjModel *mj_model = mj_loadXML(MODEL_XML,0,error,1000);
    mjData *mj_data = mj_makeData(mj_model);
    UIctr ui(mj_model,mj_data);
    MJ_Interface mj_interface(mj_model,mj_data);
    Pin_KinDyn KinDynSolve("/home/lj/project/force_control/model/iiwa_description/urdf/iiwa14.urdf");
    DataBus robotState(KinDynSolve.model_nv);
    pvt_ctrl pvtCtr(mj_model->opt.timestep,"/home/lj/project/force_control/control_algorithm/config/joint_ctrl_config.json");
    planner rrtConectPlanner;
   
    // -------------sim loop------------
    double simEndTime = 15.0;
    mjtNum simStart = mj_data->time;
    double simTime = mj_data->time;
    // init
    ui.iniGLFW();
    ui.enableTracking();
    ui.createWindow("Demo",false);

    // std::ofstream q_err("/home/lj/project/force_control/control_algorithm/data/q_err.txt",std::ios::trunc);
    std::ofstream q("/home/lj/project/force_control/control_algorithm/data/q.txt",std::ios::trunc);

    
    std::vector<double> motors_pos_des(7);
    std::vector<double> motors_vel_des(7);
    std::vector<double> motors_tau_des(7);
    Eigen::VectorXd tau_des(7),ddq_des(7),dq_des(7),q_des(7);
    tau_des.setZero();
    ddq_des.setZero();dq_des.setZero();q_des.setZero();
    motors_pos_des.assign(7,0.0);
    motors_vel_des.assign(7,0.0);
    motors_tau_des.assign(7,0.0);

    Eigen::Vector3d tool0_pos_des;
    // tool0_pos_des<<0.6649, 0.0027, 0.2755;
    tool0_pos_des<<0.4,0.4,0.3;
    Eigen::Matrix3d tool0_rot_des;
    // tool0_rot_des<<-0.7160 0.0024 -0.6981,
    //             -0.0001, 1.0000, 0.0072,
    //             0.6895, 0.0052, -0.7242;
    // tool0_rot_des<<-0.71595  ,0.00008 , 0.69815,
    //                 0.00239  ,0.99999 , 0.00233,
    //                 -0.69815  ,0.00333 ,-0.71595;
    tool0_rot_des = eul2Rot(0.0,3.1415,0.0);
    auto res = KinDynSolve.computeIK(tool0_rot_des,tool0_pos_des);

    std::cout<<"逆解值"<<res.jointPoseRes.transpose()<<"\n";
    std::cout<<"逆解状态"<<res.status<<"\n";
    std::cout<<"逆解误差"<<res.err.transpose()<<"\n";

    std::vector<std::vector<double>> trajectory;
    pvtCtr.enablePV();
    int num = 0;
    bool plan_flag = false;
    Eigen::VectorXd jointPoseRes_(7);
    Eigen::VectorXd lastJointPoseRes_(7);
    jointPoseRes_.setZero();lastJointPoseRes_.setZero();
    while (!glfwWindowShouldClose(ui.window))
    {   
        simStart = mj_data->time;
        while (mj_data->time - simStart < 1.0/60.0 )
        {   
            mj_step(mj_model,mj_data);
            simTime = mj_data->time;
            
            mj_interface.updateSensorValues();
            // printWorldPose(mj_model,mj_data,"attachment_site",0);
            // std::cout<<"----------------------"<<std::endl;
            ui.record_trajectory_site(mj_model,mj_data,"attachment_site");
            mj_interface.dataBusWrite(robotState);// 写入关节数据实时值
            for (size_t i = 0; i < 7; i++)
            {
                q<<robotState.motors_pos_cur[i];
                q<<",";
            }
            q<<"\n";
            if (simTime<=3)
            {
                robotState.motors_pos_des = {0.0, 0, 0.0, 0, 0.0, 0.0, 0.0};
                robotState.motors_vel_des = {0.0, 0, 0.0, 0, 0.0, 0.0, 0.0};
                robotState.motors_tor_des = {0.0, 0, 0.0, 0, 0.0, 0.0, 0.0};
            }
            if (simTime>3 )
            {   
                if (!plan_flag)
                {
                    Eigen::VectorXd startJoints = std2eigen(robotState.motors_pos_cur);
                    bool flag = rrtConectPlanner.plan(startJoints,res.jointPoseRes);
                    if (flag)
                    {   plan_flag = true;
                        trajectory = rrtConectPlanner.trajectory;
                        // std::cout<<"trajectory点数"<<trajectory.size()<<"\n";
                        // for (size_t i = 999; i < trajectory.size(); i++)
                        // {   
                        //     for (size_t j = 0; j < 7; j++)
                        //     {
                        //     std::cout<<trajectory[i][j]<<" ";
                        //         /* code */
                        //     }
                        //     std::cout<<"\n";
                        // }
                    }else
                    {
                        std::cout<<"轨迹规划失败！"<<"\n";
                        break;
                    }
                }

                for (size_t i = 0; i < 7; i++)
                {
                    jointPoseRes_[i] = trajectory[num][i];
                }
                // std::cout<<"8"<<"\n";
                // if (isPositionReached(robotState.q,lastJointPoseRes_))
                // {
                //     robotState.motors_pos_des = eigen2std(jointPoseRes_);
                //     tau_des = KinDynSolve.computeForwordTau(jointPoseRes_,dq_des,ddq_des); 
                // }else{
                //     robotState.motors_pos_des = eigen2std(lastJointPoseRes_);
                //     tau_des = KinDynSolve.computeForwordTau(lastJointPoseRes_,dq_des,ddq_des); 
                // }
                
                robotState.motors_pos_des = eigen2std(jointPoseRes_);
                tau_des = KinDynSolve.computeForwordTau(jointPoseRes_,dq_des,ddq_des); 
                robotState.motors_vel_des = {0.0, 0, 0.0, 0, 0.0, 0.0, 0.0};
                robotState.motors_tor_des = eigen2std(tau_des);
                if (num <trajectory.size()-1)
                {
                    num++;
                }
                lastJointPoseRes_ = jointPoseRes_;
            }
            
            pvtCtr.dataBusRead(robotState); // pvt读取期望值与实时值
            if (simTime<2)
            {
                pvtCtr.calMotorsPVT(1.0/1000.0/180.0*3.1415);// 在每个控制周期内，每个关节的期望位置变化最大不能超过 0.1°（角度），也就是约 0.001745 弧度。30/1000.0/180.0*3.1415
            }else
            {
                pvtCtr.calMotorsPVT(30/1000.0/180.0*3.1415);
            }
            pvtCtr.dataBusWrite(robotState); // 将计算得到的转矩写入数据总线
            mj_interface.setMotorsTorque(robotState.motors_tor_out);
            
        }
        if (mj_data->time>=simEndTime)
        {   
            std::cout<<"----------------------"<<std::endl;
            printWorldPose(mj_model,mj_data,"attachment_site",0);
            // printWorldPose(mj_model,mj_data,"link7");
            std::cout<<"----------------------"<<std::endl;
            KinDynSolve.print_tool0_pos();
            break;
        }
        ui.updateScene();

        
    }
    ui.Close();
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    q.close();
    
    return 0;
}
// link7 in World Frame == base :
// Position in World Frame:
//   X:   0.6353, Y:   0.0011, Z:   0.3113
// Rotation Matrix (World Frame):
//   [ -0.71595  0.00008  0.69815 ]
//   [  0.00239  0.99999  0.00233 ]
//   [ -0.69815  0.00333 -0.71595 ]


// joint7 Position in World Frame:
//   X:   0.4280, Y:   0.0004, Z:   0.5233
// Rotation Matrix (World Frame):
//   [  0.71445 -0.00008  0.69969 ]
//   [ -0.00239 -0.99999  0.00232 ]
//   [  0.69968 -0.00333 -0.71445 ]

// site in base
// Site position (world frame): [0.6649, 0.0027, 0.2755]
// Site rotation matrix (world frame):
// [-0.7242 0.0049 -0.6895]
// [-0.0001 1.0000 0.0072]
// [0.6895 0.0052 -0.7242]

// Site position (world frame): [0.6667, 0.0012, 0.2791]
// Site rotation matrix (world frame):
// [-0.7160 0.0024 -0.6981]
// [0.0001 1.0000 0.0033]
// [0.6982 0.0023 -0.7159]

// X:   0.6353, Y:   0.0011, Z:   0.3113
// 0.637102 2.72558e-17    0.316865





