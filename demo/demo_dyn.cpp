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

const double dt = 0.001;
char error[1000] = "Could  not load binary model";
const char* MODEL_XML = "../model/iiwa_description/urdf/scene.xml";

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

void printRelativePose(const mjModel* model, const mjData* data,
                       const std::string& baselink, const std::string& hand_link)
{
    int base_id = mj_name2id(model, mjOBJ_BODY, baselink.c_str());
    // int hand_id = mj_name2id(model, mjOBJ_JOINT, hand_link.c_str());
    // int base_id = mj_name2id(model, mjOBJ_BODY, baselink.c_str());
    int hand_id = mj_name2id(model, mjOBJ_BODY, hand_link.c_str());
    if (base_id == -1 || hand_id == -1) {
    std::cerr << "Error: Body names not found!" << std::endl;
    std::cout<<base_id<<"  "<<hand_id<<std::endl;
    return;
    }

    const mjtNum* base_pos = data->xpos + 3 * base_id;
    const mjtNum* base_rot = data->xmat + 9 * base_id;
    const mjtNum* hand_pos = data->xpos + 3 * hand_id;
    const mjtNum* hand_rot = data->xmat + 9 * hand_id;
     // 1. 计算相对位置
    mjtNum rel_pos[3];
    // 计算位置差：hand_pos - base_pos
    rel_pos[0] = hand_pos[0] - base_pos[0];
    rel_pos[1] = hand_pos[1] - base_pos[1];
    rel_pos[2] = hand_pos[2] - base_pos[2];

    // 2. 计算相对旋转矩阵
    mjtNum base_rot_inv[9];  // 基座旋转矩阵的逆（转置）
    mjtNum rel_rot[9];      // 相对旋转矩阵
    
    // 计算基座旋转矩阵的逆（转置）
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            base_rot_inv[3*i + j] = base_rot[3*j + i];
        }
    }
    // 计算相对旋转：rel_rot = base_rot_inv * hand_rot
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rel_rot[3*i + j] = 0;
            for (int k = 0; k < 3; k++) {
                rel_rot[3*i + j] += base_rot_inv[3*i + k] * hand_rot[3*k + j];
            }
        }
    }
    mjtNum pos_in_base[3];
    for (int i = 0; i < 3; i++) {
        pos_in_base[i] = 0;
        for (int j = 0; j < 3; j++) {
            pos_in_base[i] += base_rot_inv[3*i + j] * rel_pos[j];
        }
    }
    printf("Hand position relative to base:\n");
    printf("  X: %8.4f, Y: %8.4f, Z: %8.4f\n", 
           pos_in_base[0], pos_in_base[1], pos_in_base[2]);
    
    // 打印相对旋转矩阵（hand_link相对于baselink的姿态）
    printf("Hand rotation relative to base (3x3 matrix):\n");
    for (int i = 0; i < 3; i++) {
        printf("  ");
        for (int j = 0; j < 3; j++) {
            printf("% 8.5f ", rel_rot[3*i + j]);
        }
        printf("\n");
    }
    printf("----------------------------------\n");

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

    // -------------sim loop------------
    double simEndTime = 10.0;
    mjtNum simStart = mj_data->time;
    double simTime = mj_data->time;
    // init
    ui.iniGLFW();
    ui.enableTracking();
    ui.createWindow("Demo",false);

    // std::ofstream q_err("/home/lj/project/force_control/control_algorithm/data/q_err.txt",std::ios::trunc);
    
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
    tool0_pos_des<<0.6649, 0.0027, 0.2755;
    // tool0_pos_des<<0.6667,0.0,0.2791;
    Eigen::Matrix3d tool0_rot_des;
    // tool0_rot_des<<-0.7160 0.0024 -0.6981,
    //             -0.0001, 1.0000, 0.0072,
    //             0.6895, 0.0052, -0.7242;
    tool0_rot_des<<-0.71595  ,0.00008 , 0.69815,
                    0.00239  ,0.99999 , 0.00233,
                    -0.69815  ,0.00333 ,-0.71595;
    tool0_rot_des = eul2Rot(0.0,3.1415,0.0);
    auto res = KinDynSolve.computeIK(tool0_rot_des,tool0_pos_des);
    pvtCtr.enablePV();
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
            if (simTime>=2)
            {
                /* code */
                res = KinDynSolve.computeIK(tool0_rot_des,tool0_pos_des);
                tau_des = KinDynSolve.computeForwordTau(res.jointPoseRes,dq_des,ddq_des); 
                robotState.motors_pos_des = eigen2std(res.jointPoseRes);
                robotState.motors_tor_des = eigen2std(tau_des);
            }else{
            robotState.motors_pos_des = {0.0, 0, 0.0, 0, 0.0, 0.0, 0.0};
            }
            robotState.motors_vel_des = motors_vel_des;
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

            printWorldPose(mj_model,mj_data,"attachment_site",0);
            std::cout<<"----------------------"<<std::endl;
            KinDynSolve.print_tool0_pos();
            break;
        }
        ui.updateScene();

        
    }
    ui.Close();
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    // q_err.close();
    
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





