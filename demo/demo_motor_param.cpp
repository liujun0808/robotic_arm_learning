#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callback.h"
#include "data_bus.h"
#include "MJ_interface.h"
#include "pino_kin_dyn.h"
#include <string>
#include <iostream>
#include "pvt_ctrl.h"
// #include "useful_math.h"


const double dt = 0.001;
char error[1000] = "Could  not load binary model";
const char* MODEL_XML = "../model/kuka_iiwa_14/scene.xml";


int main(int argc, char **argv)
{
    mjModel *mj_model = mj_loadXML(MODEL_XML,0,error,1000);
    mjData *mj_data = mj_makeData(mj_model);

    UIctr ui(mj_model,mj_data);
    MJ_Interface mj_interface(mj_model,mj_data);
    Pin_KinDyn KinDynSolve("/home/lj/project/force_control/model/kuka_iiwa_14/iiwa14.urdf");
    DataBus robotState(KinDynSolve.model_nv);
    pvt_ctrl pvtCtr(mj_model->opt.timestep,"/home/lj/project/force_control/control_algorithm/config/joint_ctrl_config.json");
    // -------------sim loop------------
    double simEndTime = 30.0;
    mjtNum simStart = mj_data->time;
    double simTime = mj_data->time;

    // init
    ui.iniGLFW();
    ui.enableTracking();
    ui.createWindow("Demo",false);

    std::ofstream q_err("/home/lj/project/force_control/control_algorithm/data/q_err.txt",std::ios::trunc);
    
    // data set
    // int key_id = mj_name2id(mj_model, mjOBJ_KEY, "home");
    // if (key_id == -1) {
    //     std::cerr << "Keyframe 'home' not found!" << std::endl;
    //     std::terminate();
    // }
    std::vector<double> motors_pos_des(7);
    std::vector<double> motors_vel_des(7);
    std::vector<double> motors_tau_des(7);

    motors_pos_des.assign(7,0.0);
    motors_vel_des.assign(7,0.0);
    motors_tau_des.assign(7,0.0);

    while (!glfwWindowShouldClose(ui.window))
    {   
        simStart = mj_data->time;
        while (mj_data->time - simStart < 1.0/60.0 )
        {   
            mj_step(mj_model,mj_data);
            simTime = mj_data->time;
            
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(robotState);// 写入关节数据实时值
            robotState.motors_pos_des = {0.0, 0.785398, 0.0, -1.5708, 0.0, 0.0, 0.0};
            robotState.motors_vel_des = motors_vel_des;
            robotState.motors_tor_des = motors_tau_des;
            // if (simTime>3)
            // {
            //     robotState.motors_pos_des[6] = 1.0;
            // }

            for (size_t i = 0; i < 7; i++)
            {
                q_err<<(robotState.motors_pos_des[i] -robotState.motors_pos_cur[i]);
                if (i != robotState.motors_pos_cur.size() - 1)
                {
                q_err << ",";
                }
            }
            q_err<<std::endl;

            pvtCtr.dataBusRead(robotState); // pvt读取期望值与实时值
            if (simTime<3)
            {
                pvtCtr.calMotorsPVT(1.0/1000.0/180.0*3.1415);// 在每个控制周期内，每个关节的期望位置变化最大不能超过 0.1°（角度），也就是约 0.001745 弧度。
            }else
            {
                pvtCtr.calMotorsPVT(70.0/1000.0/180.0*3.1415);
            }
            // pvtCtr.calMotorsPVT();
            pvtCtr.dataBusWrite(robotState); // 将计算得到的转矩写入数据总线
            mj_interface.setMotorsTorque(robotState.motors_tor_out);
            
        }
        if (mj_data->time>=simEndTime)
        {
            break;
        }
        ui.updateScene();

        
    }
    ui.Close();
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    q_err.close();
    
    return 0;
}
