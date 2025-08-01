#include "MJ_interface.h"

MJ_Interface::MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn) {
    mj_model=mj_modelIn;
    mj_data=mj_dataIn;
    timeStep=mj_model->opt.timestep;
    jointNum=JointName.size();
    jntId_qpos.assign(jointNum,0);
    jntId_qvel.assign(jointNum,0);
    jntId_dctl.assign(jointNum,0);
    motor_pos.assign(jointNum,0);
    motor_vel.assign(jointNum,0);
    motor_tau.assign(jointNum,0);
    motor_pos_Old.assign(jointNum,0);
    for (int i=0;i<jointNum;i++)
    {
        int tmpId= mj_name2id(mj_model,mjOBJ_JOINT,JointName[i].c_str());
        if (tmpId==-1)
        {
            std::cerr <<JointName[i]<< " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_qpos[i]=mj_model->jnt_qposadr[tmpId]; // 获取关节索引位置
        jntId_qvel[i]=mj_model->jnt_dofadr[tmpId];
        
        tmpId= mj_name2id(mj_model,mjOBJ_ACTUATOR,motorName[i].c_str());
        if (tmpId==-1)
        {
            std::cerr <<motorName[i]<< " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_dctl[i]=tmpId;
    }
}

void MJ_Interface::updateSensorValues() {
    for (int i=0;i<jointNum;i++){
        motor_pos_Old[i]=motor_pos[i];
        motor_pos[i]=mj_data->qpos[jntId_qpos[i]];
        motor_vel[i]=mj_data->qvel[jntId_qvel[i]];
        motor_tau[i]= mj_data->qfrc_actuator[jntId_dctl[i]];
    }

}


void MJ_Interface::dataBusWrite(DataBus &busIn) {
    busIn.motors_pos_cur=motor_pos;
    busIn.motors_vel_cur=motor_vel;
    busIn.motors_tor_cur = motor_tau;

}

void MJ_Interface::setMotorsTorque(std::vector<double> &tauIn) {
    for (int i=0;i<jointNum;i++){

        mj_data->ctrl[i]=tauIn.at(i);
        // const char* actuator_name = mj_id2name(mj_model, mjOBJ_ACTUATOR, i);
        // std::cout << "ctrl[" << i << "] -> " << actuator_name << std::endl;
    }
}