#include "pvt_ctrl.h"
// timeStepIn 控制周期
pvt_ctrl::pvt_ctrl(double timeStepIn, const char *jsonPath){
    jointNum = motorName.size();

    tau_out_lpf.assign(jointNum,lpf_fst());
    motor_vel.assign(jointNum,0);
    motor_pos_cur.assign(jointNum,0);
    motor_pos_des_old.assign(jointNum,0);
    motor_tor_out.assign(jointNum,0);
    pvt_Kp.assign(jointNum,0);
    pvt_Kd.assign(jointNum,0);
    maxTor.assign(jointNum,400);
    maxVel.assign(jointNum,50);
    maxPos.assign(jointNum,3.14);
    minPos.assign(jointNum,-3.14);
    PV_enable.assign(jointNum,1);

    // 读取个关节PVT控制参数
    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in(jsonPath,std::ios::binary);
    reader.parse(in,root_read);
    for (int i=0;i<jointNum;i++){
    pvt_Kp[i]=root_read[motorName[i]]["kp"].asDouble();
    pvt_Kd[i]=root_read[motorName[i]]["kd"].asDouble();
    // pvt_Kp[i]=;
    // pvt_Kd[i]=20;
    maxTor[i]=root_read[motorName[i]]["maxTorque"].asDouble();
    maxVel[i]=root_read[motorName[i]]["maxSpeed"].asDouble();
    maxPos[i]=root_read[motorName[i]]["maxPos"].asDouble();
    minPos[i]=root_read[motorName[i]]["minPos"].asDouble();
    double fc=root_read[motorName[i]]["PVT_LPF_Fc"].asDouble();
    tau_out_lpf[i].setPara(fc, timeStepIn);
    tau_out_lpf[i].ftOut(0);
    }
}
// 读取期望值与实时值
void pvt_ctrl::dataBusRead(DataBus &busIn){
    for (int i = 0; i < jointNum; i++)
    {
        motor_pos_cur[i]=busIn.motors_pos_cur[i];
        motor_vel[i]=busIn.motors_vel_cur[i];
    }
    motor_pos_des=busIn.motors_pos_des;
    motor_vel_des=busIn.motors_vel_des;
    motor_tor_des=busIn.motors_tor_des;
}

void pvt_ctrl::dataBusWrite(DataBus &busIn) {
    busIn.motors_tor_out=motor_tor_out;
    busIn.motors_tor_cur=motor_tor_out;
}

double pvt_ctrl::sign(double in) {
    if (in>=0)
        return 1.0;
    else
        return -1.0;
}

void pvt_ctrl::setJointPD(double kp, double kd, const char *jointName){
    auto it = std::find(motorName.begin(),motorName.end(),jointName); // 指向 jointName 的迭代器it
    int id=-1;
    if (it != motorName.end()) {
        id = std::distance(motorName.begin(), it); // 返回该关节的索引下标
    } else {
        std::cout << jointName << " NOT found!" << std::endl;
    }
    pvt_Kp[id]=kp;
    pvt_Kd[id]=kd;
}

// MIT 控制模式 tau_new = tau + kp*(q_des-q) + kd*(q_dot_des -q_dot ) 
void pvt_ctrl::calMotorsPVT(){ 
    for (int i = 0; i < jointNum; i++)
    {
        double tauDes{0};
        tauDes = PV_enable[i] * pvt_Kp[i]*(motor_pos_des[i]-motor_pos_cur[i]) + PV_enable[i]*pvt_Kd[i]*(motor_vel_des[i]-motor_vel[i]);
        // 对速度 位置项进行滤波后 在加力矩前馈
        tauDes = tau_out_lpf[i].ftOut(tauDes) + motor_tor_des[i];
        if (fabs(tauDes)>=fabs(maxTor[i]))
        {
            tauDes = sign(tauDes)*maxTor[i]; // 限制力矩范围 但保持方向不变
        }
        motor_tor_out[i] = tauDes;
        motor_pos_des_old[i] = motor_pos_des[i];
    }
    
}

// 带有位置增量限制的PVT控制模式  即速度限制
void pvt_ctrl::calMotorsPVT(double deltaP_Lim){
    for (int i = 0; i < jointNum; i++)
    {
        double tauDes{0};
        double delta = motor_pos_des[i] - motor_pos_des_old[i];
        if (fabs(delta)>= fabs(deltaP_Lim))
        {
            delta = deltaP_Lim * sign(delta); // 对位置增量进行限制 但保持其增量方向
        }
        double pDes = delta + motor_pos_des_old[i];
        tauDes = PV_enable[i] * pvt_Kp[i] * (pDes - motor_pos_cur[i]) + PV_enable[i] * pvt_Kd[i] * (motor_vel_des[i] - motor_vel[i]);
        tauDes = tau_out_lpf[i].ftOut(tauDes) + motor_tor_des[i];
        // tauDes += motor_tor_des[i];
        if (fabs(tauDes) >= fabs(maxTor[i]))
            tauDes = sign(tauDes)*maxTor[i]; // 限制力矩范围 但保持方向不变
        motor_tor_out[i] = tauDes;
        motor_pos_des_old[i] = pDes;
    }
    
}

// 使能所有或某个关节的PV控制项，默认构造函数时已全部使能
void pvt_ctrl::enablePV() {
    PV_enable.assign(jointNum,1);
}

void pvt_ctrl::enablePV(int jtId) {
    PV_enable[jtId]=1;
}

// Disable PV control item for all joints.
void pvt_ctrl::disablePV() {
    PV_enable.assign(jointNum,0);
}

void pvt_ctrl::disablePV(int jtId) {
    PV_enable[jtId]=0;
}

