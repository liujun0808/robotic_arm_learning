#include "pino_kin_dyn.h"


Pin_KinDyn::Pin_KinDyn(std::string urdf_path){
    pinocchio::urdf::buildModel(urdf_path,arm);
    std::cout << "arm.nv:"<<arm.nv<<";model_biped.nq:"<<arm.nq<< std::endl;
    data_arm = pinocchio::Data(arm);
    model_nv = arm.nv;
    J_tool0 = Eigen::MatrixXd::Zero(6,7);
    dJ_tool0 = Eigen::MatrixXd::Zero(6,7);
    q.setZero();
    dq.setZero();
    ddq.setZero();
    tool0_pos_b.setZero();
    tool_R.Identity();
    
    // 动力学模型初始化
    dyn_M = Eigen::MatrixXd::Zero(model_nv,model_nv); // 质量矩阵
    dyn_M_inv = Eigen::MatrixXd::Zero(model_nv,model_nv); // 质量矩阵的逆
    dyn_C = Eigen::MatrixXd::Zero(model_nv,model_nv); // 科氏项
    dyn_G = Eigen::MatrixXd::Zero(model_nv,model_nv); // 重力项

    // 从urdf中获取关节id
    joint1 = arm.getJointId("iiwa_joint_1");
    joint2 = arm.getJointId("iiwa_joint_2");
    joint3 = arm.getJointId("iiwa_joint_3");
    joint4 = arm.getJointId("iiwa_joint_4");
    joint5 = arm.getJointId("iiwa_joint_5");
    joint6 = arm.getJointId("iiwa_joint_6"); 
    joint7 = arm.getJointId("iiwa_joint_7");
    link1 = arm.getFrameId("iiwa_link_1");
    link2 = arm.getFrameId("iiwa_link_2");
    link3 = arm.getFrameId("iiwa_link_3");
    link4 = arm.getFrameId("iiwa_link_4");
    link5 = arm.getFrameId("iiwa_link_5");
    link6 = arm.getFrameId("iiwa_link_6");
    link7 = arm.getFrameId("iiwa_link_7");
    ee_link = arm.getFrameId("iiwa_link_ee");

    // joint1 = arm.getJointId("joint1");
    // joint2 = arm.getJointId("joint2");
    // joint3 = arm.getJointId("joint3");
    // joint4 = arm.getJointId("joint4");
    // joint5 = arm.getJointId("joint5");
    // joint6 = arm.getJointId("joint6"); 
    // joint7 = arm.getJointId("joint7");
    // link1 = arm.getFrameId("link1");
    // link2 = arm.getFrameId("link2");
    // link3 = arm.getFrameId("link3");
    // link4 = arm.getFrameId("link4");
    // link5 = arm.getFrameId("link5");
    // link6 = arm.getFrameId("link6");
    // link7 = arm.getFrameId("link7");

    // 使用 JSON 解析器读取一个包含关节参数（如最大力矩、限位等）的配置文件。
    Json::Reader reader; // 定义了一个 JSON 读取器 reader 和一个用于存储读取结果的 JSON 对象 root_read
    Json::Value root_read;
    std::ifstream in("/home/lj/project/force_control/control_algorithm/config/joint_ctrl_config.json",std::ios::binary); // 以二进制读取方式
    // std::ifstream in("/home/lj/humanrobot/OpenLoong-Dyn-Control/common/joint_ctrl_config.json",std::ios::binary); // 以二进制读取方式
    reader.parse(in,root_read); // 读取并保存至root_read

    motorMaxTorque = Eigen::VectorXd::Zero(motorName.size());
    motorMaxPos = Eigen::VectorXd::Zero(motorName.size());
    motorMinPos = Eigen::VectorXd::Zero(motorName.size());

    for (size_t i = 0; i < motorName.size(); i++)
    {
        motorMaxTorque(i) = root_read[motorName[i]]["maxTorque"].asDouble();
        motorMaxPos(i) = root_read[motorName[i]]["maxPos"].asDouble();
        motorMinPos(i) = root_read[motorName[i]]["minPos"].asDouble();
    }
    motorReachLimit.assign(motorName.size(),false); // 初始化电机“是否到达限制”标志数组，全部设置为 false
    tauJointOld = Eigen::VectorXd::Zero(motorName.size()); // 初始化上一周期的关节力矩
}
void  Pin_KinDyn::dataRead(const DataBus  &robotState){
    q = robotState.q;
    dq = robotState.dq; 
    dq.block(0,0,3,1) = robotState.tool0_rot_W.transpose() * dq.block(0,0,3,1); // 线速度 mujoco 接口得到的数据为世界坐标系下的速度,需转为本体下的速度
    dq.block(3,0,3,1) = robotState.tool0_rot_W.transpose() * dq.block(3,0,3,1); // 角速度
    ddq = robotState.ddq; // 加速度
}
void Pin_KinDyn::dataWrite(DataBus &robotState){
    robotState.J_tool0 = J_tool0;
    robotState.dJ_tool0=dJ_tool0;
    robotState.tool0_pos_b=tool0_pos_b;
    robotState.tool_R=tool_R;

    robotState.dyn_M=dyn_M;
    robotState.dyn_M_inv=dyn_M_inv;
    robotState.dyn_C=dyn_C;
    robotState.dyn_G=dyn_G;
    robotState.dyn_Non=dyn_Non;
}

void Pin_KinDyn::computeDyn(){
    // M
    pinocchio::crba(arm,data_arm,q);
    data_arm.M.triangularView<Eigen::Lower>() = data_arm.M.transpose().triangularView<Eigen::Lower>();  // 补齐M的下三角
    dyn_M = data_arm.M;
    std::cout<<data_arm.Minv<<std::endl;
    std::cout<<"-------------"<<std::endl;

    // M_inv
    pinocchio::computeMinverse(arm,data_arm,q);
    data_arm.Minv.triangularView<Eigen::Lower>() = data_arm.Minv.transpose().triangularView<Eigen::Lower>();  // 补齐M的下三角
    dyn_M_inv = data_arm.Minv;
    std::cout<<dyn_M_inv<<std::endl;

    // C 计算科氏力
    pinocchio::computeCoriolisMatrix(arm,data_arm,q,dq);
    dyn_C = data_arm.C;

    // G
    pinocchio::computeGeneralizedGravity(arm,data_arm,q);
    dyn_G = data_arm.g;

    dyn_Non = dyn_C *dq + dyn_G;
}
// 逆动力学 计算前馈力矩
Eigen::VectorXd Pin_KinDyn::computeForwordTau(Eigen::VectorXd &q_des,Eigen::VectorXd &dq_des,Eigen::VectorXd &ddq_des){
    Eigen::VectorXd forwordTau = pinocchio::rnea(arm,data_arm,q_des,dq_des,ddq_des);
    return forwordTau;
}


Pin_KinDyn::IkRes Pin_KinDyn::computeIK(const Eigen::Matrix3d &tool_R_res,const Eigen::Vector3d &tool0_pos_res)
{   
    // Eigen::Vector3d tool0_pos_res_;
    // Eigen::Vector3d tool_joint(0.0,0.0,0.045);
    // tool0_pos_res_ = tool_R_res * tool_joint + tool0_pos_res;
    // tool0_pos_res_ = tool0_pos_res;
    //  tool0_pos_res_[2] = tool0_pos_res[2] - 0.1575; 
    const pinocchio::SE3 oMdesL(tool_R_res,tool0_pos_res); // 在base 坐标系下tool0位姿，齐次变换矩阵转为 特殊欧式群
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(7);
    qIk<< 0,0.785398, 0 ,-1.5708 ,0, 0 ,0;
    // qIk<< arm.lowerPositionLimit[0],arm.lowerPositionLimit[1],
    //  arm.lowerPositionLimit[2], arm.lowerPositionLimit[3], arm.lowerPositionLimit[4], arm.lowerPositionLimit[5], arm.lowerPositionLimit[7];
    // pinocchio::forwardKinematics(arm,data_arm,qIk);
    // pinocchio::updateFramePlacements(arm,data_arm);
    // pinocchio::SE3 test_q = data_arm.oMi[joint7];
    // pinocchio::FrameIndex link_set[8] = { link1,link2,link3,link4,link5,link6,link7,ee_link};
    // pinocchio::JointIndex joint_set[7] = { joint1,joint2,joint3,joint4,joint5,joint6,joint7};
    // for (size_t i = 0; i < 8; i++)
    // {
    //     pinocchio::SE3 test_q = data_arm.oMf[link_set[i]];
    //     std::cout <<"link id in urdf  "<<link_set[i]<< "  Translation: " << test_q.translation().transpose() << std::endl;
    //     /* code */
    // }
    
    // 迭代参数设置
    const double eps = 1e-4;
    const int IT_MAX  = 1000;
    const double DT   = 1;
    const double damp = 1e-6;

    pinocchio::Data::Matrix6x J(6,7);
    J.setZero();
    bool success = false;
    Eigen::Matrix<double,6,1> err;
    Eigen::VectorXd q_delta(model_nv);

    int itr_count{0};
    for (itr_count = 0;; itr_count++)
    {
        pinocchio::forwardKinematics(arm,data_arm,qIk);
        pinocchio::updateFramePlacements(arm,data_arm);
        // pinocchio::SE3 test_q = data_arm.oMf[ee_link];
        // std::cout <<"ee_link id in urdf"<<ee_link<< "Translation: " << test_q.translation().transpose() << std::endl;
        const pinocchio::SE3 iMd = data_arm.oMf[ee_link].actInv(oMdesL); // link7在base（world与base重合）下的位姿求逆后右乘期望点位姿 得到当前与期望位姿间的误差位姿
        err = pinocchio::log6(iMd).toVector();//SE(3)->se(3)  李群 对数映射 至李代数   将位姿误差转为twist 空间向量 【线速度， 角速度】T
        if (err.norm()<=eps)
        {
            success = true;
            break;
        }
        if (itr_count>= IT_MAX)
        {
            success = false;
            break;
        }
        pinocchio::computeJointJacobian(arm,data_arm,qIk,joint7,J);
        pinocchio::Data::Matrix6 Jlog;

        pinocchio::Jlog6(iMd.inverse(),Jlog); // err = delta_v = jlog * delta_q  ;Jlog6即求jlog（对oMdesL.inverse()求导）

        J = -Jlog * J; // J * delta_q 得到的delta_v并非误差空间 是末端的delta_v  ，通过左乘jlog 转到误差空间的delta_v  用于后续迭代计算
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        q_delta.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        qIk = pinocchio::integrate(arm, qIk, q_delta*DT);
         for (int j = 0; j < 7; ++j) {
            // 方法1：硬截断
            qIk[j] = clamp(qIk[j],arm.lowerPositionLimit[j],arm.upperPositionLimit[j]);
            
            // 方法2：带惩罚的软限位（更平滑）
            // if (qIk[j] < arm.lowerPositionLimit[j] + 0.1) {
            //     qIk[j] = arm.lowerPositionLimit[j] + 0.1 * (1 - std::exp(-10*(arm.lowerPositionLimit[j] + 0.1 - qIk[j])));
            // }
            // if (qIk[j] > arm.upperPositionLimit[j] - 0.1) {
            //     qIk[j] = arm.upperPositionLimit[j] - 0.1 * (1 - std::exp(-10*(qIk[j] - (arm.upperPositionLimit[j] - 0.1))));
            // }
        }
    }
    IkRes res;
    res.err = err;
    res.itr = itr_count;
    if (success)
    {
        res.status = 0;
    }else{
        res.status = -1;
    }
    res.jointPoseRes = qIk;
    return res;
}

void Pin_KinDyn::print_tool0_pos(){
    pinocchio::SE3 test_q = data_arm.oMf[ee_link];
    std::cout <<"tool0 Position in Pinocchio(base):" << test_q.translation().transpose() << std::endl;
    // 打印旋转矩阵
    Eigen::Matrix3d rot = test_q.rotation();
    printf("Rotation Matrix in Pinocchio(base):\n");
    for (int row = 0; row < 3; row++) {
        printf("  [ ");
        for (int col = 0; col < 3; col++) {
            printf("% 8.5f ", rot(row,col));
        }
        printf("]\n");
    }
    std::cout<<"----------------------"<<std::endl;
}
