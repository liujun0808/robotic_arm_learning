#pragma once
// 一阶低通滤波 连续系统模型：tau*y_dot+y=x 

class lpf_fst
{
private:
    double alpha{0};
    double dataOld{0};
    bool isIni{false};
public:
    lpf_fst();
    lpf_fst(double fc, double Ts);
    void setPara(double fc, double Ts);
    double ftOut(double dataIn);
};
