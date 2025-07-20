#include "lpf_fst.h"

lpf_fst::lpf_fst() {
    alpha=0;
}

lpf_fst::lpf_fst(double fc, double Ts){
    alpha = Ts/(Ts+1/(2*3.1415*fc)); // Ts 控制周期， fc 截止评率； 其中时间常数tau = 1/（2*3.14*fc）
}

double lpf_fst::ftOut(double dataIn){
    double res{0};
    if (isIni)
    {
        res = (1-alpha)*dataOld + alpha*dataIn;
    }else
    {
        res = dataIn;
        isIni = true;
    }
    dataOld = res;
    return res;
}

void lpf_fst::setPara(double fc, double Ts){
        alpha=Ts/(Ts+1.0/(2*3.1415*fc));
}

