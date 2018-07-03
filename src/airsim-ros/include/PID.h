#include <math.h>
#include <iostream>

class PIDctrl
{
	public:
	double Kp;  //比例系数
	double Ki;   //积分系数
	double Kd;  //微分系数
	double preErr;    //前一拍误差
	double sumErr;    //误差累积
	double dErr;
	double output,pre_output;
	double outMax,outMin;//输出最大值
	bool stop;
	
	PIDctrl()
	{
		
		preErr=0.0;sumErr=0.0;
	};
	void init(double _Kp,double _Ki,double _Kd,double _max);
	double calc(double &curErr);
	void reset();
};

void PIDctrl::init(double _Kp, double _Ki, double _Kd, double _max)
{
    Kp=_Kp;
    Ki=_Ki;
    Kd=_Kd;
    outMax=_max;
    stop=false;
}


void PIDctrl::reset()
{
	preErr=0.0;
	sumErr=0.0; 
	dErr=0.0;
}

 

double PIDctrl::calc(double &curErr)
{
	sumErr += curErr;
	double Kisum = Ki*sumErr;
	if (Kisum > 1)
		Kisum = 1;
	if (Kisum < -1)
		Kisum = -1;

	dErr    = curErr - preErr;
	preErr = curErr;

	output = Kp*curErr + Kisum + Kd*dErr;

	double d_throttle = output;
	
	return output;
}
