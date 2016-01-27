#include "StepMotor.h"

bool StepMotor::bInitSPWM = false;
float StepMotor::SPWM[600] = {0};


void StepMotor::InitSPWM()
{
	for(int i=0;i<600;i++)
	{
		SPWM[i] = sin(i*2*3.1415926/600.0);
	}
}

StepMotor::StepMotor(PWM *pwma, u8 cha, PWM *pwmb, u8 chb, PWM *pwmc, u8 chc, float power)
{
	_pwm_a = pwma;
	_pwm_b = pwmb;
	_pwm_c = pwmc;
	_ch_a = cha;
	_ch_b = chb;
	_ch_c = chc;
	_power = power;
	if(bInitSPWM) return;
	bInitSPWM = true;
	InitSPWM();
}

void StepMotor::Initialize(PWM *pwma, u8 cha, PWM *pwmb, u8 chb, PWM *pwmc, u8 chc, float power)
{
	_pwm_a = pwma;
	_pwm_b = pwmb;
	_pwm_c = pwmc;
	_ch_a = cha;
	_ch_b = chb;
	_ch_c = chc;
	_power = power;
}


void StepMotor::SetSpeed(int speed)
{
	static int pos = 0;
	
	float a,b,c;
	
	if(speed>100 || speed<-100) return;
	
	pos += speed;
	if(pos>=600)   pos -=600;
	else if(pos<0) pos +=600; 
	
	a = SPWM[pos];
	b = SPWM[(pos+200)%600];
	c = SPWM[(pos+400)%600];
	
	a = _power*a + 0.5; 
	b = _power*b + 0.5; 
	c = _power*c + 0.5; 
	_pwm_a->SetDuty(_ch_a,a*100);
	_pwm_b->SetDuty(_ch_b,b*100);
	_pwm_c->SetDuty(_ch_c,c*100);
}



