#ifndef _STEP_MOTOR_H_
#define _STEP_MOTOR_H_

#include "stm32f10x.h"
#include "math.h"
#include "PWM.h"

class StepMotor
{
	private:
		static bool bInitSPWM;
		static float SPWM[600];
		float _power;
		PWM *_pwm_a;
		PWM *_pwm_b;
		PWM *_pwm_c;
		u8 _ch_a;
		u8 _ch_b;
		u8 _ch_c;
	private:
		static void InitSPWM();
	public:
		StepMotor(PWM *pwma, u8 cha, PWM *pwmb, u8 chb, PWM *pwmc, u8 chc, float power);
		void Initialize(PWM *pwma, u8 cha, PWM *pwmb, u8 chb, PWM *pwmc, u8 chc, float power);
		void SetSpeed(int speed);
};

#endif
