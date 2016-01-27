#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "stm32f10x.h"
#include "StepMotor.h"

class Gimbal
{
	private:
		StepMotor &_motor_roll;
		StepMotor &_motor_pitch;
	public:
		Gimbal(StepMotor &motorRoll,StepMotor &motorPitch);
		void SetSpeed(int vRoll,int vPitch);
};



#endif

