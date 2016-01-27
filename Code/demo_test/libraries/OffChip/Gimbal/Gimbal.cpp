#include "Gimbal.h"

Gimbal::Gimbal(StepMotor &motorRoll,StepMotor &motorPitch):_motor_roll(motorRoll),_motor_pitch(motorPitch)
{
	
	
}

void Gimbal::SetSpeed(int vRoll,int vPitch)
{
	_motor_roll.SetSpeed(vRoll);
	_motor_pitch.SetSpeed(vPitch);
}

