#ifndef __APP_H
#define __APP_H

#include "USART.h"
#include "Taskmanager.h"
#include "LED.h"
#include "I2C.h"
#include "mpu6050.h"
#include "AHRS.h"
#include "HMC5883L.h"

class App
{
	
private:
	USART mCom1;
	GPIO mGPIOledGreen;
	GPIO mGPIOledRed;
	LED mLedGreen;
	LED mLedRed;
	
	I2C mI2C2;
	mpu6050 mMPU6050;
	HMC5883L mMag;
	
public:
	App();
	void Loop();
	void HardwareInit();
	void SoftwareInit();
};


#endif

