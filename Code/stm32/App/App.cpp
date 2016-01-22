#include "App.h"

/**
 * Constructor
 */ 
App::App()
:mCom1(1,115200,true),mGPIOledGreen(GPIOB,6,GPIO_Mode_Out_PP,GPIO_Speed_50MHz),mGPIOledRed(GPIOB,7,GPIO_Mode_Out_PP,GPIO_Speed_50MHz),
mLedGreen(mGPIOledGreen,false),mLedRed(mGPIOledRed,false),
mI2C2(2),
mMPU6050(mI2C2)
{
	
}

/**
 * Initialize hardware
 */
void App::HardwareInit()
{
	mLedGreen.Off();
	mLedRed.Off();
	mLedGreen.Blink3(mLedRed,8,100);
	mMPU6050.Init();
}

/**
 * Initialize software
 */
void App::SoftwareInit()
{

}


/**
 * loop function 循环函数
 */
void App::Loop()
{
	static Vector3<double> angle;
	mLedGreen.Toggle();
	if(MOD_ERROR==mMPU6050.Update())
		mCom1<<"Update Error!\r\n";
//	mCom1<<mMPU6050.GetAccRaw().x<<"\t"<<mMPU6050.GetAccRaw().y<<"\t"<<mMPU6050.GetAccRaw().z<<"\t";
//	mCom1<<mMPU6050.GetGyrRaw().x<<"\t"<<mMPU6050.GetGyrRaw().y<<"\t"<<mMPU6050.GetGyrRaw().z<<"\t";

	angle = AHRS::GetAngle(mMPU6050.GetAccRaw(),mMPU6050.GetGyrRaw());
	mCom1<<angle.x<<"\t"<<angle.y<<"\t"<<angle.z<<"\r\n";
	
	TaskManager::DelayMs(2);
}

