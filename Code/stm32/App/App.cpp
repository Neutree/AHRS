#include "App.h"

/**
 * Constructor
 */ 
App::App()
:mCom1(1,115200,true),mGPIOledGreen(GPIOB,6,GPIO_Mode_Out_PP,GPIO_Speed_50MHz),mGPIOledRed(GPIOB,7,GPIO_Mode_Out_PP,GPIO_Speed_50MHz),
mLedGreen(mGPIOledGreen,false),mLedRed(mGPIOledRed,false),
mI2C2(2),
mMPU6050(mI2C2),
mMag(mI2C2)
{
	
}

/**
 * Initialize hardware
 */
void App::HardwareInit()
{
	mLedGreen.Off();
	mLedRed.Off();
	mLedGreen.Blink3(mLedRed,5,100);
	if(!mMPU6050.Init())
		mCom1<<"mpu init error\n";
	if(!mMag.Init())
		mCom1<<"mag init error\n";

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
	static uint16_t count=0;
	static Vector3<double> angle;
	mLedGreen.Toggle();
	if(MOD_ERROR==mMPU6050.Update())
		mCom1<<"mpu Update Error!\r\n";
	if(MOD_ERROR==mMag.Update())
		mCom1<<"mag Update Error!\r\n";
//	mCom1<<mMPU6050.GetAccRaw().x<<"\t"<<mMPU6050.GetAccRaw().y<<"\t"<<mMPU6050.GetAccRaw().z<<"\t";
//	mCom1<<mMPU6050.GetGyrRaw().x<<"\t"<<mMPU6050.GetGyrRaw().y<<"\t"<<mMPU6050.GetGyrRaw().z<<"\t";


	Vector3<int> acc=mMPU6050.GetAccRaw();
	Vector3<int> gyr=mMPU6050.GetGyrRaw();
//	mCom1<<acc.x<<"\t"<<acc.y<<"\t"<<acc.z<<"\t";
//	mCom1<<gyr.x<<"\t"<<gyr.y<<"\t"<<gyr.z<<"\r\n";
	angle = AHRS::GetAngle(acc,gyr,mMag.GetDataRaw());
	if(++count>100)
	{
		mCom1<<angle.x<<"\t"<<angle.y<<"\t"<<angle.z<<"\r\n";
		count=0;
	}
	
	TaskManager::DelayMs(2);
}

