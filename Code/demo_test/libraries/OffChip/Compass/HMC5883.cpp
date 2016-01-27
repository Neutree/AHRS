#include "HMC5883.h"
#include "TaskManager.h"


HMC5883::HMC5883(I2C &i2c):mI2C(i2c)
{
	Initialize();
}

bool HMC5883::Initialize()
{
	mHealthy = false;                                  //set HMC5883 not healthy
	if(!mI2C.WaitFree(50))                             //wait for i2c work and free
		if(!mI2C.Initialize()) return false;             //if time out, reset and initialize i2c
	
	u8 configData[3][2] = {HMC5883_Config_RA, HMC5883L_AVERAGING_8|HMC5883L_RATE_75|HMC5883L_BIAS_NORMAL, //Config Register A  :number of samples averaged->8  Data Output rate->30Hz
	                       HMC5883_Config_RB, HMC5883L_GAIN_1090,                                         //Config Register B:  Gain Configuration as : Sensor Field Range->(+-)1.3Ga ; Gain->1090LSB/Gauss; Output Range->0xF800-0x07ff(-2048~2047)
		                     HMC5883_Mode,      HMC5883L_MODE_CONTINUOUS                                    //Config Mode as: Continous Measurement Mode
	                      };
	for(u8 i=0; i<3; i++)
  {
		bool isTaskTail = ((i==2) ? true : false);
		//mpu init cmd:    i2c addr      txdata[]       txNum  rxdata[] rxNum
		mI2C.AddCommand(HMC5883_ADDRESS, configData[i],   2,      0,      0,   this, isTaskTail);
	}
	mI2C.Start();  //start to rum i2c command	
	if(!mI2C.WaitFree(50)) return false;               //wait HMC5883 initialize complete, if time out, keep nuhealthy state
	mHealthy = true;                                   //initialize success
	return true;
}

bool HMC5883::Update(Vector3f &mag)
{
	if(!mI2C.IsHealthy())//if i2c not work correctly
	{
		mI2C.Initialize();    //initialize i2c
		Initialize();	        //initialize mpu6050
		return false;
	}
	if(mIsUpdated==false) 
	{
		if(tskmgr.Time()-mUpdatedTime > 1)
		{
			mI2C.Initialize();    //initialize i2c
			Initialize();	        //initialize HMC5883
			mIsUpdated = true;
		}
		return false;
	}
	mIsUpdated = false;
	u8 reg = HMC5883_XOUT_M;    //form mag x high register, read 6 bytes
	mI2C.AddCommand(HMC5883_ADDRESS, &reg, 1, mRawData, 6, this, true);
	mI2C.Start();  //start run i2c command
	
	//convert sensor data
	mag.x   = s16(mRawData[0]<<8 | mRawData[1]);
	mag.y   = s16(mRawData[2]<<8 | mRawData[3]);
	mag.z   = s16(mRawData[4]<<8 | mRawData[5]);

	return true;
}

