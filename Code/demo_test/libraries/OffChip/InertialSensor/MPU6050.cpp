#include "MPU6050.h"

//constructor
MPU6050::MPU6050(I2C &i2c):_i2c(i2c)
{
	Initialize();                      //initialize mpu6050
}

//initialze mpu6050
bool MPU6050::Initialize(void)
{
	mHealthy = false;                                  //set mpu6050 not healthy
	if(!_i2c.WaitFree(50))                             //wait for i2c work and free
		if(!_i2c.Initialize()) return false;             //if time out, reset and initialize i2c
	
	u8 configData[10][2] = {PWR_MGMT_1,      0x00,      //power management setting
	                       INT_PIN_CFG,     0x02,      //interrupt setting ?????
		                     SMPLRT_DIV,      0x07,      //sample rate of gyro
		                     USER_CTRL,       0x07,      //fifo, i2c master, etc.
		                     CONFIG,          0x06,      //low pass filter
		                     GYRO_CONFIG,     0x18,      //gyro range, selftest
		                     ACCEL_CONFIG,    0x10,      //acc range, selftest, high pass
												 I2C_MST_CTRL,    0x0D,      //config the mpu6050  master bus rate as 400kHz
		                     USER_CTRL,       0x00,      //Disable mpu6050 Master mode
		                     INT_PIN_CFG,     0x02       //Enable bypass mode 
	                      };
	for(u8 i=0; i<10; i++)
  {
		bool isTaskTail = ((i==9) ? true : false);
		//mpu init cmd:    i2c addr      txdata[]       txNum  rxdata[] rxNum
		_i2c.AddCommand(MPU6050_ADDRESS, configData[i],   2,      0,      0,   this, isTaskTail);
	}
	_i2c.Start();  //start to rum i2c command	
	
	if(!_i2c.WaitFree(50)) return false;               //wait mpu6050 initialize complete, if time out, keep nuhealthy state
	mHealthy = true;                                   //initialize success
	return true;
}



//update sensor data
bool MPU6050::Update(Vector3f &acc, Vector3f &gyro)
{
	if(!_i2c.IsHealthy())//if i2c not work correctly
	{
		_i2c.Initialize();    //initialize i2c
		Initialize();	        //initialize mpu6050
		return false;
	}
	if(mIsUpdated==false) 
	{
		if(tskmgr.Time()-mUpdatedTime > 1)
		{
			_i2c.Initialize();    //initialize i2c
			Initialize();	        //initialize mpu6050
			mIsUpdated = true;
		}
		return false;
	}
	mIsUpdated = false;
	u8 reg = ACCEL_XOUT_H;    //form acc x high register, read 14 bytes
	_i2c.AddCommand(MPU6050_ADDRESS, &reg, 1, _raw_data, 14, this, true);
	_i2c.Start();  //start run i2c command
	
	//convert sensor data
	acc.x   = s16(_raw_data[0]<<8 | _raw_data[1])/4096.0f;
	acc.y   = s16(_raw_data[2]<<8 | _raw_data[3])/4096.0f;
	acc.z   = s16(_raw_data[4]<<8 | _raw_data[5])/4096.0f;
	gyro.x  = s16(_raw_data[8]<<8 | _raw_data[9])*0.0010652644;
	gyro.y  = s16(_raw_data[10]<<8 | _raw_data[11])*0.0010652644;
	gyro.z  = s16(_raw_data[12]<<8 | _raw_data[13])*0.0010652644;
	return true;
}
//start to calibrate gyro
void MPU6050::StartCalibrateGyro()
{
	_is_calibrate_gyro = true;
	_gyro_calibrate_cnt = 0;
	_gyro_calibrate_sum.Zero();
}
//stop calibrate gyro
void MPU6050::StopCalibrateGyro()
{
	_is_calibrate_gyro = false;
}


