#ifndef _REMOTER_H_
#define _REMOTER_H_

#include "stm32f10x.h"
#include "TaskManager.h"

class Remoter
{
protected:
	float mRawT[10];
	float mMaxT[10];  //the max duration(ms) of channel x
  float mMinT[10];  //the min duration(ms) of channel x
  bool mLocked;     //true: remoter is locked, false: unlocked
  bool mCalibrating;//true: currently calibrate rc, false: not calibrate rc
	float mLockVal[4];
  float mUnlockVal[4];
public:
	Remoter()        //Constructor
	{
		for(u8 i=0;i<10;i++)
		{
			mMaxT[i] = 1.9f;
			mMinT[i] = 1.1f;
		}
		mLocked = true;
		mCalibrating = false;
		mLockVal[0] = 0;
		mLockVal[1] = 0;
		mLockVal[2] = 0;
		mLockVal[3] = 0;
		mUnlockVal[0] = 100;
		mUnlockVal[1] = 0;
		mUnlockVal[2] = 0;
		mUnlockVal[3] = 0;
	}
	virtual float operator[](u8 chNum) = 0; //return the percentage of channel x  (0.0 ~ 100.0), the same as Channel
	virtual float Channel(u8 chNum) = 0;    //return the percentage of channel x  (0.0 ~ 100.0), the same as operator[]
	virtual float ChannelRaw(u8 chNum) = 0; //return the raw value of channel x (ms)
	virtual u8 Update() = 0;
	bool IsLocked() { return mLocked;}      //return the locked status of remoter
	void SetLockState(float roll, float pitch, float yaw, float throttle)
	{
		mLockVal[0] = roll;
		mLockVal[1] = pitch;
		mLockVal[2] = yaw;
		mLockVal[3] = throttle;
	}
	void SetUnlockState(float roll, float pitch, float yaw, float throttle)
	{
		mUnlockVal[0] = roll;
		mUnlockVal[1] = pitch;
		mUnlockVal[2] = yaw;
		mUnlockVal[3] = throttle;
	}
	void StartCalibrate()
	{
		if(mLocked) mCalibrating = true;
		for(u8 i=0;i<10;i++)
		{
			mMaxT[i] = 1.5f;
			mMinT[i] = 1.5f;
		}
	}
	void StopCalibrate()
	{
		mCalibrating = false;
	}
	
	
};


#endif



