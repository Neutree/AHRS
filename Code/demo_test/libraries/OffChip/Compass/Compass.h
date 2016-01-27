#ifndef _COMPASS_H_
#define _COMPASS_H_

#include "stm32f10x.h"
#include "Mathtool.h"
#include "Sensor.h"

class Compass : public Sensor
{
public:
	virtual bool Initialize()=0;
	virtual bool Update(Vector3f &mag)=0;
};


#endif

