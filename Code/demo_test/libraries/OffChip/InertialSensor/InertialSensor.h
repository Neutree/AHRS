#ifndef _INERTIAL_SENSOR_H_
#define _INERTIAL_SENSOR_H_

#include "stm32f10x.h"
#include "Mathtool.h"
#include "Sensor.h"

class InertialSensor : public Sensor
{
	protected:
		Vector3f _acc_offset;  //acc offset
		Vector3f _gyro_offset; //gyro offset
	public:
		virtual bool Initialize(void) = 0;
		virtual bool Update(Vector3f &acc, Vector3f &gyro)=0;
};


#endif

