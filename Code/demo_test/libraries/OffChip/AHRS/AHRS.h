#ifndef _AHRS_H_
#define _AHRS_H_

#include "stm32f10x.h"
#include "InertialSensor.h"
#include "Compass.h"
#include "Barometer.h"
#include "GPS.h"

class AHRS
{
protected:
	InertialSensor &_ins;      //inertial sensor
	Compass *_compass;         //compass
	Barometer *_baro;          //barometer
	GPS *_gps;                 //GPS

	Vector3f _acc;             //3-aixs acceleration data
	Vector3f _gyro;            //3-aixs gyroscope data
	Vector3f _mag;             //3-aixs compass magnetic
	float _pressure;           //air pressure

	Vector3f _angle;      		 //roll, pitch, yaw
	Quaternion _q;             //quternion: q1,q2,q3,q4


	uint8_t _gps_min_satellite;//最少允许的卫星数量

public:
	AHRS(InertialSensor &ins,Compass *compass=0, Barometer *baro=0):_ins(ins),_compass(compass),_baro(baro),_pressure(0){};
	virtual bool Update()=0;             //update inertial sensor data
  Vector3f GetAcc()  {return _acc; }         // get acceleration data
	Vector3f GetGyro() {return _gyro;}        //get gyroscope data
	Vector3f GetMag()  {return _mag;}   
	float GetPressure(){return _pressure;}

	const Vector3f& getAngle() const {
		return _angle;
	}



public:
    // flags structure
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
    } _flags;
};

#endif

