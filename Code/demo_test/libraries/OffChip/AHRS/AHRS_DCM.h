#ifndef _AHRS_DCM_H_
#define _AHRS_DCM_H_

#include "stm32f10x.h"
#include "AHRS.h"
#include "Mathtool.h"

class AHRS_DCM:public AHRS
{
private:
	Matrix3<float> _dcm_matrix;
	
public:
	AHRS_DCM(InertialSensor &ins,Compass *compass=0, Barometer *baro=0);
	virtual bool Update(); 
};

#endif

