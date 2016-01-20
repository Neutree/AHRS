#ifndef __AHRS__H
#define __AHRS__H
#include "Vector3.h"

class AHRS
{
public:
	AHRS();
	~AHRS();
	Vector3<double> GetAngle(Vector3<double> acc,Vector3<double> gyr);
};


#endif

