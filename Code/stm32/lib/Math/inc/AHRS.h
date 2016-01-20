#ifndef __AHRS__H
#define __AHRS__H
#include "Vector3.h"

class AHRS
{
public:
	AHRS();
	~AHRS();
	static Vector3<double> GetAngle(Vector3<int> acc,Vector3<int> gyr);
};


#define RtA 		57.324841f				
#define AtR    		0.0174533f				
#define Acc_G 		0.0011963f				
#define Gyro_G 		0.0610351f				
#define Gyro_Gr		0.0010653f			
#define FILTER_NUM 	20

#define Kp 1.6f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.001f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                   // half the sample period???????


#endif

