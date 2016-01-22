#include "AHRS.h"

/**
 * 
 */
AHRS::AHRS()
{

}
/**
 * 
 */
AHRS::~AHRS()
{

}

/**
 * @brief get angle according to accelerator and gyro
 * @param raw accelerator value
 * @param raw gyro value
 * @return angle value
 */
Vector3<double> AHRS::GetAngle(Vector3<int> acc,Vector3<int> gyr)
{
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
	static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
	static float 	AngleOffset_Rol=0,AngleOffset_Pit=0;
	
	Vector3<double> angle;
	angle.x=0;
	angle.y=0;
	angle.z=0;
	
	
	
	float ax = acc.x,ay = acc.y,az = acc.z;
	float gx = gyr.x,gy = gyr.y,gz = gyr.z;
	float norm;
	//  float hx, hy, hz, bx, bz;
	float vx, vy, vz;// wx, wy, wz;
	float ex, ey, ez;

	// ???????????
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	//  float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	//  float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;

	if(ax*ay*az==0)
		return angle;

	gx *= Gyro_Gr;
	gy *= Gyro_Gr;
	gz *= Gyro_Gr;

	norm = sqrt(ax*ax + ay*ay + az*az);       //acc?????
	ax = ax /norm;
	ay = ay / norm;
	az = az / norm;
	if(norm>16500)
		return angle;
	// estimated direction of gravity and flux (v and w)              ?????????/??
	vx = 2*(q1q3 - q0q2);												//????xyz???
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) ;                           					 //???????????????
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

	exInt = exInt + ex * Ki;								  //???????
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;					   							//???PI???????,???????
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;				   							//???gz????????????????,??????????????

	// integrate quaternion rate and normalise						   //????????
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	//angle.z += gyr.z*Gyro_G*0.002f;
	angle.z =atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3;
	
	angle.y = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 - AngleOffset_Pit; // pitch
	angle.x = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 - AngleOffset_Rol; // roll

	return angle;
}

