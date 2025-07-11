#ifndef __MAHONY_AHRS_H
#define __MAHONY_AHRS_H
#include "math.h"
#include "Vector3.h"

#define RtA 		57.324841f	//角度转弧度制			


class MahonyAHRS
{
private:
	//! Auxiliary variables to reduce number of repeated operations
	float q0, q1, q2 , q3 ;	/** quaternion of sensor frame relative to auxiliary frame */
	float dq0, dq1, dq2 , dq3;	/** quaternion of sensor frame relative to auxiliary frame */
	float gyro_bias[3]; /** bias estimation */
	float q0q0, q0q1, q0q2, q0q3;
	float q1q1, q1q2, q1q3;
	float q2q2, q2q3;
	float q3q3;
	unsigned char bFilterInit;

	float mKp,mKi;
public:
	MahonyAHRS(float kP,float kI)
	{
		mKp = kP;
		mKi = kI;
		memset(gyro_bias, 0, sizeof(gyro_bias));
	}

	void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
	{
		float initialRoll, initialPitch;
		float cosRoll, sinRoll, cosPitch, sinPitch;
		float magX, magY;
		float initialHdg, cosHeading, sinHeading;

		q0 =1;
		
		initialRoll = atan2(-ay, -az);
		initialPitch = atan2(ax, -az);

		cosRoll = cosf(initialRoll);
		sinRoll = sinf(initialRoll);
		cosPitch = cosf(initialPitch);
		sinPitch = sinf(initialPitch);

		magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

		magY = my * cosRoll - mz * sinRoll;

		initialHdg = atan2f(-magY, magX);

		cosRoll = cosf(initialRoll * 0.5f);
		sinRoll = sinf(initialRoll * 0.5f);

		cosPitch = cosf(initialPitch * 0.5f);
		sinPitch = sinf(initialPitch * 0.5f);

		cosHeading = cosf(initialHdg * 0.5f);
		sinHeading = sinf(initialHdg * 0.5f);

		q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
		q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
		q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
		q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

		// auxillary variables to reduce number of repeated operations, for 1st pass
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;
		bFilterInit = 1;
	}
	void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt) 
	{
		float recipNorm;
		float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

		// Make filter converge to initial solution faster
		// This function assumes you are in static position.
		// WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
		if(bFilterInit == 0) {
			MahonyAHRSinit(ax,ay,az,mx,my,mz);
			return;
		}
				
		//! If magnetometer measurement is available, use it.
		if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
			float hx, hy, hz, bx, bz;
			float halfwx, halfwy, halfwz;
		
			// Normalise magnetometer measurement
			// Will sqrt work better? PX4 system is powerful enough?
			recipNorm = 1.0 / sqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;
		
			// Reference direction of Earth's magnetic field
			hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
			hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
			hz = 2.0f * (mx * (q1q3 - q0q2) +  my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
			bx = sqrt(hx * hx + hy * hy);
			bz = hz;
		
			// Estimated direction of magnetic field
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
		
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += (my * halfwz - mz * halfwy);
			halfey += (mz * halfwx - mx * halfwz);
			halfez += (mx * halfwy - my * halfwx);
			
		}

		//增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 	
		{
			float halfvx, halfvy, halfvz;
		
			// Normalise accelerometer measurement
			//归一化，得到单位加速度
			recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);

			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Estimated direction of gravity and magnetic field
			halfvx = q1q3 - q0q2;
			halfvy = q0q1 + q2q3;
			halfvz = q0q0 - 0.5f + q3q3;
		
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += ay * halfvz - az * halfvy;
			halfey += az * halfvx - ax * halfvz;
			halfez += ax * halfvy - ay * halfvx;
//			DEBUG_LOG<<"\t\t\t\t\t\t"<<halfex<<"\t"<<halfey<<"\t"<<halfez<<"\n";
		}

		// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
		if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
			// Compute and apply integral feedback if enabled
			if(twoKi > 0.0f) {
				gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
				gyro_bias[1] += twoKi * halfey * dt;
				gyro_bias[2] += twoKi * halfez * dt;
				
				// apply integral feedback
				gx += gyro_bias[0];
				gy += gyro_bias[1];
				gz += gyro_bias[2];
			}
			else {
				gyro_bias[0] = 0.0f;	// prevent integral windup
				gyro_bias[1] = 0.0f;
				gyro_bias[2] = 0.0f;
			}

			// Apply proportional feedback
			gx += twoKp * halfex;
			gy += twoKp * halfey;
			gz += twoKp * halfez;
//			DEBUG_LOG<<"\t\t\t\t\t\t"<<gx<<"\t"<<gy<<"\t"<<gz<<"\n";
		}
		
		// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
		//! q_k = q_{k-1} + dt*\dot{q}
		//! \dot{q} = 0.5*q \otimes P(\omega)
		dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
		dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
		dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
		dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

		q0 += dt*dq0;
		q1 += dt*dq1;
		q2 += dt*dq2;
		q3 += dt*dq3;
		
		// Normalise quaternion
		recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;   
	}
	
	Vector3f GetAngle(Vector3<int> acc, Vector3<float> gyro,Vector3<int> mag,float deltaT)
	{
		Vector3f angle;
		
		MahonyAHRSupdate(gyro.x,gyro.y,gyro.z,acc.x,acc.y,acc.z,mag.x,mag.y,mag.z,mKp,mKi,deltaT);		
		
		angle.y = asin(-2 * q1 * q3 + 2 * q0* q2)* RtA; // pitch
		angle.x = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll
		angle.z = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))* RtA;//yaw
		
		return angle;
	}
	
};

#endif

