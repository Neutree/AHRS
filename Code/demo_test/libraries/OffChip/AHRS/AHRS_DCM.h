#ifndef _AHRS_DCM_H_
#define _AHRS_DCM_H_

#include "stm32f10x.h"
#include "AHRS.h"
#include "Mathtool.h"


#define GRAVITY 9.80065f

class AHRS_DCM:public AHRS
{
private:
	Matrix3<float> _dcm_matrix;
    Vector3f _omega;                 // Corrected Gyro_Vector data
    Vector3f _omega_P;               // accel Omega proportional correction
    Vector3f _omega_I;               // Omega Integrator correction
    Vector3f _omega_yaw_P;           // proportional yaw correction

    // state to support status reporting
	float _renorm_val_sum;
	uint16_t _renorm_val_count;

	//加速度校准
	float _ra_deltat;  //时间累积值
	uint32_t _ra_sum_start;
	Vector3f _last_velocity;

    // state of accel drift correction（加速度计漂移修正的状态）
	Vector3f _ra_sum;

	// whether we have GPS lock
	bool _have_gps_lock;

public:
	AHRS_DCM(InertialSensor &ins,Compass *compass=0, Barometer *baro=0);
	virtual bool Update(); 
	void MatrixUpdate(float delta_t);
	// Normalize the DCM matrix
	void
	Normalize();

	/**
	 * renormalise one vector component of the DCM matrix
	 * this will return false if renormalization fails
	 */
	bool renorm(Vector3f const &a, Vector3f &result);

	// Perform drift correction
	void
	DriftCorrection(float delta_t);

	void drift_correction_yaw();



	// paranoid check for bad values in the DCM matrix
	void
	CheckMatrix();

	// Calculate pitch, roll, yaw for stabilization and navigation
	void
	EulerAngles();

	// update trig values including _cos_roll, cos_pitch
	void
	UpdateTrig();
	
	Vector3f
	ra_delayed(const Vector3f &ra);
};

#endif

