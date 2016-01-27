
#include "AHRS_DCM.h"

AHRS_DCM::AHRS_DCM(InertialSensor &ins,Compass *compass, Barometer *baro):AHRS(ins,compass,baro)
{
	_dcm_matrix.Identity();
}
/**
 *更新飞机的姿态
 */
bool AHRS_DCM::Update()
{
	
	//update raw data from sensor
	if(_compass) _compass->Update(_mag);
	if(_baro) _baro->Update(_pressure);
	if(!_ins.Update(_acc,_gyro))
		return false;

	//get time of twice update
	float delta_t=_ins.Interval();

	//interval check
	if(delta_t>1){

		return false;
	}

    // Integrate the DCM matrix using gyro inputs
	MatrixUpdate(delta_t);

	// Normalize the DCM matrix
	Normalize();

	// Perform drift correction
	DriftCorrection(delta_t);

	// paranoid check for bad values in the DCM matrix
	CheckMatrix();

	// Calculate pitch, roll, yaw for stabilization and navigation
	EulerAngles();

	// update trig values including _cos_roll, cos_pitch
	UpdateTrig();

	return true;
}

void AHRS_DCM::MatrixUpdate(float delta_t) {
	// note that we do not include the P terms in _omega. This is
	// because the spin_rate is calculated from _omega.length(),
	// and including the P terms would give positive feedback into
	// the _P_gain() calculation, which can lead to a very large P
	// value
	_omega.Zero();

	//get angle only by gyro
	_omega=_ins.GetAngleByGyro(_gyro,delta_t);

	if(delta_t>0){
		_omega+=_omega_I/delta_t;//PI controller correct gyro's data by integral
		_dcm_matrix.Rotate((_omega+_omega_P+_omega_yaw_P)*delta_t);//update DCM Matrix with PI controller
	}


}

void AHRS_DCM::Normalize() {
	float error;
	Vector3f t0,t1,t2;

	//检测正交性（orthogonal）（三个向量两两垂直），算出正交性误差
	error = _dcm_matrix.a * _dcm_matrix.b;    //Eqn.18
	//将误差分到两个向量上，使矩阵满足正交性约束
	t0 = _dcm_matrix.a - (_dcm_matrix.b*(error*0.5f));    //Eqn.19
	t1 = _dcm_matrix.b - (_dcm_matrix.a*(error*0.5f));    //Eqn.19
	t2 = t0%t1;//算出第三行的向量（第一行和第二行的叉乘）       //Eqn.20

	//归一化（让每一行的向量为范数为1）
	if(  !renorm(t0,_dcm_matrix.a) ||
         !renorm(t1, _dcm_matrix.b)||
         !renorm(t2, _dcm_matrix.c)  ){

	}
}
// renormalise one vector component of the DCM matrix
// this will return false if renormalization fails
bool AHRS_DCM::renorm(Vector3f const &from, Vector3f &result)
{
    float renorm_val;

    // numerical errors will slowly build up over time in DCM,
    // causing inaccuracies. We can keep ahead of those errors
    // using the renormalization technique from the DCM IMU paper
    // (see equations 18 to 21).

    //如果不使用泰勒展开式计算平方根，减少运算时间，这里就使用平方根进行计算

    //向量的模的倒数
    renorm_val = 1.0f / from.Length();           //Eqn.21

    // keep the average for reporting
    _renorm_val_sum += renorm_val;
    _renorm_val_count++;

    if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
        // this is larger than it should get - log it as a warning
        if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
            // we are getting values which are way out of
            // range, we will reset the matrix and hope we
            // can recover our attitude using drift
            // correction before we hit the ground!
            //Serial.printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
            //	   renorm_val);
            return false;
        }
    }

    result = from * renorm_val;//将向量单位化了
    return true;
}

void AHRS_DCM::DriftCorrection(float delta_t) {


}

void AHRS_DCM::CheckMatrix() {
}

void AHRS_DCM::EulerAngles() {
}

void AHRS_DCM::UpdateTrig() {
}
