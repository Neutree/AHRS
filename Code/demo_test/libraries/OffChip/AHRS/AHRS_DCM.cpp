
#include "AHRS_DCM.h"

AHRS_DCM::AHRS_DCM(InertialSensor &ins,Compass *compass, Barometer *baro):AHRS(ins,compass,baro)
{
	_dcm_matrix.Identity();
}

bool AHRS_DCM::Update()
{
	
	if(_compass) _compass->Update(_mag);
	if(_baro) _baro->Update(_pressure);
	if(_ins.Update(_acc,_gyro))
	{
		_dcm_matrix.Rotate(_gyro*_ins.Interval());
		_dcm_matrix.Normalize();
	}
	return true;
}



