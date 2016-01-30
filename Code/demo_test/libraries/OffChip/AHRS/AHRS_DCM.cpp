
#include "AHRS_DCM.h"
#include "GPIO.h"
#include "LED.h"
AHRS_DCM::AHRS_DCM(InertialSensor &ins,Compass *compass, Barometer *baro,GPS *gps)
:AHRS(ins,compass,baro,gps)
{
	_dcm_matrix.Identity();
	_gps_min_satellite=6;//允许的最少卫星数量为6
	_Kp = 0.1f;
	_Ki = 0.01f;
}
//	GPIO ledGpio(GPIOB,6,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
//	LED ledGreen(ledGpio);
/**
 *更新飞机的姿态
 */
bool AHRS_DCM::Update()
{
	//update raw data from sensor
//	if(_compass) _compass->Update(_mag);
//	if(_baro) _baro->Update(_pressure);
//	if(!_ins.Update(_acc,_gyro)){
//		//return false;
//	}

	//获取两次更新传感器数据之间的间隔
	float delta_t=0.006;//_ins.Interval();

	//两次更新的时间超过了限制的时间，直接返回
	if(delta_t>1){

		return false;
	}
//test
//	this->_angle+=_gyro*(1.0/(4000.0/65536.0))*delta_t;

    //用角速度来更新旋转矩阵
	MatrixUpdate(delta_t);

	// 归一化旋转矩阵（使旋转矩阵满足正交性约束）
	Normalize();

	// 对由陀螺仪获取信息而计算的的旋转矩阵进行漂移矫正
	DriftCorrection(delta_t);

	// 看方向余旋矩阵里面的数据是否发散出现无穷大或无穷小
	CheckMatrix();

	// 将方向余旋矩阵转化为欧拉角（yaw pitch roll）
	EulerAngles();

	// 用来提前计算yaw ,pitch,roll的正余旋用在其它地方
	UpdateTrig();

	return true;
}

/**
 * 仅由角速度数据来更新旋转矩阵
 * @param delta_t 两次更新传感器数据的间隔时间
 */
void AHRS_DCM::MatrixUpdate(float delta_t) {
	// note that we do not include the P terms in _omega. This is
	// because the spin_rate is calculated from _omega.length(),
	// and including the P terms would give positive feedback into
	// the _P_gain() calculation, which can lead to a very large P
	// value
	_omega.Zero();

	//由角速度计算出角度变化量
//	_omega=_ins.GetAngleByGyro(_gyro,delta_t);
	_omega = _gyro;
//	(*pCOM1)<<_omega_I.x<<"\t"<<_omega_I.y<<"\t"<<_omega_I.z<<"\n";
//	(*pCOM1)<<_omega_P.x<<"\t"<<_omega_P.y<<"\t"<<_omega_P.z<<"\n";
//	_omega=_omega*delta_t;
//	(*pCOM1)<<_omega.x<<"\t"<<_omega.y<<"\t"<<_omega.z<<"\n";
	if(delta_t>0){
		_omega+=_omega_I;//PI controller correct gyro's data by integral
		_dcm_matrix.Rotate((_omega+_omega_P+_omega_yaw_P)*delta_t);//update DCM Matrix with PI controller
//if(_omega_P.x==0&&_omega_P.y==0&&_omega_P.z==0)
//	(*pCOM1)<<"aaaaaaa\n";
//else
//	(*pCOM1)<<_omega_P.x*100000<<"\t"<<_omega_P.y*100000<<"\t"<<_omega_P.z*100000<<"\n";
//		(*pCOM1)<<temp.x*1000<<"\t"<<temp.y*1000<<"\t"<<temp.z*1000<<"\n";
		//(*pCOM1)<<_dcm_matrix.a.x*100000<<"\t"<<_dcm_matrix.a.y*100000<<"\t"<<_dcm_matrix.a.z*100000<<"\n";
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
		//归一化失败，飞机状态有问题，重置状态（清零），标记出问题的时间
		Reset(true);
		(*pCOM1)<<"normalize wrong\n";
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
    if(from.Length()==0)
    {
    	_dcm_matrix.Zero();
    	return true;
    }
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

    (*pCOM1)<<from.Length()<<"\n\n";
    result = from * renorm_val;//将向量单位化了
    return true;
}

// perform drift correction. This function aims to update _omega_P and
// _omega_I with our best estimate of the short term and long term
// gyro error. The _omega_P value is what pulls our attitude solution
// back towards the reference vector quickly. The _omega_I term is an
// attempt to learn the long term drift rate of the gyros.
//
// This drift correction implementation is based on a paper
// by Bill Premerlani from here:
//   http://gentlenav.googlecode.com/files/RollPitchDriftCompensation.pdf
void AHRS_DCM::DriftCorrection(float delta_t) {
	Vector3f velocity;
	uint32_t last_correction_time;

	//如果有航向参考向量（比如磁力计或者GPS），则进行航向矫正
	drift_correction_yaw();

	//旋转加速度值到地面坐标系下
	Vector3f accelErthRef = _dcm_matrix *_acc;//地球坐标系下的加速度
	_ra_sum +=  accelErthRef*delta_t;

	_ra_deltat+=delta_t;//累积时间，记录总的整合时间

	//没有GPS或者卫星数量小于最小允许的卫星数量或者不是3D定位
	if(!_gps ||
			_gps->Status()<GPS::GPS_OK_FIX_3D||
			_gps->NumberOfSatellite()<_gps_min_satellite){
			//需要6颗以上的卫星来获得可靠的位置信息
			if(_ra_deltat<0.2f){
				//累积时间不够
				return;
			}
	        //float airSpeed;//空速
	        //使用空速减去风速来估计在地面坐标系下的地速

			last_correction_time = TaskManager::Time();//获取当前的时间
			_have_gps_lock = false;
	}
	else{//有GPS
		if(_gps->LastFixTimeMs()==_ra_sum_start){
			//GPS没有更新值，不做处理
			return;
		}
		velocity = _gps->Velocity();//从GPS获得速度
		last_correction_time = _gps->LastFixTimeMs();//更新时间
		if(!_have_gps_lock){
			//如果上次没有锁住GPS
			_last_velocity=velocity;
		}
		_have_gps_lock=true;

//		// keep last airspeed estimate for dead-reckoning purposes
//		Vector3f airspeed = velocity - _wind;
//		airspeed.z = 0;
//		_last_airspeed = airspeed.length();
	}
	if(_gps){

	}else{

	}
	
	//看是不是第一次进来 - 也就是需要设置开始的时间并且返回
	if(_ra_sum_start==0){
		_ra_sum_start = last_correction_time;
		_last_velocity = velocity;
		return;
	}
	// equation 9: get the corrected acceleration vector in earth frame.
	//Units(得到一个地球参考系下的修正的加速度矢量)
    // 单位 m/s/s
	 Vector3f GA_e;
     GA_e = Vector3f(0, 0, -1.0f); //重力加速度方向
	 if (_ra_deltat <= 0) {
        // waiting for more data
        return;
      }
    bool using_gps_corrections = false;
    float ra_scale = 1.0f/(_ra_deltat*GRAVITY);  //1/（重力*时间间隔）

	if (1/*_flags.correct_centrifugal && (_have_gps_lock || _flags.fly_forward)*/) {
	  	   //如果需要矫正离线力 且 gps上锁 或假设航向是沿X轴的
		GA_e += (velocity - _last_velocity) * ra_scale;
		GA_e.Normalize();
		if(GA_e.IsInf()){
			return;
		}
        using_gps_corrections = true;

	}
	else{
		//(*pCOM1)<<"..."<<"\n";
	}
	  

	Vector3f GA_b;
	Vector3f error; //保存方程3左右两边的叉乘 （误差值）
	float error_dirn ;
	
	_ra_sum *= ra_scale;//	方程5左边
	  
	//传感器健康状况判断

	//如果不使用GPS修正
	GA_b= _ra_sum;
	GA_b.Normalize();

	error=GA_b % GA_e;	//方程三在左右两边的误差

	error=_dcm_matrix.Transpose()*error; //将地球坐标系下的误差传化为机体坐标系

	_omega_P=error * _Kp; //比例误差

	_omega_I_sum+=error * _Ki * _ra_deltat;
	_omega_I_sum_time += _ra_deltat;

	if(_omega_I_sum_time >=5) //如果时间大于5秒
	{
		_omega_I_sum_time = 0;
		_omega_I +=_omega_I_sum;
		_omega_I_sum.Zero();
	}

	_ra_deltat = 0; //清空累积时间
	_ra_sum_start = last_correction_time;

	// remember the velocity for next time
	_last_velocity = velocity; //保存上速度
}





void AHRS_DCM::drift_correction_yaw() {
	//如果有磁力计，并且有航向值更新

	//如果有GPS，并且有航向值更新

	//如果没有了新的航向矫正值，将_omega_yaw_P*0.97f进行缓慢衰减，最终趋近于0

}



void AHRS_DCM::CheckMatrix() {
	if(_dcm_matrix.IsNan()){
		Reset(true);
		return;
	}
	if(!(_dcm_matrix.c.x<1.0f &&
			_dcm_matrix.c.x>-1.0f)){
		Normalize();
		if(_dcm_matrix.IsNan()||
				fabsf(_dcm_matrix.c.x)>10){
				Reset(true);
		}
	}
}


void AHRS_DCM::EulerAngles() {
	Matrix3<float> body_dcm_matrix;
    body_dcm_matrix = _dcm_matrix;
    body_dcm_matrix.ToEuler(&_angle.x, &_angle.y, &_angle.z);


}



void AHRS_DCM::UpdateTrig() {
}

void AHRS_DCM::Reset(bool recoverEulers) {
	_omega_I.Zero();
	_omega_P.Zero();
	_omega_yaw_P.Zero();
	_omega.Zero();
	_angle.Zero();
}

//存入传入数据，读出上一次存入的数据
Vector3f AHRS_DCM::ra_delayed(const Vector3f &ra)
{
	   //取出旧的数据，然后用填充新的数据
}

void AHRS_DCM::UpdateSensor() {
//	if(_compass) _compass->Update(_mag);
//	if(_baro) _baro->Update(_pressure);
	if(!_ins.Update(_acc,_gyro)){
		//return false;
	}

}
