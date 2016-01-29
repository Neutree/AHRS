#include "stm32f10x.h"
#include "Configuration.h"
#include "TaskManager.h"

#include "USART.h"
#include "I2C.h"
//#include "Timer.h"
//#include "ADC.h"
//#include "InputCapture_TIM.h"
//#include "InputCapture_EXIT.h"

#include "MPU6050.h"
//#include "Gimbal.h"
//#include "AHRS.h"
//#include "Ultrasonic.h"
//#include "Remoter_PWM_TIM.h"
//#include "Remoter_PWM_EXIT.h"
#include "HMC5883.h"
#include "MS561101.h"
#include "GPS.h"
#include "Mathtool.h"
#include "AHRS_DCM.h"

//////////////////////////////////////////////////////////////////////////////
////////////////            Peripheral & Object       ////////////////////////
//////////////////////////////////////////////////////////////////////////////
USART com(USART1,115200);   //USART1
I2C i2c(I2C2);               //I2C2 for MPU6050
//PWM t2(TIM2,1,1,1,1,20000);  //TIM2 as PWM for roll
//PWM t3(TIM3,1,1,0,0,20000);  //TIM3 as PWM for pitch
//InputCapture_TIM t4(TIM4, 400, true, true, true, true); //TIM4 as InputCapture for remoter controller
//InputCapture_EXIT ch1(GPIOB,6);
//InputCapture_EXIT ch2(GPIOB,7);
//InputCapture_EXIT ch3(GPIOB,8);
//InputCapture_EXIT ch4(GPIOB,9);
//ADC adc(9,6,8,7);            //adc channel 6,7,8,9 enable
//Timer T1(TIM1,1);
//Ultrasonic sonar(GPIOB,6,GPIOB,7);


//StepMotor rollMotor(&t2,1,&t2,2,&t2,3,0.3);  //roll motor
//StepMotor pitchMotor(&t2,4,&t3,1,&t3,2,0.3); //pitch motor
//Gimbal gimbal(rollMotor,pitchMotor);         //2-aixs gimbal

//Remoter_PWM_TIM rc(&t4,1,&t4,2,&t4,4,&t4,3);
//Remoter_PWM_EXIT rc(&ch1,&ch2,&ch4,&ch3);


MPU6050 ins(i2c);        //MPU6050
HMC5883 compass(i2c);
MS561101 baro(i2c);
GPS gps;


AHRS_DCM ahrs(ins,&compass,&baro,&gps);


//////////////////////////////////////////////////////////////////////////////
//////////////////            Global Variable       //////////////////////////
//////////////////////////////////////////////////////////////////////////////


Matrix3<float> m;
Vector3f acc, gyro, mag;
double newTime = 0, oldTime = 0, oldTime2=0;

float pitch,roll,yaw;
int main()
{	
	while(1)
	{
		newTime = tskmgr.Time();
		
		if(newTime-oldTime2>=0.02){
			ahrs.Update();
			oldTime2=newTime;
		}
		if((newTime-oldTime)>0.1)
		{				
			com<<ahrs.GetAcc().x<<","<<ahrs.GetGyro().x*57.29f<<","<<ahrs.GetMag().x<<","<<ahrs.GetPressure()<<"\n";
			com<<ahrs.getAngle().x<<","<<ahrs.getAngle().y<<","<<ahrs.getAngle().z<<"\n\n\n";
			oldTime = newTime;
		}
	}
}

#ifdef USE_TIMER1
void Timer1_IRQ()
{
}
#endif

#ifdef USE_TIMER2
void Timer2_IRQ()
{
}
#endif
#ifdef USE_TIMER3
void Timer3_IRQ()
{
}
#endif
#ifdef USE_TIMER4
void Timer4_IRQ()
{
}
#endif
#ifdef USE_EXTI0
void	EXTI0_IRQ()
{
}
#endif
#ifdef USE_EXTI1
void	EXTI1_IRQ()
{
}
#endif
#ifdef USE_EXTI2
void	EXTI2_IRQ()
{
}
#endif
#ifdef USE_EXTI3
void	EXTI3_IRQ()
{
}
#endif
#ifdef USE_EXTI4
void	EXTI4_IRQ()
{
}
#endif
#ifdef USE_EXTI5
void	EXTI5_IRQ()
{
}
#endif
#ifdef USE_EXTI6
void	EXTI6_IRQ()
{
}
#endif
#ifdef USE_EXTI7
void	EXTI7_IRQ()
{
}
#endif
#ifdef USE_EXTI8
void	EXTI8_IRQ()
{
}
#endif
#ifdef USE_EXTI9
void	EXTI9_IRQ()
{
}
#endif
#ifdef USE_EXTI10
void	EXTI10_IRQ()
{
}
#endif
#ifdef USE_EXTI11
void	EXTI11_IRQ()
{
}
#endif
#ifdef USE_EXTI12
void	EXTI12_IRQ()
{
}
#endif
#ifdef USE_EXTI13
void	EXTI13_IRQ()
{
}
#endif
#ifdef USE_EXTI14
void	EXTI14_IRQ()
{
}
#endif
#ifdef USE_EXTI15
void	EXTI15_IRQ()
{
}
#endif
