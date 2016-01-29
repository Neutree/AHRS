/**
  *@file TaskManager.h
  *@author cq_dan3 lissettecarlr Neucrack(Neutree) Allen-Make
  *@brief time manager(time tick and delay function) for stm32f10x by use systick
  *@version v1.5 
  *         last edit on :   2016-01-05
  *@copyright CQUT IOT LIB all right reserved
  */



#include "TaskManager.h"

//////////////////
///define TaskManager object
/////////////////
TaskManager tskmgr;

////////////////////////////
///Initialize static variable
////////////////////////////
double TaskManager::_it_time = 0;
double TaskManager::_new_time = 0;
double TaskManager::_old_time = 0;



//////////////////////
///Initialize the systick
/////////////////////
TaskManager::TaskManager()
{
	SysTick->CTRL &= 0xFFFFFFFB; //Clock div 8 = 9M
	SysTick->LOAD = 16200000;     //1.8s
	SysTick->CTRL |= 0x00000003; //INT +ENABLE
}	



/////////////////////
///get current time since power on
///@retval Return curent time (unit: S    precision:double)
////////////////////
double TaskManager::Time(void)
{
	_new_time = _it_time + 1.8 - SysTick->VAL/9000000.0; //update current time
	
	if(_new_time - _old_time > 1.799) //check if breaked by SysTick interrupt
	{	
		_new_time -= 1.8;	              //calibrate current time
	}		          
	_old_time = _new_time;            //update old time
	return _new_time;
}



///////////////////
///delay some time
///@param nus duration (unit: us)
//////////////////
void TaskManager::DelayUs(u16 nus)
{
	double OldT=Time();
	while((Time()-OldT)<double(nus)/1000000.0);
}



///////////////////
///delay some time
///@param nms duration (unit: ms)
//////////////////
void TaskManager::DelayMs(u16 nms)
{
	double OldT=Time();
	while((Time()-OldT)<double(nms)/1000.0);
}


///////////////////
///delay some time
///@param s duration (unit: s)
//////////////////
void TaskManager::DelayS(u16 s)
{
	double OldT=Time();
	while((Time()-OldT)<double(s));
}


////////////////////////////////
///SysTick interrupt IRQ handler
////////////////////////////////
extern "C"
{
	void SysTick_Handler(void)
	{
		TaskManager::_it_time += 1.8;
	}
}


