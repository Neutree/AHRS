/**
  *@file TaskManager.h
  *@author cq_dan3 lissettecarlr Neucrack(Neutree) Allen-Make
  *@brief time manager(time tick and delay function) for stm32f10x by use systick
  *@version v1.5 
  *         last edit on :   2016-01-05
  *@copyright CQUT IOT LIB all right reserved
  */


#ifndef _TASK_MANAGER_H_
#define _TASK_MANAGER_H_
#include "stm32f10x.h"

class TaskManager
{
private:
	static double _new_time;		 //current updated time
	static double _old_time;     //last updated time
public:
	static double _it_time;      //time = SysTick interrupt counter*1.8s
public:
	//////////////////////
	///Initialize the systick
	/////////////////////
	TaskManager();

	/////////////////////
	///get current time since power on
	///@retval Return curent time (unit: S    precision:double)
	////////////////////
	static double Time(void);

	///////////////////
	///delay some time
	///@param nus duration (unit: us)
	//////////////////
	static void DelayUs(u16 nus);

	///////////////////
	///delay some time
	///@param nms duration (unit: ms)
	//////////////////
	static void DelayMs(u16 nms);
	
	///////////////////
	///delay some time
	///@param s duration (unit: s)
	//////////////////
	static void DelayS(u16 s);
};

//////////////////////
///declare TaskManager object
/////////////////////
extern TaskManager tskmgr;

#define	MOD_ERROR  0x00
#define	MOD_READY  0x01
#define	MOD_BUSY   0x02
#define	MOD_LOCK   0x04
#define	MOD_UNLOCK 0x08


#endif
