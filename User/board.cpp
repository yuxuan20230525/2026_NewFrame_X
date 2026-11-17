/******************************
File name: TDT_Task\src\board.cpp
Description: 滴答定时器的初始化和延时功能，初始化
function:
	——————————————————————————————————————————————————————————————————————————
	void TDT_Board_ALL_Init(void)
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "board.h"
#include "cycle.h"
#include "iwdg.h"
#include "can.h"
#include "vision.h"
#include "flash_var.h"
#include "dbus.h"
#include "pwm.h"
#include "adc.h"
#include "soft_pwm.h"
#include "schedule.h"
#include "task_virtual.h"
#include "judgement.h"
#include "buzzer.h"
#include "oled.h"
#include "vofa.h"
#include "power_task.h"
#include "remotevt03.h"
#include "camera_Timematch.h"
#include "gimbal.h"
#include "Steerwheel.h"  

/*初始化完成标志*/
u8 Init_OK = 0;
u8 flashNum=0;
/**
 * @brief 总初始化函数
 * @note 禁止使用延时
 */
void boardALLInit(void)
{
	/* 禁止全局中断*/
	__disable_irq();
	/*CAN1初始化*/
	canInit(CAN1);
	/*CAN2初始化*/
	canInit(CAN2);

#if USE_TEMP_CTRL
	bmiTempCtrl.tempCtrlInit();
#endif

#if USE_RC
	RC.init();
#endif
	/*视觉串口初始化*/

#if USE_VISION
	Vision.init();
#endif
	
#if USE_USB_VISION | USE_LOG
	Vision.USB_init();
#endif

#if USE_LOG
	log_task.init();
#endif

#if USE_RCVT03
	RCVT03.init();
#endif

#if USE_JUDGEMENT
	judgement.init();
#endif

#if USE_Judgement_Vofa
	vofa.init();
#endif 

	taskControl.AllTask_Init();

#if USE_BUZZER
	buzzer.init();
#endif
	power_Task.init();
	
#if USE_CAMERATM
    cameraTM.init();
#endif

	gimbal.Init();
	steer.Init();

//	/*看门狗初始化-喂狗在LED*/
	iwdgInit(4, 50);
	/*  使能全局中断 */
	__enable_irq();
	/*初始化完成*/
	Init_OK = 1;
}
/***********End of file*****************  */

/**
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次记录
	——————————————————————————————————————————————————————————————————————————
*/
