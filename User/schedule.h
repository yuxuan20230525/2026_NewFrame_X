/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-01-25 19:09:02
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-01-25 19:32:35
 * @FilePath: \2025_NewFrame\2025_NewFram\Frame Base\Frame2025\User\schedule.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _SCHEDULE_H
#define _SCHEDULE_H

#include "board.h"

///@addtogroup TDT_SCHEDULE_API
///@{
struct Schedule
{
	/// 运行频率计数
	///@{
	uint16_t cnt_ms;

	///@}
	/// CPU占用率(相对)
	float CPU_usage;
	/// 运行时间统计, 单位微秒
	///@{
	uint32_t runTime_1ms;
	uint32_t runTime_2ms;
	uint32_t runTime_5ms;
	uint32_t runTime_10ms;
	uint32_t runTime_20ms;
	uint32_t runTime_50ms;
	uint32_t runTime_100ms;
	uint32_t runTime_500ms;
	uint32_t runTime_1000ms;
	///@}
};

void TDT_Loop_1000Hz(void); ///< 1ms执行一次
void TDT_Loop_500Hz(void);  ///< 2ms执行一次
void TDT_Loop_200Hz(void);  ///< 5ms执行一次
void TDT_Loop_100Hz(void);  ///< 10ms执行一次
void TDT_Loop_50Hz(void);   ///< 20ms执行一次
void TDT_Loop_20Hz(void);   ///< 50ms执行一次
void TDT_Loop_10Hz(void);   ///< 100ms执行一次
void TDT_Loop_2Hz(void);    ///< 500ms执行一次
void TDT_Loop_1Hz(void);    ///< 1000ms执行一次

void cntUpdate(struct Schedule *robotSchedule);
void Run_UdTime(struct Schedule *robotSchedule);

#ifdef __cplusplus
extern "C" {
#endif

void TDT_Loop(void);

#ifdef __cplusplus
}
#endif

///@}
#endif
