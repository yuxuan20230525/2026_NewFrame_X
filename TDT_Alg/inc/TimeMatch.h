#ifndef __TIME_MATCH_H
#define __TIME_MATCH_H

#include "board.h"

class TimeSimultaneity
{
public:
	TimeSimultaneity(int bit_rate = 460800);

	void Get_time_diff(double vision_time, int sizeofRecvStruct); // 获取视觉时间和主控时间差值

	double GetSimuTime_s(); // 返回同步后的mcu时间(s)  （校准后）

	double GetSimuTime_s(double mcuTime); // 将mcu时间转化为视觉时间 单位（s）

	double GetImu_VisionTime(double imuTime, int sizeofSendStruct);

private:
	int bit_rate_=460800; // 串口波特率

	double time_diff=0; // 时间差值
};

#endif