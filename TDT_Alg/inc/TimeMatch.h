#ifndef __TIME_MATCH_H
#define __TIME_MATCH_H

#include "board.h"

class TimeSimultaneity
{
public:
	TimeSimultaneity(int bit_rate = 0);

	void Get_time_diff(double vision_time_us, int recv_size, uint64_t mcu_receive_time_us);

	double GetSimuTime_us();

	double GetSimuTime_us(double mcu_time_us);

	double GetImu_VisionTime(double imu_time_s, int send_size);

private:
	int bit_rate_ = 0; // USB 为 0；串口链路填写实际波特率

	double time_diff_us_ = 0.0; // MCU 时间减视觉时间，单位 us
};

#endif
