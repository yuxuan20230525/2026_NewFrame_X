
// 视觉时间同步，协议时间单位统一为 us
#include "TimeMatch.h"

TimeSimultaneity::TimeSimultaneity(int bit_rate)
{
	this->bit_rate_ = bit_rate;
}

void TimeSimultaneity::Get_time_diff(double vision_time_us, int recv_size, uint64_t mcu_receive_time_us)
{
	double transfer_delay_us = 0.0;
	if (bit_rate_ > 0)
		transfer_delay_us = 10.0 * recv_size * 1e6 / bit_rate_;

	time_diff_us_ = (double)mcu_receive_time_us - vision_time_us - transfer_delay_us;
}

double TimeSimultaneity::GetSimuTime_us()
{
	return (double)getSysTimeUs() - time_diff_us_;
}

double TimeSimultaneity::GetSimuTime_us(double mcu_time_us)
{
	return mcu_time_us - time_diff_us_;
}

double TimeSimultaneity::GetImu_VisionTime(double imu_time_s, int send_size)
{
	(void)send_size;
	return GetSimuTime_us(imu_time_s * 1e6);
}
