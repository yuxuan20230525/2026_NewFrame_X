
// 视觉时间同步
#include "TimeMatch.h"

TimeSimultaneity::TimeSimultaneity(int bit_rate)
{
	this->bit_rate_ = bit_rate;
}

void TimeSimultaneity::Get_time_diff(double vision_time, int sizeofRecvStruct) // 获取视觉时间和主控时间差值
{
	time_diff = getSysTimeUs() / 1e6f - vision_time - (double)10.0 * sizeofRecvStruct / bit_rate_;
}

double TimeSimultaneity::GetSimuTime_s() // 返回同步后的mcu时间(s)  （校准后）
{
	return getSysTimeUs() / 1e6f - time_diff;
}

double TimeSimultaneity::GetSimuTime_s(double mcuTime) // 将mcu时间转化为视觉时间 单位（s）
{
	return mcuTime - time_diff;
}

double TimeSimultaneity::GetImu_VisionTime(double imuTime, int sizeofSendStruct)
{
	return GetSimuTime_s(imuTime);
}
