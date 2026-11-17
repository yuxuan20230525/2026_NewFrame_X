#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "board.h"
#include "timer.h"

struct Music
{
	float frequency = 0;	// 当前音乐的频率
	float contain_time = 0; // 音乐持续的时间（单位：秒）
};

// 音乐播放控制结构体
class MusicPlayer
{
private:
	int8_t currentIndex = 0;	// 当前音乐索引
	float currentFrequency = 0; // 当前音乐频率
	float startTime = 0;		// 播放开始时间
	bool last_enable = 0;

public:
	bool enable = 0;		   // 使能播放
	myvector<Music> musicList; // 音乐列表
	float play()
	{ // enable置1后开始轮询，输出频率，自动置0
		if (musicList.empty() || enable == 0)
		{
			currentFrequency = 0;
			currentIndex = 0;
			last_enable = 0;
			return 0;
		}

		if (enable == 1 && last_enable == 0)
		{
			startTime = getSysTimeUs();
			currentFrequency = musicList[currentIndex].frequency;
		}
		last_enable = enable;

		// 检查是否到达当前音乐的播放时间
		if (timeIntervalFrom_f(startTime / 1e6f) > musicList[currentIndex].contain_time)
		{
			// 跳到下一个音乐
			currentIndex++;

			if (currentIndex >= musicList.size())
			{
				// 如果播放完所有音乐，重置
				currentIndex = 0;
				currentFrequency = 0;
				enable = 0;
				last_enable = 0;
			}
			else
			{
				// 播放下一个音乐
				currentFrequency = musicList[currentIndex].frequency;
				startTime = getSysTimeUs();
			}
		}

		return currentFrequency;
	}
};

class Buzzer
{
public:
	Buzzer();
	void init();
	void Run();

	struct Error
	{
		uint8_t *iferr = nullptr;
		uint8_t last_iferr = 0;
		u8 setNow;
		u8 lastsetNow;
		float setnow_time;
	} ErrorCode[10]; ///< 状态值缓存区，最多允许同时存在10个状态值,前面优先级高

	void ErrorLink(uint8_t *iferr, uint8_t priority);
	int8_t getError();

	void sethz(uint16_t frequency);
	
	void setnow(u8 index);  //调用之后立刻响应index的err一次，直到0.3s不调用后才会复位。
	MusicPlayer music[10];
};

extern Buzzer buzzer;

#endif