#ifndef __MTF02_H__
#define __MTF02_H__
#include "stm32f4xx.h"                  // Device header

#define MT0X_RC_FRAME_LENGTH 27u
#define MICOLINK_MSG_HEAD    0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN  MICOLINK_MAX_PAYLOAD_LEN + 7

typedef struct
{
	uint32_t  time_ms;			    // 系统时间 ms
    uint32_t  distance;			    // 距离(mm) 最小值为10，0表示数据不可用
    uint8_t   strength;	            // 信号强度
    uint8_t   precision;	        // 精度
    uint8_t   tof_status;	        // 状态
    uint8_t   reserved1;			// 预留
    int16_t   flow_vel_x;	        // 光流速度x轴
    int16_t   flow_vel_y;	        // 光流速度y轴
    uint8_t   flow_quality;	        // 光流质量
    uint8_t   flow_status;	        // 光流状态
    uint16_t  reserved2;	        // 预留
}Output;//外部接口

class MT0X_RC
{
public:
	void init();
	void run();
	Output Now_OP;
private:
	void Config();
	bool handleData(volatile const uint8_t *MTF_rx_buffer);
	void ProcessRxBytes(const uint8_t *data, uint16_t length);


	uint8_t frameBuffer[MT0X_RC_FRAME_LENGTH] = {};
	uint8_t frameLength = 0;

public:
	u8 updated = 0;
	uint64_t updateTime = 0;
};

extern MT0X_RC MT02_RC;

#endif