/*****************************************************************************
File name: TDT_Device\src\dbus.h
Description: 遥控器接收机
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加切换为原始数据处理的变量
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _REMOTEVT03_H
#define _REMOTEVT03_H

#include "stm32f4xx.h"
#include "TDT_User.h"
#include <stdio.h>
#include "my_math.h"

/**
 * @addtogroup DBUS 
 * @{
 */

///包括摇杆、拨杆、鼠标、键盘在内的各种信息
///@image html RC_Sketch_Map.jpg "遥控器摇杆拨杆示意图"
#pragma pack(1)
struct _KeyVt
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    union
		{
			uint16_t keyValue;
			///必须按顺序
			struct
			{
				uint16_t W : 1;		///<0x0001
				uint16_t S : 1;		///<0x0002
				uint16_t A : 1;		///<0x0004
				uint16_t D : 1;		///<0x0008
				uint16_t SHIFT : 1; ///<0x0010
				uint16_t CTRL : 1;	///<0x0020
				uint16_t Q : 1;		///<0x0040
				uint16_t E : 1;		///<0x0080
				uint16_t R : 1;		///<0x0100
				uint16_t F : 1;		///<0x0200
				uint16_t G : 1;		///<0x0400
				uint16_t Z : 1;		///<0x0800
				uint16_t X : 1;		///<0x1000
				uint16_t C : 1;		///<0x2000
				uint16_t V : 1;		///<0x4000
				uint16_t B : 1;		///<0x8000
			};
	};
  uint16_t crc16;
};

#pragma pack()

class _RCVT03
{
private:
	int pauseCnt=0;
	void Dbus_Config(void);
	void remoteUpdate();
	void taskSchedule();
public:
	u8 updated;
	uint64_t updateTime;
	void init();
	///1000Hz调用函数，进行数据处理、离线检测、按键与VisualTask调用
	void run_1000Hz();
	///数据解析
	///@image html RC_Sketch_Map.jpg "遥控器摇杆拨杆示意图"
	void handleData();
	///摇杆拨杆特定手势检测与调用
	void rstCount();
	void remote_data_process_advanced(void);
	_KeyVt Key;			 //当前值
	_KeyVt LastKey;			 //当前值
	_KeyVt KeyTick;			 //当前值
	_KeyVt KeyPress;			 //当前值
	int16_t CH[4];
};

///遥控器实例句柄
extern _RCVT03 RCVT03;
///脱力标志位
extern uint8_t deforceFlagvt03;
///遥控器在线标志位
extern uint8_t dbusOnlineFlag;
static uint16_t get_dma_current_position(void);
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);
///@warning C++必须调用将中断的函数放置于extern"C"中，以避免C++编译器对变量名碎片化
#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

void USART3_IRQHandler(void);
	
	
#ifdef __cplusplus
}
#endif /*__cplusplus*/

/** @} */

#endif /*_Dbus_H*/
