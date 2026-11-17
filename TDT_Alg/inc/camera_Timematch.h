#ifndef __CAMERA_TIMEMATCH_H
#define __CAMERA_TIMEMATCH_H

#include "board.h"

class CameraTM
{
public:
	CameraTM();
	void init();
	void run();
  
  volatile uint32_t recieve_frameID = 0;//接收到相机取帧完发回来的信号对应的ID号
  volatile uint32_t send_frameID = 0;//发送给相机的定频触发信号
  uint8_t last_digit;//高电平校验末尾数字
  uint16_t idcheckFreq;//第xx帧高电平校验
  float camera_yaw;//相机取帧完一瞬间对应yaw
  float camera_pitch;//相机取帧完一瞬间对应pitch
  float camera_roll;//相机取帧完一瞬间对应roll
private:
};

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

 void EXTI9_5_IRQHandler(void);
 void TIM1_UP_TIM10_IRQHandler(void);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

extern CameraTM cameraTM;

#endif
