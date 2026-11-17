#ifndef __PWM_H
#define __PWM_H
#include "board.h"

// 使用航模电调时，配置频率为50Hz，高电平时间1ms为不转，高电平时间为2ms为满速转动
// 电调上电时必须接收到1ms的高电平才能启动
// 校准电调的步骤：向电调输出100%油门信号，电调上电，听到响声后向电调输出0%油门信号

void select_TIM_APB_AHB(TIM_TypeDef *_TIMx, GPIO_TypeDef *_GPIOx, uint32_t *_rccAPB2Periph_TIM, uint32_t *_rccAPB1Periph_TIM, uint32_t *_rccAHB1Periph_GPIO, 
						bool _useDMA, DMA_Stream_TypeDef* _DMAx_Streamy, uint32_t* _rccAHB1Periph_DMA);
uint8_t get_GPIO_AF(TIM_TypeDef *_TIMx);
uint32_t get_GPIO_Pin(uint32_t _GPIO_PinSource);
void select_TIM_OCInit(TIM_TypeDef *_TIMx, TIM_OCInitTypeDef *_TIM_OCInitStruct, uint8_t _TIM_ch, bool _useDMA);
uint16_t get_TIM_DMA_CC(uint8_t _TIM_ch);
void select_TIM_DMA_PeripheralBaseAddr(TIM_TypeDef *_TIMx, uint8_t _TIM_ch, DMA_InitTypeDef* _DMA_InitStruct);
uint32_t get_DMA_ICIF(DMA_Stream_TypeDef* _DMAx_Streamy);
uint8_t get_DMA_IRQn(DMA_Stream_TypeDef* _DMAx_Streamy);

struct PWM
{
public:
	uint32_t CCR = 0;
	uint16_t* buffer = nullptr;

	PWM(TIM_TypeDef *TIMx, uint8_t _ch, GPIO_TypeDef *GPIOx, uint32_t _GPIO_PinSource, uint16_t _PSC = 168 - 1, uint32_t _ARR = 20000 - 1, 
		bool _useDMA = false, DMA_Stream_TypeDef* _DMAx_Streamy = DMA1_Stream0, uint32_t DMA_Channel = 0, uint32_t _DMA_size = 0, uint16_t* _buffer = nullptr);
	void init(void);

	void setCCR(void);
	void setCCR(uint32_t _newCCR);

	void setDMAenable(void);

private:
	uint8_t TIM_ch = 0;
	TIM_TypeDef *TIMx;
	GPIO_TypeDef *GPIOx;
	uint32_t GPIO_PinSource;
	uint32_t rccAPB2Periph_TIM;
	uint32_t rccAPB1Periph_TIM;
	uint32_t rccAHB1Periph_GPIO;
	uint32_t rccAHB1Periph_DMA;

	uint16_t PSC;
	uint32_t ARR;
	
	DMA_Stream_TypeDef* DMAx_Streamy;
	uint32_t DMA_Channel;
	uint32_t DMA_size;

	bool useDMA = 0;
};

#endif
