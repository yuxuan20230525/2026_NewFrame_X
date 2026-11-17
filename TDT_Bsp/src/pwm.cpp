#include "pwm.h"

PWM::PWM(TIM_TypeDef *_TIMx, uint8_t _ch, GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinSource, uint16_t _PSC, uint32_t _ARR, 
		bool _useDMA, DMA_Stream_TypeDef* _DMAx_Streamy, uint32_t _DMA_Channel, uint32_t _DMA_size, uint16_t *_buffer)
{
    this->TIMx = _TIMx;
    this->TIM_ch = _ch;
    this->GPIOx = _GPIOx;
	this->GPIO_PinSource = _GPIO_PinSource;
	this->PSC = _PSC;
	this->ARR = _ARR;
	this->useDMA = _useDMA;
	if(useDMA)
	{
		this->DMAx_Streamy = _DMAx_Streamy;
		this->DMA_Channel = _DMA_Channel;
		this->DMA_size = _DMA_size;
		this->buffer = _buffer;
	}
}

void PWM::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	select_TIM_APB_AHB(this->TIMx, this->GPIOx, &this->rccAPB2Periph_TIM, &this->rccAPB1Periph_TIM, &this->rccAHB1Periph_GPIO, 
					   this->useDMA, this->DMAx_Streamy, &this->rccAHB1Periph_DMA);
	if(TIMx == TIM1 || TIMx == TIM8)
		RCC_APB2PeriphClockCmd(rccAPB2Periph_TIM, ENABLE);
	else
		RCC_APB1PeriphClockCmd(rccAPB1Periph_TIM, ENABLE);
	RCC_AHB1PeriphClockCmd(rccAHB1Periph_GPIO, ENABLE);

	GPIO_PinAFConfig(this->GPIOx, this->GPIO_PinSource, get_GPIO_AF(this->TIMx));
	GPIO_InitStructure.GPIO_Pin = get_GPIO_Pin(this->GPIO_PinSource);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = this->PSC;
	TIM_TimeBaseStructure.TIM_Period = this->ARR;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	if(TIMx == TIM1 || TIMx == TIM8)
	{
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	}
	
	select_TIM_OCInit(TIMx, &TIM_OCInitStructure, this->TIM_ch, this->useDMA);
	
	if(TIMx == TIM1 || TIMx == TIM8)
		TIM_CtrlPWMOutputs(TIMx, ENABLE);
	TIM_ARRPreloadConfig(TIMx, ENABLE);
	TIM_Cmd(TIMx, ENABLE);
	
	if(this->useDMA)
	{
		TIM_DMACmd(this->TIMx, get_TIM_DMA_CC(this->TIM_ch), ENABLE);
		
		RCC_AHB1PeriphClockCmd(this->rccAHB1Periph_DMA, ENABLE);
		
		DMA_InitTypeDef DMA_InitStruct;
		DMA_DeInit(this->DMAx_Streamy);
		DMA_InitStruct.DMA_Channel = this->DMA_Channel;
		select_TIM_DMA_PeripheralBaseAddr(this->TIMx, this->TIM_ch, &DMA_InitStruct);
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)this->buffer;
		DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStruct.DMA_BufferSize = this->DMA_size;
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		DMA_InitStruct.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
		DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(this->DMAx_Streamy, &DMA_InitStruct);
		
		DMA_Cmd(this->DMAx_Streamy, DISABLE);
		
		DMA_ITConfig(this->DMAx_Streamy, DMA_IT_TC, ENABLE);
		
		NVIC_InitTypeDef NVIC_InitStruct;
		NVIC_InitStruct.NVIC_IRQChannel = get_DMA_IRQn(this->DMAx_Streamy);
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStruct);
		DMA_ClearITPendingBit(this->DMAx_Streamy, get_DMA_ICIF(this->DMAx_Streamy));
	}
}

void PWM::setCCR(void)
{
	switch(this->TIM_ch)
	{
        case 1: this->TIMx->CCR1 = this->CCR; break;
        case 2: this->TIMx->CCR2 = this->CCR; break;
        case 3: this->TIMx->CCR3 = this->CCR; break;
        case 4: this->TIMx->CCR4 = this->CCR; break;
    }
}

// 设置PWM占空比
void PWM::setCCR(uint32_t _newCCR)
{
	this->CCR = (uint32_t)_newCCR;
    switch(this->TIM_ch)
	{
        case 1: this->TIMx->CCR1 = this->CCR; break;
        case 2: this->TIMx->CCR2 = this->CCR; break;
        case 3: this->TIMx->CCR3 = this->CCR; break;
        case 4: this->TIMx->CCR4 = this->CCR; break;
    }
}

void PWM::setDMAenable(void)
{
	DMA_SetCurrDataCounter(this->DMAx_Streamy, (uint16_t)this->DMA_size);
	DMA_Cmd(this->DMAx_Streamy, ENABLE);
	switch(this->TIM_ch)
	{
		case 1: TIM_OC1PreloadConfig(this->TIMx, TIM_OCPreload_Enable); break;
        case 2: TIM_OC2PreloadConfig(this->TIMx, TIM_OCPreload_Enable); break;
        case 3: TIM_OC3PreloadConfig(this->TIMx, TIM_OCPreload_Enable); break;
        case 4: TIM_OC4PreloadConfig(this->TIMx, TIM_OCPreload_Enable); break;
	}
	TIM_Cmd(TIMx, ENABLE);
}

void select_TIM_APB_AHB(TIM_TypeDef *_TIMx, GPIO_TypeDef *_GPIOx, uint32_t *_rccAPB2Periph, uint32_t *_rccAPB1Periph, uint32_t *_rccAHB1Periph, 
						bool _useDMA, DMA_Stream_TypeDef* _DMAx_Streamy, uint32_t* _rccAHB1Periph_DMA)
{
	if(_TIMx == TIM1 || _TIMx == TIM8)
		*_rccAPB2Periph = (uint32_t)(1 << ((((uint32_t)_TIMx - APB2PERIPH_BASE) >> 8) / 0x04));
	else
		*_rccAPB1Periph = (uint32_t)(1 << ((((uint32_t)_TIMx - APB1PERIPH_BASE) >> 8) / 0x04));
	*_rccAHB1Periph = (uint32_t)(1 << ((((uint32_t)_GPIOx - AHB1PERIPH_BASE) >> 8) / 0x04));
		
	if(_useDMA)
		*_rccAHB1Periph_DMA = (uint32_t)(1 << (20 + (((((uint32_t)_DMAx_Streamy - AHB1PERIPH_BASE) >> 8) - 0x60) / 0x04 + 1)));
}

uint8_t get_GPIO_AF(TIM_TypeDef *_TIMx)
{
	if (_TIMx == TIM1 || _TIMx == TIM2)
	{
		return (uint8_t)0x01;
	}
	else if (_TIMx == TIM3 || _TIMx == TIM4 || _TIMx == TIM5)
	{
		return (uint8_t)0x02;
	}
	else if (_TIMx == TIM8 || _TIMx == TIM9 || _TIMx == TIM10 || _TIMx == TIM11)
	{
		return (uint8_t)0x03;
	}
	else
	{
		return (uint8_t)0xFF;
	}
}

uint32_t get_GPIO_Pin(uint32_t _GPIO_PinSource)
{
	return (uint32_t)(1 << _GPIO_PinSource);
}

void select_TIM_OCInit(TIM_TypeDef *_TIMx, TIM_OCInitTypeDef *_TIM_OCInitStruct, uint8_t _TIM_ch, bool _useDMA)
{
    switch (_TIM_ch)
    {
    case 1:
        TIM_OC1Init(_TIMx, _TIM_OCInitStruct);
        TIM_OC1PreloadConfig(_TIMx, _useDMA ? TIM_OCPreload_Disable : TIM_OCPreload_Enable);
        break;
    case 2:
        TIM_OC2Init(_TIMx, _TIM_OCInitStruct);
        TIM_OC2PreloadConfig(_TIMx, _useDMA ? TIM_OCPreload_Disable : TIM_OCPreload_Enable);
        break;
    case 3:
        TIM_OC3Init(_TIMx, _TIM_OCInitStruct);
        TIM_OC3PreloadConfig(_TIMx, _useDMA ? TIM_OCPreload_Disable : TIM_OCPreload_Enable);
        break;
    case 4:
        TIM_OC4Init(_TIMx, _TIM_OCInitStruct);
        TIM_OC4PreloadConfig(_TIMx, _useDMA ? TIM_OCPreload_Disable : TIM_OCPreload_Enable);
        break;
    default:
        break;
    }
}

uint16_t get_TIM_DMA_CC(uint8_t _TIM_ch)
{
	return 1 << (8+_TIM_ch);
}

void select_TIM_DMA_PeripheralBaseAddr(TIM_TypeDef *_TIMx, uint8_t _TIM_ch, DMA_InitTypeDef* _DMA_InitStruct)
{
	switch(_TIM_ch)
	{
		case 1: _DMA_InitStruct->DMA_PeripheralBaseAddr = (uint32_t)&(_TIMx->CCR1); break;
		case 2: _DMA_InitStruct->DMA_PeripheralBaseAddr = (uint32_t)&(_TIMx->CCR2); break;
		case 3: _DMA_InitStruct->DMA_PeripheralBaseAddr = (uint32_t)&(_TIMx->CCR3); break;
		case 4: _DMA_InitStruct->DMA_PeripheralBaseAddr = (uint32_t)&(_TIMx->CCR4); break;
	}
}

uint8_t xxx = 0;
uint8_t yyy = 0;

uint32_t get_DMA_ICIF(DMA_Stream_TypeDef* _DMAx_Streamy)
{
	uint8_t DMA_x;
	uint8_t stream_x;
	
	DMA_x = ((((uint32_t)_DMAx_Streamy - AHB1PERIPH_BASE) >> 8) - 0x60) / 0x04 + 1;

	switch(DMA_x)
	{
		case 1: stream_x = ((uint32_t)_DMAx_Streamy - DMA1_Stream0_BASE) / 0x18; break;
		case 2: stream_x = ((uint32_t)_DMAx_Streamy - DMA2_Stream0_BASE) / 0x18; break;
		default: return 0;
	}
	
	switch(stream_x)
	{
		case 0: return DMA_IT_TCIF0; break;
		case 1: return DMA_IT_TCIF1; break;
		case 2: return DMA_IT_TCIF2; break;
		case 3: return DMA_IT_TCIF3; break;
		case 4: return DMA_IT_TCIF4; break;
		case 5: return DMA_IT_TCIF5; break;
		case 6: return DMA_IT_TCIF6; break;
		case 7: return DMA_IT_TCIF7; break;
		default: return 0;
	}
}

/*
DMA1_Stream0_IRQn = 11,
DMA1_Stream1_IRQn = 12,
DMA1_Stream2_IRQn = 13,
DMA1_Stream3_IRQn = 14,
DMA1_Stream4_IRQn = 15,
DMA1_Stream5_IRQn = 16,
DMA1_Stream6_IRQn = 17,
DMA1_Stream7_IRQn = 47,
DMA2_Stream0_IRQn = 56,
DMA2_Stream1_IRQn = 57,
DMA2_Stream2_IRQn = 58,
DMA2_Stream3_IRQn = 59,
DMA2_Stream4_IRQn = 60,
DMA2_Stream5_IRQn = 68,
DMA2_Stream6_IRQn = 69,
DMA2_Stream7_IRQn = 70,
*/

uint8_t get_DMA_IRQn(DMA_Stream_TypeDef* _DMAx_Streamy)
{
	uint8_t DMA_x;
	uint8_t stream_x;
	
	DMA_x = ((((uint32_t)_DMAx_Streamy - AHB1PERIPH_BASE) >> 8) - 0x60) / 0x04 + 1;

	switch(DMA_x)
	{
		case 1: 
			stream_x = ((uint32_t)_DMAx_Streamy - DMA1_Stream0_BASE) / 0x18; 
			switch(stream_x)
			{
				case 0:
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
					return 11 + stream_x;
				case 7:
					return DMA1_Stream7_IRQn;
			}
			break;
		
		case 2: 
			stream_x = ((uint32_t)_DMAx_Streamy - DMA2_Stream0_BASE) / 0x18; 
			switch(stream_x)
			{
				case 0:
				case 1:
				case 2:
				case 3:
				case 4:
					return 56 + stream_x;
				case 5:
				case 6:
				case 7:
					return 63 + stream_x;
			}
			break;
		
		default: 
			return 0;
	}
	return 0;
}
