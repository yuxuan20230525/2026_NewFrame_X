/*
 * @Date: 2021-12-10 20:23:21
 * @LastEditors: CCsherlock
 * @LastEditTime: 2021-12-10 22:23:13
 * @FilePath: \Projectd:\TDT2022\TDT-sentry\BotGimbal\TDT_Bsp\inc\adc.h
 * @Description:
 */
#ifndef __ADC_H__
#define __ADC_H__
#include "board.h"
struct Adc
{
private:
	/* data */
public:
	Adc(GPIO_TypeDef *_pinPort, uint16_t _pin, ADC_TypeDef *_ADCx, uint8_t _ch);
	GPIO_TypeDef *pinPort = nullptr;	  ///< SCL端口
	ADC_TypeDef *ADCx = nullptr;		  ///< SDA端口
	uint16_t pin = 0;			  ///< SCL引脚
	uint32_t ioRccAHB1Periph = 0; ///< SCL端口时钟
	uint32_t adcAHB2Periph = 0;	  ///< SDA端口时钟
	uint8_t ch = 0;
	void adcInit();
	void Get_Adc();
	int rawData = 0;
	float temperature = 0.0f;
	void readAdc();
};
extern Adc adcMpuTemp;
#endif