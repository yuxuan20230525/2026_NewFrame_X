
/**
	大疆c板的蜂鸣器程序
	使用方法：
	MusicPlayer  music 声明音频
	music[i].musicList={
		{300,0.15},
		{400,0.15},
		{500,0.2}
	};                对应 频率 持续时间  （频率最大4000hz）

	ErrorLink(&Init_OK,0);  当变量从0跳变成1的时候 播放 0 号音频
	
	setnow(index),立刻播放一次index号音频，如果0.3s未调用，则复位。


*/

#include "buzzer.h"
#include "my_math.h"

Buzzer buzzer;

Buzzer::Buzzer()
{
	music[0].musicList = {
		{300, 0.15},
		{400, 0.15},
		{500, 0.2}};
	
	music[1].musicList = {
		{200, 0.15},
	};
			
	music[2].musicList = {
		{200, 0.15},
		{300, 0.15},};
	
	music[3].musicList = {
	{200, 0.15},
	{300, 0.15},
	{400, 0.15},};
	
	music[4].musicList = {
	{200, 0.15},
	{300, 0.15},
	{400, 0.15},
	{500, 0.15}};
	ErrorLink(&Init_OK, 0);
}

int8_t index;
void Buzzer::Run()
{
	index = getError();

	if (index != -1)
	{
		#if USE_DJIC
		TIM_Cmd(TIM4, ENABLE);
		#endif
		#if USE_OTHER
		TIM_Cmd(TIM8, ENABLE);
		#endif
		music[index].enable = 1;
		return;
	}

	for (int i = 0; i < 10; i++)
	{
		if (music[i].enable == 1)
		{
			sethz(music[i].play());
			return;
		}
	}
	#if USE_DJIC
	TIM_Cmd(TIM4, DISABLE);
	#endif
	#if USE_OTHER
	TIM_Cmd(TIM8, DISABLE);
	#endif
}

void Buzzer::init()
{

	GPIO_InitTypeDef gpioInitStructure; // 蜂鸣器初始化
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	#if USE_DJIC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	gpioInitStructure.GPIO_Pin = GPIO_Pin_14;
	gpioInitStructure.GPIO_Mode = GPIO_Mode_AF;		  // 设置为复用功能模式
	gpioInitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 设置为 100 MHz 的高速模式
	gpioInitStructure.GPIO_OType = GPIO_OType_PP;	  // 设置为推挽输出
	gpioInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	  // 不使用上拉或下拉
	GPIO_Init(GPIOD, &gpioInitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); // 设置复用功能为 TIM4
	// 使能 TIM4 时钟

	// 配置定时器：PWM 模式
	TIM_TimeBaseInitStruct.TIM_Prescaler = 10 - 1;				 // 设置预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
	TIM_TimeBaseInitStruct.TIM_Period = 1049;					 // 设置自动重载值
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	 // 不分频
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;			 // 重复计数器值为 0
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	// 配置 TIM4 Channel 2 为 PWM 输出模式
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;			   // 设置为 PWM 模式 1
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; // 启用输出
	TIM_OCInitStruct.TIM_Pulse = 0;							   // 设置占空比为 0%
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;	   // 高电平有效
	TIM_OC3Init(TIM4, &TIM_OCInitStruct);					   // 配置通道 2 的 PWM 输出

	// 启动 TIM4 Channel 2 输出 PWM
	TIM_Cmd(TIM4, ENABLE);
  #endif
	#if USE_OTHER
	// 1. 使能时钟 (TIM8属于APB2总线，GPIOI属于AHB1总线)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE); // 使能GPIOI时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  // 使能TIM8时钟

	// 2. 配置GPIO引脚 PI7 为 TIM8_CH3
	gpioInitStructure.GPIO_Pin = GPIO_Pin_7;
	gpioInitStructure.GPIO_Mode = GPIO_Mode_AF;          // 复用功能模式
	gpioInitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	gpioInitStructure.GPIO_OType = GPIO_OType_PP;        // 推挽输出
	gpioInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     // 无上下拉
	GPIO_Init(GPIOI, &gpioInitStructure);
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_TIM8); // 将PI7复用为TIM8

	// 3. 配置定时器时基（这部分与之前相同）
	TIM_TimeBaseInitStruct.TIM_Prescaler =  10 - 1;              // 设置预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseInitStruct.TIM_Period = 1049;                   // 设置自动重载值ARR
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;           // 重复计数器，高级定时器特有，设为0
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

	// 4. 配置TIM8通道3为PWM模式
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;            // PWM模式1
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable; // 互补输出不使能
	TIM_OCInitStruct.TIM_Pulse = 0;                           // 初始占空比为0
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;    // 输出极性高电平有效
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;   // 互补输出极性
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC3Init(TIM8, &TIM_OCInitStruct);                     // 初始化通道3

	// 5. 使能定时器及PWM输出（高级定时器关键步骤）
	TIM_Cmd(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);                         // 必须调用此函数使能主输出
	#endif
}

void Buzzer::sethz(uint16_t frequency)
{

	uint16_t arr = 0;
	#if USE_DJIC
	if (frequency <= 50)
	{
		TIM_SetCompare3(TIM4, 0);
		arr = 1049;
		return;
	}
	arr = 4000.0f / (float)LIMIT(frequency, 50, 4000) * 1049;
	TIM4->ARR = arr;
	TIM_SetCompare3(TIM4, arr / 2);
	#endif
	
	#if USE_OTHER
	if (frequency <= 50)
	{
		TIM_SetCompare3(TIM8, 0);
		arr = 1049;
		return;
	}
	arr = 4000.0f / (float)LIMIT(frequency, 50, 4000) * 1049;
	TIM8->ARR = arr;
	TIM_SetCompare3(TIM8, arr / 2);
	#endif
}

void Buzzer ::ErrorLink(uint8_t *iferr, uint8_t priority)
{
	if (priority >= 10)
		return;
	ErrorCode[priority].iferr = iferr;
}

int8_t Buzzer::getError()
{
	for (u8 i = 0; i < 10; i++)
	{
		if(timeIntervalFrom_f(ErrorCode[i].setnow_time/1e6f)>0.3)
		{
			ErrorCode[i].setNow=0;
		}
	}
	
	// LED只展示位置最先的异常（表现为位于数组的第一个有效数据）
	for (u8 i = 0; i < 10; i++)
	{
		if ((ErrorCode[i].iferr != nullptr && *(ErrorCode[i].iferr) == 1 && ErrorCode[i].last_iferr == 0 )|| (ErrorCode[i].setNow==1 &&ErrorCode[i].lastsetNow==0))
		{
			ErrorCode[i].last_iferr = *(ErrorCode[i].iferr);
			ErrorCode[i].lastsetNow=ErrorCode[i].setNow;
			return i;
		}
		ErrorCode[i].last_iferr = *(ErrorCode[i].iferr);
		ErrorCode[i].lastsetNow=ErrorCode[i].setNow;
	}

	return -1;
}

void Buzzer :: setnow(u8 index)
{
	if(index >=10)
		return;
	ErrorCode[index].setNow=1;
	ErrorCode[index].setnow_time= getSysTimeUs();
}