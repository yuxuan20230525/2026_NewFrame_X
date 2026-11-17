
/**
	��c��ķ���������
	ʹ�÷�����
	MusicPlayer  music ������Ƶ
	music[i].musicList={
		{300,0.15},
		{400,0.15},
		{500,0.2}
	};                ��Ӧ Ƶ�� ����ʱ��  ��Ƶ�����4000hz��

	ErrorLink(&Init_OK,0);  ��������0�����1��ʱ�� ���� 0 ����Ƶ
	
	setnow(index),���̲���һ��index����Ƶ�����0.3sδ���ã���λ��


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

int8_t buzzer_index;
void Buzzer::Run()
{
	buzzer_index = getError();

	if (buzzer_index != -1)
	{
		#if USE_DJIC
		TIM_Cmd(TIM4, ENABLE);
		#endif
		#if USE_OTHER
		TIM_Cmd(TIM8, ENABLE);
		#endif
		music[buzzer_index].enable = 1;
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

	GPIO_InitTypeDef gpioInitStructure; // ��������ʼ��
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	#if USE_DJIC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	gpioInitStructure.GPIO_Pin = GPIO_Pin_14;
	gpioInitStructure.GPIO_Mode = GPIO_Mode_AF;		  // ����Ϊ���ù���ģʽ
	gpioInitStructure.GPIO_Speed = GPIO_Speed_100MHz; // ����Ϊ 100 MHz �ĸ���ģʽ
	gpioInitStructure.GPIO_OType = GPIO_OType_PP;	  // ����Ϊ�������
	gpioInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	  // ��ʹ������������
	GPIO_Init(GPIOD, &gpioInitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); // ���ø��ù���Ϊ TIM4
	// ʹ�� TIM4 ʱ��

	// ���ö�ʱ����PWM ģʽ
	TIM_TimeBaseInitStruct.TIM_Prescaler = 10 - 1;				 // ����Ԥ��Ƶ��
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_Period = 1049;					 // �����Զ�����ֵ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	 // ����Ƶ
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;			 // �ظ�������ֵΪ 0
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	// ���� TIM4 Channel 2 Ϊ PWM ���ģʽ
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;			   // ����Ϊ PWM ģʽ 1
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; // �������
	TIM_OCInitStruct.TIM_Pulse = 0;							   // ����ռ�ձ�Ϊ 0%
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;	   // �ߵ�ƽ��Ч
	TIM_OC3Init(TIM4, &TIM_OCInitStruct);					   // ����ͨ�� 2 �� PWM ���

	// ���� TIM4 Channel 2 ��� PWM
	TIM_Cmd(TIM4, ENABLE);
  #endif
	#if USE_OTHER
	// 1. ʹ��ʱ�� (TIM8����APB2���ߣ�GPIOI����AHB1����)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE); // ʹ��GPIOIʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  // ʹ��TIM8ʱ��

	// 2. ����GPIO���� PI7 Ϊ TIM8_CH3
	gpioInitStructure.GPIO_Pin = GPIO_Pin_7;
	gpioInitStructure.GPIO_Mode = GPIO_Mode_AF;          // ���ù���ģʽ
	gpioInitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	gpioInitStructure.GPIO_OType = GPIO_OType_PP;        // �������
	gpioInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     // ��������
	GPIO_Init(GPIOI, &gpioInitStructure);
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_TIM8); // ��PI7����ΪTIM8

	// 3. ���ö�ʱ��ʱ�����ⲿ����֮ǰ��ͬ��
	TIM_TimeBaseInitStruct.TIM_Prescaler =  10 - 1;              // ����Ԥ��Ƶ��
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���
	TIM_TimeBaseInitStruct.TIM_Period = 1049;                   // �����Զ�����ֵARR
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;           // �ظ����������߼���ʱ�����У���Ϊ0
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

	// 4. ����TIM8ͨ��3ΪPWMģʽ
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;            // PWMģʽ1
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; // ʹ�����
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable; // ���������ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                           // ��ʼռ�ձ�Ϊ0
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;    // ������Ըߵ�ƽ��Ч
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;   // �����������
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC3Init(TIM8, &TIM_OCInitStruct);                     // ��ʼ��ͨ��3

	// 5. ʹ�ܶ�ʱ����PWM������߼���ʱ���ؼ����裩
	TIM_Cmd(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);                         // ������ô˺���ʹ�������
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
	
	// LEDֻչʾλ�����ȵ��쳣������Ϊλ������ĵ�һ����Ч���ݣ�
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