/******************************
File name: TDT_Device\src\camera_Timematch.cpp
Description: 硬同步
————————————————————————————————————————————————————————————————————————————————————————————————————————
基本原理：C板通过GPIOE11给相机发送定频触发信号，相机取帧完之后通过GPIOE9给c板发送取帧完的信号，
          接收到的取帧信号逐帧编号，编号的同时记录某帧下对应的imu值，方便视觉算法逐帧匹配对应的imu值。
————————————————————————————————————————————————————————————————————————————————————————————————————————
特殊处理：(1)高电平校验：每xx帧C板通过GPIOE13发送一次高电平，此时视觉算法检测是否是xx帧的整数倍。
          (2)异常情况处理：电控方出现异常，视觉串口中发送的请求信号置0；视觉算法方出现异常，视觉串口中接收的请求信号为1/2，
					                 此时接收取帧信号计数器/ID号清零；只有双方请求信号都置2时可以正常工作。
————————————————————————————————————————————————————————————————————————————————————————————————————————
可配置参数：(1)触发信号频率  发送频率=(168M/TIM_Prescaler/TIM_Period)hz 
            (2)idcheckFreq   实现第idcheckFreq帧触发一次高电平校验
————————————————————————————————————————————————————————————————————————————————————————————————————————
线序：触发线上不同序号对应C板引脚——4对应GPIOE9，2对应GPIOE11，3对应GPIOE13，5和6对应地
————————————————————————————————————————————————————————————————————————————————————————————————————————
相关视觉串口内容：发送——(1)cameraID,填充该文件的cameraTM.recieve_frameID
                        (2)send_request_signal，接收到recieve_request_signal为1时发送2，接收到recieve_request_signal为0时发送0
												(3)current_yaw/pitch/roll,实时填充imu值；camera_yaw/pitch/roll,填充该文件的cameraTM.camera_yaw/pitch/roll
									接收——recieve_request_signal
—————————————————————————————————————————————————————————————————————————————————————————————————————————
PS:在board.h将USE_CAMERATM置1
   填充上述所需视觉串口内容，解注释本文件run()中的if(vision_Recv_Struct.recieve_request_signal==1 || vision_Recv_Struct.recieve_request_signal==0)。
********************************/
#include "camera_Timematch.h"
#include "imu_task.h"
#include "vision.h"

CameraTM cameraTM;

CameraTM::CameraTM()
{
}
/**
 * @brief 硬同步相关引脚初始化
 */
void CameraTM::init()
{
	EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);//使能定时器1时钟
	
	//接收相机取帧信号引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//GPIOE9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//速度100MHz/高低电平翻转频率
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//没有外部信号时引脚默认高电平
	GPIO_Init(GPIOE,&GPIO_InitStructure);//初始化PE9

	//发送定频触发信号引脚
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);//GPIOE11复用为定时器1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//GPIOE11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//速度100MHz/高低电平翻转频率
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无上下拉（输出模式推荐）
	GPIO_Init(GPIOE,&GPIO_InitStructure);//初始化PE11
	
	//发送高电平校验信号引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//GPIOE13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//速度100MHz/高低电平翻转频率
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无上下拉（输出模式推荐）
	GPIO_Init(GPIOE,&GPIO_InitStructure);//初始化PE13
	GPIO_ResetBits(GPIOE, GPIO_Pin_13); //输出默认低电平
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9); //将GPIO端口与中断线映射起来
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line9;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;//中断模式
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;//启用外部中断
  EXTI_Init(&EXTI_InitStruct);
    
  NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;//EXTI5-9共享中断
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
	
	//配置触发信号发送频率
	//TIM1时钟频率为168MHz 发送频率 = (168M/TIM_Prescaler/TIM_Period) hz
	TIM_TimeBaseStructure.TIM_Period = 10000-1;//设置自动重装载值 10000/1M = 10ms->100hz
  TIM_TimeBaseStructure.TIM_Prescaler = 168-1;//设置预分频值 168MHz/168=1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//设置时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);//初始化TIM1

	TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择PWM模式1，当计数器<CCR时输出有效电平
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//有效电平为低电平
	TIM_OCInitStructure.TIM_Pulse = 5000; //占空比设置为5000/10000
  TIM_OC2Init(TIM1, &TIM_OCInitStructure); //初始化TIM1
	
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);//使能TIM1预装载寄存器，修改CCR1值后等到下一个更新事件才生效，确保波形完整
	TIM_ARRPreloadConfig(TIM1,ENABLE);//修改ARR后，等到下一个更新事件才生效
  TIM_CtrlPWMOutputs(TIM1, ENABLE);//显式使能主输出才有PWM信号
	TIM_Cmd(TIM1, ENABLE); //使能TIM1
	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);	// 允许定时器1更新中断
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除中断标志位
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;//TIM1更新中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief 硬同步特殊处理
 */
void CameraTM::run()
{
	//高电平校验处理
	//实现第idcheckFreq帧的时候拉高电平，其他时刻默认低电平
	idcheckFreq = 10;
	cameraTM.last_digit = cameraTM.recieve_frameID % idcheckFreq;
	if(cameraTM.last_digit== 0 && cameraTM.recieve_frameID!=0)
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
	else
		GPIO_ResetBits(GPIOE, GPIO_Pin_13); 
	
	//异常情况相机取帧信号清零处理
	//如果接收到算法请求计数器清零标志位，此时相机已停止，电控计数器需清零
//	if(vision_Recv_Struct.recieve_request_signal==1 || vision_Recv_Struct.recieve_request_signal==0)
	{
		cameraTM.recieve_frameID = 0;  
	}
}

/**
 * @brief 接收相机取帧信号
 */
void EXTI9_5_IRQHandler(void) 
{
    if (EXTI_GetITStatus(EXTI_Line9) != RESET) 
    {		
				cameraTM.recieve_frameID++;
			  cameraTM.camera_yaw = boardImu->FusionAngle.yaw;
        cameraTM.camera_pitch = boardImu->FusionAngle.pitch;
			  cameraTM.camera_roll = boardImu->FusionAngle.roll;
			
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

/**
* @brief 发送硬同步定频触发信号
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    //检查是否是更新中断
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
			  cameraTM.send_frameID++;
        //清除中断标志
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
