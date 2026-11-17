#include "vofa.h"

VOFA vofa;
volatile bool vofaIdle = true;

#if USE_Judgement_Vofa
void VOFA::init()
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

#if IF_USE_V5_V5PLUS_V6
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#if IF_USE_DJIC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
#endif

	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStructure);
	USART_Cmd(USART6, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void VOFA::DMA_Send(uint8_t *data, uint16_t size)
{
	static DMA_InitTypeDef DMA_InitStructure;
	static NVIC_InitTypeDef NVIC_InitStructure;
	static u8 firstLoad = 1; // 首次加载时加载配置缺省值
	if (firstLoad == 1)
	{
		firstLoad = 0;

		DMA_InitStructure.DMA_Channel = DMA_Channel_5;						  // 配置为 USART6 的 DMA 通道
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR); // 外设基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				  // 数据传输方向：内存到外设

		// 配置 DMA 缓存大小和地址递增模式
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		// 外设地址不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// 内存地址递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度为八位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			// 数据宽度为八位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// 工作模式为正常
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;						// DMA 通道拥有高优先级

		// NVIC 配置
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;	  // 修改为 DMA2_Stream6 的中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 设定抢占优先级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 设定子优先级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // 使能中断
		DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
		NVIC_Init(&NVIC_InitStructure);
	}

	// 设置 DMA 内存基地址和缓存大小
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)data; // 内存基地址
	DMA_InitStructure.DMA_BufferSize = size;				// DMA 缓存大小

	// 清除 DMA 标志位
	DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6); // 清除传输完成标志位

	// 初始化 DMA2 Stream6
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);

	// 启用 DMA2 Stream6
	DMA_Cmd(DMA2_Stream6, ENABLE);
	vofaIdle = false;
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6)) // 修改为 Stream6 的中断标志位
	{
		DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6); // 清除中断挂起位
		DMA_ClearFlag(DMA2_Stream6, DMA_IT_TCIF6);		   // 清除 Stream6 的标志位
		vofaIdle = true;								   // 设置 UART 空闲状态
	}
}


void VOFA::run()
{
	if (vofaIdle && !sendPackage.empty())
	{
		SendMessage &msg = sendPackage.front_element();
		sendPackage.pop();
		vofa.DMA_Send(msg.data, msg.size); // 使用指针和大小进行发送
	}
}


#endif

#if USE_VISION_Vofa


void VOFA::init()
{
	Vision.init();
}

void VOFA::run()
{
	if (Vision.uartIdle && !sendPackage.empty())
	{
		SendMessage &msg = sendPackage.front_element();
		sendPackage.pop();
		Vision.DMA_SendData(msg.data, msg.size); // 使用指针和大小进行发送
	}
}


#endif 


void VOFA::printf(float value,u8 id)
{
	sendpack.fdata[id]=value;
	sendPackage.push({(u8*)&sendpack, sizeof(sendpack)});
};
	
