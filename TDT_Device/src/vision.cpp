/******************************
Class:  默认配置 空闲中断 dma发送完成中断
	——————————————————————————————————————————————————————————————————————————
	void init();        //初始化
	——————————————————————————————————————————————————————————————————————————
	void run_1000hz();  //运行函数,检查队列里有无数据，有则自动调用发送
	——————————————————————————————————————————————————————————————————————————
	void SendMessage(uint8_t* data, uint16_t size); //队列发送函数
	——————————————————————————————————————————————————————————————————————————
	默认全部接收到 tmp_RecvBuff（在构造函数中填入的地址）
	——————————————————————————————————————————————————————————————————————————
	void ProcessReceivedData(uint8_t* recvBuf); //接收处理，在接收中断时默认运行
	——————————————————————————————————————————————————————————————————————————
	offlineFlag

	若要修改配置，直接在构造函数中修改即可
*/
#include <stdio.h>
#include <math.h>
#include "vision.h"
#include "board.h"
#include "crc.h"
#include "TimeMatch.h"
#include "judgement.h"
#include "imu_task.h"
#include "schedule.h"
#include "usbd_cdc_core.h"
#include "usbd_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

extern CDC_IF_Prop_TypeDef VCP_fops;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /* !< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END = {0};

/// 陀螺仪数据的时间同步
TimeSimultaneity imuTimeMatch(921600);
vision_Recv_Struct_t vision_RecvStruct = {0};
vision_Send_Struct_t vision_SendStruct = {0};
u8 tmp_RecvBuff[200] = {0}; // 缓存区大小为最长的数据包size+1

#if IF_USE_V5_V5PLUS_V6

VISION vision(GPIOC, GPIO_Pin_12, GPIO_PinSource12, GPIO_AF_UART5,
			  GPIOD, GPIO_Pin_2, GPIO_PinSource2, GPIO_AF_UART5,
			  UART5, 921600,
			  DMA_Channel_4, DMA1_Stream0,
			  DMA_Channel_4, DMA1_Stream7,
			  UART5_IRQn, DMA1_Stream7_IRQn, DMA_FLAG_TCIF7, DMA_IT_TCIF7,
			  RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, RCC_APB1Periph_UART5, RCC_AHB1Periph_DMA1,
			  tmp_RecvBuff, sizeof(tmp_RecvBuff));

// 发送中断，一次发送结束后触发，检测消息队列是否为空，若为空且串口空闲，则发送队列中的数据
extern "C" void DMA1_Stream7_IRQHandler(void)
{
	vision.Handle_DMAIC_Interrupt();
}

// 串口接收中断
extern "C" void UART5_IRQHandler(void)
{
	vision.Handle_UARTIdle_Interrupt();
}

#endif

#if IF_USE_DJIC

VISION vision  (GPIOA, GPIO_Pin_9, GPIO_PinSource9, GPIO_AF_USART1,
				GPIOB, GPIO_Pin_7, GPIO_PinSource7, GPIO_AF_USART1,
				USART1, 921600,
				DMA_Channel_4, DMA2_Stream2,
				DMA_Channel_4, DMA2_Stream7,
				USART1_IRQn, DMA2_Stream7_IRQn, DMA_FLAG_TCIF7, DMA_IT_TCIF7,
				RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, RCC_APB2Periph_USART1, RCC_AHB1Periph_DMA2,
				tmp_RecvBuff, sizeof(tmp_RecvBuff));

// HIPNUC_def / HIPNUC_FAILOVER 时 USART1 归 HiPNUC IMU(见 hipnuc_imu.cpp), 让出这两个 ISR 避免重复定义。
// (Vision 对象仍构造, 供 USB 视觉用; USE_VISION=0 时其 USART1 本就不 init)				
#if !(HIPNUC_def || HIPNUC_FAILOVER)
extern "C" void DMA2_Stream7_IRQHandler(void)
{
	vision.Handle_DMAIC_Interrupt();
}

extern "C" void USART1_IRQHandler(void)
{
	vision.offlineFlag = 0;
	vision.offline_cnt = 0;
	vision.Handle_UARTIdle_Interrupt();
}
#endif

#endif

extern "C" void USB_REC_IQRHandler(uint8_t *Buf, uint32_t Len) // usb串口接收中断
{
	vision.USB_offline_cnt = 0;
	vision.USB_offline_Flag = 0;
	memcpy(tmp_RecvBuff, Buf, Len);
	vision.ProcessReceivedData();
}

void VISION::vision_Send_Data()
{
	
}

// 接收处理，在接收中断时自动调用
void VISION::ProcessReceivedData()
{
	const uint64_t recvTimeUs = getSysTimeUs();
	/* 禁止全局中断*/
	__disable_irq();

	float recvTime = getSysTimeUs();
	if (tmp_RecvBuff[0] == 0xA5 && Verify_CRC16_Check_Sum(tmp_RecvBuff, sizeof(vision_RecvStruct))) // 若增加多种数据包，增加判断条件
	{
		switch (tmp_RecvBuff[1]) // 判断接收数据包的type
		{
			case 2:
				memcpy((uint8_t *)&vision_RecvStruct, tmp_RecvBuff, sizeof(vision_RecvStruct));
				imuTimeMatch.Get_time_diff(vision_RecvStruct.time_stamp, sizeof(vision_RecvStruct), recvTimeUs);
				break;
			
			default:
				break;
		}
	}
	/*  使能全局中断 */
	__enable_irq();
}

// 发送数据包，将数据包push进消息队列 队列发送函数
void USART_DMA::SendMessage(uint8_t *data, uint16_t size)
{
	#if USE_USB_VISION
	VCP_fops.pIf_DataTx(data, size);
	#endif
	
	#if USE_VISION
	messageQueue.push({data, size});
	#endif
	
}

// 非队列发送函数
void USART_DMA::DMA_SendData(uint8_t *data, uint16_t size)
{
	DMA_Cmd(dmaStreamTx, DISABLE);
	DMA_InitStructure_tx.DMA_Memory0BaseAddr = (uint32_t)data;
	DMA_InitStructure_tx.DMA_BufferSize = size;
	DMA_ClearFlag(dmaStreamTx, TX_Flag);
	DMA_Init(dmaStreamTx, &DMA_InitStructure_tx);
	DMA_Cmd(dmaStreamTx, ENABLE);
	uartIdle = false;
}

void USART_DMA::run_1000hz() // 需要放到schedule里面运行，检查队列并统一发送
{
	
#if USE_VISION
	if (uartIdle && !messageQueue.empty())
	{
		Message &msg = messageQueue.front_element();
		messageQueue.pop();
		DMA_SendData(msg.data, msg.size); // 使用指针和大小进行发送
	}
	

	if (offline_cnt++ > 300) // 0.3s未收到
	{
		offlineFlag = 1;
		offline_cnt = 300;
	}
#endif

#if USE_USB_VISION
	if (vision.USB_offline_cnt++ > 300)
	{
		vision.USB_offline_Flag = 1;
		vision.USB_offline_cnt = 300;
	}
#endif
}

void USART_DMA::init() // 默认使能空闲中断和dma发送完成中断
{
	RCC_AHB1PeriphClockCmd(clockGPIO, ENABLE);
	RCC_AHB1PeriphClockCmd(clockDMA, ENABLE);

	if (clockUSART == RCC_APB2Periph_USART1 || clockUSART == RCC_APB2Periph_USART6)
		RCC_APB2PeriphClockCmd(clockUSART, ENABLE);
	else
		RCC_APB1PeriphClockCmd(clockUSART, ENABLE);

	// 配置TX引脚
	GPIO_PinAFConfig(txPort, txPinSource, txAF);
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = txPin; // TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(txPort, &GPIO_InitStructure);

	// 配置RX引脚
	GPIO_PinAFConfig(rxPort, rxPinSource, rxAF);
	GPIO_InitStructure.GPIO_Pin = rxPin; // RX
	GPIO_Init(rxPort, &GPIO_InitStructure);

	// USART初始化
	USART_InitTypeDef USART_InitStructure = {0};
	USART_DeInit(usart);
	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(usart, &USART_InitStructure);
	USART_Cmd(usart, ENABLE);

	// NVIC配置接收中断
	NVIC_InitTypeDef NVIC_InitStructure = {0};
	NVIC_InitStructure.NVIC_IRQChannel = irqnRx;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// DMA配置接收
	DMA_DeInit(dmaStreamRx);
	DMA_InitStructure_rx.DMA_Channel = dmaChannelRx;
	DMA_InitStructure_rx.DMA_PeripheralBaseAddr = (uint32_t) & (usart->DR);
	DMA_InitStructure_rx.DMA_Memory0BaseAddr = (uint32_t)recvBuffer; // 使用传入的接收缓冲区
	DMA_InitStructure_rx.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure_rx.DMA_BufferSize = bufferSize;
	DMA_InitStructure_rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_rx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure_rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure_rx.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_rx.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure_rx.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure_rx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure_rx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure_rx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(dmaStreamRx, &DMA_InitStructure_rx);
	DMA_Cmd(dmaStreamRx, ENABLE);

	USART_DMACmd(usart, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
	USART_ITConfig(usart, USART_IT_IDLE, ENABLE);

	// 发送DMA配置
	DMA_DeInit(dmaStreamTx);
	DMA_InitStructure_tx.DMA_Channel = dmaChannelTx;
	DMA_InitStructure_tx.DMA_PeripheralBaseAddr = (uint32_t) & (usart->DR);
	DMA_InitStructure_tx.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure_tx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_tx.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_tx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure_tx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure_tx.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_tx.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure_tx.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init(dmaStreamTx, &DMA_InitStructure_tx);
	DMA_Cmd(dmaStreamTx, DISABLE);

	// NVIC配置发送中断
	NVIC_InitStructure.NVIC_IRQChannel = irqnTx;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	DMA_ITConfig(dmaStreamTx, DMA_IT_TC, ENABLE);
	NVIC_Init(&NVIC_InitStructure);
}


void VISION::USB_init()
{
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
}

void USART_DMA::Handle_DMAIC_Interrupt()
{
	if (DMA_GetITStatus(dmaStreamTx, TX_IT_Flag))
	{
		DMA_ClearITPendingBit(dmaStreamTx, TX_IT_Flag);
		DMA_ClearFlag(dmaStreamTx, TX_IT_Flag);
		uartIdle = true;
		if (!messageQueue.empty() && messageQueue.size() > 10) // 集中释放，防止写入频率超过发送频率
		{
			Message &msg = messageQueue.front_element();
			messageQueue.pop();
			DMA_SendData(msg.data, msg.size); // 使用指针和大小进行发送
		}
	}
}

void USART_DMA::Handle_UARTIdle_Interrupt()
{
	uint8_t tmp = 0;

	// 清除中断标志位
	if (USART_GetITStatus(usart, USART_IT_IDLE) != RESET)
	{
		tmp = usart->SR;
		tmp = usart->DR;

		// 禁用DMA
		DMA_Cmd(dmaStreamRx, DISABLE);
		USART_ClearITPendingBit(usart, USART_IT_IDLE);

		// 调用处理接收数据的函数
		ProcessReceivedData();

		// 等待DMA禁用完成
		while (DMA_GetCmdStatus(dmaStreamRx) != DISABLE)
			;

		// 重新初始化DMA
		DMA_DeInit(dmaStreamRx);
		DMA_Init(dmaStreamRx, &DMA_InitStructure_rx);
		DMA_SetCurrDataCounter(dmaStreamRx, bufferSize);
		DMA_Cmd(dmaStreamRx, ENABLE);
	}
}
