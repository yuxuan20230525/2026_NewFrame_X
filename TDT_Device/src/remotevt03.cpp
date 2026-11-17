/******************************
File name: TDT_Device\src\dbus.cpp
Description: 遥控器接收机
function:
	——————————————————————————————————————————————————————————————————————————
	void Dbus_Config(void)
	——————————————————————————————————————————————————————————————————————————
	void USART2_IRQHandler(void)
	——————————————————————————————————————————————————————————————————————————
	static void Handle_data(volatile const uint8_t *sbus_buf, struct _RC *rc_ctrl)
	——————————————————————————————————————————————————————————————————————————
	int Get_Keypress(uint16_t Key)
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加切换为原始数据处理的变量
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次记录
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "remotevt03.h"
#include "board.h"
#include "string.h"
#include "task_virtual.h"
#include "KeyProcess.h"
#include "judgement.h"


/**
 * @ingroup TDT_DEVICE
 * @defgroup DBUS 遥控器解算
 * @brief 该类提供了遥控器的数据解算，以及常用的重要标志位
 */
_RCVT03 RCVT03;
uint8_t deforceFlagvt03 = 0;
int dbusOnlineCnt=0;
uint8_t dbusOnlineFlagvt03 = 0;

#define VT03SIZEBUFF 256
#define REMOTE_PACKET_SIZE 21


#if USE_RCVT03
uint8_t SBUS_rx_bufvt03[VT03SIZEBUFF];//DMA的原始数据
uint8_t TxBuffer[50];
static uint16_t crc16_init = 0xffff; 
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
void _RCVT03::init()
{
	deforceFlagvt03 = 1;
	/*DBUS初始化*/
	Dbus_Config();
}


/**
  * @brief 遥控器初始化
  */
void _RCVT03::Dbus_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
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
	USART_InitStructure.USART_BaudRate = 921600;
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

	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SBUS_rx_bufvt03;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = sizeof(SBUS_rx_bufvt03);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);
}



void _RCVT03::run_1000Hz()
{
	dbusOnlineCnt++;
	if(dbusOnlineCnt>50)
		dbusOnlineFlagvt03=0;
	else
		dbusOnlineFlagvt03=1;
	if(dbusOnlineFlagvt03)
	{
		handleData();
		updated = 0;
		//RCVT03.taskSchedule();
		return;
	}
	if (timeIntervalFrom(updateTime) > 500000) //500ms
	{
		deforceFlagvt03 = 1;
		/*离线检测*/
		if (dbusOnlineFlagvt03 == 1)//后面置一代表相当于离线跳变
		{
			//清除旧数据
			memset(&Key, 0, sizeof(Key));
			//仍然进行按键处理，不过所有键松开
			KeyProcess::keyHandle(0);
			deforceFlagvt03 = 1;
			//仍然进行任务调度
			//taskSchedule();
		}
		dbusOnlineFlagvt03 = 0;
	}
}

void _RCVT03::handleData()
{
	remote_data_process_advanced();
	CH[0]=Key.ch_0-1024;
	CH[1]=Key.ch_1-1024;
	CH[2]=Key.ch_2-1024;
	CH[3]=Key.ch_3-1024;
	
}



#include "imu_task.h"

void _RCVT03::rstCount(void)
{
	static u8 RST_Delay_Cnt = 0;
	static u8 HasRST = 1;
	if (((int)CH[0]) > 600 && ((int)CH[1]) < -600 && ((int)CH[2]) < -600 && ((int)CH[3]) < -600)
	{
		//▼ 自检
		if (Key.fn_1 == 1)
		{
		}
		//▼ 重启
		// if (Key.SW1 == Down && HasRST == 0)
		// {
		// 	RST_Delay_Cnt++;
		// 	if (RST_Delay_Cnt > 150) //150*14ms大约为2s
		// 	{
		// 		__disable_irq(); //关闭所有中断
		// 		NVIC_SystemReset(); //复位
		// 		while (1)
		// 		{
		// 		} //仅等待复位
		// 	}
		// }
	}
	//校准陀螺仪
	else if (((int)CH[0]) < -600 && ((int)CH[1]) < -600 && ((int)CH[2]) > 600 && ((int)CH[3]) < -600)
	{
		//▼ 校准陀螺仪
		if (Key.fn_1 == 1)
		{
			imutask.forceGetOffset = 1;
		}
	}
	else
	{
		RST_Delay_Cnt = 0; //清空计数器
		HasRST = 0;
	}
}

void _RCVT03::remoteUpdate()
{
	//调用其他任务的遥控器更新函数
	for (int i = 0; i < taskControl.taskNum; i++)
	{
		taskControl.taskList[i]->remoteCtrlUpdate();
	}
	KeyProcess::keyHandle(Key.keyValue);
}



/**
  * @brief 任务调度
  * @note 通过遥控器指令执行相关任务的挂起和恢复
  * @warning 脱力控制也在这边，注意一下
  */
void _RCVT03::taskSchedule()
{
	//▲ 如果到脱力键，发送通知到紧急停止任务，不带通知值，且不保留接受任务的通知值，接受任务的通知值加一
	if(Key.pause&&!deforceFlag)
		pauseCnt++;
	else
		pauseCnt=0;
	if(pauseCnt==100)
	{
		if(deforceFlagvt03==1)
			deforceFlagvt03=0;
	}
	if((pauseCnt==1)&&!deforceFlagvt03)
	{
		if(deforceFlagvt03==0)
			deforceFlagvt03=1;
	}
	if (!deforceFlagvt03)
	{ //调用其他任务的脱力回调函数
		for (int i = 0; i < taskControl.taskNum; i++)
		{
			taskControl.taskList[i]->deforceCallBack();
		}
		//执行重启计数函数，等待特定的重启操作后将重启
		rstCount();
	}
	//脱力状态解除，恢复挂起的任务，执行一些初始化
	else
	{
			imutask.forceGetOffset = 0;
			//调用其他任务的脱力回调函数
			for (int i = 0; i < taskControl.taskNum; i++)
			{
				taskControl.taskList[i]->deforceCancelCallBack();
			}
	}
}


// 上一次DMA读取位置
static volatile uint16_t dma_last_pos = 0;
// 接收到的遥控器数据
volatile uint8_t remote_data_ready = 0;
uint32_t remote_packet_count = 0;
uint32_t remote_error_count = 0;



void _RCVT03::remote_data_process_advanced(void)
{
    uint16_t current_pos, bytes_available, bytes_processed = 0;
    uint16_t buffer_index, i;
    static uint8_t packet_buffer[REMOTE_PACKET_SIZE];
    static uint8_t packet_index = 0;
    static uint8_t packet_state = 0;
    
    current_pos = get_dma_current_position();
    
    // 计算可用字节数
    if (current_pos >= dma_last_pos) {
        bytes_available = current_pos - dma_last_pos;
    } else {
        bytes_available = (VT03SIZEBUFF - dma_last_pos) + current_pos;
    }
    
    // 处理所有可用数据
    while (bytes_processed < bytes_available) {
        // 获取当前字节
        buffer_index = (dma_last_pos + bytes_processed) % VT03SIZEBUFF;
        uint8_t byte = SBUS_rx_bufvt03[buffer_index];
        bytes_processed++;
        
        // 状态机解析
        if (packet_state == 0) {
            // 状态0: 寻找包头1
            if (byte == 0xA9) {
                packet_buffer[0] = byte;
                packet_index = 1;
                packet_state = 1;
            }
        } else if (packet_state == 1) {
            // 状态1: 检查包头2
            if (byte == 0x53) {
                packet_buffer[1] = byte;
                packet_index = 2;
                packet_state = 2;
            } else {
                // 包头2不匹配，重新开始
                packet_state = 0;
            }
        } else if (packet_state == 2) {
            // 状态2: 接收数据
            if (packet_index < REMOTE_PACKET_SIZE) {
                packet_buffer[packet_index] = byte;
                packet_index++;
                
                // 检查是否收到完整包
                if (packet_index == REMOTE_PACKET_SIZE) {
                    // CRC校验
                    if (verify_crc16_check_sum(packet_buffer, REMOTE_PACKET_SIZE)) {
                        // 复制数据
                        memcpy(&RCVT03.Key, packet_buffer, REMOTE_PACKET_SIZE);
                        remote_data_ready = 1;
                        remote_packet_count++;
                    } else {
                        remote_error_count++;
                    }
                    packet_state = 0;  // 准备接收下一包
                }
            } else {
                // 异常情况，重置
                packet_state = 0;
            }
        }
    }
    
    // 更新读指针
    dma_last_pos = (dma_last_pos + bytes_processed) % VT03SIZEBUFF;
}







/**
 * @brief 接收DMA定义
 * @note  由于裁判系统发送一帧的中间会停止发送，故不能用空闲中断+DMA进行读取
 * 		  而使用DMA循环接收+各个字节轮询遍历的方法
 */
extern "C" void DMA2_Stream1_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
	{
		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		dbusOnlineCnt=0;
	}
}



static uint16_t get_dma_current_position(void)
{
    // 计算DMA当前写位置
    return VT03SIZEBUFF - DMA_GetCurrDataCounter(DMA2_Stream1);
}



static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }
    return crc16;
}


bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len) 
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

#endif
