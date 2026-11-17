#include "stm32f4xx.h"                  // Device header
#include "mtf02.h"
#include "board.h"
#include <string.h>

MT0X_RC MT02_RC;
constexpr uint16_t MTF02_RX_BUF_NUM = 64;
// DMA 双缓冲
static uint8_t mtf02_rx_buf[2][MTF02_RX_BUF_NUM] = {};
// 软件暂存区
static uint8_t mtf02_pending_buf[MTF02_RX_BUF_NUM] = {};
static volatile uint16_t rxLength = 0;
static volatile uint8_t rxReady = 0;
static volatile uint8_t rxLost = 0;

void MT0X_RC::Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
#if IF_USE_DJIC

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);	// 复位USART1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE); // 停止复位USART1

	/* -------------- GPIO 引脚配置 ---------------------------------- */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); // PB7 为 USART1 RX

	/* -------------- USART 配置 ------------------------------------- */
	USART_DeInit(USART1);

	USART_InitStructure.USART_BaudRate = 115200; // 波特率可根据需求调整
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No; // 校验位
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStructure);

	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

	USART_ClearFlag(USART1, USART_FLAG_IDLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

	USART_Cmd(USART1, ENABLE);

	/* -------------- NVIC 配置 -------------------------------------- */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//	/* -------------- DMA 配置 --------------------------------------- */
	DMA_DeInit(DMA2_Stream2);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4; // USART1 Rx 对应的 DMA 通道
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mtf02_rx_buf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = MTF02_RX_BUF_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);

	DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)mtf02_rx_buf[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream2, ENABLE);

#endif
}
#if USE_MTF0X
extern "C" void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) == RESET)
        return;

    // 清除空闲中断标志
    (void)USART1->SR;
    (void)USART1->DR;
	
	 // 检查是否发生了 DMA 缓冲区满传输
    uint8_t dmaFull =
        (DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) != RESET);
	
    // 暂停 DMA
    DMA_Cmd(DMA2_Stream2, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE) {}

    // 获取当前 DMA 缓冲区
    uint8_t bank =
        (DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0) ? 0 : 1;

    // 获取实际接收长度
    uint16_t length =
        MTF02_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);

    // 重置 DMA
    DMA_SetCurrDataCounter(DMA2_Stream2, MTF02_RX_BUF_NUM);

    if (bank == 0)
        DMA2_Stream2->CR |= DMA_SxCR_CT;
    else
        DMA2_Stream2->CR &= ~DMA_SxCR_CT;

    DMA_ClearFlag(DMA2_Stream2,
                  DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);

    // 恢复 DMA
    DMA_Cmd(DMA2_Stream2, ENABLE);

    // 满缓冲区的连续接收暂不在此版本中处理
    if (dmaFull)
    {
        rxLost = 1;
        return;
    }

    if (length == 0)
        return;

    // 暂存区尚未被主循环取走
    if (rxReady || rxLost)
    {
        rxLost = 1;
        return;
    }

    // 复制本次实际接收到的数据
    memcpy(mtf02_pending_buf, mtf02_rx_buf[bank], length);

    rxLength = length;
    __DMB();
    rxReady = 1;
}
#endif

void MT0X_RC::init(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();

    frameLength = 0;
    rxLength = 0;
    rxReady = 0;
    rxLost = 0;
    updated = 0;
    updateTime = 0;
    Config();
    __DMB();
    __set_PRIMASK(mask);
}

void MT0X_RC::run(void)
{
    updated = 0;

    uint8_t data[MTF02_RX_BUF_NUM];
    uint16_t length = 0;

    // 临时关闭中断，安全地复制暂存数据
    uint32_t mask = __get_PRIMASK();
    __disable_irq();

    if (rxLost)
    {
        // 发生丢包，丢弃暂存数据并重新同步
        rxReady = 0;
        rxLength = 0;
        rxLost = 0;
        frameLength = 0;
    }
    else if (rxReady)
    {
        length = rxLength;

        memcpy(data, mtf02_pending_buf, length);

        rxReady = 0;
        rxLength = 0;
    }

    __DMB();
    __set_PRIMASK(mask);

    // 恢复中断后再进行组帧
    if (length > 0)
    {
        ProcessRxBytes(data, length);
    }
}

bool MT0X_RC::handleData(volatile const uint8_t *MTF_rx_buffer)
{
	if (MTF_rx_buffer[0] != 0xEF ||
	MTF_rx_buffer[1] != 0x0F ||MTF_rx_buffer[2] != 0x00 ||
	MTF_rx_buffer[3] != 0x51 ||MTF_rx_buffer[5] != 0x14)
	{
		return false;
	}
	
	uint8_t checksum = 0;// 前26个字节累加，保留低8位

	for (uint8_t i = 0; i < 26; ++i)
	{
		checksum = uint8_t(checksum + MTF_rx_buffer[i]);
	}
	if (checksum != MTF_rx_buffer[26])
	{
		return false;
	}
	Now_OP.time_ms = ((uint32_t)MTF_rx_buffer[9]<<24)|((uint32_t)MTF_rx_buffer[8]<<16)|
	((uint32_t)MTF_rx_buffer[7]<<8)|MTF_rx_buffer[6];// 系统时间 ms
	Now_OP.distance = ((uint32_t)MTF_rx_buffer[13]<<24)|((uint32_t)MTF_rx_buffer[12]<<16)|
	((uint32_t)MTF_rx_buffer[11]<<8)|MTF_rx_buffer[10];// 距离(mm) 最小值为10，0表示数据不可用
    Now_OP.strength = MTF_rx_buffer[14];// 信号强度
    Now_OP.precision = MTF_rx_buffer[15];// 精度
    Now_OP.tof_status = MTF_rx_buffer[16];// 状态
    Now_OP.reserved1 = MTF_rx_buffer[17];// 预留1
    Now_OP.flow_vel_x = (uint16_t)MTF_rx_buffer[19]<<8|MTF_rx_buffer[18];// 光流速度x轴
    Now_OP.flow_vel_y = (uint16_t)MTF_rx_buffer[21]<<8|MTF_rx_buffer[20];// 光流速度y轴
    Now_OP.flow_quality = MTF_rx_buffer[22];// 光流质量
    Now_OP.flow_status = MTF_rx_buffer[23];// 光流状态
	Now_OP.reserved2 =(uint16_t)MTF_rx_buffer[24]|((uint16_t)MTF_rx_buffer[25]<<8);//预留2
	
	updated = 1;
	updateTime = getSysTimeUs();
	return true;
}
void MT0X_RC::ProcessRxBytes(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i)
    {
        if (frameLength == 0 && data[i] != MICOLINK_MSG_HEAD)
        {
            continue;
        }
        frameBuffer[frameLength++] = data[i];
        if (frameLength < MT0X_RC_FRAME_LENGTH)
        {
            continue;
        }
        if (handleData(frameBuffer))
        {
            frameLength = 0;
        }
        else
        {
            memmove(frameBuffer, frameBuffer + 1,MT0X_RC_FRAME_LENGTH - 1);
            frameLength = MT0X_RC_FRAME_LENGTH - 1;
        }
    }
}