#ifndef _VISION_H
#define _VISION_H
#include "board.h"
#include "TimeMatch.h"

#pragma pack(1) // 自行添加数据包需要一字节对齐，添加在两行pack之间
/// 主控发送（MCU->NUC）.
struct vision_Send_Struct_t
{
	uint8_t header = 0xA5;
    uint8_t type = 2;
		
	float camera_yaw;
	float camera_pitch;
	
	float camera_roll;
		
    float current_imu_yaw;
    float current_imu_pitch;
    float current_imu_roll;

    double time_stamp;         // 初始为电控系时间,同步视觉时间后,为电控那边的绝对时间

    uint32_t camera_imu_id;    // imu序号,触发相机取一帧后同步绑定imu数据,即该帧图像对应的imu序号
    uint8_t syncState;         // 同步状态: 0-请求同步, 1-确认同步, 2-正常工作
	uint8_t camera_id;
	
    uint8_t enemycolor : 2;  // 0--Blue   1--error   2--Red
    uint8_t beat_mode : 4;  // 0--非打符模式   1--打符模式     2--反符模式
    uint8_t buildingShootMode : 1;  // 0--非建筑击打模式    1--建筑击打模式
    uint8_t lockcommand : 1;
    uint8_t spin_command : 1;
    uint8_t if_invincible;  // 敌方哨兵，5，4，3，2，1，自己颜色（红1蓝0）
    uint8_t sentryCommand : 1;
    uint8_t buff_type : 1;

    float nominal_bulletspeed;  // 标称弹速，即裁判系统限制的最高弹速
    float bullet_speed;         // 实际弹速，即上一次发射的弹速

    int8_t offset;             // 1是pitch,2是yaw

    double self_vel;
    double vel_yaw;
	
	uint8_t flight_index = 255;
	uint8_t takeoff = 0;
	uint8_t fly_direction = 255;
    
	float dial_angle;
	uint16_t bullet_number;
	uint32_t frame_ID;

    uint16_t CRC16CheckSum;
};

/// 主控接收（NUC->MCU）.
struct vision_Recv_Struct_t
{
	uint8_t header = 0xA5;
    uint8_t type = 2;
    double time_stamp;  //视觉时间系下时间戳，用于给电控做时间同步用

    uint8_t syncState ;          // 同步状态: 0-请求同步, 1-确认同步, 2-正常工作
    
    float yaw;           // yaw角度
    float pitch;         // pitch角度
    float distance = 0;  // 预测目标的距离，单位：cm
    float predict_point_angle;  //

    uint8_t obj_id : 4;  // 操作手UI.发送击打目标
    uint8_t no_obj : 1;  // boolean
    uint8_t beat : 1;    // boolean
    // uint8_t isLockingAxis : 1;  // boolean
    uint8_t unLimitedFireTime : 6;
    uint8_t obj_spining : 1;
    uint8_t IsEnergyMode : 1;   // boolean
    uint8_t clearInterval : 1;  // 大符扇叶切换的时候清空开火间隔
    uint8_t state;
    int8_t newYawOffset;
    int8_t newPitchOffset;
		
	float feedforward_v_yaw;
    float feedforward_v_pitch;
    float feedforward_torque_yaw;
    float feedforward_torque_pitch;
    bool buff_done;
	
	float body_yaw;
	uint8_t flight_mode;
	
    uint16_t CRC16CheckSum;
};


#pragma pack()

class USART_DMA
{
public:
	USART_DMA(GPIO_TypeDef *txPort, uint16_t txPin, uint8_t txPinSource, uint8_t txAF,
			  GPIO_TypeDef *rxPort, uint16_t rxPin, uint8_t rxPinSource, uint8_t rxAF,
			  USART_TypeDef *usart, uint32_t baudRate,
			  uint32_t dmaChannelRx, DMA_Stream_TypeDef *dmaStreamRx,
			  uint32_t dmaChannelTx, DMA_Stream_TypeDef *dmaStreamTx,
			  IRQn_Type irqnRx, IRQn_Type irqnTx, uint32_t TX_Flag, int32_t TX_IT_Flag,
			  uint32_t clockGPIO, uint32_t clockUSART, uint32_t clockDMA,
			  uint8_t *recvBuffer, uint32_t bufferSize) // 注意传入的buf的地址
	{
		this->txPort = txPort;
		this->txPin = txPin;
		this->txPinSource = txPinSource;
		this->txAF = txAF;
		this->rxPort = rxPort;
		this->rxPin = rxPin;
		this->rxPinSource = rxPinSource;
		this->rxAF = rxAF;
		this->usart = usart;
		this->baudRate = baudRate;

		this->dmaChannelRx = dmaChannelRx;
		this->dmaStreamRx = dmaStreamRx;
		this->dmaChannelTx = dmaChannelTx;
		this->dmaStreamTx = dmaStreamTx;

		this->irqnRx = irqnRx;
		this->irqnTx = irqnTx;
		this->TX_IT_Flag = TX_IT_Flag;
		this->TX_Flag = TX_Flag;
		this->clockGPIO = clockGPIO;
		this->clockUSART = clockUSART;
		this->clockDMA = clockDMA;
		this->recvBuffer = recvBuffer;
		this->bufferSize = bufferSize;
	}

	bool offlineFlag = true;
	uint32_t offline_cnt = 0;

	void Handle_UARTIdle_Interrupt(); // 接收中断
	void Handle_DMAIC_Interrupt();	  // 发送中断

	void init();	   // 初始化
	void run_1000hz(); // 运行函数,检查队列里有无数据，有则自动调用发送

	void SendMessage(uint8_t *data, uint16_t size);	 // 队列发送函数
	void DMA_SendData(uint8_t *data, uint16_t size); // 非队列发送函数  直接发送数据的地址和大小

	virtual void ProcessReceivedData() = 0; // 在中断处理时运行 recvBuf时接收dma接收的值
	uint32_t baudRate = 0;
	volatile bool uartIdle = true; // 串口空闲的标志
private:
	GPIO_TypeDef *txPort = nullptr;
	uint16_t txPin = 0;
	uint8_t txPinSource = 0;
	uint8_t txAF = 0;
	GPIO_TypeDef *rxPort = nullptr;
	uint16_t rxPin = 0;
	uint8_t rxPinSource = 0;
	uint8_t rxAF = 0;
	USART_TypeDef *usart = nullptr;
	uint32_t dmaChannelRx = 0;
	DMA_Stream_TypeDef *dmaStreamRx = nullptr;
	uint32_t dmaChannelTx = 0;
	DMA_Stream_TypeDef *dmaStreamTx = nullptr;
	IRQn_Type irqnRx = (IRQn_Type)0;
	IRQn_Type irqnTx = (IRQn_Type)0;
	uint32_t TX_IT_Flag = 0;
	uint32_t TX_Flag = 0;
	uint32_t clockGPIO = 0;
	uint32_t clockUSART = 0;
	uint32_t clockDMA = 0;
	uint8_t *recvBuffer = nullptr;
	uint32_t bufferSize = 0;
	DMA_InitTypeDef DMA_InitStructure_rx = {0}, DMA_InitStructure_tx = {0};

	

	struct Message
	{
		uint8_t *data = nullptr;
		uint16_t size = 0;
	};
	myqueue<Message> messageQueue; // 串口消息队列
};

class VISION : public USART_DMA
{
public:
	VISION(GPIO_TypeDef *txPort, uint16_t txPin, uint8_t txPinSource, uint8_t txAF,
		   GPIO_TypeDef *rxPort, uint16_t rxPin, uint8_t rxPinSource, uint8_t rxAF,
		   USART_TypeDef *usart, uint32_t baudRate,
		   uint32_t dmaChannelRx, DMA_Stream_TypeDef *dmaStreamRx,
		   uint32_t dmaChannelTx, DMA_Stream_TypeDef *dmaStreamTx,
		   IRQn_Type irqnRx, IRQn_Type irqnTx, uint32_t TX_Flag, int32_t TX_IT_Flag,
		   uint32_t clockGPIO, uint32_t clockUSART, uint32_t clockDMA,
		   uint8_t *recvBuffer, uint32_t bufferSize)
		: USART_DMA(txPort, txPin, txPinSource, txAF,
					rxPort, rxPin, rxPinSource, rxAF,
					usart, baudRate,
					dmaChannelRx, dmaStreamRx, dmaChannelTx, dmaStreamTx,
					irqnRx, irqnTx, TX_Flag, TX_IT_Flag,
					clockGPIO, clockUSART, clockDMA, recvBuffer, bufferSize) {}

	void ProcessReceivedData() override;

	void USB_init();
	uint32_t USB_offline_cnt = 0;
	bool USB_offline_Flag = true;

	void vision_Send_Data();
};

extern VISION vision;

extern vision_Recv_Struct_t vision_RecvStruct;
extern vision_Send_Struct_t vision_SendStruct;

extern TimeSimultaneity imuTimeMatch;

#endif
