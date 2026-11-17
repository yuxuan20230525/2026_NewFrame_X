/******************************
File: TDT_Device/inc/customclient.h
Description: 0x0310 机器人→自定义客户端 (图传链路) 发送
Channel:     USART6 @ 921600 共用 VT03 通道
             RX 留给 remotevt03 (DMA2_Stream1),本模块只用 TX (DMA2_Stream6)
Constraints: 仅在 USE_JUDGEMENT=0 且 USE_Judgement_Vofa=0 时编译
             否则 DMA2_Stream6 会与 judgement / vofa 冲突
Note:        从 2026balancehero/top 移植。本工程无 vision H264 基础设施,
             已去掉 customClientSendH264Chunk 及 CC_MODE_H264 转发路径,
             仅保留遥测(雷达)发送链路 (见 radar_telemetry.h)
****************************  */
#ifndef _CUSTOMCLIENT_H
#define _CUSTOMCLIENT_H

#include "board.h"
#include <stdint.h>

enum CustomClientMode
{
    CC_MODE_H264      = 0,
    CC_MODE_TELEMETRY = 1,
};

extern CustomClientMode gCustomClientMode;

#if !USE_JUDGEMENT && !USE_Judgement_Vofa

#pragma pack(1)
struct CustomClientFrameHeader
{
    uint8_t  sof;        // 0xA5
    uint16_t dataLength; // data 段长度,固定 300
    uint8_t  seq;        // 包序号,自增
    uint8_t  crc8;       // 前 5 字节 CRC8
    uint16_t cmdid;      // 0x0310
};

struct CustomClientPacket
{
    CustomClientFrameHeader header;
    uint8_t  user_data[300];
    uint16_t crc16;      // 整包 CRC16
};
#pragma pack()

/**
 * @brief 初始化 USART6 TX DMA (DMA2_Stream6 + Channel_5)
 *        必须在 RCVT03.init() 之后调用,因为 USART6 本体由 remotevt03 配置
 */
void customClientInit(void);

/**
 * @brief 0x0310 机器人→自定义客户端发送
 *        data 段固定 300 字节,len<300 自动补 0,len>300 截断,上限 50Hz
 *        若上一帧仍在 DMA 发送中,本次调用直接丢弃 (不阻塞)
 *
 * @param data 用户负载指针 (可为 nullptr,此时整段清 0)
 * @param len  实际负载字节数
 */
void customClientSend(uint8_t *data, uint16_t len);

/**
 * @brief 查询发送通道是否忙
 */
bool customClientBusy(void);

void RadarTelemetrySend(void);

#endif // !USE_JUDGEMENT && !USE_Judgement_Vofa

#endif
