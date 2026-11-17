/******************************
File: TDT_Device/src/customclient.cpp
Description: 0x0310 (机器人→自定义客户端,图传链路) 发送实现
Channel:     USART6 @ 921600 共用 VT03 通道, TX 走 DMA2_Stream6 / Channel_5
Note:        USART6 主体(GPIO/Clock/Baud/USART_DMACmd Tx|Rx) 已由 remotevt03 配好,
             本模块只接管 TX DMA 流; RX 由 remotevt03 独占 DMA2_Stream1
             从 2026balancehero/top 移植,已去掉 H264 转发,默认遥测模式
****************************  */
#include "customclient.h"

#include "vision.h"

CustomClientMode gCustomClientMode = CC_MODE_TELEMETRY;  // 本工程无 H264 路径,默认遥测

#if !USE_JUDGEMENT && !USE_Judgement_Vofa

#include "crc.h"
#include <string.h>

static CustomClientPacket gPkt;            // DMA 发送缓冲(每次发送拷贝到这里)
static volatile uint8_t   gBusy = 0;       // DMA 进行中标志
static uint8_t            gSeq  = 0;       // 包序号,每次自增

void customClientInit(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_DeInit(DMA2_Stream6);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;            // USART6_TX
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&gPkt;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize         = sizeof(gPkt);
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream6, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
    // 不在 init 中 enable DMA 流,等首次 send 时再 enable
}

void customClientSend(uint8_t *data, uint16_t len)
{
    if (gBusy) return;   // 上一帧未发完直接丢

    if (len > 300) len = 300;
    if (data && len) memcpy(gPkt.user_data, data, len);
    if (len < 300)   memset(gPkt.user_data + len, 0, 300 - len);

    gPkt.header.sof        = 0xA5;
    gPkt.header.dataLength = 300;
    gPkt.header.seq        = gSeq++;
    gPkt.header.crc8       = 0;
    gPkt.header.cmdid      = 0x0310;
    Append_CRC8_Check_Sum((unsigned char *)&gPkt, 5);
    Append_CRC16_Check_Sum((uint8_t *)&gPkt, sizeof(gPkt));

    gBusy = 1;
    //gBusy==0 已保证上一帧 ISR 里 disable 过 DMA;这里不再自旋等待状态位翻转,避免极端下死循环
    DMA_Cmd(DMA2_Stream6, DISABLE);
    DMA_ClearFlag(DMA2_Stream6,
                  DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 |
                  DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_TEIF6);
    DMA_SetCurrDataCounter(DMA2_Stream6, sizeof(gPkt));
    DMA_Cmd(DMA2_Stream6, ENABLE);
}

bool customClientBusy(void)
{
    return gBusy != 0;
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6))
    {
        DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);
        DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);
        DMA_Cmd(DMA2_Stream6, DISABLE);
        gBusy = 0;
    }
}
void RadarTelemetrySend(void)
{
//	static uint16_t seq = 0;
//	CustomTelemetryPacket pkt;
//	memset(&pkt, 0, sizeof(pkt));

//	pkt.magic        = 0x524D;
//	pkt.seq          = seq++;
//	pkt.timestamp_ms = (uint32_t)(getSysTimeUs() / 1000);

//	pkt.hero_17mm       = receiveMsg.RadarAmmo1.hero_17mm;
//	pkt.infantry_3_17mm = receiveMsg.RadarAmmo1.infantry_3_17mm;
//	pkt.infantry_4_17mm = receiveMsg.RadarAmmo1.infantry_4_17mm;
//	pkt.aerial_17mm     = receiveMsg.RadarAmmo1.aerial_17mm;
//	pkt.sentry_17mm     = receiveMsg.RadarAmmoGold.sentry_17mm;

//	pkt.remaining_gold = receiveMsg.RadarAmmoGold.remaining_gold;
//	pkt.total_gold     = receiveMsg.RadarAmmoGold.total_gold;

//	pkt.hero_heal_percent          = receiveMsg.RadarBuffHero.heal;
//	pkt.hero_heat_cooling          = receiveMsg.RadarBuffHero.cooling;
//	pkt.hero_defense_percent       = receiveMsg.RadarBuffHero.defense;
//	pkt.hero_vulnerability_percent = receiveMsg.RadarBuffHero.vulnerability;
//	pkt.hero_attack_percent        = receiveMsg.RadarBuffHero.attack;

//	pkt.engineer_heal_percent          = receiveMsg.RadarBuffEngineer.heal;
//	pkt.engineer_heat_cooling          = receiveMsg.RadarBuffEngineer.cooling;
//	pkt.engineer_defense_percent       = receiveMsg.RadarBuffEngineer.defense;
//	pkt.engineer_vulnerability_percent = receiveMsg.RadarBuffEngineer.vulnerability;
//	pkt.engineer_attack_percent        = receiveMsg.RadarBuffEngineer.attack;

//	pkt.infantry_3_heal_percent          = receiveMsg.RadarBuffInfantry3.heal;
//	pkt.infantry_3_heat_cooling          = receiveMsg.RadarBuffInfantry3.cooling;
//	pkt.infantry_3_defense_percent       = receiveMsg.RadarBuffInfantry3.defense;
//	pkt.infantry_3_vulnerability_percent = receiveMsg.RadarBuffInfantry3.vulnerability;
//	pkt.infantry_3_attack_percent        = receiveMsg.RadarBuffInfantry3.attack;

//	pkt.infantry_4_heal_percent          = receiveMsg.RadarBuffInfantry4.heal;
//	pkt.infantry_4_heat_cooling          = receiveMsg.RadarBuffInfantry4.cooling;
//	pkt.infantry_4_defense_percent       = receiveMsg.RadarBuffInfantry4.defense;
//	pkt.infantry_4_vulnerability_percent = receiveMsg.RadarBuffInfantry4.vulnerability;
//	pkt.infantry_4_attack_percent        = receiveMsg.RadarBuffInfantry4.attack;

//	pkt.sentry_heal_percent          = receiveMsg.RadarBuffSentry.heal;
//	pkt.sentry_heat_cooling          = receiveMsg.RadarBuffSentry.cooling;
//	pkt.sentry_defense_percent       = receiveMsg.RadarBuffSentry.defense;
//	pkt.sentry_vulnerability_percent = receiveMsg.RadarBuffSentry.vulnerability;
//	pkt.sentry_attack_percent        = receiveMsg.RadarBuffSentry.attack;

//	pkt.sentry_posture = receiveMsg.RadarAmmoGold.sentry_posture;
//	
//	pkt.hero_state = receiveMsg.RadarBuffHero.state;
//	pkt.engineer_state = receiveMsg.RadarBuffEngineer.state;
//	pkt.infantry_3_state = receiveMsg.RadarBuffInfantry3.state;
//	pkt.infantry_4_state = receiveMsg.RadarBuffInfantry4.state;
//	pkt.sentry_state = receiveMsg.RadarBuffSentry.state;

//	pkt.fangda = vision_SendStruct.camera_id;

//	Append_CRC16_Check_Sum((uint8_t *)&pkt, sizeof(pkt));

//	customClientSend((uint8_t *)&pkt, sizeof(pkt));
}
#endif // !USE_JUDGEMENT && !USE_Judgement_Vofa
