/*****************************************************************************
File name: TDT_Device\src\hipnuc_imu.cpp
Description: HiPNUC HI 系列外置 IMU 驱动实现 (UART5 + HI91 + SYNC_IN 硬件时间同步)
设计文档: docs/superpowers/specs/2026-06-19-hipnuc-imu-uart5-sync-design.md
*****************************************************************************/
#include "hipnuc_imu.h"
#include "imu_task.h" // HIPNUC_def / HIPNUC_FAILOVER
#include "timer.h"    // getSysTimeUs
#include "iwdg.h"
#include "led_task.h"
#include <math.h>
#include <string.h>

HipnucDbg g_hipnucDbg = {};

// 失效降级(HIPNUC_FAILOVER)也需要 HiPNUC 对象/ISR/实现
#if (HIPNUC_def || HIPNUC_FAILOVER)

/* USART1 接收缓冲 (与 vision 的 tmp_RecvBuff 同理, ISR 内直接用此全局) */
static uint8_t hipnuc_rxbuf[HIPNUC_RXBUF_SIZE] = {0};

/* 全局对象: USART1(PA9=TX/PB7=RX, C板4-pin对外串口连接器, 壳印"UART2") 921600,
   RX=DMA2_Stream2, TX=DMA2_Stream7, USART1_IRQ。
   参数与 vision.cpp 的 IF_USE_DJIC 块一致(仅波特率 921600), 该块因 USE_VISION=0 从不 init,
   故 USART1 物理空闲; 其两个 ISR 在 vision.cpp 已用 !HIPNUC_def guard 让出。 */
HipnucImu hipnucImu(GPIOA, GPIO_Pin_9, GPIO_PinSource9, GPIO_AF_USART1,
                    GPIOB, GPIO_Pin_7, GPIO_PinSource7, GPIO_AF_USART1,
                    USART1, HIPNUC_BAUD,
                    DMA_Channel_4, DMA2_Stream2,
                    DMA_Channel_4, DMA2_Stream7,
                    USART1_IRQn, DMA2_Stream7_IRQn, DMA_FLAG_TCIF7, DMA_IT_TCIF7,
                    RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, RCC_APB2Periph_USART1, RCC_AHB1Periph_DMA2,
                    hipnuc_rxbuf, sizeof(hipnuc_rxbuf));

/* USART1 空闲中断 -> 解析一帧 (帧尾触发) */
extern "C" void USART1_IRQHandler(void)
{
    hipnucImu.HandleUartIdleInterrupt();
}

/* USART1 TX DMA(DMA2_Stream7) 完成中断 (发 ONMARK 配置用) */
extern "C" void DMA2_Stream7_IRQHandler(void)
{
    hipnucImu.Handle_DMAIC_Interrupt();
}

/* ============================ CRC-16/CCITT (手册 §5.10) ============================ */
void HipnucImu::crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currectCrc;
    for (uint32_t j = 0; j < len; ++j)
    {
        crc ^= (uint32_t)src[j] << 8;
        for (uint32_t i = 0; i < 8; ++i)
        {
            uint32_t tmp = crc << 1;
            if (crc & 0x8000)
                tmp ^= 0x1021;
            crc = tmp;
        }
    }
    *currectCrc = (uint16_t)crc;
}

/* ============================ 触发流水线 (仅 tick 上下文) ============================ */
void HipnucImu::trigPush(uint64_t t)
{
    trigFifo_[trigHead_] = t;
    trigHead_ = (trigHead_ + 1) % HIPNUC_PIPELINE_MAX;
    if (trigCount_ < HIPNUC_PIPELINE_MAX)
        trigCount_++;
    else
        trigTail_ = (trigTail_ + 1) % HIPNUC_PIPELINE_MAX; // 满, 丢最旧重同步
}

bool HipnucImu::trigPop(uint64_t &t)
{
    if (!trigCount_)
        return false;
    t = trigFifo_[trigTail_];
    trigTail_ = (trigTail_ + 1) % HIPNUC_PIPELINE_MAX;
    trigCount_--;
    return true;
}

/* ============================ SYNC_IN 触发脚 ============================ */
void HipnucImu::syncPinInit()
{
    RCC_AHB1PeriphClockCmd(HIPNUC_SYNC_RCC, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = HIPNUC_SYNC_PIN;
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(HIPNUC_SYNC_PORT, &g);
    GPIO_ResetBits(HIPNUC_SYNC_PORT, HIPNUC_SYNC_PIN); // 初始低, 首拍 syncRise 即干净上升沿
}

/* ============================ 模块配置: 补发 ONMARK ============================ */
void HipnucImu::sendOnmarkConfig()
{
    // 前置: 模块已由 CHCenter 存成 921600 + HI91 + 6轴(ATT MODE 0) + SAVECONFIG。
    // 此处每次上电补发, 防模块被复位回周期模式。不发 SAVECONFIG(不磨损 flash)。
    // 手册 §4.6.8.15: LOG <MSG> <TYPE> <VALUE>; ONMARK 的 VALUE=1 表示 SYNC_IN 脉冲触发。
    static uint8_t s_cfg[] = "LOG HI91 ONMARK 1\r\n";
    DMA_SendData(s_cfg, sizeof(s_cfg) - 1); // 去掉结尾 '\0'
}

/* ============================ 一次性配置 (provision) ============================ */
#if HIPNUC_PROVISION
#define HIPNUC_STR2(x) #x
#define HIPNUC_STR(x) HIPNUC_STR2(x)

// 阻塞延时 (provision 在 init 内跑, 此时看门狗未启, 阻塞安全)
static void prov_delay_ms(uint32_t ms)
{
    uint64_t t0 = getSysTimeUs();
    while (getSysTimeUs() - t0 < (uint64_t)ms * 1000u) { }
}

// 阻塞轮询发一串 ASCII (USART1 直发, 不依赖 DMA/中断)
static void prov_send(const char *s)
{
    while (*s)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
        USART_SendData(USART1, (uint16_t)(uint8_t)*s++);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }
}

// 切 USART1 波特率 (并补回 IDLE 中断/DMA 请求, 因 USART_Init 会清 CR1)
static void prov_set_baud(uint32_t baud)
{
    USART_InitTypeDef u = {0};
    u.USART_BaudRate = baud;
    u.USART_WordLength = USART_WordLength_8b;
    u.USART_StopBits = USART_StopBits_1;
    u.USART_Parity = USART_Parity_No;
    u.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &u);
    USART_Cmd(USART1, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
}
#endif // HIPNUC_PROVISION

void HipnucImu::provisionModule()
{
#if HIPNUC_PROVISION
    // 进来时 USART1 已在 HIPNUC_BAUD(配置版应=115200, 即模块出厂波特率)。
    // 开环发送(不读回应), 每条留足处理时间; SAVECONFIG 写 flash 给 800ms, REBOOT 给 1.5s。
    prov_delay_ms(300);                                              // 等模块上电稳定
    prov_send("CONFIG ATT MODE 0\r\n");              prov_delay_ms(150); // 6轴 VRU
    prov_send("CONFIG IMU URFR " HIPNUC_STR(HIPNUC_URFR_CODE) "\r\n");
                                                     prov_delay_ms(150); // 安装方向
    prov_send("LOG HI91 ONMARK 1\r\n");              prov_delay_ms(150); // SYNC_IN 触发
    prov_send("SAVECONFIG\r\n");                     prov_delay_ms(800); // 存 模式/方向/ONMARK
    prov_send("SERIALCONFIG 921600\r\n");            prov_delay_ms(150); // 模块切 921600
    prov_set_baud(921600);                           prov_delay_ms(50);  // 板子也切 921600
    prov_send("SAVECONFIG\r\n");                     prov_delay_ms(800); // 存波特率
    prov_send("REBOOT\r\n");                         prov_delay_ms(1500);// 模块复位生效
    // 完成: 模块 = 921600 + 6轴 + ONMARK(已存)。请改烧正式版固件(HIPNUC_PROVISION=0)。
#endif
}

/* ============================ 初始化 ============================ */
void HipnucImu::init()
{
    cycle.getCycleT();   // 同基类 init 语义, 取周期时钟
    USART_DMA::init();   // 起 USART1 + RX/TX DMA + 空闲中断 (波特率=HIPNUC_BAUD)

    // HI91 1 kHz 时每帧只有约 0.11 ms 空档。原 USART_DMA 的 IDLE 回调会停 DMA、
    // 在中断内跑 CRC、再 DeInit/Init DMA，高 CAN 负载时容易丢下一帧帧头。
    // 改为循环 DMA，IDLE 只根据 NDTR 消费新字节，接收全程不停。
    DMA_Cmd(DMA2_Stream2, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE) { }
    DMA2_Stream2->CR |= DMA_SxCR_CIRC;
    DMA_SetCurrDataCounter(DMA2_Stream2, HIPNUC_RXBUF_SIZE);
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 |
                                  DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2);
    DMA_Cmd(DMA2_Stream2, ENABLE);

    rxReadPos_ = 0;
    parseLen_ = 0;
    syncPinInit();       // PI7 输出, 初始低
#if HIPNUC_PROVISION
    provisionModule();   // 一次性配置(阻塞~4s): 配完模块=921600/6轴/ONMARK 并复位
#elif HIPNUC_SEND_ONMARK
    sendOnmarkConfig();  // 补发 LOG HI91 ONMARK 1
#endif

    trigHead_ = trigTail_ = trigCount_ = 0;
    lastSeq_ = rxSeq_;
    offlineCnt_ = 0;
    imuValid = 0;
    offlineFlag = true;
    g_hipnucDbg.syncMode = syncMode_;
}

/* ============================ 循环 DMA 流式解析 ============================ */
void HipnucImu::ProcessReceivedData()
{
    consumeDmaBytes();
}

void HipnucImu::HandleUartIdleInterrupt()
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) == RESET)
        return;

    volatile uint32_t clear = USART1->SR;
    clear = USART1->DR;
    (void)clear;
    g_hipnucDbg.isrCnt++;
    consumeDmaBytes();
}

void HipnucImu::consumeDmaBytes()
{
    const uint16_t writePos = (uint16_t)(HIPNUC_RXBUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream2));
    const uint16_t available = (uint16_t)((writePos + HIPNUC_RXBUF_SIZE - rxReadPos_) % HIPNUC_RXBUF_SIZE);

    for (uint16_t i = 0; i < 8; ++i)
        g_hipnucDbg.rxHead[i] = i < available ? hipnuc_rxbuf[(rxReadPos_ + i) % HIPNUC_RXBUF_SIZE] : 0;

    while (rxReadPos_ != writePos)
    {
        parseByte(hipnuc_rxbuf[rxReadPos_]);
        rxReadPos_ = (uint16_t)((rxReadPos_ + 1) % HIPNUC_RXBUF_SIZE);
        g_hipnucDbg.byteCnt++;
    }
}

void HipnucImu::parseByte(uint8_t byte)
{
    parseCommandReplyByte(byte);

    if (parseLen_ == 0)
    {
        if (byte == HIPNUC_SOF0)
            parseBuf_[parseLen_++] = byte;
        return;
    }

    if (parseLen_ == 1)
    {
        if (byte == HIPNUC_SOF1)
        {
            parseBuf_[parseLen_++] = byte;
            g_hipnucDbg.sofSeen++;
        }
        else if (byte != HIPNUC_SOF0)
        {
            parseLen_ = 0;
        }
        return;
    }

    parseBuf_[parseLen_++] = byte;
    if (parseLen_ == 4)
    {
        const uint16_t len = (uint16_t)parseBuf_[2] | ((uint16_t)parseBuf_[3] << 8);
        if (len != HIPNUC_HI91_PAYLEN)
            parseLen_ = byte == HIPNUC_SOF0 ? 1 : 0;
        return;
    }

    if (parseLen_ < HIPNUC_HI91_FRAMELEN)
        return;

    HipnucFrame frame;
    if (decodeFrame(parseBuf_, frame))
    {
        rxFrame_ = frame;
        rxTimeUs_ = getSysTimeUs();
        rxSeq_++;
        offlineFlag = false;
    }
    parseLen_ = 0;
}

void HipnucImu::parseCommandReplyByte(uint8_t byte)
{
    if (!awaitingReply_)
        return;

    static const uint8_t okReply[] = {'O', 'K', '\r', '\n'};
    static const uint8_t errReply[] = {'E', 'R', 'R', '\r', '\n'};

    if (okMatch_ == 2 && byte == '\n') // 同时兼容 OK\n 与 OK\r\n
    {
        commandReply_ = 1;
        awaitingReply_ = false;
        return;
    }
    if (errMatch_ == 3 && byte == '\n') // 同时兼容 ERR\n 与 ERR\r\n
    {
        commandReply_ = 2;
        awaitingReply_ = false;
        return;
    }

    if (byte == okReply[okMatch_])
        okMatch_++;
    else
        okMatch_ = byte == okReply[0] ? 1 : 0;

    if (byte == errReply[errMatch_])
        errMatch_++;
    else
        errMatch_ = byte == errReply[0] ? 1 : 0;

    if (okMatch_ == sizeof(okReply))
    {
        commandReply_ = 1;
        awaitingReply_ = false;
    }
    else if (errMatch_ == sizeof(errReply))
    {
        commandReply_ = 2;
        awaitingReply_ = false;
    }
}

bool HipnucImu::decodeFrame(const uint8_t *frame, HipnucFrame &out)
{
    const uint16_t crcRx = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
    uint16_t crc = 0;
    crc16_update(&crc, frame, 4);
    crc16_update(&crc, frame + 6, HIPNUC_HI91_PAYLEN);
    if (crc != crcRx)
    {
        g_hipnucDbg.crcFail++;
        return false;
    }

    const uint8_t *p = frame + 6;
    if (p[0] != HIPNUC_HI91_TAG)
        return false;

    memcpy(&out.status, p + HI91_OFF_STATUS, 2);
    memcpy(&out.systime, p + HI91_OFF_SYSTIME, 4);
    memcpy(out.acc, p + HI91_OFF_ACC, 12);
    memcpy(out.gyr, p + HI91_OFF_GYR, 12);
    memcpy(out.mag, p + HI91_OFF_MAG, 12);
    memcpy(&out.eul[0], p + HI91_OFF_ROLL, 4);
    memcpy(&out.eul[1], p + HI91_OFF_PITCH, 4);
    memcpy(&out.eul[2], p + HI91_OFF_YAW, 4);
    memcpy(out.quat, p + HI91_OFF_QUAT, 16);

    if (!framePlausible(out))
    {
        g_hipnucDbg.invalidFrame++;
        return false;
    }
    return true;
}

bool HipnucImu::framePlausible(const HipnucFrame &frame)
{
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (!isfinite(frame.acc[i]) || fabsf(frame.acc[i]) > 32.0f ||
            !isfinite(frame.gyr[i]) || fabsf(frame.gyr[i]) > 4000.0f ||
            !isfinite(frame.eul[i]) || fabsf(frame.eul[i]) > 720.0f)
            return false;
    }

    float quatNormSq = 0.0f;
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (!isfinite(frame.quat[i]))
            return false;
        quatNormSq += frame.quat[i] * frame.quat[i];
    }
    return quatNormSq > 0.25f && quatNormSq < 2.25f;
}

/* ============================ 帧 -> 解算结果 ============================ */
void HipnucImu::applyFrame(const HipnucFrame &f)
{
    // 轴向重映射(向量: gyro/acc 有效)。局部名带 V 后缀, 避免遮蔽成员 acc/gyro
    float gyrV[3], accV[3];
    for (int i = 0; i < 3; ++i)
    {
        gyrV[i] = gyroSign_[i] * axisSign_[i] * f.gyr[axisMap_[i]]; // gyroSign 额外翻(配套 euler)
        accV[i] = accSign_[i] * axisSign_[i] * f.acc[axisMap_[i]];  // accSign 额外翻(可只翻 pitch 轴 acc[1])
    }
    for (int i = 0; i < 3; ++i)
    {
        acc.accValue.data[i] = accV[i] * 9.81f;      // G -> m/s^2 (成员 acc)
        gyro.dps.data[i]     = gyrV[i];              // deg/s (成员 gyro)
        gyro.radps.data[i]   = gyrV[i] * ANGLE_TO_RAD;
    }

    // 欧拉角(符号可配)
    float roll  = eulSign_[0] * f.eul[0];
    float pitch = eulSign_[1] * f.eul[1];
    float yaw   = eulSign_[2] * f.eul[2];

    // FusionAngle: ±200° 过圈累加 (镜像 imu.cpp:161-169), 复用 fusion_temp
    if (yaw - fusion_temp.lastyaw >= 200)
        fusion_temp.roundnum -= 1;
    else if (yaw - fusion_temp.lastyaw <= -200)
        fusion_temp.roundnum += 1;
    FusionAngle.yaw   = yaw + 360.0f * fusion_temp.roundnum + yawOffset;
    FusionAngle.pitch = pitch + pitchOffset;
    FusionAngle.roll  = roll + rollOffset;
    fusion_temp.lastyaw = yaw;

    // My_data / AHRS_data 连续角(±180° ImuCrossRoundHandle), 供读这两组的消费者
    My_data.nowAngle.roll = roll;
    My_data.nowAngle.pitch = pitch;
    My_data.nowAngle.yaw = yaw;
    ImuCrossRoundHandle(My_data.LastAngle, My_data.nowAngle, My_data.Angle, My_data.round);
    My_data.LastAngle = My_data.nowAngle;
    My_data.quaternion.q0 = f.quat[0];
    My_data.quaternion.q1 = f.quat[1];
    My_data.quaternion.q2 = f.quat[2];
    My_data.quaternion.q3 = f.quat[3];

    AHRS_data.nowAngle.roll = roll;
    AHRS_data.nowAngle.pitch = pitch;
    AHRS_data.nowAngle.yaw = yaw;
    ImuCrossRoundHandle(AHRS_data.LastAngle, AHRS_data.nowAngle, AHRS_data.Angle, AHRS_data.round);
    AHRS_data.LastAngle = AHRS_data.nowAngle;
    AHRS_data.ins_quat[0] = f.quat[0];
    AHRS_data.ins_quat[1] = f.quat[1];
    AHRS_data.ins_quat[2] = f.quat[2];
    AHRS_data.ins_quat[3] = f.quat[3];

    // 调试快照
    memcpy(g_hipnucDbg.acc, accV, sizeof(accV));
    memcpy(g_hipnucDbg.gyr, gyrV, sizeof(gyrV));
    g_hipnucDbg.eul[0] = roll;
    g_hipnucDbg.eul[1] = pitch;
    g_hipnucDbg.eul[2] = yaw;
    memcpy(g_hipnucDbg.quat, f.quat, sizeof(f.quat));
}

/* ============================ 1kHz 主更新 ============================ */
void HipnucImu::TDT_IMU_update()
{
    // 1) 硬件同步模式: PI7 上升沿触发模块采样, 同处 latch 触发时刻(与控制环同一时基)
    uint64_t trigUs = 0;
    if (syncMode_ == HIPNUC_SYNC_HW)
    {
        syncRise();
        trigUs = getSysTimeUs();
    }

    // 2) 帧内容、到达时刻和序号必须同一个临界区快照。原逻辑先读 seq 再禁中断，
    // ISR 若卡在两者之间，会把新帧当旧帧并在下一拍重复消费。
    HipnucFrame f;
    uint64_t rxUs = 0;
    uint32_t seq;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    seq = rxSeq_;
    if (seq != lastSeq_)
    {
        f = rxFrame_;
        rxUs = rxTimeUs_;
    }
    if (!primask)
        __enable_irq();

    const uint32_t newFrames = seq - lastSeq_;
    if (newFrames)
    {
        lastSeq_ = seq;

        uint64_t sampleUs;
        if (syncMode_ == HIPNUC_SYNC_HW)
        {
            // 触发流水线配对(1拍): 来了 newFrames 帧, 弹 newFrames 个在途触发, 最后一个配最新帧
            sampleUs = trigUs; // 兜底(流水线空时)
            for (uint32_t k = 0; k < newFrames; ++k)
            {
                uint64_t t;
                if (trigPop(t))
                    sampleUs = t;
            }
        }
        else
        {
            // 软同步: 用帧到达(空闲中断)时刻, 无触发线
            sampleUs = rxUs;
        }
        readTime = (double)sampleUs / 1e6;
        applyFrame(f);

        // 推进板载 Fusion AHRS(喂模块 acc/gyro): 只为让 ahrs.initialising 按 2s 收敛清零,
        // 满足 actuator_task 的 "boardImu->ahrs.initialising 才上力" 门控。
        // 输出不取用 —— FusionAngle 仍由 applyFrame 用模块自带融合填。
        FusionVector gyroVec = {gyro.dps.data[0], gyro.dps.data[1], gyro.dps.data[2]};
        FusionVector accVec  = {acc.accValue.data[0] / 9.81f, acc.accValue.data[1] / 9.81f, acc.accValue.data[2] / 9.81f};
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroVec, accVec, cycle.getCycleT());

        imuValid = 1;
        offlineCnt_ = 0;
        offlineFlag = false;

        g_hipnucDbg.readTime = readTime;
        g_hipnucDbg.frameCnt = seq;
        g_hipnucDbg.offline = 0;
    }
    else
    {
        if (offlineCnt_ != 0xFFFFFFFFu)
            offlineCnt_++;
        if (offlineCnt_ > HIPNUC_OFFLINE_TICKS)
        {
            offlineFlag = true;
            imuValid = 0;
            ahrs.initialising = true; // 离线: 重新封锁上力(纯HiPNUC安全; failover下已切BMI088故无影响)
            trigHead_ = trigTail_ = trigCount_ = 0; // 离线清流水线, 防堆积错位
            g_hipnucDbg.offline = 1;
        }
        if (offlineCnt_ > g_hipnucDbg.maxLossTicks)
            g_hipnucDbg.maxLossTicks = offlineCnt_;
    }

    // 3) 硬件模式: 记录本拍触发, 落低准备下次上升沿
    if (syncMode_ == HIPNUC_SYNC_HW)
    {
        trigPush(trigUs);
        syncFall();
    }
}

/* ============================ 触发线掉 -> 转软同步 ============================ */
void HipnucImu::switchToSoftwareSync()
{
    if (syncMode_ == HIPNUC_SYNC_SW)
        return;
    // 把模块从 ONMARK(等触发) 切到周期 1kHz 自发, 之后用 idle-latch 软件打戳
    static uint8_t s_ontime[] = "LOG HI91 ONTIME 0.002\r\n";
    DMA_SendData(s_ontime, sizeof(s_ontime) - 1);
    syncMode_ = HIPNUC_SYNC_SW;
    imuValid = 0; // 软同步真正收到新帧前，禁止控制环继续使用陈旧姿态
    offlineCnt_ = 0; // 给模块完整的软同步启动宽限，不沿用硬同步的丢帧计数
    trigHead_ = trigTail_ = trigCount_ = 0; // 触发流水线作废
    g_hipnucDbg.syncMode = syncMode_;
}

/* ============================ 模块内部姿态自动校平 ============================ */
bool HipnucImu::sendCommandAndWait(uint8_t *command, uint16_t size, uint32_t timeoutMs)
{
    const uint64_t idleStart = getSysTimeUs();
    while (!uartIdle)
    {
        if (getSysTimeUs() - idleStart > 100000u)
            return false;
        iwdgFeed();
    }

    commandReply_ = 0;
    okMatch_ = 0;
    errMatch_ = 0;
    awaitingReply_ = true;
    DMA_SendData(command, size);

    const uint64_t replyStart = getSysTimeUs();
    while (getSysTimeUs() - replyStart < (uint64_t)timeoutMs * 1000u)
    {
        if (commandReply_ != 0)
            break;
        iwdgFeed();
        delayMs(1);
    }
    awaitingReply_ = false;
    return commandReply_ == 1;
}

bool HipnucImu::calibrateAttitude()
{
    g_hipnucDbg.calibState = 1;
    g_hipnucDbg.calibFailReason = 0;

    if (!imuValid || offlineFlag)
    {
        g_hipnucDbg.calibState = 4;
        g_hipnucDbg.calibFailReason = 1;
        return false;
    }
    // 连续约 300 ms 确认在线、接近水平且静止，再把命令交给模块。
    uint32_t lastFrame = g_hipnucDbg.frameCnt;
    uint16_t stableFrames = 0;
    uint16_t noFrameTicks = 0;
    while (stableFrames < 150)
    {
        TDT_IMU_update();
        const uint32_t frame = g_hipnucDbg.frameCnt;
        if (frame == lastFrame)
        {
            if (++noFrameTicks > 50)
            {
                g_hipnucDbg.calibState = 4;
                g_hipnucDbg.calibFailReason = 1;
                return false;
            }
        }
        else
        {
            lastFrame = frame;
            noFrameTicks = 0;
            g_hipnucDbg.calibPitch = g_hipnucDbg.eul[1];
            g_hipnucDbg.calibRoll = g_hipnucDbg.eul[0];
            // 手册要求 Pitch/Roll 均小于 5°；本地先拦截，避免错误姿态写入模块。
            if (fabsf(g_hipnucDbg.calibPitch) >= 5.0f || fabsf(g_hipnucDbg.calibRoll) >= 5.0f)
            {
                g_hipnucDbg.calibState = 4;
                g_hipnucDbg.calibFailReason = 2;
                return false;
            }
            if (fabsf(gyro.dps.data[0]) >= 3.0f ||
                fabsf(gyro.dps.data[1]) >= 3.0f ||
                fabsf(gyro.dps.data[2]) >= 3.0f)
            {
                g_hipnucDbg.calibState = 4;
                g_hipnucDbg.calibFailReason = 3;
                return false;
            }
            stableFrames++;
        }
        iwdgFeed();
        delayMs(2);
    }

    g_hipnucDbg.calibCmdCnt++;
    ledtask.SetNow(LedES_BlinkFast);
    laser.stateShow(LedES_BlinkFast);

    static uint8_t calibrateCommand[] = "CONFIG ATT RST 3\r\n";
    if (!sendCommandAndWait(calibrateCommand, sizeof(calibrateCommand) - 1, 500))
    {
        g_hipnucDbg.calibState = 4;
        g_hipnucDbg.calibFailReason = 4;
        return false;
    }
    g_hipnucDbg.calibState = 2;

    static uint8_t saveCommand[] = "SAVECONFIG\r\n";
    g_hipnucDbg.calibState = 3;
    if (!sendCommandAndWait(saveCommand, sizeof(saveCommand) - 1, 1500))
    {
        g_hipnucDbg.calibState = 4;
        g_hipnucDbg.calibFailReason = 5;
        return false;
    }

    ledtask.SetNow(LedES_ConstantLight);
    iwdgFeed();
    delayMs(50);
    __set_FAULTMASK(1);
    NVIC_SystemReset();
    while (1) { }
    return true;
}

/* ============================ 轴向定标接口 ============================ */
void HipnucImu::setAxisRemap(const uint8_t map[3], const float sign[3])
{
    for (int i = 0; i < 3; ++i)
    {
        axisMap_[i] = map[i];
        axisSign_[i] = sign[i];
    }
}

void HipnucImu::setEulerSign(float rollSign, float pitchSign, float yawSign)
{
    eulSign_[0] = rollSign;
    eulSign_[1] = pitchSign;
    eulSign_[2] = yawSign;
}

void HipnucImu::setGyroSign(float xSign, float ySign, float zSign)
{
    gyroSign_[0] = xSign;
    gyroSign_[1] = ySign;
    gyroSign_[2] = zSign;
}

void HipnucImu::setAccSign(float xSign, float ySign, float zSign)
{
    accSign_[0] = xSign;
    accSign_[1] = ySign;
    accSign_[2] = zSign;
}

#endif // HIPNUC_def || HIPNUC_FAILOVER
