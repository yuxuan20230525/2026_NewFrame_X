/*****************************************************************************
File name: TDT_Device\inc\hipnuc_imu.h
Description: HiPNUC HI 系列外置 IMU 驱动 (USART1 + HI91 二进制帧 + SYNC_IN 硬件时间同步)
            派生自 ImuCalc, 复用模块自带 6/9 轴融合(euler/quat), 接入 imutask 接口。
            数据口走 C 板 4-pin 对外串口连接器 = STM32 USART1(PA9 TX/PB7 RX, 壳印"UART2")。
            硬件时间同步: C 板当主, PI7(7路PWM排针C7) 发上升沿触发模块(ONMARK)。
设计文档: docs/superpowers/specs/2026-06-19-hipnuc-imu-uart5-sync-design.md
*****************************************************************************/
#ifndef _HIPNUC_IMU_H
#define _HIPNUC_IMU_H

#include "board.h"
#include "vision.h" // USART_DMA 基类(UART5 + 空闲中断 + DMA 收发机)
#include "imu.h"    // ImuCalc 基类

/* ============ HI91 帧格式 (手册 §5.9.10) ============ */
#define HIPNUC_SOF0          0x5A
#define HIPNUC_SOF1          0xA5
#define HIPNUC_HI91_TAG      0x91
#define HIPNUC_HI91_PAYLEN   76  ///< payload 字节数(LEN 字段值)
#define HIPNUC_HI91_FRAMELEN 82  ///< 整帧 = 6(SOF+LEN+CRC) + 76(payload)
/* payload 内偏移(从 tag 0x91 起, 即整帧 buf[6] 起) */
#define HI91_OFF_STATUS  1
#define HI91_OFF_SYSTIME 8
#define HI91_OFF_ACC     12 ///< float[3], 单位 G
#define HI91_OFF_GYR     24 ///< float[3], 单位 deg/s
#define HI91_OFF_MAG     36 ///< float[3], 单位 uT
#define HI91_OFF_ROLL    48 ///< float, deg
#define HI91_OFF_PITCH   52 ///< float, deg
#define HI91_OFF_YAW     56 ///< float, deg
#define HI91_OFF_QUAT    60 ///< float[4], WXYZ

/* ============ 驱动参数 ============ */
#define HIPNUC_RXBUF_SIZE    512 ///< 循环 DMA 缓冲, 921600 baud 下可覆盖约 6 ms 中断阻塞
#define HIPNUC_OFFLINE_TICKS 100 ///< 连续多少拍(1ms)无有效帧判离线
/// 在途触发缓存深度上限, 覆盖 1~2 拍管线延迟; 溢出丢最旧重同步。真机定标可调。
#define HIPNUC_PIPELINE_MAX  2
/// 板子上电是否补发 "LOG HI91 ONMARK 1"(重申 SYNC_IN 触发模式)。
/// 调试链路时想让模块跑周期模式(CHCenter 设 LOG HI91 ONTIME 0.001), 置 0 以免被覆盖。
#define HIPNUC_SEND_ONMARK   1

/// USART1 波特率。HI91 1kHz 需要 921600(115200 上限仅 100Hz)。
/// 探针调试可临时设 115200 看模块出厂默认数据(见首次上板记录)。
#define HIPNUC_BAUD          921600

/// 一次性配置: 让 C 板经 USART1 把模块从出厂态配好(6轴+安装方向+ONMARK+切921600+存flash+复位)。
/// 用法: 烧"配置版"(HIPNUC_PROVISION=1 且 HIPNUC_BAUD=115200) 上电跑一遍(开机阻塞约4s),
///       模块配好并复位到 921600/ONMARK 后, 再烧正式版(HIPNUC_PROVISION=0, HIPNUC_BAUD=921600)。
/// 注意: 配置版别长期留着——下次上电它又按 115200 去配已是 921600 的模块会失败。
#define HIPNUC_PROVISION     0
/// 安装方向编码(手册 §4.6.7.6), 24=默认 X右Y前Z上; 按实际贴装改。
/// 本机贴装=传感器 X后/Y右/Z上。214 输出系 X右Y前Z上(roll绕右=前后倾) → roll/pitch 与期望对调;
/// 改 134: 输出系 X前/Y左/Z上(标准机器人系), roll绕前=侧倾、pitch绕左=前后(平衡)倾。
#define HIPNUC_URFR_CODE     134

/// 同步模式(失效降级用)
#define HIPNUC_SYNC_HW       0  ///< 硬件: PI7 触发 ONMARK + 1拍流水线打戳(默认)
#define HIPNUC_SYNC_SW       1  ///< 软件: 模块周期输出 + 空闲中断打戳(触发线掉时降级)

/* ============ SYNC_IN 触发脚: PI7 (TIM8_CH3, 7路PWM排针C7), 当普通GPIO用 ============ */
#define HIPNUC_SYNC_PORT GPIOI
#define HIPNUC_SYNC_PIN  GPIO_Pin_7
#define HIPNUC_SYNC_RCC  RCC_AHB1Periph_GPIOI

/// HI91 一帧解析结果
struct HipnucFrame
{
    float    acc[3];  ///< G
    float    gyr[3];  ///< deg/s
    float    mag[3];  ///< uT
    float    eul[3];  ///< roll, pitch, yaw (deg)
    float    quat[4]; ///< w, x, y, z
    uint32_t systime; ///< ms (模块时钟, 仅调试/冗余)
    uint16_t status;  ///< main_status
};

/// J-Link/JScope 真机观测用快照 (含链路分层诊断)
struct HipnucDbg
{
    float    acc[3];
    float    gyr[3];
    float    eul[3];
    float    quat[4];
    float    readTime;
    uint32_t frameCnt;   ///< 解析成功的有效 HI91 帧数 (CRC+tag 都过)
    uint8_t  offline;
    /* ---- 链路分层诊断 (定位卡在哪一级) ---- */
    uint32_t isrCnt;     ///< USART1 空闲中断次数 = 有数据到达且线路转空闲
    uint32_t crcFail;    ///< 找到 5A A5+LEN=76 但 CRC 不过的次数
    uint32_t sofSeen;    ///< 流式解析器找到 5A A5 帧头的次数
    uint8_t  rxHead[8];  ///< 最近一次 IDLE 分片的头 8 字节
    uint32_t byteCnt;    ///< DMA 流式解析器消费的总字节数
    uint32_t invalidFrame; ///< CRC 正确但数值无效/越界的帧数
    uint32_t maxLossTicks; ///< 历史最大连续无新帧拍数
    uint8_t  syncMode;     ///< 0=硬件 ONMARK, 1=软件 ONTIME
    uint8_t  calibState;   ///< 0=空闲 1=检查中 2=校平成功 3=保存中 4=失败
    uint8_t  calibFailReason; ///< 1=离线 2=未接近水平 3=仍在运动 4=校平命令失败 5=保存失败 6=未脱力
    uint32_t calibCmdCnt;  ///< 已发起的内部自动校平次数
    float    calibPitch;   ///< 触发校平时的 Pitch(deg)
    float    calibRoll;    ///< 触发校平时的 Roll(deg)
};
extern HipnucDbg g_hipnucDbg;

/**
 * @brief HiPNUC 外置 IMU 设备类
 * @note  多继承: ImuCalc(接 imutask) + USART_DMA(UART5 收发)。
 *        用模块自带融合, 重写 TDT_IMU_update 直接填解算结果, 不跑板载 Fusion AHRS。
 */
class HipnucImu : public ImuCalc, public USART_DMA
{
public:
    HipnucImu(GPIO_TypeDef *txPort, uint16_t txPin, uint8_t txPinSource, uint8_t txAF,
              GPIO_TypeDef *rxPort, uint16_t rxPin, uint8_t rxPinSource, uint8_t rxAF,
              USART_TypeDef *usart, uint32_t baudRate,
              uint32_t dmaChannelRx, DMA_Stream_TypeDef *dmaStreamRx,
              uint32_t dmaChannelTx, DMA_Stream_TypeDef *dmaStreamTx,
              IRQn_Type irqnRx, IRQn_Type irqnTx, uint32_t TX_Flag, int32_t TX_IT_Flag,
              uint32_t clockGPIO, uint32_t clockUSART, uint32_t clockDMA,
              uint8_t *recvBuffer, uint32_t bufferSize)
        : ImuCalc(),
          USART_DMA(txPort, txPin, txPinSource, txAF,
                    rxPort, rxPin, rxPinSource, rxAF,
                    usart, baudRate,
                    dmaChannelRx, dmaStreamRx, dmaChannelTx, dmaStreamTx,
                    irqnRx, irqnTx, TX_Flag, TX_IT_Flag,
                    clockGPIO, clockUSART, clockDMA, recvBuffer, bufferSize)
    {
    }

    /* ---- ImuCalc 接口 ---- */
    void init() override;            ///< 起 UART5+DMA+IRQ、配 PI7、发 ONMARK
    void GetOriginData() override {} ///< 不走 int16 origin 管线(空实现)
    void TDT_IMU_update() override;  ///< 发触发 + 消费帧 + 填解算结果(1kHz)

    /* ---- USART_DMA 接口 ---- */
    void ProcessReceivedData() override; ///< 兼容基类接口, 实际走循环 DMA 流式解析
    void HandleUartIdleInterrupt();      ///< USART1 IDLE: 不停 DMA, 消费新收到的字节

    /* ---- 轴向重映射(真机定标; 默认单位映射) ---- */
    /// gyro/acc 向量重映射: out[i] = sign[i] * in[ map[i] ]
    void setAxisRemap(const uint8_t map[3], const float sign[3]);
    /// euler 符号: roll/pitch/yaw 各自取反(默认 +1)
    void setEulerSign(float rollSign, float pitchSign, float yawSign);
    /// gyro 角速度符号: x/y/z(roll/pitch/yaw 轴) 各自取反(默认 +1)。
    /// 与 setEulerSign 配套: 翻了 euler 某轴角度, 同轴角速度也要翻, 否则 PD 角度/角速度打架。
    void setGyroSign(float xSign, float ySign, float zSign);
    /// acc 符号: x/y/z 各自取反(默认 +1)。单独于 gyro, 可只翻 pitch 轴(acc[1]) 而留垂直轴(acc[2])不动。
    void setAccSign(float xSign, float ySign, float zSign);

    /// 手册 4.6.7.5: 静止且接近水平时执行 CONFIG ATT RST 3，保存模块配置后复位 top 主控。
    bool calibrateAttitude();

    /* ---- 失效降级支持 ---- */
    uint32_t lossTicks() const { return offlineCnt_; }                ///< 连续丢帧拍数(供监督切换判定)
    bool isSoftwareSync() const { return syncMode_ == HIPNUC_SYNC_SW; }
    void switchToSoftwareSync(); ///< 触发线掉: 发 LOG HI91 ONTIME 0.001 转周期, 改用 idle-latch 打戳

private:
    /* 接收侧(ISR 写, tick 读) */
    HipnucFrame       rxFrame_ = {};   ///< 最近解析成功的帧(tick 侧 __disable_irq 读)
    volatile uint32_t rxSeq_   = 0;    ///< 有效帧序号, ISR 递增, tick 比较增量
    volatile uint64_t rxTimeUs_ = 0;   ///< 帧到达(空闲中断)时刻, 软同步打戳用
    uint8_t  syncMode_ = HIPNUC_SYNC_HW; ///< HW=PI7触发 / SW=周期+软件打戳

    /* 循环 DMA + 跨 IDLE 分片的流式解析 */
    uint16_t rxReadPos_ = 0;
    uint8_t  parseBuf_[HIPNUC_HI91_FRAMELEN] = {};
    uint16_t parseLen_ = 0;
    void consumeDmaBytes();
    void parseByte(uint8_t byte);
    bool decodeFrame(const uint8_t *frame, HipnucFrame &out);
    static bool framePlausible(const HipnucFrame &frame);
    void parseCommandReplyByte(uint8_t byte);

    /* 触发流水线(仅 tick 上下文访问, 无 ISR 竞争) */
    uint64_t trigFifo_[HIPNUC_PIPELINE_MAX] = {};
    uint8_t  trigHead_ = 0, trigTail_ = 0, trigCount_ = 0;
    uint32_t lastSeq_   = 0;
    uint32_t offlineCnt_ = 0;
    void trigPush(uint64_t t);
    bool trigPop(uint64_t &t);

    /* ASCII 配置命令应答(接收中断写，校准任务读) */
    volatile uint8_t commandReply_ = 0; ///< 0=等待 1=OK 2=ERR
    volatile bool awaitingReply_ = false;
    uint8_t okMatch_ = 0;
    uint8_t errMatch_ = 0;
    bool sendCommandAndWait(uint8_t *command, uint16_t size, uint32_t timeoutMs);

    /* 轴向 */
    uint8_t axisMap_[3]  = {0, 1, 2};
    float   axisSign_[3] = {1.f, 1.f, 1.f}; ///< gyro+acc 共用的轴重映射符号
    float   eulSign_[3]  = {1.f, 1.f, 1.f}; ///< euler roll/pitch/yaw 符号
    float   gyroSign_[3] = {1.f, 1.f, 1.f}; ///< gyro x/y/z 额外符号(与 euler 配套)
    float   accSign_[3]  = {1.f, 1.f, 1.f}; ///< acc x/y/z 额外符号(单独, 可只翻 pitch 轴留垂直轴)

    /* SYNC_IN 触发脚 */
    void syncPinInit();
    inline void syncRise() { GPIO_SetBits(HIPNUC_SYNC_PORT, HIPNUC_SYNC_PIN); }
    inline void syncFall() { GPIO_ResetBits(HIPNUC_SYNC_PORT, HIPNUC_SYNC_PIN); }

    /* HI91 解析辅助 */
    static void crc16_update(uint16_t *crc, const uint8_t *src, uint32_t len);
    void applyFrame(const HipnucFrame &f); ///< 帧 -> acc/gyro/euler/quat(含过圈)

    /* 模块配置 */
    void sendOnmarkConfig(); ///< 经 USART1 TX 发 "LOG HI91 ONMARK 1"(重申触发模式)
    void provisionModule();  ///< 一次性: 从出厂态(115200)阻塞配到 921600+6轴+ONMARK+存flash+复位
};

extern HipnucImu hipnucImu;

#endif
