/**
  ******************************************************************************
  * @file    uart_protocol.h
  * @brief   USART1 通信协议: 组帧 / 拆帧 / CRC16 / 非阻塞收发
  *
  *  ============================ 帧格式 ============================
  *
  *  上位机 -> STM32 (下行命令):
  *      +------+------+------+------------------+--------+--------+
  *      | 0xAA | CMD  | LEN  | DATA[0..LEN-1]   | CRC_LO | CRC_HI |
  *      +------+------+------+------------------+--------+--------+
  *
  *  STM32 -> 上位机 (上行上报):
  *      +------+------+------+------------------+--------+--------+
  *      | 0x55 | CMD  | LEN  | DATA[0..LEN-1]   | CRC_LO | CRC_HI |
  *      +------+------+------+------------------+--------+--------+
  *
  *    LEN  = DATA 段的字节数 (不含 CMD/LEN/CRC)
  *    CRC  = CRC-16/MODBUS, 多项式 0xA001(反射), 初值 0xFFFF,
  *           覆盖范围 = 帧头 0xAA/0x55 一直到最后一个 DATA 字节 (不含 CRC 本身),
  *           **低字节在前**。
  *
  *  ---------------------------- 上行 CMD ----------------------------
  *    0x10 温度上报:  DATA = 32 个 float32 (小端 IEEE-754), 单位 °C。
  *                    通道顺序: ch[2n] = 第 n 片 AIN0/AIN1
  *                              ch[2n+1] = 第 n 片 AIN2/AIN3
  *                    无效通道填 NaN (0x7FC00000) —— 断线/通信失败/无数据。
  *
  *    0x11 状态上报:  DATA = 24 字节, 见下方 UART_STATUS_DATA_LEN 布局
  *
  *  ---------------------------- 下行 CMD ----------------------------
  *    0x01 设置采样率:
  *           LEN = 2: DATA[0] = DR   (索引 0..6, 也接受已移位的 0x00/0x20/.../0xC0)
  *                    DATA[1] = 50/60 抑制 (索引 0..3, 也接受 0x00/0x10/0x20/0x30)
  *           LEN = 1: DATA[0] = (DR索引 << 4) | (50/60索引)
  *           索引:  0=20SPS 1=45 2=90 3=175 4=330 5=600 6=1000
  *           注意:  50/60 抑制只能配合 20SPS + 正常模式 (手册 Table 8-13);
  *                  若 DR != 20SPS 却请求了抑制, 固件会自动关掉抑制并在状态帧报告。
  *           应答:  0x11 状态帧
  *
  *    0x02 启动/停止采集: DATA[0] = 0 停止 / 1 启动。  应答: 0x11 状态帧
  *
  *    0x03 读取单次温度: DATA 可省略;
  *           LEN = 0 或 DATA[0] = 0xFF  -> 全部通道 (协议固定 32 槽)
  *           LEN = 1 且 DATA[0] = 0..31 -> 只报该通道 (其余填 NaN)
  *           应答: 0x10 温度帧
  ******************************************************************************
  */

#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "board_config.h"

/*==============================================================================
 * 帧常量
 *============================================================================*/
#define UART_FRAME_HEAD_DOWN   0xAAu   /**< 下行帧头 (PC -> MCU) */
#define UART_FRAME_HEAD_UP     0x55u   /**< 上行帧头 (MCU -> PC) */

#define UART_CMD_UP_TEMP       0x10u   /**< 上行: 温度帧 (固定 32 槽, 本板用前 8 槽) */
#define UART_CMD_UP_STATUS     0x11u   /**< 上行: 状态 */

#define UART_CMD_DOWN_SET_RATE 0x01u   /**< 下行: 设置采样率 */
#define UART_CMD_DOWN_RUN      0x02u   /**< 下行: 启动/停止采集 */
#define UART_CMD_DOWN_SINGLE   0x03u   /**< 下行: 读取单次温度 */

#define UART_DATA_MAX_LEN      200u    /**< 允许的最大 DATA 长度 */
#define UART_FRAME_MAX_LEN     (3u + UART_DATA_MAX_LEN + 2u)

/** ★ 协议温度槽位数: 与板子实际通道数**无关**, 永远是 32。
 *
 *  这样做的好处: 4 路板 / 8 路板 / 16 路板 用**完全相同的协议和上位机软件**,
 *  没接的槽位填 NaN (0x7FC00000), 上位机显示成"未配置/断线"。
 *  状态帧里会上报"本板实际片数", 上位机据此只显示真实存在的通道。 */
#define UART_TEMP_SLOT_COUNT   32u
/** 温度帧 DATA 长度 = 32 x float32 = 128 字节 (固定) */
#define UART_TEMP_DATA_LEN     (UART_TEMP_SLOT_COUNT * 4u)
#define UART_STATUS_DATA_LEN   24u

#define UART_CH_SINGLE_ALL     0xFFu   /**< CMD=0x03 时表示"全部通道" */

/** 帧内字节间超时(ms): 超时就丢弃半截帧, 重新找帧头 */
#define UART_FRAME_GAP_MS      50u

/** 发送环形缓冲区大小。一帧温度 = 133 字节, 512 能缓存 ~3 帧。 */
#define UART_TX_RING_SIZE      512u

/** 无效温度标记 (float32 小端 00 00 C0 7F) */
#define UART_TEMP_NAN_BITS     0x7FC00000u

/*==============================================================================
 * 上行状态帧 DATA 布局 (LEN = 24)
 *============================================================================*/
#define UART_ST_RUN_STATE      0u   /**< u8  0=停止 1=运行 */
#define UART_ST_DR_BITS        1u   /**< u8  当前 DR[2:0] (已移位值) */
#define UART_ST_REJECT         2u   /**< u8  高4位=50/60[1:0](已移位值), 低4位=本板片数 */
#define UART_ST_CHIP_OK_LO     3u   /**< u16 小端: bit n=1 第 n 片通信正常 */
#define UART_ST_CHIP_OK_HI     4u
#define UART_ST_CHIP_ERR_LO    5u   /**< u16 小端: bit n=1 第 n 片本轮出错 */
#define UART_ST_CHIP_ERR_HI    6u
#define UART_ST_ROUND_LO       7u   /**< u32 小端: 已完成测量轮次 */
#define UART_ST_ROUND_HI      10u
#define UART_ST_UPTIME_LO     11u   /**< u32 小端: 运行时间 ms */
#define UART_ST_UPTIME_HI     14u
#define UART_ST_DRDY_CNT_LO   15u   /**< u16 小端: "全部就绪"次数(低16位) */
#define UART_ST_DRDY_CNT_HI   16u
#define UART_ST_SPI_ERR_LO    17u   /**< u16 小端: SPI 事务失败累计 */
#define UART_ST_SPI_ERR_HI    18u
#define UART_ST_OPEN_CNT_LO   19u   /**< u16 小端: 断线累计次数 */
#define UART_ST_OPEN_CNT_HI   20u
#define UART_ST_FLAGS         21u   /**< u8  异常标志, 见 UART_ST_FLAG_xxx */
#define UART_ST_TIMEOUT_LO    22u   /**< u16 小端: 等待 DRDY 超时累计次数 */
#define UART_ST_TIMEOUT_HI    23u

/* 状态帧 flags 位定义 */
#define UART_ST_FLAG_SELFTEST   0x01u  /**< 上电自检失败 (热电偶多项式) */
#define UART_ST_FLAG_DRDY_SILENT 0x02u /**< 长时间收不到 DRDY (所有片都哑了?) */
#define UART_ST_FLAG_VERIFY_FAIL 0x04u /**< 上电回读校验不通过 (有片没焊/虚焊?) */
#define UART_ST_FLAG_TX_OVERFLOW 0x08u /**< 串口发送缓冲溢出(丢过帧) */
#define UART_ST_FLAG_CRC_ERR     0x10u /**< 收到过 CRC 错的下行帧 */
#define UART_ST_FLAG_DRDY_PARTIAL 0x20u /**< ★新增: 有片一直不就绪(看 chip_err 位图定位) */

/*==============================================================================
 * 事件
 *============================================================================*/
typedef enum
{
  UART_EV_NONE = 0,
  UART_EV_SET_RATE,   /**< 设置采样率 */
  UART_EV_START,      /**< 启动采集 */
  UART_EV_STOP,       /**< 停止采集 */
  UART_EV_SINGLE      /**< 读取单次温度 */
} UART_Event_t;

typedef struct
{
  UART_Event_t ev;
  uint8_t dr_bits;    /**< SET_RATE: 已移位的 DR[2:0] */
  uint8_t reject;     /**< SET_RATE: 已移位的 50/60[1:0] */
  uint8_t channel;    /**< SINGLE: 0..31 或 UART_CH_SINGLE_ALL */
} UART_EventMsg_t;

/** 状态帧的数据来源 */
typedef struct
{
  uint8_t  run_state;
  uint8_t  dr_bits;
  uint8_t  reject;            /**< 已移位的 50/60[1:0] (0x00/0x10/0x20/0x30) */
  uint8_t  chip_count;        /**< ★本板实际片数 (1..16), 填进状态帧 byte2 低 4 位 */
  uint16_t chip_ok_mask;
  uint16_t chip_err_mask;
  uint32_t round_count;
  uint32_t uptime_ms;
  uint16_t drdy_irq_count;    /**< "全部就绪"次数 (老版本是合并 DRDY 中断次数) */
  uint16_t spi_err_count;
  uint16_t open_tc_count;
  uint16_t phase_timeout_count;
  uint8_t  flags;
} UART_Status_t;

/*==============================================================================
 * API
 *============================================================================*/
/** 启动接收中断。必须在 MX_USART1_UART_Init() 之后调用。 */
void UART_Protocol_Init(void);

/** 由 HAL_UART_RxCpltCallback 调用 (uart_protocol.c 内部已实现该回调)。 */
void UART_Protocol_OnRxByte(uint8_t byte);

/** 主循环调用: 帧内超时复位 + 接收中断自愈 + 发送续传。 */
void UART_Protocol_Poll(void);

/** 取一个待处理事件。@retval 1 = 取到, 0 = 无事件 */
uint8_t UART_Protocol_GetEvent(UART_EventMsg_t *msg);

/** CRC-16/MODBUS (多项式 0xA001 反射, 初值 0xFFFF)。 */
uint16_t UART_CRC16(const uint8_t *data, uint16_t len);

/*----------------------------------------------------------------------------
 * 发送 (全部走中断 + 环形缓冲, 不阻塞主循环)
 *--------------------------------------------------------------------------*/
/** 通用组帧发送: 自动加 0x55 帧头 / LEN / CRC。 @retval 1 = 已入队, 0 = 缓冲满已丢弃 */
uint8_t UART_Protocol_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len);

/** 上报温度帧 (CMD=0x10)。
 *  ★ 固定 32 个槽位 (UART_TEMP_SLOT_COUNT), 与板子通道数无关;
 *    本板没接的槽位由 main.c 填 NaN, 上位机会显示成"未配置"。
 *    temps 里 NaN 会被原样发出表示无效。 */
uint8_t UART_Protocol_SendTempFrame(const float temps[UART_TEMP_SLOT_COUNT]);

/** 上报状态 (CMD=0x11)。 */
uint8_t UART_Protocol_SendStatusFrame(const UART_Status_t *st);

/** 组一个 float32 小端到 buf (供上位机参考实现对齐) */
void UART_PutFloatLE(uint8_t *buf, float v);
/** 组一个 uint16/uint32 小端 */
void UART_PutU16LE(uint8_t *buf, uint16_t v);
void UART_PutU32LE(uint8_t *buf, uint32_t v);

/** 返回协议规定的"无效温度"值 (0x7FC00000, 即 quiet NaN)。
 *  用它而不是 NAN 宏, 保证位型在 Keil/GCC 下完全一致。 */
float UART_NaN(void);

/** 统计: 收到过的 CRC 错帧数 */
uint16_t UART_Protocol_GetCrcErrCount(void);
/** 统计: 发送环形缓冲溢出(丢帧)次数 */
uint16_t UART_Protocol_GetTxOverflowCount(void);

/** CRC / 组帧自检。 @retval 1 = 通过 */
uint8_t UART_Protocol_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_PROTOCOL_H */
