/**
  ******************************************************************************
  * @file    ads1220.h
  * @brief   ADS1220 驱动 (寄存器位域 / 指令 / 数据读取 / 批量配置)
  *
  *  寄存器与指令的**唯一事实来源**: TI ADS1220 数据手册 SBAS501D
  *    - Table 8-7  Command Definitions
  *    - Table 8-8  Configuration Register Map
  *    - Table 8-10/8-11/8-13/8-14 各寄存器位定义
  *    - Table 8-4  Conversion Times
  *    - 8.5.1.3 Data Ready / 8.5.4 Reading Data
  *
  *  ⚠️ 需求文档中的寄存器描述与数据手册的位位置有几处不一致, 本驱动按**数据手册**
  *     实现, 并在下面逐条标注。功能意图完全按需求保留。
  *
  *   ┌──────────┬──────────────────────────────────────────────────────────────┐
  *   │ 需求原文 │ 实际情况 (SBAS501D)                                          │
  *   ├──────────┼──────────────────────────────────────────────────────────────┤
  *   │ CONFIG0  │ CONFIG0 = MUX[7:4] | GAIN[3:1] | PGA_BYPASS[0]               │
  *   │  "DR=20  │ DR 在 CONFIG1[7:5], 不在 CONFIG0 → 放到 CONFIG1 设置          │
  *   │   SPS"   │                                                              │
  *   ├──────────┼──────────────────────────────────────────────────────────────┤
  *   │ CONFIG1  │ CONFIG1 = DR[7:5] | MODE[4:3] | CM[2] | TS[1] | BCS[0]        │
  *   │ "FIR=11  │ FIR/50-60 抑制在 CONFIG2[5:4], 不在 CONFIG1 → 见 CONFIG2      │
  *   │  同时抑  │ 而且 "同时抑制 50 和 60Hz" 是 **01b**, 11b 只是"仅抑制 60Hz"  │
  *   │  制50/60"│ → 本驱动用 ADS1220_REJECT_50_60 = 0x10 (=01b)                 │
  *   │ "BCS=断  │ BCS 在 CONFIG1[0] ✔ (需求放在 CONFIG2 是笔误)                 │
  *   │  线检测" │                                                              │
  *   ├──────────┼──────────────────────────────────────────────────────────────┤
  *   │ CONFIG2  │ CONFIG2 = VREF[7:6] | 50/60[5:4] | PSW[3] | IDAC[2:0]         │
  *   │ "VREF 内 │ VREF=内部2.048V ✔ (00b)   IDAC=关闭 ✔ (000b)                  │
  *   │  部/IDAC │ "DRDY 使能" 不在 CONFIG2 → 见 CONFIG3.DRDYM                   │
  *   │  关/DRDY"│                                                              │
  *   ├──────────┼──────────────────────────────────────────────────────────────┤
  *   │ CONFIG3  │ CONFIG3 = I1MUX[7:5] | I2MUX[4:2] | DRDYM[1] | RSVD[0]        │
  *   │ "连续转  │ "连续转换模式"是 CONFIG1.CM[2], 不在 CONFIG3 → 见 CONFIG1     │
  *   │  换/DRDY │ "DRDY 专用引脚使能" = DRDYM=0 (0b 只用专用 DRDY 引脚)          │
  *   │  专用"   │ → CONFIG3 = 0x00                                              │
  *   ├──────────┼──────────────────────────────────────────────────────────────┤
  *   │ "SELF    │ ADS1220 **没有** 0x04 这条指令。六条指令只有 RESET/START/    │
  *   │ CAL 0x04"│ POWERDOWN/RDATA/RREG/WREG。                                   │
  *   │          │ 手册 9.1.5 给出的等效做法 = 把 MUX 置 1110b(输入内部短接到  │
  *   │          │ (AVDD+AVSS)/2), 采多次求平均存到 MCU, 之后每次读数减去它。    │
  *   │          │ → 本驱动实现 ADS1220_OffsetCalibrateAll() 完成这件事。        │
  *   └──────────┴──────────────────────────────────────────────────────────────┘
  ******************************************************************************
  */

#ifndef __ADS1220_H
#define __ADS1220_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "board_config.h"
#include "spi_driver.h"

/** 驱动返回码沿用总线层定义, 方便直接透传 */
typedef SPI_Driver_Status_t ADS1220_Status_t;

/*==============================================================================
 * 1. 指令 (Table 8-7, SBAS501D)
 *    RESET       0000 011x   -> 0x06
 *    START/SYNC  0000 100x   -> 0x08   (单次模式启动一次; 连续模式必须发一次才开始)
 *    POWERDOWN   0000 001x   -> 0x02
 *    RDATA       0001 xxxx   -> 0x10
 *    RREG        0010 rrnn
 *    WREG        0100 rrnn
 *    rr = 起始寄存器地址(0..3), nn = 字节数-1 (0..3)
 *============================================================================*/
#define ADS1220_CMD_RESET          0x06u
#define ADS1220_CMD_START_SYNC     0x08u
#define ADS1220_CMD_POWERDOWN      0x02u
#define ADS1220_CMD_RDATA          0x10u

/** RREG 指令字节: 从寄存器 start 起连续读 cnt 个 (cnt = 1..4) */
#define ADS1220_CMD_RREG(start, cnt) \
        ((uint8_t)(0x20u | (((start) & 0x03u) << 2) | (((cnt) - 1u) & 0x03u)))
/** WREG 指令字节: 从寄存器 start 起连续写 cnt 个 (cnt = 1..4) */
#define ADS1220_CMD_WREG(start, cnt) \
        ((uint8_t)(0x40u | (((start) & 0x03u) << 2) | (((cnt) - 1u) & 0x03u)))

/* 写全部 4 个寄存器: 0x40 | (0<<2) | 3 = 0x43
   读全部 4 个寄存器: 0x20 | (0<<2) | 3 = 0x23   (手册 9.1.6 伪代码使用的就是这两个) */

/*==============================================================================
 * 2. 寄存器地址
 *============================================================================*/
#define ADS1220_REG_CONFIG0        0x00u
#define ADS1220_REG_CONFIG1        0x01u
#define ADS1220_REG_CONFIG2        0x02u
#define ADS1220_REG_CONFIG3        0x03u
#define ADS1220_REG_COUNT          4u

/*==============================================================================
 * 3. CONFIG0 (00h) = MUX[3:0] | GAIN[2:0] | PGA_BYPASS
 *    复位值 00h
 *============================================================================*/
/* --- MUX[3:0]: 输入多路复用 (Table 8-10) --- */
#define ADS1220_MUX_AIN0_AIN1      0x00u   /* AINP=AIN0, AINN=AIN1  ← CH_A */
#define ADS1220_MUX_AIN0_AIN2      0x10u
#define ADS1220_MUX_AIN0_AIN3      0x20u
#define ADS1220_MUX_AIN1_AIN2      0x30u
#define ADS1220_MUX_AIN1_AIN3      0x40u
#define ADS1220_MUX_AIN2_AIN3      0x50u   /* AINP=AIN2, AINN=AIN3  ← CH_B */
#define ADS1220_MUX_AIN1_AIN0      0x60u
#define ADS1220_MUX_AIN3_AIN2      0x70u
#define ADS1220_MUX_AIN0_AVSS      0x80u   /* 以下单端设置要求 PGA_BYPASS=1, 增益<=4 */
#define ADS1220_MUX_AIN1_AVSS      0x90u
#define ADS1220_MUX_AIN2_AVSS      0xA0u
#define ADS1220_MUX_AIN3_AVSS      0xB0u
#define ADS1220_MUX_VREF_MON       0xC0u   /* (VREFPx-VREFNx)/4 监测 (PGA 旁路) */
#define ADS1220_MUX_AVDD_MON       0xD0u   /* (AVDD-AVSS)/4 监测 (PGA 旁路) */
#define ADS1220_MUX_SHORTED        0xE0u   /* AINP/AINN 内部短接到 (AVDD+AVSS)/2
                                              ← 用于失调校准, 见 9.1.5 / 8.3.12 */
#define ADS1220_MUX_RESERVED       0xF0u

/* --- GAIN[2:0] --- */
#define ADS1220_GAIN_1             0x00u
#define ADS1220_GAIN_2             0x02u
#define ADS1220_GAIN_4             0x04u
#define ADS1220_GAIN_8             0x06u
#define ADS1220_GAIN_16            0x08u
#define ADS1220_GAIN_32            0x0Au
#define ADS1220_GAIN_64            0x0Cu
#define ADS1220_GAIN_128           0x0Eu

/* --- PGA_BYPASS --- */
#define ADS1220_PGA_ENABLED        0x00u   /* PGA 使能 */
#define ADS1220_PGA_BYPASSED       0x01u   /* PGA 关闭并旁路 (增益只能 1/2/4),
                                              共模范围可到 AVSS-0.1V ~ AVDD+0.1V */
/*==============================================================================
 * 4. CONFIG1 (01h) = DR[2:0] | MODE[1:0] | CM | TS | BCS
 *    复位值 00h
 *============================================================================*/
/* --- DR[2:0]: 数据速率 (Table 8-12, 正常模式) --- */
#define ADS1220_DR_20SPS           0x00u
#define ADS1220_DR_45SPS           0x20u
#define ADS1220_DR_90SPS           0x40u
#define ADS1220_DR_175SPS          0x60u
#define ADS1220_DR_330SPS          0x80u
#define ADS1220_DR_600SPS          0xA0u
#define ADS1220_DR_1000SPS         0xC0u
#define ADS1220_DR_RESERVED        0xE0u   /* 111b 保留, 禁止使用 */

/* --- MODE[1:0]: 调制器工作模式 --- */
#define ADS1220_MODE_NORMAL        0x00u   /* 00b 正常模式 256kHz 调制器 */
#define ADS1220_MODE_DUTY_CYCLE    0x08u   /* 01b 占空比模式 */
#define ADS1220_MODE_TURBO         0x10u   /* 10b Turbo 模式 512kHz 调制器 */
#define ADS1220_MODE_RESERVED      0x18u   /* 11b 保留 */

/* --- CM: 转换模式 (需求中的"连续转换模式") --- */
#define ADS1220_CM_SINGLE_SHOT     0x00u
#define ADS1220_CM_CONTINUOUS      0x04u

/* --- TS: 内部温度传感器 (需求中的"TS=1b 读内部温度") --- */
#define ADS1220_TS_ADC             0x00u   /* 0b 正常 ADC 转换(差分/单端由 MUX 决定) */
#define ADS1220_TS_TEMPERATURE     0x02u   /* 1b 温度传感器模式; 此时 CONFIG0 无效,
                                              且强制使用内部基准 */

/* --- BCS: 10uA 烧断电流源 (断线检测) --- */
#define ADS1220_BCS_OFF            0x00u
#define ADS1220_BCS_ON             0x01u

/*==============================================================================
 * 5. CONFIG2 (02h) = VREF[1:0] | 50/60[1:0] | PSW | IDAC[2:0]
 *    复位值 00h
 *============================================================================*/
/* --- VREF[1:0] --- */
#define ADS1220_VREF_INTERNAL_2V048 0x00u  /* 00b 内部 2.048V  ← 需求 */
#define ADS1220_VREF_EXTERNAL_REFP0 0x40u  /* 01b 外部 REFP0/REFN0 */
#define ADS1220_VREF_EXTERNAL_AIN   0x80u  /* 10b 外部 AIN0/REFP1, AIN3/REFN1 */
#define ADS1220_VREF_ANALOG_SUPPLY  0xC0u  /* 11b AVDD-AVSS */

/* --- 50/60[1:0]: 内部 FIR 滤波器 (Table 8-13) ---
 *  ⚠️ 只能配合"正常模式 + DR=20SPS"使用; 其它数据率必须写 00b。 */
#define ADS1220_REJECT_NONE        0x00u   /* 00b 不抑制 */
#define ADS1220_REJECT_50_60       0x10u   /* 01b **同时**抑制 50Hz 和 60Hz ← 需求 */
#define ADS1220_REJECT_50_ONLY     0x20u   /* 10b 只抑制 50Hz */
#define ADS1220_REJECT_60_ONLY     0x30u   /* 11b 只抑制 60Hz (需求写成 11 是笔误) */

/* --- PSW: 低边电源开关 --- */
#define ADS1220_PSW_OPEN           0x00u
#define ADS1220_PSW_AUTO           0x08u

/* --- IDAC[2:0] --- */
#define ADS1220_IDAC_OFF           0x00u   /* 000b 关闭 ← 需求 */
#define ADS1220_IDAC_10UA          0x01u
#define ADS1220_IDAC_50UA          0x02u
#define ADS1220_IDAC_100UA         0x03u
#define ADS1220_IDAC_250UA         0x04u
#define ADS1220_IDAC_500UA         0x05u
#define ADS1220_IDAC_1000UA        0x06u
#define ADS1220_IDAC_1500UA        0x07u

/*==============================================================================
 * 6. CONFIG3 (03h) = I1MUX[2:0] | I2MUX[2:0] | DRDYM | RESERVED
 *    复位值 00h
 *============================================================================*/
#define ADS1220_I1MUX_OFF          0x00u
#define ADS1220_I1MUX_AIN0         0x20u
#define ADS1220_I1MUX_AIN1         0x40u
#define ADS1220_I1MUX_AIN2         0x60u
#define ADS1220_I1MUX_AIN3         0x80u
#define ADS1220_I1MUX_REFP0        0xA0u
#define ADS1220_I1MUX_REFN0        0xC0u

#define ADS1220_I2MUX_OFF          0x00u
#define ADS1220_I2MUX_AIN0         0x04u
#define ADS1220_I2MUX_AIN1         0x08u
#define ADS1220_I2MUX_AIN2         0x0Cu
#define ADS1220_I2MUX_AIN3         0x10u
#define ADS1220_I2MUX_REFP0        0x14u
#define ADS1220_I2MUX_REFN0        0x18u

/** DRDYM: DOUT/DRDY 引脚行为
 *  0b: 只有专用 DRDY 引脚指示数据就绪  ← 需求"DRDY 专用引脚使能"
 *      ★ 多片共享总线时必须用 0, 因为 CS 为高时 DOUT/DRDY 是高阻的,
 *        把它当"数据就绪"指示在多片场景下根本不可用 (手册 8.5.1.5 明确说明)。
 *  1b: DOUT/DRDY 与 DRDY 同时指示 */
#define ADS1220_DRDYM_DRDY_PIN_ONLY 0x00u
#define ADS1220_DRDYM_BOTH_PINS     0x02u

/*==============================================================================
 * 7. 配置结构体 (使用"已移位"的常量, 与数据手册位位置一一对应)
 *============================================================================*/
typedef struct
{
  uint8_t mux;        /**< MUX[3:0]      见 ADS1220_MUX_xxx        */
  uint8_t gain;       /**< GAIN[2:0]     见 ADS1220_GAIN_xxx       */
  uint8_t pga_bypass; /**< PGA_BYPASS    见 ADS1220_PGA_xxx        */
  uint8_t dr;         /**< DR[2:0]       见 ADS1220_DR_xxx         */
  uint8_t mode;       /**< MODE[1:0]     见 ADS1220_MODE_xxx       */
  uint8_t cm;         /**< CM            见 ADS1220_CM_xxx         */
  uint8_t ts;         /**< TS            见 ADS1220_TS_xxx         */
  uint8_t bcs;        /**< BCS           见 ADS1220_BCS_xxx        */
  uint8_t vref;       /**< VREF[1:0]     见 ADS1220_VREF_xxx       */
  uint8_t reject;     /**< 50/60[1:0]    见 ADS1220_REJECT_xxx     */
  uint8_t psw;        /**< PSW           见 ADS1220_PSW_xxx        */
  uint8_t idac;       /**< IDAC[2:0]     见 ADS1220_IDAC_xxx       */
  uint8_t i1mux;      /**< I1MUX[2:0]    见 ADS1220_I1MUX_xxx      */
  uint8_t i2mux;      /**< I2MUX[2:0]    见 ADS1220_I2MUX_xxx      */
  uint8_t drdym;      /**< DRDYM         见 ADS1220_DRDYM_xxx      */
} ADS1220_Config_t;

/*==============================================================================
 * 8. 数据速率表 (Table 8-4, 正常模式, 内部 4.096MHz 振荡器)
 *============================================================================*/
typedef struct
{
  uint8_t  dr_bits;   /**< DR[2:0] 已移位常量 */
  uint16_t sps;       /**< 标称数据率 */
  uint32_t conv_us;   /**< 实际转换时间 (连续转换模式的 DRDY 间隔) */
  uint8_t  reject_ok; /**< 1 = 允许打开 50/60 抑制 (仅 20SPS 正常模式) */
} ADS1220_RateInfo_t;

/*==============================================================================
 * 9. API
 *============================================================================*/

/** 用本工程默认配置(需求所描述的那套)填充 cfg。 */
void ADS1220_DefaultConfig(ADS1220_Config_t *cfg);

/** 把 cfg 打包成 4 个寄存器字节 regs[0..3] = CONFIG0..CONFIG3。 */
void ADS1220_BuildRegs(const ADS1220_Config_t *cfg, uint8_t regs[ADS1220_REG_COUNT]);

/** 取数据率信息; dr_bits 非法时返回 20SPS 的信息。 */
const ADS1220_RateInfo_t *ADS1220_RateInfo(uint8_t dr_bits);

/** 取转换时间(us), 用于"等一个转换周期"。 */
uint32_t ADS1220_ConversionTimeUs(uint8_t dr_bits);

/*---- 单片刻操作 ----*/
ADS1220_Status_t ADS1220_ResetChip(uint8_t chip);
ADS1220_Status_t ADS1220_WriteReg(uint8_t chip, uint8_t addr, uint8_t val);
ADS1220_Status_t ADS1220_WriteRegs(uint8_t chip, uint8_t start, const uint8_t *vals, uint8_t cnt);
ADS1220_Status_t ADS1220_ReadRegs(uint8_t chip, uint8_t start, uint8_t *vals, uint8_t cnt);
ADS1220_Status_t ADS1220_Start(uint8_t chip);
ADS1220_Status_t ADS1220_PowerDown(uint8_t chip);

/** 读一次转换结果。
 *  连续转换模式下, DRDY 下降沿后数据就在输出移位寄存器里, 直接锁 24 个 SCLK
 *  即可读出 (手册 9.1.6 伪代码 / Figure 8-25), 不需要 RDATA 指令。
 *  @param code  24bit 二进制补码, 已符号扩展
 *  @param raw   可选, 返回 3 个原始字节 (raw[0]=MSB) */
ADS1220_Status_t ADS1220_ReadData(uint8_t chip, int32_t *code, uint8_t raw[3]);

/** 用 RDATA 指令读一次 (任何时候都能读, 不依赖 DRDY; 调试/单次用)。 */
ADS1220_Status_t ADS1220_ReadDataByCmd(uint8_t chip, int32_t *code, uint8_t raw[3]);

/** 把一份完整配置写进 16 片 (WREG 0x43 + 4 字节), 并记录为当前配置镜像。 */
ADS1220_Status_t ADS1220_SetConfigAll(const ADS1220_Config_t *cfg);

/** 取回当前生效的配置镜像 (未初始化时返回默认配置)。
 *  典型用法: Get -> 改 dr/reject -> SetConfigAll。 */
void ADS1220_GetConfig(ADS1220_Config_t *cfg);

/*---- 16 片批量操作 (广播: 每片单独一次 CS, 其余 CS 保持高) ----*/
ADS1220_Status_t ADS1220_ResetAll(void);
ADS1220_Status_t ADS1220_WriteRegAll(uint8_t addr, uint8_t val);
ADS1220_Status_t ADS1220_WriteRegsAll(uint8_t start, const uint8_t *vals, uint8_t cnt);
ADS1220_Status_t ADS1220_StartAll(void);
ADS1220_Status_t ADS1220_PowerDownAll(void);
ADS1220_Status_t ADS1220_SetMuxAll(uint8_t mux);
ADS1220_Status_t ADS1220_SetTempSensorAll(uint8_t enable);
ADS1220_Status_t ADS1220_ReadDataAll(int32_t code[ADS1220_CHIP_COUNT]);

/** 回读 4 个寄存器并与本次写入的值比对。
 *  @param ok_mask  bit n = 1 表示第 n 片回读正确 (通信正常)
 *  @param written 本次刚写入的 4 个字节, 传 NULL 则跳过比对只做回读 */
ADS1220_Status_t ADS1220_VerifyAll(const uint8_t written[ADS1220_REG_COUNT],
                                   uint16_t *ok_mask);

/** 完整上电初始化序列 (对应需求"初始化流程"):
 *    1) 等 >= 50ms  (电源/上电复位)
 *    2) 逐片 RESET(0x06) -> 等 1ms   (手册要求 >= 50us + 32*tCLK)
 *    3) 逐片 WREG 写 CONFIG0..3
 *    4) 逐片回读校验 -> ok_mask
 *    5) 逐片 START/SYNC(0x08)
 *  @param ok_mask 可为 NULL */
ADS1220_Status_t ADS1220_PowerUpInit(const ADS1220_Config_t *cfg, uint16_t *ok_mask);

/** 内部短路失调校准 (手册 9.1.5 / 8.3.12):
 *    把 16 片 MUX 置 1110b(输入短接到 (AVDD+AVSS)/2) -> 采样 samples 次求平均
 *    -> 存下每次读数的平均码值 -> 恢复 MUX。
 *  @param offset_code  输出 16 片的平均失调码值 (后续读数减去它)
 *  @param samples      平均次数 (>=1)
 *  @note  本函数是**阻塞**的, 耗时约 samples x 转换周期 + 若干 ms, 只在上电时调用。
 *  @note  该失调是在"当前 GAIN/PGA_BYPASS 设置"下测得的, 改增益后需要重做。 */
ADS1220_Status_t ADS1220_OffsetCalibrateAll(const ADS1220_Config_t *cfg,
                                            int32_t offset_code[ADS1220_CHIP_COUNT],
                                            uint8_t samples);

/*---- 数值换算 ----*/
/** 3 字节原始数据 -> 24bit 有符号码值 (二进制补码) */
int32_t ADS1220_RawToCode(const uint8_t raw[3]);

/** 码值 -> 输入电压 (uV)。1LSB = (2*VREF/Gain)/2^24 = (VREF/Gain)/2^23
 *  内部基准 VREF = 2.048V 时: 1LSB = 0.244140625 uV / Gain */
float ADS1220_CodeToMicroVolt(int32_t code, uint8_t gain_bits);

/** 温度传感器模式码值 -> 摄氏度。
 *  14bit 结果左对齐在 24bit 数据里, 1 个 14bit LSB = 0.03125°C (8.3.13)。*/
float ADS1220_TempCodeToCelsius(int32_t code);

/** 从 GAIN[2:0] 常量取增益倍数 (1..128) */
uint16_t ADS1220_GainValue(uint8_t gain_bits);

#ifdef __cplusplus
}
#endif

#endif /* __ADS1220_H */
