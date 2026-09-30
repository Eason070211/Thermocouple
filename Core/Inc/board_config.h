/**
  ******************************************************************************
  * @file    board_config.h
  * @brief   16 x ADS1220 / 32 路热电偶测温系统 —— 硬件板级配置(单一事实来源)
  *
  *          所有"和具体板子有关"的东西都集中在本文件:
  *            - 芯片数量、通道数量
  *            - SPI1 引脚、片选引脚、合并 DRDY 中断引脚
  *            - UART 波特率
  *            - 全部关键时序常量 (tCLK / tRESET / tDATA / tCSSC / tSCCS ...)
  *
  *          === 目标硬件 (已确认) ===
  *            MCU      : STM32F103C8T6  LQFP48  72MHz
  *            SPI      : SPI1  Mode1 (CPOL=0, CPHA=1)  SCK=PA5 MISO=PA6 MOSI=PA7
  *            CS       : PB0..PB15  -> ADS1220 #0..#15  (低有效, 推挽 50MHz, 初始高)
  *            DRDY     : 16 片 DRDY 经 2x74HC30 + 1x74HC132 与非合并 -> PA0 (EXTI0 下降沿)
  *            UART     : USART1  TX=PA9  RX=PA10  115200(可 460800) 8N1
  *            DEBUG    : SWD  SWDIO=PA13  SWCLK=PA14
  *
  *          === 引脚预算核算 (LQFP48 共 35 个可用 IO) ===
  *            占用: PA5,PA6,PA7 (SPI1) + PA9,PA10 (USART1) + PA13,PA14 (SWD)
  *                  + PA0 (DRDY) + PB0..PB15 (CS x16)              = 24
  *            剩余: PA1,PA2,PA3,PA4,PA8,PA11,PA12,PA15,PC13,PC14,PC15 = 11
  *            => 刚好够用, 不要再把 PB 口挪作他用。
  *
  *          === DRDY 合并逻辑说明 ===
  *            ADS1220 的 DRDY 为 **低有效、推挽输出**, 且 "DRDY pin is always actively
  *            driven, even when CS is high" (SBAS501D 8.5.1.3), 所以可以直接进逻辑门。
  *            NAND(all16_H) = NOT(AND) = OR(any_low) , 因此
  *                合并输出 = 低  <=>  至少一片 ADS1220 有新的转换结果
  *            74HC132 是带施密特触发的 2 输入 NAND, 用来整形并驱动 EXTI0。
  *
  *            重要: 该合并信号 **无法分辨是哪一片** 拉低的。因此固件采用
  *            "广播 START/SYNC 让 16 片重新同步 -> 等合并 DRDY 下降沿 -> 逐片 RDATA"
  *            的策略 (详见 main.c 的轮询状态机注释)。
  ******************************************************************************
  */

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/*==============================================================================
 * 0. 目标 MCU 检查 / 系列选择
 *    用 HAL 头文件的 include guard 来判断系列, 比依赖 STM32F1xx 之类的宏可靠
 *    (CMSIS 的 stm32f1xx.h 并不会定义 STM32F1xx)。
 *============================================================================*/
#if defined(__STM32F1xx_HAL_H)
  #define BOARD_MCU_FAMILY_F1   1
#elif defined(__STM32F4xx_HAL_H)
  #define BOARD_MCU_FAMILY_F4   1
#else
  #error "board_config.h: 未识别的 HAL 系列 (期望 STM32F1xx 或 STM32F4xx)"
#endif

#if !defined(STM32F103xB) && !defined(STM32F103xE) && !defined(STM32F103xG)
  /* Keil 工程的 C/C++ Define 必须是 USE_HAL_DRIVER,STM32F103xB (见 docs/BUILD.md)。
     换型号时只改这一个宏 + 启动文件即可。 */
  #warning "board_config.h: 期望 STM32F103xB(或 xE/xG) 宏, 请检查 Keil 的 C/C++ Define"
#endif

/*==============================================================================
 * 1. 系统规模
 *============================================================================*/
/** ADS1220 片数 (PE 片 2 路差分 => 总通道 = 2 x 片数) */
#define ADS1220_CHIP_COUNT        16u
/** 每片 ADS1220 的差分通道数 (AIN0/AIN1 与 AIN2/AIN3) */
#define ADS1220_CH_PER_CHIP       2u
/** 总测温通道数 */
#define TC_CHANNEL_COUNT          (ADS1220_CHIP_COUNT * ADS1220_CH_PER_CHIP)   /* 32 */
/** 协议/缓冲区支持的上限 (片选位图用 16bit, 不要超过 16) */
#define ADS1220_MAX_CHIP          16u

#if (ADS1220_CHIP_COUNT > ADS1220_MAX_CHIP)
  #error "ADS1220_CHIP_COUNT 不能大于 16 (片选位图/CS 掩码按 16 位设计)"
#endif

/** 芯片 n 的 A 通道在温度数组中的下标; B 通道为 (n*2+1) */
#define TC_CH_INDEX_A(chip)       ((uint8_t)((chip) * ADS1220_CH_PER_CHIP))
#define TC_CH_INDEX_B(chip)       ((uint8_t)((chip) * ADS1220_CH_PER_CHIP + 1u))

/*==============================================================================
 * 2. SPI1 引脚  (SCK=PA5 / MISO=PA6 / MOSI=PA7)
 *============================================================================*/
#define SPI1_GPIO_PORT            GPIOA
#define SPI1_SCK_PIN              GPIO_PIN_5
#define SPI1_MISO_PIN             GPIO_PIN_6
#define SPI1_MOSI_PIN             GPIO_PIN_7
#define SPI1_GPIO_PINS            (SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN)

/** SPI1 时钟: APB2 = SYSCLK = 72MHz, 分频 /16 => 4.5MHz
 *  ADS1220 数据手册 tc(SC) >= 150ns  =>  f_SCLK <= 6.67MHz。
 *  (需求写的是 "<=10MHz", 但按 SBAS501D 6.6 节的 SCLK 周期下限,
 *   10MHz 会超规格; 4.5MHz 有 48% 裕量, 且是整数分频, 最稳妥。)
 *
 *  ★ 这里的三个量必须彼此一致:
 *      BOARD_APB2_HZ / ADS1220_SPI_DIV == ADS1220_SPI_SCLK_HZ == 实际 SCK
 *    ADS1220_SPI_BAUDRATE_PSC 是给 HAL 的枚举, 必须对应同一个分频值。
 *    改时钟树时只改 BOARD_APB2_HZ 和 ADS1220_SPI_DIV,
 *    下面的编译期检查会在写错时报错 (见 §6 末尾)。 */
#define BOARD_SYSCLK_HZ           72000000u   /* HSE 8MHz x PLL9 */
#define BOARD_APB2_HZ             72000000u   /* SystemClock_Config(): APB2 = HCLK/1 */
#define ADS1220_SPI_DIV           16u         /* HAL: SPI_BAUDRATEPRESCALER_16 */
#define ADS1220_SPI_BAUDRATE_PSC  SPI_BAUDRATEPRESCALER_16
#define ADS1220_SPI_SCLK_HZ       (BOARD_APB2_HZ / ADS1220_SPI_DIV)   /* 4.5MHz */
#define ADS1220_SPI_TIMEOUT_MS    2u       /* 单次 SPI 事务超时(正常 4 字节约 8us) */
#define SPI_DRIVER_SHORT_NOP_CNT  8u       /* SPI_DRIVER_SHORT_DELAY() 里的 NOP 个数 */

/*==============================================================================
 * 3. 片选 CS0..CS15 -> PB0..PB15
 *    片选直接映射成 GPIOB 的位掩码, 一次写 BSRR 即可, 不需要查表。
 *    若改到别的端口/引脚, 只需改 ADS1220_CS_PORT / ADS1220_CS_PIN()。
 *============================================================================*/
#define ADS1220_CS_PORT           GPIOB
#define ADS1220_CS_MASK_ALL       ((uint16_t)0xFFFFu)
/** 第 chip 片的片选掩码 (chip = 0..15 => PB0..PB15) */
#define ADS1220_CS_PIN(chip)      ((uint16_t)(1u << (chip)))

/*==============================================================================
 * 4. 合并 DRDY -> PA0 (EXTI0, 下降沿)
 *============================================================================*/
#define ADS1220_DRDY_PORT         GPIOA
#define ADS1220_DRDY_PIN          GPIO_PIN_0
#define ADS1220_DRDY_EXTI_IRQn    EXTI0_IRQn
#define ADS1220_DRDY_IRQ_PRIO     1u       /* NVIC 抢占优先级 (0 最高, 分组 NVIC_PRIORITYGROUP_4) */

/*==============================================================================
 * 5. USART1
 *============================================================================*/
#define UART1_GPIO_PORT           GPIOA
#define UART1_TX_PIN              GPIO_PIN_9
#define UART1_RX_PIN              GPIO_PIN_10
#define UART1_IRQ_PRIO            2u       /* 低于 DRDY */

/** 波特率: 115200 (默认) / 460800 (高速采样率时必须, 见 docs/TIMING.md) */
#define UART_BAUDRATE_115200      115200u
#define UART_BAUDRATE_460800      460800u
#ifndef UART_BAUDRATE
  #define UART_BAUDRATE           UART_BAUDRATE_115200
#endif

/*==============================================================================
 * 6. ADS1220 时序常量  (全部来自 TI 数据手册 SBAS501D)
 *============================================================================*/
/** 内部振荡器 4.096MHz => tCLK = 244.14ns (SBAS501D 6.6 节脚注) */
#define ADS1220_TCLK_NS           244.14f

/** 上电到可通信: 手册要求 >= 50us; 这里额外等 50ms 让 LDO/磁珠后的
 *  AVDD/DVDD 与 2.048V 基准彻底稳定 (TPS7A4901 软启动 + RC)。 */
#define ADS1220_T_POWERUP_MS      50u

/** RESET 指令后必须等待 (50us + 32 x tCLK) = 57.8us (SBAS501D 8.5.3.1)。
 *  实际取 1ms, 远大于最小值, 与需求一致。 */
#define ADS1220_T_RESET_MIN_US    58u
#define ADS1220_T_RESET_US        1000u

/** SPI 时序 (SBAS501D 6.6 / 6.7):
 *    td(CSSC)  CS 下降到第一个 SCLK 上升沿 >= 50ns
 *    td(SCCS)  最后一个 SCLK 下降到 CS 上升  >= 25ns
 *    tw(CSH)   CS 高电平最小宽度            >= 50ns
 *    tc(SC)    SCLK 周期                    >= 150ns
 *    tsu(DI)   DIN 在 SCLK 下降沿前建立     >= 50ns
 *    th(DI)    DIN 在 SCLK 下降沿后保持     >= 25ns
 *    tp(SCDO)  SCLK 上升沿到 DOUT 更新      <= 50ns
 *  结论: DIN 在 SCLK **下降沿**被采样, DOUT 在 SCLK **上升沿**更新
 *        => 正好是 SPI Mode 1 (CPOL=0, CPHA=1)。 */
#define ADS1220_T_CSSC_NS         50u
#define ADS1220_T_SCCS_NS         25u
#define ADS1220_T_CSH_NS          50u
#define ADS1220_T_SCLK_MIN_NS     150u

/** DRDY 下降沿到可以安全读数据之间的等待时间。
 *  注: 使用 **专用 DRDY 引脚** 时, 数据在 DRDY 下降沿就已经准备好 (手册 8.5.4:
 *      "Data can be read directly from this buffer on DOUT/DRDY when DRDY falls low
 *       without concern of data corruption"), 手册并未定义 tDATA。
 *      这里仍然保守地等 2us (约 8 个 tCLK), 用来覆盖:
 *        - 74HC30/74HC132 的传播延迟
 *        - EXTI 中断响应延迟
 *        - td(CSSC) 建立时间
 *  需求中"至少 1 个 CLK 周期"即指此值。 */
#define ADS1220_T_DATA_US         2u

/** 等待"下一片/下一通道"转换完成的窗口:
 *  最小等待 = 标称转换时间 x 9/10 (宁可晚一点, 保证读到的是新通道的数据)
 *  超时     = 标称转换时间 + 25ms (超时则判故障, 但流程继续, 绝不卡死) */
#define ADS1220_WAIT_MIN_NUM      9u
#define ADS1220_WAIT_MIN_DEN      10u
#define ADS1220_WAIT_MARGIN_MS    25u

/*==============================================================================
 * 6.1 编译期时序自检
 *     把上面那些"文档常量"真正用起来: 时钟树、SPI 分频、CS 建立延时
 *     只要有一项配错, 直接编译失败, 而不是等到示波器上才发现。
 *============================================================================*/
/* (1) SPI 时钟不能超过 ADS1220 的 SCLK 周期下限 tc(SC) >= 150ns */
#if (ADS1220_SPI_SCLK_HZ > (1000000000u / ADS1220_T_SCLK_MIN_NS))
  #error "SPI1 时钟超过 ADS1220 tc(SC)>=150ns 的上限(约6.67MHz), 请加大 ADS1220_SPI_DIV"
#endif

/* (2) SPI 时钟也不能太低, 否则 16 片 x 4 字节的批量读会挤掉转换窗口 */
#if (ADS1220_SPI_SCLK_HZ < 2000000u)
  #warning "SPI1 时钟低于 2MHz, 16 片批量读会明显变慢, 建议 >= 4MHz"
#endif

/* (3) CS 建立/保持用的 NOP 延时必须覆盖 td(CSSC)/td(SCCS)/tw(CSH) 里最大的那个。
 *     写成 (NOP数 x 1000) / (MHz) 的形式, 避免中间结果超过 32 位。 */
#if (((SPI_DRIVER_SHORT_NOP_CNT * 1000u) / (BOARD_SYSCLK_HZ / 1000000u)) < ADS1220_T_CSSC_NS)
  #error "SPI_DRIVER_SHORT_NOP_CNT 个 NOP 不足以满足 td(CSSC), 请加大 NOP 个数"
#endif

/* (4) 16 片 x 2 通道的片选位图必须放得下 */
#if (ADS1220_MAX_CHIP != 16u)
  #error "片选位图/芯片掩码按 16 位设计, ADS1220_MAX_CHIP 必须是 16"
#endif

/*==============================================================================
 * 7. 应用行为配置
 *============================================================================*/
/** 每完成 N 次转换切换一次 MUX (需求中的 N, 可配置)。
 *  N=1: 每次转换后切换 (默认, 通道更新最快)
 *  N>1: 每个通道连续读 N 次再切换, 减少 MUX 切换次数, 代价是每通道速率降为 1/N */
#ifndef TC_MUX_HOLD_CONVERSIONS
  #define TC_MUX_HOLD_CONVERSIONS 1u
#endif

/** BCS (10uA 烧断电流源) 使用方式 —— 断线检测:
 *    1 = BCS 常开 (严格按需求 "CONFIG1: BCS=断线检测使能"), 默认
 *    0 = BCS 关闭; 此时仍靠"采样值超量程"判断断线, 但要求模拟前端
 *        带有 TI 参考设计里的 1M~50M 偏置电阻 RB1/RB2 把开路输入拉满量程
 *  提示: 热电偶回路电阻 R 上会流过 10uA => 附加失调 10uA x R。
 *        2m 的 K 型偶丝回路电阻约几欧姆 => 几十 nV, 可忽略;
 *        若使用很长的补偿导线(上百欧姆), 建议改成 0 并在需要时临时开启。 */
#ifndef TC_BCS_ALWAYS_ON
  #define TC_BCS_ALWAYS_ON        1
#endif

/** 上位机多久没发命令时, 主动推送一帧状态 (0 = 不自动推送, 只应答) */
#define TC_STATUS_PUSH_PERIOD_MS  5000u

/** 上电初始化时对 16 片做一次内部短路失调校准 (手册 9.1.5 推荐, 用于替代
 *  ADS1220 并不存在的 "SELF CAL 0x04" 指令)。平均次数见 .c */
#ifndef TC_OFFSET_CAL_ENABLE
  #define TC_OFFSET_CAL_ENABLE    1
#endif
#define TC_OFFSET_CAL_SAMPLES     4u

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CONFIG_H */
