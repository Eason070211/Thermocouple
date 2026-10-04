/**
  ******************************************************************************
  * @file    board_config.h
  * @brief   N x ADS1220 / 2N 路热电偶测温系统 —— 硬件板级配置(单一事实来源)
  *
  *          所有"和具体板子有关"的东西都集中在本文件:
  *            - 芯片数量(= 实际贴装的 ADS1220 片数)、通道数量
  *            - SPI1 引脚、片选引脚、每片独立的 DRDY 引脚
  *            - UART 波特率
  *            - 全部关键时序常量 (tCLK / tRESET / tDATA / tCSSC / tSCCS ...)
  *
  *          === 本板规模(可调) ===
  *            贴 2 片 -> 4 路    (先打样验证用)
  *            贴 4 片 -> 8 路    (目标配置, 默认)
  *            贴 8 片 -> 16 路   (需要扩展 DRDY 引脚, 见第 4 节说明)
  *          改规模只改 ADS1220_CHIP_COUNT 一个宏, 其余全部自动推导。
  *
  *          === 目标硬件 (8 路: 4 x ADS1220) ===
  *            MCU      : STM32F103C8T6  LQFP48  72MHz
  *            SPI      : SPI1  Mode1 (CPOL=0, CPHA=1)  SCK=PA5 MISO=PA6 MOSI=PA7
  *            CS       : PB0..PB3  -> ADS1220 #0..#3  (低有效, 推挽 50MHz, 初始高)
  *            DRDY     : 每片一根, PA0..PA3 (输入 + 内部上拉, 软件轮询)
  *            UART     : USART1  TX=PA9  RX=PA10  115200(可 460800) 8N1
  *            DEBUG    : SWD  SWDIO=PA13 SWCLK=PA14
  *
  *          === 为什么改成"每片一根 DRDY"而不是原来的合并逻辑门 ===
  *            16 片版本用 2x74HC30 + 74HC132 把 DRDY 与非合并成一路 (PA0/EXTI0)。
  *            8 路版本只有 4 片, 引脚完全够用, 于是改成每片一根 DRDY 直接进 MCU:
  *
  *              1) 省掉外部逻辑芯片和它的去耦, 板子更小、BOM 更短、少一个故障点;
  *              2) 没贴的芯片那一根 DRDY 用 MCU 内部上拉钳到高 = "永远没有新数据",
  *                 不需要外部上拉电阻, 也不存在悬空导致误判的风险
  *                 (这是"先贴 2 片、以后再贴满"能直接跑通的关键);
  *              3) 固件能分辨"是哪一片没响应", 调试时直接定位到具体芯片
  *                 (状态帧的 chip_err 位图 + UART_ST_FLAG_DRDY_PARTIAL 标志);
  *              4) 不再需要外部逻辑门的传播延迟, 时序余量更大。
  *
  *            代价: 每增加一片就要多占一个 MCU 引脚 (见 ADS1220_DRDY_PIN)。
  *
  *          === 引脚预算核算 (LQFP48 共 35 个可用 IO, 8 路配置) ===
  *            占用: PA5,PA6,PA7 (SPI1) + PA9,PA10 (USART1) + PA13,PA14 (SWD)
  *                  + PA0..PA3 (DRDY x4) + PB0..PB3 (CS x4)          = 16
  *            剩余: PA4,PA8,PA11,PA12,PA15,PB4..PB15,
  *                  PC13,PC14,PC15 等约 19 个 IO, 扩到 8 片(16 路)也够用。
  *
  *          === 贴装策略 (板子绝不需要重画) ===
  *            板上预留 4 个 ADS1220 焊盘, 先只贴 #0/#1 -> 固件 ADS1220_CHIP_COUNT=2;
  *            四路验证通过后补焊 #2/#3  -> 固件改成 4, 重新编译下载即可。
  *            没贴的芯片: CS 保持高(释放), DRDY 被内部上拉钳高, 固件不扫描它、也不等它。
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
 * 1. 系统规模  ★★★ 换板子 / 增减芯片只改这一个宏 ★★★
 *============================================================================*/
/** 实际贴装的 ADS1220 片数。
 *
 *    2 -> 4 路   (先贴 2 片, 验证四路能不能跑通)
 *    4 -> 8 路   (目标配置, 默认)
 *    8 -> 16 路  (需要扩展 DRDY 引脚, 见第 4 节)
 *
 *  这个宏同时决定:
 *    - 片选用了哪几个脚 (PB0 .. PB<CHIP_COUNT-1>)
 *    - 固件上电回读校验 / 失调校准 / 每轮采集 扫描哪几片
 *    - 状态帧里上报的"本板片数"(上位机据此只显示真实存在的通道)
 *  协议帧格式不变: 始终 32 个 float 槽位, 没接的槽位填 NaN。 */
#define ADS1220_CHIP_COUNT        4u

/** 每片 ADS1220 的差分通道数 (AIN0/AIN1 与 AIN2/AIN3) */
#define ADS1220_CH_PER_CHIP       2u
/** 本板实际测温通道数 = 片数 x 2 (8 路配置时为 8) */
#define TC_CHANNEL_COUNT          (ADS1220_CHIP_COUNT * ADS1220_CH_PER_CHIP)

/** 协议/缓冲区支持的最大片数 (片选位图用 16bit, 不要超过 16) */
#define ADS1220_MAX_CHIP          16u

#if (ADS1220_CHIP_COUNT < 1u) || (ADS1220_CHIP_COUNT > ADS1220_MAX_CHIP)
  #error "ADS1220_CHIP_COUNT 必须在 1..16 之间"
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
 * 3. 片选 CS0..CS(n-1) -> PB0..PB(n-1)
 *    片选直接映射成 GPIOB 的位掩码, 一次写 BSRR 即可, 不需要查表。
 *
 *    ★ JTAG 冲突提醒: STM32 复位后默认 SWJ 全使能, PB3(JTDO)/PB4(NJTRST)/
 *      PA15(JTDI)/PA13/PA14 被 JTAG+SWD 占用。本板 4 片会用到 PB3, 所以
 *      spi_driver.c 里统一执行了 __HAL_AFIO_REMAP_SWJ_NOJTAG() (关 JTAG 留 SWD),
 *      PB3/PB4 才能当普通推挽输出。
 *      (16 片老版本用 PB0..PB15 却没关 JTAG -> CS3/CS4 实际是无效的, 见
 *       docs/HARDWARE_8CH.md 的说明)
 *============================================================================*/
#define ADS1220_CS_PORT           GPIOB
/** 第 chip 片的片选掩码 (chip = 0..CHIP_COUNT-1 => PB0..PB3) */
#define ADS1220_CS_PIN(chip)      ((uint16_t)(1u << (chip)))
/** 本板实际用到的全部片选掩码 (只覆盖已配置的片数, 不会去动 PB4..PB15) */
#define ADS1220_CS_MASK_ALL       ((uint16_t)((1u << ADS1220_CHIP_COUNT) - 1u))

/*==============================================================================
 * 4. DRDY: 每片一根 -> PA0..PA3  (输入 + 内部上拉, 软件轮询, 不用逻辑门)
 *    低有效: DRDY = 0 表示该片有未读走的新数据。
 *    没贴的芯片: 该引脚被 MCU 内部上拉钳高, 读回来恒为 1 = "永远没数据";
 *    同时固件只扫描/只等待 ADS1220_CHIP_COUNT 片里校验通过的片, 所以既不会
 *    误判也不会拖慢采集。
 *============================================================================*/
#define ADS1220_DRDY_PORT         GPIOA
/** 第 chip 片的 DRDY 掩码 (chip = 0..3 => PA0..PA3) */
#define ADS1220_DRDY_PIN(chip)    ((uint16_t)(1u << (chip)))
/** 本板实际用到的 DRDY 掩码 */
#define ADS1220_DRDY_MASK_ALL     ((uint16_t)((1u << ADS1220_CHIP_COUNT) - 1u))

/** DRDY 目前定义到 PA0..PA3, 即最多 4 片(8 路)。
 *  想在一片板上做到 16 路(8 片), 把本宏和 spi_driver.c 的初始化掩码一起扩展:
 *  可用的空脚有 PA4, PA8, PA11, PA12 (都在 GPIOA, 一次 HAL_GPIO_Init 就能配完),
 *  再不够还可以用 PB4..PB15 / PC13..PC15。 */
#define ADS1220_DRDY_MAX_CHIP     4u

#if (ADS1220_CHIP_COUNT > ADS1220_DRDY_MAX_CHIP)
  #error "DRDY 只定义到 PA0..PA3(4 片)。做 8 片(16 路)请先扩展 ADS1220_DRDY_PIN() 和 spi_driver.c"
#endif

/*==============================================================================
 * 5. USART1
 *============================================================================*/
#define UART1_GPIO_PORT           GPIOA
#define UART1_TX_PIN              GPIO_PIN_9
#define UART1_RX_PIN              GPIO_PIN_10
#define UART1_IRQ_PRIO            2u       /* 只给 UART 用 (DRDY 已改成轮询, 无中断) */

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
 *        - 每片 DRDY 走线的延迟 (已无逻辑门, 比 16 片版本更短)
 *        - 轮询/中断响应延迟
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

/* (2) SPI 时钟也不能太低, 否则逐片批量读会挤掉转换窗口 */
#if (ADS1220_SPI_SCLK_HZ < 2000000u)
  #warning "SPI1 时钟低于 2MHz, 多片批量读会明显变慢, 建议 >= 4MHz"
#endif

/* (3) CS 建立/保持用的 NOP 延时必须覆盖 td(CSSC)/td(SCCS)/tw(CSH) 里最大的那个。
 *     写成 (NOP数 x 1000) / (MHz) 的形式, 避免中间结果超过 32 位。 */
#if (((SPI_DRIVER_SHORT_NOP_CNT * 1000u) / (BOARD_SYSCLK_HZ / 1000000u)) < ADS1220_T_CSSC_NS)
  #error "SPI_DRIVER_SHORT_NOP_CNT 个 NOP 不足以满足 td(CSSC), 请加大 NOP 个数"
#endif

/* (4) 片选/错误位图按 16 位设计 */
#if (ADS1220_MAX_CHIP != 16u)
  #error "片选位图/芯片掩码按 16 位设计, ADS1220_MAX_CHIP 必须是 16"
#endif

/* (5) ★ 8 路拓扑新增: DRDY 引脚必须够分 (每片一根) */
#if (ADS1220_DRDY_MAX_CHIP < ADS1220_CHIP_COUNT)
  #error "DRDY 引脚不够: 每片需要一根独立 DRDY, 请扩展 board_config.h 第 4 节"
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

/** ★ 上行数据模式 —— "温度由谁换算"。
 *
 *    0 = 只发温度帧 (CMD=0x10): 固件用 NIST 多项式算出 °C, 上位机直接画
 *        (老行为; 分度号写死在固件里)
 *    1 = 只发原始帧 (CMD=0x12): 固件只发"实测热电势 µV + 冷端温度 °C",
 *        温度由上位机查分度表算  ← **默认**
 *    2 = 两帧都发 (双倍带宽, 只在固件/上位机对比验证时用)
 *
 *  === 为什么默认 1 (让上位机算) ===
 *    固件用 float32 + NIST 逆多项式, 精度其实够(<0.03°C, 见 TC_SelfTest),
 *    但"用哪个分度号 / 哪张表 / 单支标定修正"全都写死在固件里:
 *      - 换 J 型热电偶要改固件 + 重编译 + 重新下载;
 *      - 想用厂家给的粗表(10/100°C 步长) 或单支标定曲线做不到。
 *    改成上报 µV 之后, 上位机 (pc_ui/tc_table.py) 可以:
 *      - 换分度号 K/J/T/E/N 而完全不碰固件;
 *      - 用 float64 + 单调三次插值(pchip), 比固件 float32 多项式更准;
 *      - CSV 里同时存 µV 和 °C, 换表后能**离线重算**历史数据;
 *      - 用单支标定数据做逐点修正。
 *
 *  ★ 传输是无损的: ADS1220 增益是 2 的幂, 且 2048000/8388608 = 0.244140625
 *    是二进制精确值, float32 尾数 24 位 == 码值 24 位, 所以
 *    "int32 码值 -> float32 µV" 这一步没有任何精度损失, 上位机拿到的 µV
 *    和固件内部算出来的完全一致。
 *
 *  详见 docs/TC_TABLE_FORMAT.md 第 8 节。 */
#ifndef TC_UPLINK_MODE
  #define TC_UPLINK_MODE          1
#endif

/** 断线判定用的"热电势物理窗口"(µV)。
 *  ⚠️ 这个窗口是**分度号相关**的: 固件默认按"常见几种型号的并集"取值
 *     (E 型 -270..1000°C 约 [-9835, +76373] µV 是最宽的), 所以取了 ±80mV。
 *      想收紧成 K 型专用: TC_OPEN_EMF_LO_UV=-12000, TC_OPEN_EMF_HI_UV=60000
 *      想完全关掉这一条(只靠"码值被拉死到满量程"判开路): TC_OPEN_EMF_CHECK=0
 *  ★ 关掉它是安全的: 断线时 BCS 的 10µA 会把输入拉到 ±满量程码值, 那一条
 *     独立判定与分度号无关, 仍然有效。 */
#ifndef TC_OPEN_EMF_CHECK
  #define TC_OPEN_EMF_CHECK       1
#endif
#ifndef TC_OPEN_EMF_LO_UV
  #define TC_OPEN_EMF_LO_UV       (-12000.0f)
#endif
#ifndef TC_OPEN_EMF_HI_UV
  #define TC_OPEN_EMF_HI_UV       (80000.0f)
#endif

/** BCS (10uA 烧断电流源) 使用方式 —— 断线检测:
 *    1 = BCS 常开 (严格按需求 "CONFIG1: BCS=断线检测使能"), 默认
 *    0 = BCS 关闭; 此时仍靠"采样值超量程"判断断线, 但要求模拟前端
 *        带有 TI 参考设计里的 1M~50M 偏置电阻 RB1/RB2 把开路输入拉满量程
 *
 *  ⚠️ 热电偶回路电阻 R 上会流过 10uA => 附加失调 = 10uA x R。
 *     2m 的 K 型偶丝回路电阻约几欧姆 => 几十 nV, 可忽略;
 *     若使用很长的补偿导线(上百欧姆), 建议改成 0 并在需要时临时开启。
 *
 *  ★ 8 路板要特别注意: 如果输入端串了 RC 滤波电阻(比如常见 1kΩ),
 *    10uA x 1kΩ = 10mV 的固定失调, 折合 K 型约 240°C 的误差!
 *    这时必须三选一: (a) 把本宏改 0; (b) 串阻取很小(<=10Ω);
 *    (c) 只在测量窗口之外开 BCS。详见 docs/HARDWARE_8CH.md。 */
#ifndef TC_BCS_ALWAYS_ON
  #define TC_BCS_ALWAYS_ON        1
#endif

/** 上位机多久没发命令时, 主动推送一帧状态 (0 = 不自动推送, 只应答) */
#define TC_STATUS_PUSH_PERIOD_MS  5000u

/** 上电初始化时对每片做一次内部短路失调校准 (手册 9.1.5 推荐, 用于替代
 *  ADS1220 并不存在的 "SELF CAL 0x04" 指令)。平均次数见 .c */
#ifndef TC_OFFSET_CAL_ENABLE
  #define TC_OFFSET_CAL_ENABLE    1
#endif
#define TC_OFFSET_CAL_SAMPLES     4u

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CONFIG_H */
