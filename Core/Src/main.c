/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : N x ADS1220 / 2N 路热电偶测温系统 主程序
  *                  (N = ADS1220_CHIP_COUNT, 默认 4 片 = 8 路; 改一个宏即可变 2/4/8 片)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board_config.h"
#include "spi_driver.h"
#include "ads1220.h"
#include "thermocouple.h"
#include "uart_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
  * @brief 采集轮询状态机。
  *
  *  === 为什么仍然"广播 START/SYNC + 等一个转换周期" ===
  *  本板每片 ADS1220 各有一根 DRDY (PA0..PA3), 直接进 MCU, 不用外部逻辑门。
  *  SPI 总线是**共享**的, 同一时刻只能跟一片说话, 所以还是采用"先让所有片
  *  在时间上对齐, 再逐片读"的策略:
  *
  *    1) 每次切换 MUX / TS 之后, 对每片 **逐片** 发 WREG + START/SYNC。
  *       START/SYNC 会复位数字滤波器并重新开始转换, 于是所有片在时间上重新对齐
  *       (彼此只差几十微秒的 SPI 下发时间)。
  *    2) 等待期间同时满足两个条件才继续:
  *         a. 已经过了 >= 0.9 个转换周期 (保证读到的是新通道的数据, 不是上一次的);
  *         b. 所有"被等待的片"的 DRDY 都变低 —— 由 SPI_Driver_DrdyTakeFlag()
  *            把 N 根 DRDY 软件合成一个"全部就绪" (等价于原来的合并信号,
  *            但多了一个能力: 能分辨出到底是哪一片没就绪)。
  *       超时 (转换周期 + 25ms) 则记一次故障、把没就绪的片从等待集合里摘掉
  *       (同时置 chip_err 位), 流程继续, 保证永不卡死。
  *    3) 读完数据后, 该片的 DRDY 会在下一个 SCLK 上升沿自动变高,
  *       "全部就绪"随之解除, 等待下一轮转换完成, 不需要任何复位操作。
  *
  *  ★ "被等待的片" = 上电回读校验通过的片 (见 App_StartWait / s_chip_ok_mask)。
  *    这样"板上留了 4 个位置但只焊了 2 片"时, 没焊的片根本不在等待集合里,
  *    既不会每轮超时, 也不会把采集拖慢 —— 这是分步贴装能直接跑通的关键。
  *
  *  一个完整轮次 = A 通道 -> 切 MUX -> B 通道 -> 切 TS -> 冷端 -> 恢复 + 上报,
 *  每个阶段各占 s_avg_n 个转换周期 (20SPS 下一次 50ms), 因此 20SPS + 不平均
 *  时约 150ms/轮; 若把平均次数设为 N, 则每轮约 3 x N x 50ms。
 */
typedef enum
{
  APP_ST_STOPPED = 0,   /**< 已停止 (所有片处于 POWERDOWN) */
  APP_ST_WAIT_A,        /**< 等 A 通道 (AIN0/AIN1) 转换完成 */
  APP_ST_WAIT_B,        /**< 等 B 通道 (AIN2/AIN3) 转换完成 */
  APP_ST_WAIT_T         /**< 等内部温度传感器 (冷端) 转换完成 */
} App_State_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** 采集运行中, 超过这个时间一次"全部就绪"都没出现 -> 置"DRDY 静默"故障位。
 *  正常运行时每个转换周期至少来一次 (20SPS x 3 阶段 ≈ 20 次/秒),
 *  1 秒都没有说明总线或器件出问题了。 */
#define TC_DRDY_SILENT_MS      1000u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*---- 配置与采集数据 ----*/
static ADS1220_Config_t s_cfg;                                  /**< 各片共用配置镜像 */

static int32_t  s_codeA[ADS1220_CHIP_COUNT];                    /**< A 通道 24bit 码值 */
static int32_t  s_codeB[ADS1220_CHIP_COUNT];                    /**< B 通道 24bit 码值 */
static int32_t  s_codeT[ADS1220_CHIP_COUNT];                    /**< 内部温度传感器码值 */
static int32_t  s_offset[ADS1220_CHIP_COUNT];                   /**< 上电失调校准结果(码值) */
static uint8_t  s_errA[ADS1220_CHIP_COUNT];                     /**< 0 = SPI 成功 */
static uint8_t  s_errB[ADS1220_CHIP_COUNT];
static uint8_t  s_errT[ADS1220_CHIP_COUNT];

static int32_t  s_accA[ADS1220_CHIP_COUNT];                     /**< A 通道累加器 (软件平均) */
static int32_t  s_accB[ADS1220_CHIP_COUNT];                     /**< B 通道累加器 */
static int32_t  s_accT[ADS1220_CHIP_COUNT];                     /**< 冷端温度累加器 */

#if (TC_UPLINK_MODE != 1)
/** 本板实际通道的最终温度 (长度 = 片数 x 2)。
 *  ★ 只在"固件算温度"模式 (0/2) 下需要 —— 默认模式 1 由上位机算, 这里不占空间。 */
static float    s_temps[TC_CHANNEL_COUNT];

/** 上报用的 32 槽温度缓冲区 (CMD=0x10)。协议固定 32 槽, 本板没接的槽位填 NaN。
 *  (上位机靠状态帧里的片数知道哪几路是真实存在的) */
static float    s_report[UART_TEMP_SLOT_COUNT];
#endif

#if (TC_UPLINK_MODE != 0)
/** ★ 原始上报缓冲 (CMD=0x12): 32 槽热电势 µV + 16 槽冷端温度 °C。
 *
 *  这是"温度由上位机换算"的数据源: 固件只把 ADS1220 的 24bit 码值变成
 *  µV 和冷端 °C, 其余(分度表/插值/单支标定)全部交给上位机 pc_ui/tc_table.py。
 *  码值 -> µV 这一步是无损的 (见 board_config.h 的说明)。 */
static float    s_report_uv[UART_TEMP_SLOT_COUNT];
static float    s_report_cj[UART_CJ_SLOT_COUNT];
#endif

/*---- 状态 ----*/
static App_State_t s_state;
static uint8_t   s_run;                                         /**< 1 = 采集运行中 */
static uint8_t   s_single_mode;                                 /**< 1 = 本次是"读单次"请求 */
static uint8_t   s_single_ch;                                   /**< 单次请求的通道号 */
static uint8_t   s_hold_a;                                      /**< MUX 保持计数 (CH_A) */
static uint8_t   s_hold_b;                                      /**< MUX 保持计数 (CH_B) */
static uint8_t   s_hold_t;                                      /**< MUX 保持计数 (冷端) */
static uint8_t   s_avg_n;                                       /**< ★软件平均次数 (1..TC_AVG_MAX_CONVERSIONS), CMD=0x04 可改 */

static uint16_t  s_chip_ok_mask;                                /**< 上电回读校验通过的片 */
static uint16_t  s_chip_err_mask;                               /**< 本轮 SPI 出错的片 */
static uint16_t  s_drdy_missing_mask;                           /**< 一直不就绪的片(没焊/坏了) */
static uint16_t  s_wait_mask;                                   /**< 本阶段实际要等的片 */
static uint32_t  s_round_count;                                 /**< 已完成轮次 */
static uint16_t  s_open_cnt;                                    /**< 断线累计次数 */
static uint16_t  s_phase_timeout_cnt;                           /**< 等待 DRDY 超时次数 */
static uint8_t   s_flags;                                       /**< UART_ST_FLAG_xxx */

/*---- 等待转换完成的窗口 ----*/
static uint32_t  s_wait_start;                                  /**< 本轮等待起始 tick */
static uint32_t  s_wait_min_ms;                                 /**< 最小等待 (0.9 x 转换周期) */
static uint32_t  s_wait_max_ms;                                 /**< 超时 (转换周期 + 25ms) */

/*---- 心跳 / 周期上报 ----*/
static uint32_t  s_last_drdy_cnt;
static uint32_t  s_last_drdy_tick;
static uint32_t  s_status_push_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void    App_StartWait(void);
static uint8_t App_WaitDone(void);
static void    App_ReadAll(int32_t *dst, uint8_t *err);
static void    App_ClearAvg(void);
static void    App_SetAverage(uint8_t n);
static void    App_ComputeTemps(void);
static void    App_ReportTemps(void);
static void    App_StatusFill(UART_Status_t *st);
static void    App_ReportStatus(void);
static void    App_Start(void);
static void    App_Stop(void);
static void    App_SetRate(uint8_t dr_bits, uint8_t reject);
static void    App_RequestSingle(uint8_t channel);
static void    App_HandleEvent(const UART_EventMsg_t *msg);
static void    App_Housekeeping(void);
static void    App_Run(void);
static void    App_HwInit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*==============================================================================
 * 小工具
 *============================================================================*/
/* (保留位置: 目前没有通用小工具, 全部逻辑都在下面的模块里) */

/*==============================================================================
 * 等待窗口
 *============================================================================*/
/** 开始一个新的"等转换完成"窗口。
 *  必须在广播 START/SYNC 之后调用, 这样计时基准和器件的转换起点一致。 */
static void App_StartWait(void)
{
  const ADS1220_RateInfo_t *ri = ADS1220_RateInfo(s_cfg.dr);
  uint16_t configured = (uint16_t)((1u << ADS1220_CHIP_COUNT) - 1u);

  /* 最小等待 = 0.9 x 实际转换时间, 宁可晚一点也不要把上一通道的旧数据当新的 */
  s_wait_min_ms = (ri->conv_us * ADS1220_WAIT_MIN_NUM) /
                  (1000u * ADS1220_WAIT_MIN_DEN);
  if (s_wait_min_ms == 0u)
  {
    s_wait_min_ms = 1u;                 /* 高速率下至少等 1ms */
  }

  /* 超时 = 转换时间 + 25ms 裕量 */
  s_wait_max_ms = (ri->conv_us / 1000u) + ADS1220_WAIT_MARGIN_MS;

  /* ★ 本阶段要等的片 = 上电回读校验通过 且 没有"一直不就绪"记录的片。
   *   这样"板上留了 4 个位置但只焊了 2 片"时, 没焊的片不在等待集合里,
   *   既不会每轮白等 25ms, 也不会把整轮采集拖慢。 */
  s_wait_mask = (uint16_t)(s_chip_ok_mask &
                           (uint16_t)(~s_drdy_missing_mask) &
                           configured);
  if (s_wait_mask == 0u)
  {
    /* 一片都没通过校验 (例如校验还没跑过) -> 退化成等全部, 让它超时并如实报错 */
    s_wait_mask = configured;
  }
  SPI_Driver_DrdySetExpectedMask(s_wait_mask);

  SPI_Driver_DrdyClearFlag();           /* 清掉 START 之前的残留"全就绪"锁存 */
  s_wait_start = HAL_GetTick();
}

/**
  * @brief 查询等待是否结束
  * @retval 0 = 继续等; 1 = 所有被等的片都就绪; 2 = 超时
  */
static uint8_t App_WaitDone(void)
{
  uint32_t elapsed = HAL_GetTick() - s_wait_start;

  if (elapsed < s_wait_min_ms)
  {
    return 0u;                          /* 还没到最小建立时间, 一律忽略 */
  }

  if (SPI_Driver_DrdyTakeFlag() != 0u)
  {
    return 1u;                          /* 所有被等的片都有新数据了 */
  }

  if (elapsed >= s_wait_max_ms)
  {
    /* ★ 超时诊断: 是哪几片没就绪? 把它们记进 chip_err 并从等待集合里摘掉,
     *   避免后面每一轮都在它们身上白等 25ms。摘掉之后, 上位机右侧的
     *   "芯片 ERR 位图"就能直接指出问题片号 (没焊 / 虚焊 / 供电异常)。 */
    uint16_t ready   = SPI_Driver_DrdyReadyMask();
    uint16_t missing = (uint16_t)(s_wait_mask & (uint16_t)(~ready));

    if (missing != 0u)
    {
      s_drdy_missing_mask |= missing;
      s_chip_err_mask     |= missing;
      s_flags             |= UART_ST_FLAG_DRDY_PARTIAL;
      s_wait_mask          = (uint16_t)(s_wait_mask & (uint16_t)(~missing));
      SPI_Driver_DrdySetExpectedMask(s_wait_mask);   /* 传 0 时内部会保持原值 */
    }
    return 2u;                          /* 超时: 器件没响应 */
  }

  return 0u;
}

/*==============================================================================
 * 数据读取
 *============================================================================*/
/** 逐片读 24bit。err[] 记录每片的 SPI 结果 (0 = 成功)。 */
static void App_ReadAll(int32_t *dst, uint8_t *err)
{
  uint8_t chip;

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    err[chip] = (uint8_t)ADS1220_ReadData(chip, &dst[chip], NULL);
  }
}

/** 清空三个阶段的软件平均累加器 (启动 / 单次读取 / 改平均次数时调用,
 *  避免把上一轮的半截累加结果带进新一轮)。 */
static void App_ClearAvg(void)
{
  uint8_t chip;

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    s_accA[chip] = 0;
    s_accB[chip] = 0;
    s_accT[chip] = 0;
  }
}

/** ★ 设置软件平均次数 (CMD=0x04)。
 *
 *  夹到 1..TC_AVG_MAX_CONVERSIONS (0 会被当成 1 = 不平均)。
 *  运行中修改是安全的: 立刻清累加器和保持计数, 下一阶段的读取就用新次数,
 *  不会把"新旧次数"的样本混在一起平均。 */
static void App_SetAverage(uint8_t n)
{
  if (n < 1u)
  {
    n = 1u;
  }
  if (n > (uint8_t)TC_AVG_MAX_CONVERSIONS)
  {
    n = (uint8_t)TC_AVG_MAX_CONVERSIONS;
  }

  s_avg_n  = n;
  s_hold_a = 0u;
  s_hold_b = 0u;
  s_hold_t = 0u;
  App_ClearAvg();
}

/*==============================================================================
 * 温度换算 / 原始量打包 (冷端补偿在这里完成)
 *============================================================================*/
/** 把本轮的 A/B 通道与冷端整理成上报数据, 并做断线判定。
 *
 *  ★ 这里是"温度由谁换算"的分岔点 (TC_UPLINK_MODE):
 *      0 -> 固件用 NIST 多项式算 °C, 填 s_temps      (老行为)
 *      1 -> 只填 s_report_uv / s_report_cj, 上位机查分度表算 °C  (默认)
 *      2 -> 两边都填, 用于固件/上位机对比验证
 *
 *  三种模式下 s_chip_err_mask / s_open_cnt / s_drdy_missing_mask 的维护完全一致,
 *  所以状态帧和断线计数在任何模式下都照常工作。
 *
 *  注意断线判定用的是 TC_IsOpen() 而不是 TC_Compute(): 前者只做
 *  "冷端量程 + 满量程码值 + 物理窗口"三个判断, 不含任何多项式运算,
 *  因此模式 1 下固件不需要算温度也能准确识别断线。 */
static void App_ComputeTemps(void)
{
  uint8_t chip;
  uint8_t ia;
  uint8_t ib;
  float   cj_c;
  float   emf_uv;

  s_chip_err_mask = 0u;         /* 本轮错误位图从这里开始重新累积 */

#if (TC_UPLINK_MODE != 0)
  /* 原始上报缓冲先全置 NaN: 没接的通道 / 没贴的片保持 NaN, 上位机据此显示"无效" */
  {
    uint16_t k;
    for (k = 0u; k < UART_TEMP_SLOT_COUNT; k++)
    {
      s_report_uv[k] = UART_NaN();
    }
    for (k = 0u; k < UART_CJ_SLOT_COUNT; k++)
    {
      s_report_cj[k] = UART_NaN();
    }
  }
#endif

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    ia = TC_CH_INDEX_A(chip);
    ib = TC_CH_INDEX_B(chip);

    /* 默认本片两个通道都无效 */
#if (TC_UPLINK_MODE != 1)
    s_temps[ia] = UART_NaN();
    s_temps[ib] = UART_NaN();
#endif

    /* 跳过两类片:
     *   a) 上电回读校验没通过 (没焊/虚焊) —— 避免把"空总线读回的全 0"
     *      当成 0uV, 从而报出一个看起来正常的冷端温度;
     *   b) 采集过程中一直不就绪的片 (由 App_WaitDone 的超时诊断标出来)。 */
    if (((s_chip_ok_mask & (uint16_t)(1u << chip)) == 0u) ||
        ((s_drdy_missing_mask & (uint16_t)(1u << chip)) != 0u))
    {
      s_chip_err_mask |= (uint16_t)(1u << chip);
      continue;
    }

    /* ---- 冷端温度 (ADS1220 内部温度传感器, 每轮采一次) ---- */
    if (s_errT[chip] != 0u)
    {
      s_chip_err_mask |= (uint16_t)(1u << chip);
      continue;                       /* 冷端无效 -> 该片两路都没法算 */
    }
    cj_c = ADS1220_TempCodeToCelsius(s_codeT[chip]);

#if (TC_UPLINK_MODE != 0)
    /* ★ 冷端原始量也要发给上位机: 冷端补偿 V+E(T_CJ) 必须由同一个算法完成,
     *   只给 µV 不给冷端温度, 上位机是算不出热端温度的。 */
    s_report_cj[chip] = cj_c;
#endif

    /* ---- A 通道: AIN0/AIN1 ---- */
    if (s_errA[chip] == 0u)
    {
      emf_uv = ADS1220_CodeToMicroVolt(s_codeA[chip] - s_offset[chip], s_cfg.gain);
      if (TC_IsOpen(emf_uv, cj_c, s_codeA[chip]) != 0u)
      {
        s_open_cnt++;                 /* 断线/超量程: 该通道保持 NaN */
      }
      else
      {
#if (TC_UPLINK_MODE != 0)
        s_report_uv[ia] = emf_uv;     /* 上位机算温度用的原始量 */
#endif
#if (TC_UPLINK_MODE != 1)
        s_temps[ia] = TC_CompensateHotJunction(emf_uv, cj_c);
#endif
      }
    }
    else
    {
      s_chip_err_mask |= (uint16_t)(1u << chip);
    }

    /* ---- B 通道: AIN2/AIN3 ---- */
    if (s_errB[chip] == 0u)
    {
      emf_uv = ADS1220_CodeToMicroVolt(s_codeB[chip] - s_offset[chip], s_cfg.gain);
      if (TC_IsOpen(emf_uv, cj_c, s_codeB[chip]) != 0u)
      {
        s_open_cnt++;
      }
      else
      {
#if (TC_UPLINK_MODE != 0)
        s_report_uv[ib] = emf_uv;
#endif
#if (TC_UPLINK_MODE != 1)
        s_temps[ib] = TC_CompensateHotJunction(emf_uv, cj_c);
#endif
      }
    }
    else
    {
      s_chip_err_mask |= (uint16_t)(1u << chip);
    }
  }
}

/*==============================================================================
 * 上报
 *============================================================================*/
static void App_ReportTemps(void)
{
  uint16_t i;

  /*====================================================================
   * 1) 温度帧 (CMD=0x10): 固件已经算好的 °C
   *    只在 TC_UPLINK_MODE = 0 / 2 时发。
   *==================================================================*/
#if (TC_UPLINK_MODE != 1)
  /* 先把 32 个槽位全部填成 NaN。
   * ★ 协议固定 32 槽, 本板没接的通道就保持 NaN —— 上位机看到 NaN 就知道
   *   "这一路不存在/断线", 而且 4 路板 / 8 路板 / 16 路板共用同一套上位机。 */
  for (i = 0u; i < UART_TEMP_SLOT_COUNT; i++)
  {
    s_report[i] = UART_NaN();
  }

  if ((s_single_mode != 0u) && (s_single_ch != UART_CH_SINGLE_ALL))
  {
    /* 单通道请求: 32 槽里只放目标通道那一个值, 其余保持 NaN */
    if (s_single_ch < UART_TEMP_SLOT_COUNT)
    {
      s_report[s_single_ch] = (s_single_ch < TC_CHANNEL_COUNT)
                              ? s_temps[s_single_ch]
                              : UART_NaN();
    }
  }
  else
  {
    /* 正常上报: 把本板实际通道拷进前 TC_CHANNEL_COUNT 个槽位 */
    for (i = 0u; i < TC_CHANNEL_COUNT; i++)
    {
      s_report[i] = s_temps[i];
    }
  }

  (void)UART_Protocol_SendTempFrame(s_report);
#endif

  /*====================================================================
   * 2) ★原始帧 (CMD=0x12): 热电势 µV + 冷端 °C, 温度由上位机查分度表算。
   *    只在 TC_UPLINK_MODE = 1 / 2 时发。
   *
   *    单次读取时同样只保留目标通道 (与温度帧同一约定), 其余置 NaN。
   *    这里就地改 s_report_uv 是安全的: 它每轮都会被 App_ComputeTemps()
   *    重新填满, 不会把上一轮的掩码带到下一轮。
   *==================================================================*/
#if (TC_UPLINK_MODE != 0)
  if ((s_single_mode != 0u) && (s_single_ch != UART_CH_SINGLE_ALL))
  {
    for (i = 0u; i < UART_TEMP_SLOT_COUNT; i++)
    {
      if (i != (uint16_t)s_single_ch)
      {
        s_report_uv[i] = UART_NaN();
      }
    }
  }

  (void)UART_Protocol_SendRawFrame(s_report_uv, s_report_cj);
#endif
}

static void App_StatusFill(UART_Status_t *st)
{
  st->run_state      = s_run;
  st->dr_bits        = s_cfg.dr;
  st->reject         = s_cfg.reject;
  /* ★ 上报本板实际片数: 上位机据此只显示真实存在的通道
   *   (没接的槽位虽然也发了, 但值恒为 NaN, 不必显示成"断线"吓人) */
  st->chip_count     = (uint8_t)ADS1220_CHIP_COUNT;
  st->chip_ok_mask   = s_chip_ok_mask;
  st->chip_err_mask  = s_chip_err_mask;
  st->round_count    = s_round_count;
  st->uptime_ms      = HAL_GetTick();
  st->drdy_irq_count = (uint16_t)(SPI_Driver_DrdyReadyCount() & 0xFFFFu);
  st->spi_err_count  = (uint16_t)(SPI_Driver_ErrorCount() & 0xFFFFu);
  st->open_tc_count  = s_open_cnt;
  st->phase_timeout_count = s_phase_timeout_cnt;
  st->avg_n          = s_avg_n;   /* ★ 当前生效的软件平均次数 */

  st->flags = s_flags;
#if (TC_UPLINK_MODE != 0)
  /* ★ 告诉上位机"我发的是原始帧(0x12), 温度请你算":
   *   上位机据此选用查表换算而不是直接画 0x10 的温度。 */
  st->flags |= UART_ST_FLAG_RAW_UPLINK;
#endif
  if (UART_Protocol_GetTxOverflowCount() != 0u)
  {
    st->flags |= UART_ST_FLAG_TX_OVERFLOW;
  }
  if (UART_Protocol_GetCrcErrCount() != 0u)
  {
    st->flags |= UART_ST_FLAG_CRC_ERR;
  }
}

static void App_ReportStatus(void)
{
  UART_Status_t st;

  App_StatusFill(&st);
  (void)UART_Protocol_SendStatusFrame(&st);
}

/*==============================================================================
 * 启动 / 停止 / 改速率 / 单次
 *============================================================================*/
/** 启动采集: 重新下发配置 -> 回读校验 -> 广播 START/SYNC -> 进入轮询。
 *  重新回读校验是为了支持"运行中热插拔/换线后发一次启动命令自愈"。 */
static void App_Start(void)
{
  uint8_t regs[ADS1220_REG_COUNT];

  ADS1220_BuildRegs(&s_cfg, regs);

  (void)ADS1220_SetConfigAll(&s_cfg);
  (void)ADS1220_VerifyAll(regs, &s_chip_ok_mask);
  (void)ADS1220_StartAll();

  s_run         = 1u;
  s_hold_a      = 0u;
  s_hold_b      = 0u;
  s_hold_t      = 0u;
  App_ClearAvg();             /* 丢弃上一轮残留的半截平均累加 */
  s_chip_err_mask = 0u;
  s_drdy_missing_mask = 0u;   /* 每次启动都重新信任一次: 重新校验过就再等它 */
  s_state       = APP_ST_WAIT_A;

  App_StartWait();
}

/** 停止采集: 所有片进入 POWERDOWN (寄存器值保持, 下次 START/SYNC 即可恢复)。 */
static void App_Stop(void)
{
  s_run         = 0u;
  s_single_mode = 0u;
  (void)ADS1220_PowerDownAll();
  s_state = APP_ST_STOPPED;
}

/** 修改采样率。
 *  手册 Table 8-13: 50/60Hz 抑制滤波器只能在"正常模式 + 20SPS"下打开,
 *  其它数据率必须写 00b —— 这里强制保证, 避免上位机下发非法组合。 */
static void App_SetRate(uint8_t dr_bits, uint8_t reject)
{
  const ADS1220_RateInfo_t *ri = ADS1220_RateInfo(dr_bits);

  s_cfg.dr = ri->dr_bits;

  if ((ri->reject_ok != 0u) && (s_cfg.mode == ADS1220_MODE_NORMAL))
  {
    s_cfg.reject = reject;
  }
  else
  {
    s_cfg.reject = ADS1220_REJECT_NONE;
  }

  if (s_run != 0u)
  {
    App_Start();                     /* 重新下发 + 重新同步 */
  }
  else
  {
    (void)ADS1220_SetConfigAll(&s_cfg);
  }
}

/** 单次读取: 无论当前在哪个阶段, 都把所有片拉回 CH_A + TS=0 重新跑一整轮。 */
static void App_RequestSingle(uint8_t channel)
{
  s_single_ch   = (channel < TC_CHANNEL_COUNT) ? channel : UART_CH_SINGLE_ALL;
  s_single_mode = 1u;

  if (s_run == 0u)
  {
    /* 之前是 POWERDOWN: 寄存器值还保留着, 但保险起见重发一遍配置 */
    (void)ADS1220_SetConfigAll(&s_cfg);
  }

  (void)ADS1220_SetMuxAll(ADS1220_MUX_AIN0_AIN1);
  (void)ADS1220_SetTempSensorAll(0u);
  (void)ADS1220_StartAll();

  s_hold_a        = 0u;
  s_hold_b        = 0u;
  s_hold_t        = 0u;
  App_ClearAvg();                 /* 单次读取也从干净的累加器开始 */
  s_chip_err_mask = 0u;
  s_state         = APP_ST_WAIT_A;

  App_StartWait();
}

/*==============================================================================
 * 命令分发
 *============================================================================*/
static void App_HandleEvent(const UART_EventMsg_t *msg)
{
  switch (msg->ev)
  {
    case UART_EV_SET_RATE:
      App_SetRate(msg->dr_bits, msg->reject);
      App_ReportStatus();
      break;

    case UART_EV_START:
      App_Start();
      App_ReportStatus();
      break;

    case UART_EV_STOP:
      App_Stop();
      App_ReportStatus();
      break;

    case UART_EV_SINGLE:
      App_RequestSingle(msg->channel);
      break;

    case UART_EV_SET_AVG:
      /* ★ 运行中修改软件平均次数, 改完回一帧状态让上位机确认生效值 */
      App_SetAverage(msg->avg_n);
      App_ReportStatus();
      break;

    default:
      break;
  }
}

/*==============================================================================
 * 心跳 / 周期性状态上报
 *============================================================================*/
static void App_Housekeeping(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t cnt = SPI_Driver_DrdyReadyCount();

  if (cnt != s_last_drdy_cnt)
  {
    s_last_drdy_cnt  = cnt;
    s_last_drdy_tick = now;
    s_flags = (uint8_t)(s_flags & (uint8_t)(~UART_ST_FLAG_DRDY_SILENT));
  }
  else if ((s_run != 0u) && ((now - s_last_drdy_tick) > TC_DRDY_SILENT_MS))
  {
    s_flags |= UART_ST_FLAG_DRDY_SILENT;
  }

#if (TC_STATUS_PUSH_PERIOD_MS != 0u)
  if ((now - s_status_push_tick) >= TC_STATUS_PUSH_PERIOD_MS)
  {
    s_status_push_tick = now;
    App_ReportStatus();
  }
#endif
}

/*==============================================================================
 * 主任务 (主循环里反复调用, 不含任何阻塞等待)
 *============================================================================*/
static void App_Run(void)
{
  UART_EventMsg_t msg;

  /* ---- 1) 先把上位机命令消化掉 (停止状态下也要能收命令) ---- */
  while (UART_Protocol_GetEvent(&msg) != 0u)
  {
    App_HandleEvent(&msg);
  }

  /* ---- 2) 串口: 半截帧超时复位 / 接收中断自愈 / 发送续传 ---- */
  UART_Protocol_Poll();

  /* ---- 3) 采集状态机 ---- */
  if ((s_run == 0u) && (s_single_mode == 0u))
  {
    App_Housekeeping();
    return;
  }

  switch (s_state)
  {
    /*--------------------------------------------------------------------
     * 阶段 1: A 通道 (MUX = AIN0/AIN1)
     *------------------------------------------------------------------*/
    case APP_ST_WAIT_A:
    {
      uint8_t w = App_WaitDone();
      uint8_t chip;

      if (w == 0u)
      {
        break;                                  /* 还没好, 下一圈再看 */
      }
      if (w == 2u)
      {
        s_phase_timeout_cnt++;                  /* 超时: 记故障, 但流程继续 */
      }

      App_ReadAll(s_codeA, s_errA);

      /* ★ 软件平均: 把本次读数累加进 A 通道累加器。连续读 s_avg_n 次后取
       *   算术平均再切 MUX, 把随机噪声(白噪声)压到 1/sqrt(N); N=1 即不平均。 */
      for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
      {
        s_accA[chip] += s_codeA[chip];
      }
      s_hold_a++;

      if (s_hold_a < s_avg_n)
      {
        App_StartWait();                        /* 继续读同一通道, 多累加几个样本 */
        break;
      }

      /* 达到平均次数: 求平均 (四舍五入) 并清累加器, 之后才切 MUX */
      for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
      {
        s_codeA[chip] = (s_accA[chip] + ((int32_t)s_hold_a / 2)) / (int32_t)s_hold_a;
        s_accA[chip] = 0;
      }
      s_hold_a = 0u;

      /* 切到 B 通道, 并用 START/SYNC 让所有片重新同步 */
      (void)ADS1220_SetMuxAll(ADS1220_MUX_AIN2_AIN3);
      (void)ADS1220_StartAll();
      App_StartWait();
      s_state = APP_ST_WAIT_B;
      break;
    }

    /*--------------------------------------------------------------------
     * 阶段 2: B 通道 (MUX = AIN2/AIN3)
     *------------------------------------------------------------------*/
    case APP_ST_WAIT_B:
    {
      uint8_t w = App_WaitDone();
      uint8_t chip;

      if (w == 0u)
      {
        break;
      }
      if (w == 2u)
      {
        s_phase_timeout_cnt++;
      }

      App_ReadAll(s_codeB, s_errB);

      /* ★ 软件平均 (与 A 通道同一逻辑) */
      for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
      {
        s_accB[chip] += s_codeB[chip];
      }
      s_hold_b++;
      if (s_hold_b < s_avg_n)
      {
        App_StartWait();
        break;
      }
      for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
      {
        s_codeB[chip] = (s_accB[chip] + ((int32_t)s_hold_b / 2)) / (int32_t)s_hold_b;
        s_accB[chip] = 0;
      }
      s_hold_b = 0u;

      /* 切到内部温度传感器: CONFIG1.TS = 1。
       * 注意 TS=1 时 CONFIG0 的设置全部失效, 且强制使用内部 2.048V 基准。 */
      (void)ADS1220_SetTempSensorAll(1u);
      (void)ADS1220_StartAll();
      App_StartWait();
      s_state = APP_ST_WAIT_T;
      break;
    }

    /*--------------------------------------------------------------------
     * 阶段 3: 冷端温度 -> 恢复 -> 换算 -> 上报
     *------------------------------------------------------------------*/
    case APP_ST_WAIT_T:
    {
      uint8_t w = App_WaitDone();
      uint8_t chip;

      if (w == 0u)
      {
        break;
      }
      if (w == 2u)
      {
        s_phase_timeout_cnt++;
      }

      App_ReadAll(s_codeT, s_errT);

      /* ★ 冷端也做同样的软件平均: 冷端是本片两路共用的基准, 它的噪声会变成
       *   整片的公共误差, 平均收益和热电偶通道完全一样。 */
      for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
      {
        s_accT[chip] += s_codeT[chip];
      }
      s_hold_t++;
      if (s_hold_t < s_avg_n)
      {
        App_StartWait();
        break;
      }
      for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
      {
        s_codeT[chip] = (s_accT[chip] + ((int32_t)s_hold_t / 2)) / (int32_t)s_hold_t;
        s_accT[chip] = 0;
      }
      s_hold_t = 0u;

      /* 恢复: TS=0 + MUX 回 CH_A, 然后重新同步开始下一轮的 A 通道转换。
       * 这里先发 START 再上报, 让串口发送时间和 50ms 转换时间重叠。 */
      (void)ADS1220_SetTempSensorAll(0u);
      (void)ADS1220_SetMuxAll(ADS1220_MUX_AIN0_AIN1);
      (void)ADS1220_StartAll();
      App_StartWait();

      /* 冷端补偿 + 断线判定; 出错位图在 App_ComputeTemps() 开头清零后重新累积,
       * 保留到下一轮开始, 这样周期性的状态帧能读到本轮的故障信息。 */
      App_ComputeTemps();

      /* 上报温度帧 (CMD=0x10, 固定 32 槽, 没接的槽位是 NaN) */
      App_ReportTemps();

      s_round_count++;

      if (s_single_mode != 0u)
      {
        s_single_mode = 0u;
        if (s_run == 0u)
        {
          /* 这是"停止状态下读单次": 报完就重新掉电 */
          (void)ADS1220_PowerDownAll();
          s_state = APP_ST_STOPPED;
          break;
        }
      }

      s_state = APP_ST_WAIT_A;
      break;
    }

    default:
      /* APP_ST_STOPPED */
      s_state = APP_ST_STOPPED;
      break;
  }

  App_Housekeeping();
}

/*==============================================================================
 * 硬件初始化 (上电流程)
 *============================================================================*/
static void App_HwInit(void)
{
  uint16_t ok_mask = 0u;

  /* ---- 1) 协议层 (打开接收中断) ---- */
  UART_Protocol_Init();

  /* ---- 2) 自检 ---- */
  if (TC_SelfTest() == 0u)
  {
    s_flags |= UART_ST_FLAG_SELFTEST;
  }
  if (UART_Protocol_SelfTest() == 0u)
  {
    s_flags |= UART_ST_FLAG_SELFTEST;
  }

  /* ---- 3) 默认寄存器配置 (对应需求里的 CONFIG0..CONFIG3) ---- */
  ADS1220_DefaultConfig(&s_cfg);

  /* ---- 3b) ★软件平均次数上电默认值 (可由上位机 CMD=0x04 运行时修改) ---- */
  s_avg_n  = (uint8_t)TC_MUX_HOLD_CONVERSIONS;
  s_hold_a = 0u;
  s_hold_b = 0u;
  s_hold_t = 0u;
  App_ClearAvg();

  /* ---- 4) 上电初始化:
   *        >=50ms 等待 -> 逐片 RESET(0x06) -> 1ms -> WREG 4 字节
   *        -> 回读校验 -> START/SYNC(0x08)                       ---- */
  (void)ADS1220_PowerUpInit(&s_cfg, &ok_mask);
  s_chip_ok_mask = ok_mask;
  if (ok_mask != (uint16_t)((1u << ADS1220_CHIP_COUNT) - 1u))
  {
    /* 有片没通过回读校验: 最常见的原因就是"板上留了位置但还没焊",
     * 上位机状态帧的"芯片 ERR 位图"会直接指出是第几片。 */
    s_flags |= UART_ST_FLAG_VERIFY_FAIL;
  }

  /* ★ 把"要等待就绪"的集合收窄成校验通过的片:
   *   没焊的片 DRDY 被内部上拉钳高, 如果还等它, 每一轮都会白等 25ms 超时。 */
  SPI_Driver_DrdySetExpectedMask(ok_mask);

  /* ---- 5) 内部短路失调校准 (手册 9.1.5; ADS1220 没有 "SELF CAL 0x04" 指令,
   *        这是等效且正确的做法)。默认 4 次平均, 耗时约 200ms。 ---- */
#if (TC_OFFSET_CAL_ENABLE != 0)
  {
    uint8_t i;
    for (i = 0u; i < ADS1220_CHIP_COUNT; i++)
    {
      s_offset[i] = 0;
    }
    (void)ADS1220_OffsetCalibrateAll(&s_cfg, s_offset, TC_OFFSET_CAL_SAMPLES);
  }
#else
  {
    uint8_t i;
    for (i = 0u; i < ADS1220_CHIP_COUNT; i++)
    {
      s_offset[i] = 0;
    }
  }
#endif

  /* ---- 6) 启动连续采集 ---- */
  App_Start();

  s_last_drdy_cnt   = SPI_Driver_DrdyReadyCount();
  s_last_drdy_tick  = HAL_GetTick();
  s_status_push_tick = HAL_GetTick();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  App_HwInit();     /* 自检 -> 复位/配置所有 ADS1220 -> 失调校准 -> 启动采集 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Run();      /* 非阻塞: 命令解析 + DRDY 标志消费 + 采集状态机 + 上报 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* ★ APB2 = 72MHz
       * 原模板是 DIV4(APB2=18MHz, 为了把 MAX31855 的 SCK 压到 562.5kHz)。
       * 现在 SPI1 在 APB2 上, 用 /16 得到 4.5MHz (满足 ADS1220 tc(SC)>=150ns),
       * 同时 USART1 拿到 72MHz 时钟, 115200 和 460800 都是整数分频、零误差。 */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
