/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 16 x ADS1220 / 32 路热电偶测温系统 主程序
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
  *  === 为什么必须"广播 START/SYNC + 等一个转换周期"而不是"每片独立 DRDY" ===
  *  16 片 ADS1220 的 DRDY 经 2x74HC30 + 74HC132 合并成 **一路** PA0 中断,
  *  固件无法分辨是哪一片拉低的。所以采用:
  *
  *    1) 每次切换 MUX / TS 之后, 对 16 片 **逐片** 发 WREG + START/SYNC。
  *       START/SYNC 会复位数字滤波器并重新开始转换, 于是 16 片在时间上重新对齐
  *       (彼此相差只有几十微秒的 SPI 下发时间), 合并 DRDY 就变成一次干净的脉冲。
  *    2) 等待期间同时满足两个条件才继续:
  *         a. 已经过了 >= 0.9 个转换周期 (保证读到的是新通道的数据, 不是上一次的);
  *         b. 合并 DRDY 下降沿中断标志已置位 (ISR 只置标志位)。
  *       超时 (转换周期 + 25ms) 则记一次故障并继续, 保证流程永不卡死。
  *    3) 读完数据后, 该片的 DRDY 会在下一个 SCLK 上升沿自动变高, 合并信号随之恢复,
  *       不需要任何"复位逻辑芯片"的操作。
  *
  *  一个完整轮次 = A 通道 -> 切 MUX -> B 通道 -> 切 TS -> 冷端 -> 恢复 + 上报,
  *  每个阶段各占 1 个转换周期 (20SPS 下 50ms), 因此 20SPS 时约 150ms/轮。
  */
typedef enum
{
  APP_ST_STOPPED = 0,   /**< 已停止 (16 片处于 POWERDOWN) */
  APP_ST_WAIT_A,        /**< 等 A 通道 (AIN0/AIN1) 转换完成 */
  APP_ST_WAIT_B,        /**< 等 B 通道 (AIN2/AIN3) 转换完成 */
  APP_ST_WAIT_T         /**< 等内部温度传感器 (冷端) 转换完成 */
} App_State_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** 采集运行中, 超过这个时间一次 DRDY 都没来 -> 置"DRDY 静默"故障位。
 *  16 片 x 20SPS = 理论 320 次/秒, 1 秒没有一次说明总线或器件出问题了。 */
#define TC_DRDY_SILENT_MS      1000u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*---- 配置与采集数据 ----*/
static ADS1220_Config_t s_cfg;                                  /**< 16 片共用配置镜像 */

static int32_t  s_codeA[ADS1220_CHIP_COUNT];                    /**< A 通道 24bit 码值 */
static int32_t  s_codeB[ADS1220_CHIP_COUNT];                    /**< B 通道 24bit 码值 */
static int32_t  s_codeT[ADS1220_CHIP_COUNT];                    /**< 内部温度传感器码值 */
static int32_t  s_offset[ADS1220_CHIP_COUNT];                   /**< 上电失调校准结果(码值) */
static uint8_t  s_errA[ADS1220_CHIP_COUNT];                     /**< 0 = SPI 成功 */
static uint8_t  s_errB[ADS1220_CHIP_COUNT];
static uint8_t  s_errT[ADS1220_CHIP_COUNT];

static float    s_temps[TC_CHANNEL_COUNT];                      /**< 32 路最终温度 °C */

/*---- 状态 ----*/
static App_State_t s_state;
static uint8_t   s_run;                                         /**< 1 = 采集运行中 */
static uint8_t   s_single_mode;                                 /**< 1 = 本次是"读单次"请求 */
static uint8_t   s_single_ch;                                   /**< 单次请求的通道号 */
static uint8_t   s_hold_a;                                      /**< MUX 保持计数 (CH_A) */
static uint8_t   s_hold_b;                                      /**< MUX 保持计数 (CH_B) */

static uint16_t  s_chip_ok_mask;                                /**< 上电回读校验通过的片 */
static uint16_t  s_chip_err_mask;                               /**< 本轮 SPI 出错的片 */
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

  /* 最小等待 = 0.9 x 实际转换时间, 宁可晚一点也不要把上一通道的旧数据当新的 */
  s_wait_min_ms = (ri->conv_us * ADS1220_WAIT_MIN_NUM) /
                  (1000u * ADS1220_WAIT_MIN_DEN);
  if (s_wait_min_ms == 0u)
  {
    s_wait_min_ms = 1u;                 /* 高速率下至少等 1ms */
  }

  /* 超时 = 转换时间 + 25ms 裕量 */
  s_wait_max_ms = (ri->conv_us / 1000u) + ADS1220_WAIT_MARGIN_MS;

  SPI_Driver_DrdyClearFlag();           /* 清掉 START 之前的残留边沿 */
  s_wait_start = HAL_GetTick();
}

/**
  * @brief 查询等待是否结束
  * @retval 0 = 继续等; 1 = 等到 DRDY; 2 = 超时
  */
static uint8_t App_WaitDone(void)
{
  uint32_t elapsed = HAL_GetTick() - s_wait_start;

  if (elapsed < s_wait_min_ms)
  {
    return 0u;                          /* 还没到最小建立时间, 边沿一律忽略 */
  }

  if (SPI_Driver_DrdyTakeFlag() != 0u)
  {
    return 1u;                          /* 中断里置的标志位被主循环消费 */
  }

  if (elapsed >= s_wait_max_ms)
  {
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

/*==============================================================================
 * 温度换算 (冷端补偿在这里完成)
 *============================================================================*/
static void App_ComputeTemps(void)
{
  uint8_t chip;
  uint8_t ia;
  uint8_t ib;
  float   cj_c;
  float   emf_uv;
  TC_Result_t r;

  s_chip_err_mask = 0u;         /* 本轮错误位图从这里开始重新累积 */

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    ia = TC_CH_INDEX_A(chip);
    ib = TC_CH_INDEX_B(chip);

    /* 默认本片两个通道都无效 */
    s_temps[ia] = UART_NaN();
    s_temps[ib] = UART_NaN();

    /* 上电回读校验没通过的片子直接跳过 (避免把"没焊的芯片读回全 0"
     * 当成 0uV 而报出一个看起来正常的冷端温度) */
    if ((s_chip_ok_mask & (uint16_t)(1u << chip)) == 0u)
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

    /* ---- A 通道: AIN0/AIN1 ---- */
    if (s_errA[chip] == 0u)
    {
      emf_uv = ADS1220_CodeToMicroVolt(s_codeA[chip] - s_offset[chip], s_cfg.gain);
      TC_Compute(emf_uv, cj_c, s_codeA[chip], &r);
      if (r.valid != 0u)
      {
        s_temps[ia] = r.hot_c;
      }
      else
      {
        s_open_cnt++;                 /* 断线/超量程 */
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
      TC_Compute(emf_uv, cj_c, s_codeB[chip], &r);
      if (r.valid != 0u)
      {
        s_temps[ib] = r.hot_c;
      }
      else
      {
        s_open_cnt++;
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
  /* 单通道请求: 只保留目标通道, 其余填 NaN */
  if ((s_single_mode != 0u) && (s_single_ch != UART_CH_SINGLE_ALL))
  {
    static float one[TC_CHANNEL_COUNT];
    uint8_t i;

    for (i = 0u; i < TC_CHANNEL_COUNT; i++)
    {
      one[i] = UART_NaN();
    }
    if (s_single_ch < TC_CHANNEL_COUNT)
    {
      one[s_single_ch] = s_temps[s_single_ch];
    }
    (void)UART_Protocol_SendTempFrame(one);
  }
  else
  {
    (void)UART_Protocol_SendTempFrame(s_temps);
  }
}

static void App_StatusFill(UART_Status_t *st)
{
  st->run_state      = s_run;
  st->dr_bits        = s_cfg.dr;
  st->reject         = s_cfg.reject;
  st->chip_ok_mask   = s_chip_ok_mask;
  st->chip_err_mask  = s_chip_err_mask;
  st->round_count    = s_round_count;
  st->uptime_ms      = HAL_GetTick();
  st->drdy_irq_count = (uint16_t)(SPI_Driver_DrdyIrqCount() & 0xFFFFu);
  st->spi_err_count  = (uint16_t)(SPI_Driver_ErrorCount() & 0xFFFFu);
  st->open_tc_count  = s_open_cnt;
  st->phase_timeout_count = s_phase_timeout_cnt;

  st->flags = s_flags;
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
  s_chip_err_mask = 0u;
  s_state       = APP_ST_WAIT_A;

  App_StartWait();
}

/** 停止采集: 16 片进入 POWERDOWN (寄存器值保持, 下次 START/SYNC 即可恢复)。 */
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

/** 单次读取: 无论当前在哪个阶段, 都把 16 片拉回 CH_A + TS=0 重新跑一整轮。 */
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
  uint32_t cnt = SPI_Driver_DrdyIrqCount();

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

      if (w == 0u)
      {
        break;                                  /* 还没好, 下一圈再看 */
      }
      if (w == 2u)
      {
        s_phase_timeout_cnt++;                  /* 超时: 记故障, 但流程继续 */
      }

      App_ReadAll(s_codeA, s_errA);

      /* 需求中的 "每完成 N 次转换切换一次 MUX" */
      s_hold_a++;
      if (s_hold_a < TC_MUX_HOLD_CONVERSIONS)
      {
        App_StartWait();                        /* 继续读同一通道, 不切 MUX */
        break;
      }
      s_hold_a = 0u;

      /* 切到 B 通道, 并用 START/SYNC 让 16 片重新同步 */
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

      if (w == 0u)
      {
        break;
      }
      if (w == 2u)
      {
        s_phase_timeout_cnt++;
      }

      App_ReadAll(s_codeB, s_errB);

      s_hold_b++;
      if (s_hold_b < TC_MUX_HOLD_CONVERSIONS)
      {
        App_StartWait();
        break;
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

      if (w == 0u)
      {
        break;
      }
      if (w == 2u)
      {
        s_phase_timeout_cnt++;
      }

      App_ReadAll(s_codeT, s_errT);

      /* 恢复: TS=0 + MUX 回 CH_A, 然后重新同步开始下一轮的 A 通道转换。
       * 这里先发 START 再上报, 让串口发送时间和 50ms 转换时间重叠。 */
      (void)ADS1220_SetTempSensorAll(0u);
      (void)ADS1220_SetMuxAll(ADS1220_MUX_AIN0_AIN1);
      (void)ADS1220_StartAll();
      App_StartWait();

      /* 冷端补偿 + 断线判定; 出错位图在 App_ComputeTemps() 开头清零后重新累积,
       * 保留到下一轮开始, 这样周期性的状态帧能读到本轮的故障信息。 */
      App_ComputeTemps();

      /* 上报 32 路温度 (CMD=0x10) */
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

  /* ---- 4) 上电初始化:
   *        >=50ms 等待 -> 16 片 RESET(0x06) -> 1ms -> WREG 4 字节
   *        -> 回读校验 -> START/SYNC(0x08)                       ---- */
  (void)ADS1220_PowerUpInit(&s_cfg, &ok_mask);
  s_chip_ok_mask = ok_mask;
  if (ok_mask != (uint16_t)((1u << ADS1220_CHIP_COUNT) - 1u))
  {
    s_flags |= UART_ST_FLAG_VERIFY_FAIL;
  }

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

  s_last_drdy_cnt   = SPI_Driver_DrdyIrqCount();
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

  App_HwInit();     /* 自检 -> 复位/配置 16 片 ADS1220 -> 失调校准 -> 启动采集 */

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
