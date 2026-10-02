/**
  ******************************************************************************
  * @file    spi_driver.c
  * @brief   SPI1 总线层实现 (片选 / 事务 / 每片 DRDY 轮询与软件合成 / 延时)
  *
  *  === 时序依据 (TI ADS1220 数据手册 SBAS501D) ===
  *    td(CSSC)  CS 下降到第一个 SCLK 上升沿   >= 50ns
  *    td(SCCS)  最后一个 SCLK 下降到 CS 上升  >= 25ns
  *    tw(CSH)   CS 高电平最小宽度             >= 50ns
  *    tp(CSDO)  CS 下降到 DOUT 被驱动         <= 50ns
  *    tp(CSDOZ) CS 上升到 DOUT 高阻           <= 50ns
  *    "When the serial interface is idle, hold SCLK low." (8.5.1.2) —— CPOL=0 满足。
  *
  *  === 关于 tDATA ===
  *    使用专用 DRDY 引脚时, 数据在 DRDY 下降沿即已就绪, 手册未定义 tDATA;
  *    本层在每个事务前统一留出 ADS1220_T_DATA_US (2us ≈ 8 x tCLK) 作为裕量,
  *    该等待放在 ads1220.c 的 RDATA 之前, 这里只负责 CS/SCLK 的硬时序。
  *
  *  === DRDY: 硬件简化 + 软件合成 (见 spi_driver.h 顶部的详细说明) ===
  *    每片一根 DRDY 直接进 MCU (PA0..PA3), 不用 74HC30/74HC132 逻辑门。
  *    本层把 N 根合成出一个"全部就绪"信号, 对上层保持与老合并-DRDY 相同的语义,
  *    同时额外提供逐片位图, 便于定位坏片。
  ******************************************************************************
  */

#include "spi_driver.h"
#include "spi.h"

/*==============================================================================
 * 私有变量
 *============================================================================*/
/** "全部就绪"的锁存: 1 = 自上次 DrdyClearFlag 之后见过一次"所有片都就绪"。
 *  用 uint8_t 保证单指令读写(原子)。新方案没有 DRDY 中断, 只有主循环访问。 */
static volatile uint8_t  s_all_ready_latch = 0u;
static volatile uint32_t s_drdy_ready_cnt  = 0u;

/** 本阶段"需要等待就绪"的片的掩码。
 *  默认=全部; main.c 上电回读校验之后会把它收窄成"校验通过的片",
 *  这样**没焊的芯片不会被等**, 也就不会因为缺片而每轮都超时。 */
static uint16_t s_drdy_expected = ADS1220_DRDY_MASK_ALL;

static uint32_t s_spi_err_cnt   = 0u;
static uint8_t  s_last_err_chip = 0xFFu;
static uint8_t  s_dwt_ok        = 0u;

/* 50MHz 推挽输出的片选需要的最小建立/保持时间: @72MHz 一个 NOP = 13.9ns,
 * 8 个 NOP = 111ns > max(td(CSSC)=50ns, td(SCCS)=25ns, tw(CSH)=50ns)。
 * 下面的编译期断言保证这里的 NOP 个数和 board_config.h 里的
 * SPI_DRIVER_SHORT_NOP_CNT 一致 —— 改了其中一个忘了另一个会直接编译失败。 */
#define SPI_DRIVER_SHORT_DELAY()   \
  do {                             \
    __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); \
  } while (0)

#define SPI_DRIVER_STATIC_ASSERT(cond, tag) \
  typedef char spi_driver_static_assert_##tag[(cond) ? 1 : -1]

SPI_DRIVER_STATIC_ASSERT(SPI_DRIVER_SHORT_NOP_CNT == 8u, short_delay_nop_count);

/* ---- 极短临界区: 用 PRIMASK 保存/恢复, 而不是无脑 __enable_irq()。
       否则在"本来就已经关中断"的上下文里调用会提前开中断。 ---- */
#define SPI_DRV_CRITICAL_ENTER()   uint32_t _primask = __get_PRIMASK(); __disable_irq()
#define SPI_DRV_CRITICAL_EXIT()    __set_PRIMASK(_primask)

/*==============================================================================
 * 私有函数
 *============================================================================*/

/** 使能 DWT 周期计数器 (Cortex-M3 内核自带, 用于精确 us 延时)。 */
static void SPI_Driver_DwtInit(void)
{
  uint32_t c0;
  uint32_t c1;
  uint32_t guard = 0u;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 打开跟踪与调试模块时钟 */
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* 启动周期计数 */

  /* 校验计数器是否真的在跑 (某些调试器/低功耗场景下可能被关掉) */
  c0 = DWT->CYCCNT;
  do
  {
    c1 = DWT->CYCCNT;
    guard++;
  } while ((c1 == c0) && (guard < 1000u));

  s_dwt_ok = (c1 != c0) ? 1u : 0u;
}

/*==============================================================================
 * 初始化
 *============================================================================*/
void SPI_Driver_GpioInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* ---- 时钟 ---- */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
#if defined(BOARD_MCU_FAMILY_F1)
  __HAL_RCC_AFIO_CLK_ENABLE();          /* F1: 复用/AFIO 配置必须开这个时钟 */
#elif defined(BOARD_MCU_FAMILY_F4)
  __HAL_RCC_SYSCFG_CLK_ENABLE();        /* F4: 外部中断复用靠 SYSCFG */
#endif

#if defined(BOARD_MCU_FAMILY_F1)
  /* ---- 0) 关闭 JTAG, 只保留 SWD ----
   *  ★ 这一步是必须的: STM32 复位后默认 SWJ = 全使能, PA13/PA14/PA15/PB3/PB4
   *    被 JTAG 占用。而本板的片选是 PB0..PB3(4 片), 片数 >= 4 时 PB3 = JTDO,
   *    不关 JTAG 的话 CS3 根本不是普通输出 —— 表现就是"第 4 片永远读不回来"。
   *    (16 片老版本用的是 PB0..PB15, 当时没有这一步, CS3/CS4 实际是失效的。)
   *    关掉 JTAG 后 SWD 仍然可用, 下载/调试完全不受影响。 */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
#endif

  /* ---- 1) 片选 PB0..PB(CHIP_COUNT-1): 推挽输出, 50MHz, 初始高电平(释放) ---- */
  HAL_GPIO_WritePin(ADS1220_CS_PORT, ADS1220_CS_MASK_ALL, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = ADS1220_CS_MASK_ALL;
#if defined(BOARD_MCU_FAMILY_F1)
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      /* F1: SPEED_FREQ_HIGH = 50MHz */
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#else
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      /* F4: HIGH=25MHz, 50MHz 要用 VERY_HIGH */
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#endif
  HAL_GPIO_Init(ADS1220_CS_PORT, &GPIO_InitStruct);

  /* ---- 2) DRDY: 每片一根 PA0..PA3, 输入 + 内部上拉 ----
   *  ADS1220 的 DRDY 是"主动驱动的推挽输出, CS 为高时也驱动"(SBAS501D 8.5.1.3),
   *  所以严格来说不需要上拉; 这里加上内部上拉是为了:
   *    a) 没贴芯片的焊盘 -> 该引脚被钳高 = "永远没有新数据", 不会悬空误触发;
   *    b) 芯片还没上电/复位期间, 引脚不会因为浮空被读成随机的"就绪"。
   *  低有效: 0 = 有新数据。 */
  GPIO_InitStruct.Pin  = ADS1220_DRDY_MASK_ALL;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADS1220_DRDY_PORT, &GPIO_InitStruct);

  /* 注意: 新方案没有 DRDY 中断, 因此不再配置 EXTI / NVIC。
   *       原来的 EXTI0_IRQHandler 已经从 stm32f1xx_it.c 里移除。 */

  /* ---- 3) 期望掩码复位成"全部片" ---- */
  s_drdy_expected   = ADS1220_DRDY_MASK_ALL;
  s_all_ready_latch = 0u;
  s_drdy_ready_cnt  = 0u;

  /* ---- 4) DWT 延时基准 ---- */
  SPI_Driver_DwtInit();
}

void SPI_Driver_ReleaseAllCs(void)
{
  HAL_GPIO_WritePin(ADS1220_CS_PORT, ADS1220_CS_MASK_ALL, GPIO_PIN_SET);
}

/*==============================================================================
 * 延时
 *============================================================================*/
void SPI_Driver_DelayUs(uint32_t us)
{
  if (us == 0u)
  {
    return;
  }

  if (s_dwt_ok != 0u)
  {
    uint32_t start;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);   /* 72MHz -> 72 tick/us */

    start = DWT->CYCCNT;
    /* 无符号相减天然处理 32 位回绕 */
    while ((DWT->CYCCNT - start) < ticks)
    {
      /* busy wait */
    }
  }
  else
  {
    /* 退化路径: 每次循环约 3~4 个周期 */
    volatile uint32_t n = us * ((SystemCoreClock / 3000000u) + 1u);
    while (n != 0u)
    {
      n--;
    }
  }
}

void SPI_Driver_DelayMs(uint32_t ms)
{
  if (ms == 0u)
  {
    return;
  }
  HAL_Delay(ms);   /* 基于 SysTick, 会响应中断(UART 仍能被服务) */
}

/*==============================================================================
 * SPI 事务
 *============================================================================*/
void SPI_Driver_Recover(void)
{
  /* 上一次超时可能让 HAL 状态机停在 BUSY_TX_RX, 必须显式中止,
     否则后续所有 HAL_SPI_xxx 都会立刻返回 HAL_BUSY。 */
  if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  {
    (void)HAL_SPI_Abort(&hspi1);
  }

  __HAL_SPI_DISABLE(&hspi1);
  __HAL_SPI_CLEAR_OVRFLAG(&hspi1);      /* 清 OVR: 先读 DR 再读 SR */
  hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
  __HAL_SPI_ENABLE(&hspi1);
}

SPI_Driver_Status_t SPI_Driver_Transfer(uint8_t chip,
                                        const uint8_t *tx,
                                        uint8_t *rx,
                                        uint16_t len)
{
  HAL_StatusTypeDef hal_st;
  SPI_Driver_Status_t st;

  if ((chip >= ADS1220_CHIP_COUNT) || (tx == NULL) || (len == 0u))
  {
    return SPI_DRV_ERR_PARAM;
  }

  /* 若上一次事务异常, 先把外设恢复到 READY */
  if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  {
    SPI_Driver_Recover();
  }

  /* ---- 1) 选中目标片 (CS 低有效) ---- */
  HAL_GPIO_WritePin(ADS1220_CS_PORT, ADS1220_CS_PIN(chip), GPIO_PIN_RESET);

  /* ---- 2) td(CSSC): CS 下降 -> 第一个 SCLK 上升沿 >= 50ns ---- */
  SPI_DRIVER_SHORT_DELAY();

  /* ---- 3) 收发 ----
   *  SPI Mode1 (CPOL=0/CPHA=1): 主机在第一个(上升)沿输出, 在第二个(下降)沿采样,
   *  与 ADS1220 "DIN 在 SCLK 下降沿被锁存 / DOUT 在 SCLK 上升沿更新" 完全对应。*/
  if (rx != NULL)
  {
    hal_st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len,
                                     ADS1220_SPI_TIMEOUT_MS);
  }
  else
  {
    hal_st = HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len,
                              ADS1220_SPI_TIMEOUT_MS);
  }

  /* ---- 4) td(SCCS): 最后一个 SCLK 下降 -> CS 上升 >= 25ns ---- */
  SPI_DRIVER_SHORT_DELAY();

  /* ---- 5) 释放片选 ---- */
  HAL_GPIO_WritePin(ADS1220_CS_PORT, ADS1220_CS_PIN(chip), GPIO_PIN_SET);

  /* ---- 6) tw(CSH): 下一次选中同一片之前, CS 必须保持高 >= 50ns ---- */
  SPI_DRIVER_SHORT_DELAY();

  /* ---- 7) 错误处理 ---- */
  if (hal_st == HAL_OK)
  {
    st = SPI_DRV_OK;
  }
  else
  {
    s_spi_err_cnt++;
    s_last_err_chip = chip;
    SPI_Driver_Recover();     /* 关键: 不让总线卡死, 后续芯片还能继续测 */
    st = (hal_st == HAL_TIMEOUT) ? SPI_DRV_ERR_TIMEOUT : SPI_DRV_ERR_HAL;
  }

  return st;
}

/*==============================================================================
 * DRDY: 每片一根 -> 软件合成"全部就绪"
 *
 *  硬件: DRDY0..3 = PA0..PA3, 低有效, 输入 + 内部上拉, 无中断。
 *  合成: 一次读 GPIOA->IDR 拿到全部 4 根线的电平 (单条指令, 天然原子),
 *        取反后 bit n = 1 表示第 n 片有新数据。
 *============================================================================*/
uint16_t SPI_Driver_DrdyReadyMask(void)
{
  /* IDR 里 1 = 高 = 未就绪; 取反后 1 = 低 = 已就绪 (低有效) */
  uint16_t idr = (uint16_t)ADS1220_DRDY_PORT->IDR;

  return (uint16_t)((uint16_t)(~idr) & ADS1220_DRDY_MASK_ALL);
}

void SPI_Driver_DrdySetExpectedMask(uint16_t mask)
{
  /* 掩码里只保留本板实际存在的片 */
  mask = (uint16_t)(mask & ADS1220_DRDY_MASK_ALL);

  /* 传 0 没有意义(会变成"永远就绪"), 这时保持原值不动, 让上层超时并报错 */
  if (mask != 0u)
  {
    s_drdy_expected = mask;
  }
}

uint16_t SPI_Driver_DrdyExpectedMask(void)
{
  return s_drdy_expected;
}

uint8_t SPI_Driver_DrdyTakeFlag(void)
{
  uint16_t ready = SPI_Driver_DrdyReadyMask();

  if ((uint16_t)(ready & s_drdy_expected) == s_drdy_expected)
  {
    /* 只在"从没就绪 -> 全就绪"的那一次计数, 语义上等于老方案的下降沿计数 */
    if (s_all_ready_latch == 0u)
    {
      s_all_ready_latch = 1u;
      s_drdy_ready_cnt++;
    }
    return 1u;
  }

  s_all_ready_latch = 0u;
  return 0u;
}

void SPI_Driver_DrdyClearFlag(void)
{
  s_all_ready_latch = 0u;
}

uint8_t SPI_Driver_DrdyLevel(void)
{
  /* 返回 0 表示"低有效" = 需要的片都有未读走的数据 (与老合并 DRDY 语义一致) */
  return SPI_Driver_DrdyTakeFlag() != 0u ? 0u : 1u;
}

uint32_t SPI_Driver_DrdyReadyCount(void)
{
  return s_drdy_ready_cnt;
}

uint32_t SPI_Driver_ErrorCount(void)
{
  return s_spi_err_cnt;
}

uint8_t SPI_Driver_LastErrorChip(void)
{
  return s_last_err_chip;
}
