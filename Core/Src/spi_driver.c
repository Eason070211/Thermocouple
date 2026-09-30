/**
  ******************************************************************************
  * @file    spi_driver.c
  * @brief   SPI1 总线层实现 (片选 / 事务 / 合并 DRDY / 延时)
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
  ******************************************************************************
  */

#include "spi_driver.h"
#include "spi.h"

/*==============================================================================
 * 私有变量
 *============================================================================*/
/** 合并 DRDY 标志: 中断里只写这一个字节, 主循环里读并清零。
 *  用 uint8_t 而不是位域/结构体, 保证单指令读写(原子)。 */
static volatile uint8_t  s_drdy_flag   = 0u;
static volatile uint32_t s_drdy_irq_cnt = 0u;

static uint32_t s_spi_err_cnt  = 0u;
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
  __HAL_RCC_AFIO_CLK_ENABLE();          /* F1: EXTI 复用靠 AFIO_EXTICR */
#elif defined(BOARD_MCU_FAMILY_F4)
  __HAL_RCC_SYSCFG_CLK_ENABLE();        /* F4: EXTI 复用靠 SYSCFG_EXTICR */
#endif

  /* ---- 1) 16 个片选 PB0..PB15: 推挽输出, 50MHz, 初始高电平(释放) ---- */
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

  /* ---- 2) 合并 DRDY: PA0 下降沿中断, 上拉 ----
   *  74HC132 是推挽输出, 上拉只是"逻辑门没焊/掉电"时的保护;
   *  ADS1220 的 DRDY 本身是主动驱动(CS 高时也驱动), 所以不会浮空。 */
  GPIO_InitStruct.Pin  = ADS1220_DRDY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADS1220_DRDY_PORT, &GPIO_InitStruct);

  /* ---- 3) NVIC ---- */
  HAL_NVIC_SetPriority(ADS1220_DRDY_EXTI_IRQn, ADS1220_DRDY_IRQ_PRIO, 0u);
  HAL_NVIC_EnableIRQ(ADS1220_DRDY_EXTI_IRQn);

  /* 初始化后清掉上电过程中可能残留的挂起标志 */
  __HAL_GPIO_EXTI_CLEAR_IT(ADS1220_DRDY_PIN);

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
  HAL_Delay(ms);   /* 基于 SysTick, 会响应中断(DRDY/UART 仍能被服务) */
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
 * 合并 DRDY (PA0 / EXTI0)
 *============================================================================*/
void SPI_Driver_DrdyIrqHandler(void)
{
  /* ★ 中断服务程序里只做这一件事: 置标志位 + 递增计数。
   *   绝对不要在这里做 SPI 读写 —— 一次 16 片的批量读取要几百微秒,
   *   放在 ISR 里会拖垮 UART 接收并造成中断抖动。 */
  s_drdy_flag = 1u;
  s_drdy_irq_cnt++;
}

uint8_t SPI_Driver_DrdyTakeFlag(void)
{
  uint8_t f;

  /* 关中断保护: 读-清 之间可能刚好来一个新的下降沿 */
  SPI_DRV_CRITICAL_ENTER();
  f = s_drdy_flag;
  s_drdy_flag = 0u;
  SPI_DRV_CRITICAL_EXIT();

  return f;
}

void SPI_Driver_DrdyClearFlag(void)
{
  SPI_DRV_CRITICAL_ENTER();
  s_drdy_flag = 0u;
  SPI_DRV_CRITICAL_EXIT();
}

uint8_t SPI_Driver_DrdyLevel(void)
{
  /* 返回 0 表示 DRDY 有效(低) —— 至少一片 ADS1220 有未读走的数据 */
  return (uint8_t)HAL_GPIO_ReadPin(ADS1220_DRDY_PORT, ADS1220_DRDY_PIN);
}

uint32_t SPI_Driver_DrdyIrqCount(void)
{
  uint32_t v;
  SPI_DRV_CRITICAL_ENTER();
  v = s_drdy_irq_cnt;
  SPI_DRV_CRITICAL_EXIT();
  return v;
}

uint32_t SPI_Driver_ErrorCount(void)
{
  return s_spi_err_cnt;
}

uint8_t SPI_Driver_LastErrorChip(void)
{
  return s_last_err_chip;
}
