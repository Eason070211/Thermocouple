/**
  ******************************************************************************
  * @file    ads1220.c
  * @brief   ADS1220 驱动实现
  *
  *  参考: TI ADS1220 数据手册 SBAS501D
  *        Table 8-4  转换时间            Table 8-7  指令定义
  *        Table 8-8  寄存器映射          Table 8-10/8-11/8-13/8-14 位定义
  *        8.3.13 温度传感器              8.3.12 / 9.1.5 失调校准
  *        8.5.1.3 DRDY                   8.5.4 读数据
  *        9.1.6 连续转换模式伪代码
  *
  *  本文件不直接碰 PA0/DRDY 的寄存器, 只通过 spi_driver.h 暴露的接口访问,
  *  保证"中断里只置标志位"这条约束不会被这里破坏。
  ******************************************************************************
  */

#include "ads1220.h"

/*==============================================================================
 * 私有状态
 *============================================================================*/

/** 当前全部片共用的配置镜像。SetMuxAll / SetTempSensorAll 需要它来重建寄存器字节,
 *  避免调用者自己拼位域时把别的字段写坏。 */
static ADS1220_Config_t s_cfg;
static uint8_t          s_cfg_valid = 0u;

/*==============================================================================
 * 数据速率表 (Table 8-4, 正常模式, 内部振荡器 4.096MHz)
 *   conv_us = 实际转换时间(tCLK) / 4.096
 *============================================================================*/
static const ADS1220_RateInfo_t s_rate_table[] =
{
  /* dr_bits                sps    conv_us  reject_ok */
  { ADS1220_DR_20SPS,        20u,   49992u, 1u },  /* 204768 tCLK */
  { ADS1220_DR_45SPS,        45u,   22246u, 0u },  /*  91120 tCLK */
  { ADS1220_DR_90SPS,        90u,   11262u, 0u },  /*  46128 tCLK */
  { ADS1220_DR_175SPS,      175u,    5777u, 0u },  /*  23664 tCLK */
  { ADS1220_DR_330SPS,      330u,    3043u, 0u },  /*  12464 tCLK */
  { ADS1220_DR_600SPS,      600u,    1684u, 0u },  /*   6896 tCLK */
  { ADS1220_DR_1000SPS,    1000u,    1012u, 0u },  /*   4144 tCLK */
};
#define ADS1220_RATE_TABLE_LEN  (sizeof(s_rate_table) / sizeof(s_rate_table[0]))

/** 增益倍数表, 下标 = GAIN[2:0] >> 1 */
static const uint16_t s_gain_table[8] =
{
  1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u
};

/*==============================================================================
 * 私有函数
 *============================================================================*/

/**
  * @brief 阻塞等待"合并 DRDY"有效
  * @param min_ms 最小等待(ms) —— 用来保证"至少过一个转换周期", 过滤掉上一次的残留边沿
  * @param max_ms 超时(ms)
  * @retval 1 = 等到 DRDY; 0 = 超时
  * @note  只在上电初始化(失调校准)时使用; 主循环绝不调用这个阻塞函数。
  */
static uint8_t ADS1220_WaitDataReadyBlocking(uint32_t min_ms, uint32_t max_ms)
{
  uint32_t t;

  SPI_Driver_DrdyClearFlag();

  for (t = 0u; t < max_ms; t++)
  {
    SPI_Driver_DelayMs(1u);

    /* 前 (min_ms-1) ms 内到达的边沿先"记账"但不消费:
       因为我们是在 START/SYNC 之后立即开始等的, 理论上不会有过早的边沿;
       即便如此, 下面的时间门限也保证了不会提前退出。 */
    if ((t + 1u) >= min_ms)
    {
      if (SPI_Driver_DrdyTakeFlag() != 0u)
      {
        return 1u;
      }
    }
  }

  return 0u;
}

/** 把一个配置写进 s_cfg (并标记有效) */
static void ADS1220_StoreConfig(const ADS1220_Config_t *cfg)
{
  if (cfg != NULL)
  {
    s_cfg = *cfg;
    s_cfg_valid = 1u;
  }
}

/*==============================================================================
 * 配置构造
 *============================================================================*/
void ADS1220_DefaultConfig(ADS1220_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  cfg->mux        = ADS1220_MUX_AIN0_AIN1;      /* AIN0/AIN1 差分 (CH_A) */
  cfg->gain       = ADS1220_GAIN_1;             /* 增益 = 1 (需求) */
  cfg->pga_bypass = ADS1220_PGA_BYPASSED;       /* PGA 旁路 (需求) */
  cfg->dr         = ADS1220_DR_20SPS;           /* 20 SPS (需求) */
  cfg->mode       = ADS1220_MODE_NORMAL;        /* 正常模式 256kHz 调制器 */
  cfg->cm         = ADS1220_CM_CONTINUOUS;      /* 连续转换模式 (需求) */
  cfg->ts         = ADS1220_TS_ADC;             /* 差分测量, 温度传感器关闭 (需求 TS=0) */
  cfg->bcs        = (TC_BCS_ALWAYS_ON != 0) ? ADS1220_BCS_ON : ADS1220_BCS_OFF;
  cfg->vref       = ADS1220_VREF_INTERNAL_2V048;/* 内部 2.048V (需求) */
  cfg->reject     = ADS1220_REJECT_50_60;       /* 同时抑制 50Hz+60Hz (需求) = 01b */
  cfg->psw        = ADS1220_PSW_OPEN;           /* 低边开关常开 */
  cfg->idac       = ADS1220_IDAC_OFF;           /* IDAC 关闭 (需求) */
  cfg->i1mux      = ADS1220_I1MUX_OFF;
  cfg->i2mux      = ADS1220_I2MUX_OFF;
  cfg->drdym      = ADS1220_DRDYM_DRDY_PIN_ONLY;/* 只用专用 DRDY 引脚 (需求) */
}

void ADS1220_BuildRegs(const ADS1220_Config_t *cfg, uint8_t regs[ADS1220_REG_COUNT])
{
  if ((cfg == NULL) || (regs == NULL))
  {
    return;
  }

  /* CONFIG0 = MUX[7:4] | GAIN[3:1] | PGA_BYPASS[0] */
  regs[0] = (uint8_t)((cfg->mux        & 0xF0u) |
                      (cfg->gain       & 0x0Eu) |
                      (cfg->pga_bypass & 0x01u));

  /* CONFIG1 = DR[7:5] | MODE[4:3] | CM[2] | TS[1] | BCS[0] */
  regs[1] = (uint8_t)((cfg->dr   & 0xE0u) |
                      (cfg->mode & 0x18u) |
                      (cfg->cm   & 0x04u) |
                      (cfg->ts   & 0x02u) |
                      (cfg->bcs  & 0x01u));

  /* CONFIG2 = VREF[7:6] | 50/60[5:4] | PSW[3] | IDAC[2:0] */
  regs[2] = (uint8_t)((cfg->vref   & 0xC0u) |
                      (cfg->reject & 0x30u) |
                      (cfg->psw    & 0x08u) |
                      (cfg->idac   & 0x07u));

  /* CONFIG3 = I1MUX[7:5] | I2MUX[4:2] | DRDYM[1] | RESERVED[0]=0 */
  regs[3] = (uint8_t)((cfg->i1mux & 0xE0u) |
                      (cfg->i2mux & 0x1Cu) |
                      (cfg->drdym & 0x02u));
}

const ADS1220_RateInfo_t *ADS1220_RateInfo(uint8_t dr_bits)
{
  uint8_t i;

  for (i = 0u; i < (uint8_t)ADS1220_RATE_TABLE_LEN; i++)
  {
    if (s_rate_table[i].dr_bits == (dr_bits & 0xE0u))
    {
      return &s_rate_table[i];
    }
  }

  return &s_rate_table[0];   /* 非法值 -> 退化为 20SPS */
}

uint32_t ADS1220_ConversionTimeUs(uint8_t dr_bits)
{
  return ADS1220_RateInfo(dr_bits)->conv_us;
}

uint16_t ADS1220_GainValue(uint8_t gain_bits)
{
  return s_gain_table[(uint8_t)((gain_bits >> 1) & 0x07u)];
}

/*==============================================================================
 * 单片刻操作
 *============================================================================*/
ADS1220_Status_t ADS1220_ResetChip(uint8_t chip)
{
  uint8_t cmd = ADS1220_CMD_RESET;
  return SPI_Driver_Transfer(chip, &cmd, NULL, 1u);
}

ADS1220_Status_t ADS1220_WriteRegs(uint8_t chip, uint8_t start,
                                   const uint8_t *vals, uint8_t cnt)
{
  uint8_t buf[1u + ADS1220_REG_COUNT];
  uint8_t i;

  if ((vals == NULL) || (cnt == 0u) || (cnt > ADS1220_REG_COUNT))
  {
    return SPI_DRV_ERR_PARAM;
  }

  buf[0] = ADS1220_CMD_WREG(start, cnt);
  for (i = 0u; i < cnt; i++)
  {
    buf[1u + i] = vals[i];
  }

  /* 手册 8.5.3.6: "配置寄存器在最后一个 SCLK 下降沿被更新" ——
     SPI_Driver_Transfer 结束时 CS 才拉高, 所以更新一定已经发生。 */
  return SPI_Driver_Transfer(chip, buf, NULL, (uint16_t)(cnt + 1u));
}

ADS1220_Status_t ADS1220_ReadRegs(uint8_t chip, uint8_t start,
                                  uint8_t *vals, uint8_t cnt)
{
  uint8_t tx[1u + ADS1220_REG_COUNT];
  uint8_t rx[1u + ADS1220_REG_COUNT];
  uint8_t i;

  if ((vals == NULL) || (cnt == 0u) || (cnt > ADS1220_REG_COUNT))
  {
    return SPI_DRV_ERR_PARAM;
  }

  tx[0] = ADS1220_CMD_RREG(start, cnt);
  for (i = 0u; i < cnt; i++)
  {
    tx[1u + i] = 0x00u;      /* 读寄存器期间 DIN 保持低 */
    rx[1u + i] = 0x00u;
  }

  {
    ADS1220_Status_t st = SPI_Driver_Transfer(chip, tx, rx, (uint16_t)(cnt + 1u));
    if (st != SPI_DRV_OK)
    {
      return st;
    }
  }

  for (i = 0u; i < cnt; i++)
  {
    vals[i] = rx[1u + i];
  }

  return SPI_DRV_OK;
}

ADS1220_Status_t ADS1220_WriteReg(uint8_t chip, uint8_t addr, uint8_t val)
{
  return ADS1220_WriteRegs(chip, addr, &val, 1u);
}

ADS1220_Status_t ADS1220_Start(uint8_t chip)
{
  uint8_t cmd = ADS1220_CMD_START_SYNC;
  return SPI_Driver_Transfer(chip, &cmd, NULL, 1u);
}

ADS1220_Status_t ADS1220_PowerDown(uint8_t chip)
{
  uint8_t cmd = ADS1220_CMD_POWERDOWN;
  return SPI_Driver_Transfer(chip, &cmd, NULL, 1u);
}

int32_t ADS1220_RawToCode(const uint8_t raw[3])
{
  uint32_t u;

  u = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[2];

  /* 24bit 二进制补码 -> 32bit; 显式符号扩展, 不依赖 >> 的实现定义行为 */
  if ((u & 0x00800000u) != 0u)
  {
    return (int32_t)(u | 0xFF000000u);
  }

  return (int32_t)u;
}

ADS1220_Status_t ADS1220_ReadData(uint8_t chip, int32_t *code, uint8_t raw[3])
{
  uint8_t tx[3] = { 0x00u, 0x00u, 0x00u };   /* 读数据期间 DIN 保持低 */
  uint8_t rx[3] = { 0x00u, 0x00u, 0x00u };
  ADS1220_Status_t st;

  if (code == NULL)
  {
    return SPI_DRV_ERR_PARAM;
  }

  /* ★ tDATA: DRDY 有效后先留出建立时间, 再开始 SCLK。
     使用专用 DRDY 引脚时手册不要求额外延时, 这里保留 2us(约 8 个 tCLK)
     作为逻辑门传播 + 中断延迟 + td(CSSC) 的裕量 (需求: "至少 1 个 CLK 周期")。 */
  SPI_Driver_DelayUs(ADS1220_T_DATA_US);

  st = SPI_Driver_Transfer(chip, tx, rx, 3u);
  if (st != SPI_DRV_OK)
  {
    return st;
  }

  *code = ADS1220_RawToCode(rx);
  if (raw != NULL)
  {
    raw[0] = rx[0];
    raw[1] = rx[1];
    raw[2] = rx[2];
  }

  return SPI_DRV_OK;
}

ADS1220_Status_t ADS1220_ReadDataByCmd(uint8_t chip, int32_t *code, uint8_t raw[3])
{
  /* RDATA(0x10) 之后器件从输出移位寄存器吐出 3 个数据字节。
     注意: 命令字节那 8 个 SCLK 期间 DOUT 上出现的是**上一次**装载的结果
     (连续转换模式下 DOUT 一直在输出最近一次结果), 所以有效数据取 rx[1..3]。 */
  uint8_t tx[4] = { ADS1220_CMD_RDATA, 0x00u, 0x00u, 0x00u };
  uint8_t rx[4] = { 0x00u, 0x00u, 0x00u, 0x00u };
  ADS1220_Status_t st;

  if (code == NULL)
  {
    return SPI_DRV_ERR_PARAM;
  }

  SPI_Driver_DelayUs(ADS1220_T_DATA_US);

  st = SPI_Driver_Transfer(chip, tx, rx, 4u);
  if (st != SPI_DRV_OK)
  {
    return st;
  }

  *code = ADS1220_RawToCode(&rx[1]);
  if (raw != NULL)
  {
    raw[0] = rx[1];
    raw[1] = rx[2];
    raw[2] = rx[3];
  }

  return SPI_DRV_OK;
}

/*==============================================================================
 * 批量操作
 *============================================================================*/
ADS1220_Status_t ADS1220_ResetAll(void)
{
  ADS1220_Status_t st;
  ADS1220_Status_t first_err = SPI_DRV_OK;
  uint8_t chip;

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    st = ADS1220_ResetChip(chip);
    if ((st != SPI_DRV_OK) && (first_err == SPI_DRV_OK))
    {
      first_err = st;
    }
  }

  return first_err;
}

ADS1220_Status_t ADS1220_WriteRegsAll(uint8_t start, const uint8_t *vals, uint8_t cnt)
{
  ADS1220_Status_t st;
  ADS1220_Status_t first_err = SPI_DRV_OK;
  uint8_t chip;

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    st = ADS1220_WriteRegs(chip, start, vals, cnt);
    if ((st != SPI_DRV_OK) && (first_err == SPI_DRV_OK))
    {
      first_err = st;
    }
  }

  return first_err;
}

ADS1220_Status_t ADS1220_WriteRegAll(uint8_t addr, uint8_t val)
{
  return ADS1220_WriteRegsAll(addr, &val, 1u);
}

ADS1220_Status_t ADS1220_StartAll(void)
{
  ADS1220_Status_t st;
  ADS1220_Status_t first_err = SPI_DRV_OK;
  uint8_t chip;

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    st = ADS1220_Start(chip);
    if ((st != SPI_DRV_OK) && (first_err == SPI_DRV_OK))
    {
      first_err = st;
    }
  }

  return first_err;
}

ADS1220_Status_t ADS1220_PowerDownAll(void)
{
  ADS1220_Status_t st;
  ADS1220_Status_t first_err = SPI_DRV_OK;
  uint8_t chip;

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    st = ADS1220_PowerDown(chip);
    if ((st != SPI_DRV_OK) && (first_err == SPI_DRV_OK))
    {
      first_err = st;
    }
  }

  return first_err;
}

ADS1220_Status_t ADS1220_ReadDataAll(int32_t code[ADS1220_CHIP_COUNT])
{
  ADS1220_Status_t st;
  ADS1220_Status_t first_err = SPI_DRV_OK;
  uint8_t chip;

  if (code == NULL)
  {
    return SPI_DRV_ERR_PARAM;
  }

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    st = ADS1220_ReadData(chip, &code[chip], NULL);
    if (st != SPI_DRV_OK)
    {
      code[chip] = 0;                       /* 出错时给一个安全值 */
      if (first_err == SPI_DRV_OK)
      {
        first_err = st;
      }
    }
  }

  return first_err;
}

/*==============================================================================
 * 配置下发
 *============================================================================*/
ADS1220_Status_t ADS1220_SetConfigAll(const ADS1220_Config_t *cfg)
{
  uint8_t regs[ADS1220_REG_COUNT];
  ADS1220_Status_t st;

  if (cfg == NULL)
  {
    return SPI_DRV_ERR_PARAM;
  }

  ADS1220_BuildRegs(cfg, regs);
  st = ADS1220_WriteRegsAll(ADS1220_REG_CONFIG0, regs, ADS1220_REG_COUNT);

  ADS1220_StoreConfig(cfg);

  return st;
}

void ADS1220_GetConfig(ADS1220_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  if (s_cfg_valid != 0u)
  {
    *cfg = s_cfg;
  }
  else
  {
    ADS1220_DefaultConfig(cfg);   /* 还没初始化过就返回默认配置 */
  }
}

ADS1220_Status_t ADS1220_SetMuxAll(uint8_t mux)
{
  uint8_t reg0;

  if (s_cfg_valid == 0u)
  {
    return SPI_DRV_ERR_PARAM;
  }

  s_cfg.mux = (uint8_t)(mux & 0xF0u);

  reg0 = (uint8_t)((s_cfg.mux        & 0xF0u) |
                   (s_cfg.gain       & 0x0Eu) |
                   (s_cfg.pga_bypass & 0x01u));

  return ADS1220_WriteRegAll(ADS1220_REG_CONFIG0, reg0);
}

ADS1220_Status_t ADS1220_SetTempSensorAll(uint8_t enable)
{
  uint8_t reg1;

  if (s_cfg_valid == 0u)
  {
    return SPI_DRV_ERR_PARAM;
  }

  s_cfg.ts = (enable != 0u) ? ADS1220_TS_TEMPERATURE : ADS1220_TS_ADC;

  reg1 = (uint8_t)((s_cfg.dr   & 0xE0u) |
                   (s_cfg.mode & 0x18u) |
                   (s_cfg.cm   & 0x04u) |
                   (s_cfg.ts   & 0x02u) |
                   (s_cfg.bcs  & 0x01u));

  return ADS1220_WriteRegAll(ADS1220_REG_CONFIG1, reg1);
}

ADS1220_Status_t ADS1220_VerifyAll(const uint8_t written[ADS1220_REG_COUNT],
                                   uint16_t *ok_mask)
{
  ADS1220_Status_t st;
  ADS1220_Status_t first_err = SPI_DRV_OK;
  uint8_t  rb[ADS1220_REG_COUNT];
  uint16_t mask = 0u;
  uint8_t  chip;
  uint8_t  i;
  uint8_t  same;

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    st = ADS1220_ReadRegs(chip, ADS1220_REG_CONFIG0, rb, ADS1220_REG_COUNT);
    if (st != SPI_DRV_OK)
    {
      if (first_err == SPI_DRV_OK)
      {
        first_err = st;
      }
      continue;                      /* 该片标记为"未响应" */
    }

    if (written == NULL)
    {
      mask |= (uint16_t)(1u << chip);
      continue;
    }

    same = 1u;
    for (i = 0u; i < ADS1220_REG_COUNT; i++)
    {
      /* CONFIG3 的 bit0 是保留位, 永远读 0, 一并比对没问题;
         这里逐字节严格比对 —— 只要有一片没焊/没响应, MISO 被下拉到 0,
         读回值必然对不上, 从而被识别出来。 */
      if (rb[i] != written[i])
      {
        same = 0u;
        break;
      }
    }

    if (same != 0u)
    {
      mask |= (uint16_t)(1u << chip);
    }
  }

  if (ok_mask != NULL)
  {
    *ok_mask = mask;
  }

  return first_err;
}

/*==============================================================================
 * 上电初始化序列
 *============================================================================*/
ADS1220_Status_t ADS1220_PowerUpInit(const ADS1220_Config_t *cfg, uint16_t *ok_mask)
{
  ADS1220_Config_t c;
  uint8_t  regs[ADS1220_REG_COUNT];
  ADS1220_Status_t st;
  ADS1220_Status_t first_err = SPI_DRV_OK;
  uint16_t mask = 0u;

  if (cfg == NULL)
  {
    ADS1220_DefaultConfig(&c);
  }
  else
  {
    c = *cfg;
  }

  /* 1) 上电等待: 手册要求上电复位 >= 50us; 这里等 50ms 让
   *    TPS7A4901 软启动 + 磁珠后的 AVDD/DVDD 与内部基准彻底稳定。 */
  SPI_Driver_ReleaseAllCs();
  SPI_Driver_DelayMs(ADS1220_T_POWERUP_MS);

  /* 2) 逐片 RESET (0x06) */
  st = ADS1220_ResetAll();
  if (st != SPI_DRV_OK)
  {
    first_err = st;
  }

  /* 手册 8.5.3.1: RESET 之后至少等 (50us + 32 x tCLK) = 57.8us
     才能发下一条指令。这里按需求等 ADS1220_T_RESET_US = 1ms。 */
  SPI_Driver_DelayUs(ADS1220_T_RESET_US);

  /* 3) 写 CONFIG0..CONFIG3 (一条 WREG 0x43 + 4 字节) */
  ADS1220_BuildRegs(&c, regs);
  st = ADS1220_WriteRegsAll(ADS1220_REG_CONFIG0, regs, ADS1220_REG_COUNT);
  if ((st != SPI_DRV_OK) && (first_err == SPI_DRV_OK))
  {
    first_err = st;
  }

  /* 4) 回读校验 (手册 9.1.6 伪代码里的 optional sanity check) */
  st = ADS1220_VerifyAll(regs, &mask);
  if ((st != SPI_DRV_OK) && (first_err == SPI_DRV_OK))
  {
    first_err = st;
  }

  ADS1220_StoreConfig(&c);

  /* 5) 启动连续转换。
   *    连续模式下 START/SYNC 只需发一次; 之后再发会复位数字滤波器并重新开始。 */
  st = ADS1220_StartAll();
  if ((st != SPI_DRV_OK) && (first_err == SPI_DRV_OK))
  {
    first_err = st;
  }

  if (ok_mask != NULL)
  {
    *ok_mask = mask;
  }

  return first_err;
}

/*==============================================================================
 * 内部短路失调校准 (手册 9.1.5 / 8.3.12)
 *   把 AINP/AINN 内部短接到 (AVDD+AVSS)/2, 采多次求平均, 之后每次读数减掉。
 *   这是 ADS1220 上"自校准"的正确做法 —— 器件本身没有 0x04 自校准指令。
 *============================================================================*/
ADS1220_Status_t ADS1220_OffsetCalibrateAll(const ADS1220_Config_t *cfg,
                                            int32_t offset_code[ADS1220_CHIP_COUNT],
                                            uint8_t samples)
{
  ADS1220_Config_t c;
  ADS1220_Config_t restore;
  uint8_t  regs[ADS1220_REG_COUNT];
  int32_t  code[ADS1220_CHIP_COUNT];
  int64_t  acc[ADS1220_CHIP_COUNT];
  int32_t  conv_ms;
  int32_t  min_ms;
  uint8_t  chip;
  uint8_t  i;
  uint8_t  ok;

  if ((offset_code == NULL) || (samples == 0u))
  {
    return SPI_DRV_ERR_PARAM;
  }

  if (cfg == NULL)
  {
    ADS1220_DefaultConfig(&c);
  }
  else
  {
    c = *cfg;
  }
  restore = c;

  for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
  {
    acc[chip] = 0;
    offset_code[chip] = 0;
  }

  /* 切到"内部短路"输入: MUX=1110b, TS=0。
     注意 CONFIG0 只在 TS=0 时有效。 */
  c.ts  = ADS1220_TS_ADC;
  c.mux = ADS1220_MUX_SHORTED;
  ADS1220_BuildRegs(&c, regs);

  (void)ADS1220_WriteRegsAll(ADS1220_REG_CONFIG0, regs, ADS1220_REG_COUNT);

  conv_ms = (int32_t)((ADS1220_ConversionTimeUs(c.dr) + 999u) / 1000u);
  min_ms  = (conv_ms * (int32_t)ADS1220_WAIT_MIN_NUM) / (int32_t)ADS1220_WAIT_MIN_DEN;
  if (min_ms < 1)
  {
    min_ms = 1;
  }

  for (i = 0u; i < samples; i++)
  {
    /* 每次都重新 START/SYNC: 复位数字滤波器, 保证是一份干净的新结果 */
    SPI_Driver_DrdyClearFlag();
    (void)ADS1220_StartAll();

    ok = ADS1220_WaitDataReadyBlocking((uint32_t)min_ms,
                                       (uint32_t)(conv_ms + (int32_t)ADS1220_WAIT_MARGIN_MS));
    if (ok == 0u)
    {
      break;              /* 等不到就放弃校准, 不阻塞开机 */
    }

    (void)ADS1220_ReadDataAll(code);
    for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
    {
      acc[chip] += (int64_t)code[chip];
    }
  }

  if (i > 0u)
  {
    for (chip = 0u; chip < ADS1220_CHIP_COUNT; chip++)
    {
      offset_code[chip] = (int32_t)(acc[chip] / (int64_t)i);
    }
  }

  /* 恢复原配置并重新启动转换 */
  (void)ADS1220_SetConfigAll(&restore);
  SPI_Driver_DrdyClearFlag();
  (void)ADS1220_StartAll();

  return (i > 0u) ? SPI_DRV_OK : SPI_DRV_ERR_TIMEOUT;
}

/*==============================================================================
 * 数值换算
 *============================================================================*/
float ADS1220_CodeToMicroVolt(int32_t code, uint8_t gain_bits)
{
  float gain = (float)ADS1220_GainValue(gain_bits);

  /* 1 LSB = (2 x VREF / Gain) / 2^24 = (VREF / Gain) / 2^23  (手册式 15)
     VREF = 2.048V, Gain = 1  =>  1 LSB = 0.244140625 uV */
  return ((float)code) * (2048000.0f / gain) / 8388608.0f;
}

float ADS1220_TempCodeToCelsius(int32_t code)
{
  uint32_t u = (uint32_t)code;
  int32_t  t14;

  /* 14bit 结果左对齐在 24bit 里, 低 10 位是"无关位"(手册 8.3.13 图 8-23)。
     右移 10 位取出 14bit; 用位运算显式符号扩展, 避免依赖有符号右移的实现定义行为。 */
  if ((u & 0x00800000u) != 0u)
  {
    t14 = (int32_t)((u >> 10) | 0xFFFFC000u);   /* 14bit 补码 -> 负数 */
  }
  else
  {
    t14 = (int32_t)(u >> 10);
  }

  /* 1 个 14bit LSB = 0.03125 °C */
  return ((float)t14) * 0.03125f;
}
