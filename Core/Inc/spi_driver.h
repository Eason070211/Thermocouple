/**
  ******************************************************************************
  * @file    spi_driver.h
  * @brief   SPI1 总线层: 片选管理 / 事务收发 / 合并 DRDY 中断 / 延时
  *
  *  职责边界:
  *    - 只关心"怎么在 SPI1 上安全地跟某一片 ADS1220 说一句话"
  *    - 不关心 ADS1220 的寄存器含义 (那是 ads1220.c 的事)
  *
  *  提供的功能:
  *    SPI_Driver_GpioInit()   CS(PB0..15) + DRDY(PA0/EXTI0) 的 GPIO/EXTI/NVIC 初始化
  *    SPI_Driver_Transfer()   选中片 -> tCSSC -> 收发 -> tSCCS -> 释放片 -> tCSH
  *    SPI_Driver_DrdyXxx()    合并 DRDY 的"只置标志位"中断接口与查询接口
  *    SPI_Driver_DelayUs/Ms   DWT CYCCNT 精确微秒延时 (失败时退化为 NOP 循环)
  ******************************************************************************
  */

#ifndef __SPI_DRIVER_H
#define __SPI_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "board_config.h"

/** 总线层返回码 (不直接用 HAL_StatusTypeDef, 便于区分"超时"和"参数错误") */
typedef enum
{
  SPI_DRV_OK = 0,          /**< 成功 */
  SPI_DRV_ERR_PARAM,       /**< 片号越界 / 空指针 / 长度为 0 */
  SPI_DRV_ERR_TIMEOUT,     /**< HAL 超时 (总线无响应) */
  SPI_DRV_ERR_HAL          /**< HAL 报了 BUSY/ERROR */
} SPI_Driver_Status_t;

/*----------------------------------------------------------------------------
 * 初始化 / 延时
 *--------------------------------------------------------------------------*/
/** 配置 CS(PB0..PB15, 推挽 50MHz, 初始高) 与 DRDY(PA0, EXTI0 下降沿, 上拉)。
 *  在 MX_GPIO_Init() 里调用 (见 gpio.c), 这样 CubeMX 重新生成也不会丢。 */
void SPI_Driver_GpioInit(void);

/** 上电复位后 CS 全部拉高 (释放总线)。 */
void SPI_Driver_ReleaseAllCs(void);

/** 忙等延时。DelayUs 使用 DWT->CYCCNT, 精度 ~14ns @72MHz。 */
void SPI_Driver_DelayUs(uint32_t us);
void SPI_Driver_DelayMs(uint32_t ms);

/*----------------------------------------------------------------------------
 * SPI 事务
 *--------------------------------------------------------------------------*/
/**
  * @brief  对第 chip 片 ADS1220 做一次 SPI 事务
  * @param  chip  片号 0..ADS1220_CHIP_COUNT-1 (= PB0..PB15)
  * @param  tx    发送缓冲区, 必须非 NULL
  * @param  rx    接收缓冲区; 传 NULL 表示只写不读
  * @param  len   字节数 (1..255)
  * @retval SPI_Driver_Status_t
  * @note   内部自动处理 td(CSSC)/td(SCCS)/tw(CSH), 出错后自动恢复 SPI 外设。
  *         失败时 CS 仍会被正确释放, 不会把总线锁死。
  */
SPI_Driver_Status_t SPI_Driver_Transfer(uint8_t chip,
                                        const uint8_t *tx,
                                        uint8_t *rx,
                                        uint16_t len);

/** 总线自恢复: 若 HAL SPI 状态卡在 BUSY, 中止并重新使能外设。 */
void SPI_Driver_Recover(void);

/*----------------------------------------------------------------------------
 * 合并 DRDY (PA0 / EXTI0)
 *--------------------------------------------------------------------------*/
/** 在 EXTI0_IRQHandler 中调用 —— 只置标志位 + 计数, 绝不做 SPI。 */
void     SPI_Driver_DrdyIrqHandler(void);

/** 读取并清除"有新数据"标志 (主循环用)。返回 1 表示期间有过 DRDY 下降沿。 */
uint8_t  SPI_Driver_DrdyTakeFlag(void);

/** 只清除标志, 不读取。 */
void     SPI_Driver_DrdyClearFlag(void);

/** 读合并 DRDY 的当前电平。返回 0 表示"低有效" = 至少一片有数据。 */
uint8_t  SPI_Driver_DrdyLevel(void);

/** 累计的 DRDY 中断次数 (可用于判断 16 片是否都在正常转换)。 */
uint32_t SPI_Driver_DrdyIrqCount(void);

/** 累计的 SPI 事务失败次数 (超时/错误)。 */
uint32_t SPI_Driver_ErrorCount(void);

/** 最近一次 SPI 事务的失败片号 (0xFF = 无)。 */
uint8_t  SPI_Driver_LastErrorChip(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_DRIVER_H */
