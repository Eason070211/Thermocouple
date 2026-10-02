/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"
#include "board_config.h"
#include "spi_driver.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();   /* PD0/PD1 走 HSE 晶振 (由 RCC 接管) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN 2 */
  /* ==========================================================================
   * 本工程所有的普通 GPIO 都集中在 SPI_Driver_GpioInit() 里, 由它统一配置:
   *
   *   PB0..PB3  : 4 路片选 CS0..CS3, 推挽输出(50MHz), 初始高电平 = 全部释放
   *               (具体几条由 board_config.h 的 ADS1220_CHIP_COUNT 决定;
   *                >= 4 条时会用到 PB3, 所以那里还会关掉 JTAG 只留 SWD)
   *   PA0..PA3  : 每片一根 DRDY, 输入 + 内部上拉, 主循环轮询
   *               (没有外部逻辑门, 也不再使用 EXTI 中断)
   *
   * 放在 spi_driver.c 是为了保证"引脚定义"只有一处 (board_config.h 提供宏),
   * 而这里的调用放在 USER CODE 区, CubeMX 重新生成工程也不会被删掉。
   *
   * ⚠️ 历史记录: MAX31855 版本用 PA0..PA3 做片选; 16 片 ADS1220 版本把片选搬到
   *          PB0..PB15 并把合并 DRDY 放 PA0。现在是 8 路(4 片)版本:
   *          CS = PB0..PB3, DRDY = PA0..PA3 (每片一根)。
   *          这是换芯片/换拓扑后最容易漏掉的一处改动。
   * ========================================================================== */
  SPI_Driver_GpioInit();
  /* USER CODE END 2 */

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
