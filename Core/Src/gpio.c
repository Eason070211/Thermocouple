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
   *   PB0..PB15 : 16 路片选 CS0..CS15, 推挽输出(50MHz), 初始高电平 = 全部释放
   *   PA0       : 16 片 DRDY 经 2x74HC30 + 74HC132 合并后的中断输入
   *               EXTI0 下降沿触发 + NVIC 使能
   *
   * 放在 spi_driver.c 是为了保证"引脚定义"只有一处 (board_config.h 提供宏),
   * 而这里的调用放在 USER CODE 区, CubeMX 重新生成工程也不会被删掉。
   *
   * ⚠️ 注意: MAX31855 版本用的是 PA0..PA3 做片选。现在片选全部搬到 PB0..PB15,
   *          PA0 让给合并 DRDY —— 这是换芯片/换拓扑后最容易漏掉的一处改动。
   * ========================================================================== */
  SPI_Driver_GpioInit();
  /* USER CODE END 2 */

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
