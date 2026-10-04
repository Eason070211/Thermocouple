/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"
#include "board_config.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* ==========================================================================
   * ★ 从 MAX31855 换成 ADS1220 之后, SPI 配置必须改这几项:
   *
   *  1) Direction: RXONLY  ->  2LINES (全双工)
   *     ADS1220 是"有寄存器、要发指令"的器件, 必须能发 MOSI(PA7)。
   *
   *  2) CLKPhase: SPI_PHASE_1EDGE(Mode0)  ->  SPI_PHASE_2EDGE(Mode1)
   *     数据手册 SBAS501D 6.6 节:
   *       tsu(DI)/th(DI) 是相对 **SCLK 下降沿** 定义的 -> DIN 在下降沿被锁存
   *       tp(SCDO) 是 "SCLK **上升沿** 到 DOUT 输出新数据"  -> DOUT 在上升沿更新
   *     STM32 的 Mode1 (CPOL=0, CPHA=1) 就是"第一个(上升)沿输出, 第二个(下降)沿采样",
   *     与上面完全一致。Mode0 会在错误的边沿采样, 读回的数据整体错位。
   *
   *  3) BaudRatePrescaler: /32 -> /16, SCK = APB2/16 = 72MHz/16 = 4.5MHz
   *     手册 tc(SC) >= 150ns, 即 f_SCLK <= 6.67MHz。需求里写的 "<=10MHz" 会超规格,
   *     这里按数据手册取 4.5MHz (48% 裕量, 且是整数分频)。1 字节=8bit 约 1.8us。
   * ========================================================================== */
  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;      /* ★ 全双工, 需要 MOSI(PA7) */
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;          /* ADS1220 按字节收发 */
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;        /* CPOL = 0 */
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;            /* ★ CPHA = 1 -> SPI Mode 1 */
  hspi1.Init.NSS = SPI_NSS_SOFT;                    /* 片选由 GPIO 手动管理 (逐片) */
  hspi1.Init.BaudRatePrescaler = ADS1220_SPI_BAUDRATE_PSC;  /* /16 -> 4.5MHz */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;           /* ADS1220 先出 MSB */
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;      /* F1: 50MHz */
    HAL_GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStruct);

    /* MISO 配成输入 + 下拉:
       - 多片 DOUT 并在一根线上, 未被选中的片子是高阻(手册 8.5.1.5);
       - 下拉让"某片没焊/没响应"时读回确定的全 0, 而不是随机值,
         配合 ads1220.c 里的回读校验就能把坏片识别出来。 */
    GPIO_InitStruct.Pin = SPI1_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(SPI1_GPIO_PORT, SPI1_GPIO_PINS);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
