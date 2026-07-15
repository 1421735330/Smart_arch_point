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

/* USER CODE BEGIN 0 */

/* ICM-42688-P SPI 读写超时 (ms) */
#define ICM_SPI_TIMEOUT   100U

/* --- 片选控制 --- */
static inline void ICM_CS_Low(void)
{
  HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
}

static inline void ICM_CS_High(void)
{
  HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
}

/**
  * @brief  写单个寄存器
  * @note   首字节 bit7=0 表示写, 低 7 位为寄存器地址
  */
static HAL_StatusTypeDef ICM_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];
  HAL_StatusTypeDef st;

  tx[0] = reg & 0x7FU;
  tx[1] = value;

  ICM_CS_Low();
  st = HAL_SPI_Transmit(&hspi1, tx, 2, ICM_SPI_TIMEOUT);
  ICM_CS_High();

  return st;
}

/**
  * @brief  读单个寄存器
  * @note   首字节 bit7=1 表示读
  */
static HAL_StatusTypeDef ICM_ReadReg(uint8_t reg, uint8_t *value)
{
  uint8_t tx = reg | 0x80U;
  HAL_StatusTypeDef st;

  ICM_CS_Low();
  st = HAL_SPI_Transmit(&hspi1, &tx, 1, ICM_SPI_TIMEOUT);
  if (st == HAL_OK)
  {
    st = HAL_SPI_Receive(&hspi1, value, 1, ICM_SPI_TIMEOUT);
  }
  ICM_CS_High();

  return st;
}

/**
  * @brief  连续读多个寄存器 (突发读, 地址自动递增)
  */
static HAL_StatusTypeDef ICM_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint8_t tx = reg | 0x80U;
  HAL_StatusTypeDef st;

  ICM_CS_Low();
  st = HAL_SPI_Transmit(&hspi1, &tx, 1, ICM_SPI_TIMEOUT);
  if (st == HAL_OK)
  {
    st = HAL_SPI_Receive(&hspi1, buf, len, ICM_SPI_TIMEOUT);
  }
  ICM_CS_High();

  return st;
}

/**
  * @brief  等待 INT_STATUS 中的指定状态位出现
  * @note   INT_STATUS 为 R/C, 读取后会清除已置位状态
  */
static HAL_StatusTypeDef ICM_WaitForStatus(uint8_t mask, uint32_t timeout_ms)
{
  uint32_t start_ms = HAL_GetTick();
  uint8_t status = 0U;

  do
  {
    if (ICM_ReadReg(ICM_REG_INT_STATUS, &status) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if ((status & mask) == mask)
    {
      return HAL_OK;
    }
  } while ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms);

  return HAL_TIMEOUT;
}

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
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
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  读取 WHO_AM_I, 正常返回 0x47
  */
uint8_t ICM42688P_WhoAmI(void)
{
  uint8_t id = 0x00U;
  (void)ICM_ReadReg(ICM_REG_WHO_AM_I, &id);
  return id;
}

/**
  * @brief  初始化 ICM-42688-P
  * @note   配置: 软复位 -> 校验 ID -> 量程(±2000dps/±16g)/ODR(1kHz)
  *               -> 加速度+陀螺 Low-Noise 模式 -> 开启数据就绪中断到 INT1
  * @retval HAL_OK 成功; HAL_ERROR 设备识别失败
  */
HAL_StatusTypeDef ICM42688P_Init(void)
{
  uint8_t id;
  uint8_t status;

  /* CS 默认拉高(空闲) */
  ICM_CS_High();
  HAL_Delay(10);

  /* 1. 软复位 (DEVICE_CONFIG.SOFT_RESET_CONFIG = 1), 之后需等待 >=1ms */
  if (ICM_WriteReg(ICM_REG_DEVICE_CONFIG, ICM_DEVICE_CONFIG_RESET) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(2);

  if (ICM_WaitForStatus(ICM_INT_STATUS_RESET_DONE, ICM_RESET_DONE_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 确保在 Bank 0 */
  if (ICM_WriteReg(ICM_REG_BANK_SEL, ICM_BANK_SEL_BANK0) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 2. 校验设备 ID */
  if (ICM_ReadReg(ICM_REG_WHO_AM_I, &id) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (id != ICM42688P_WHOAMI_VALUE)
  {
    return HAL_ERROR;
  }

  /* 3. 陀螺量程 ±2000dps(000) + ODR 1kHz(0110) -> 0x06 */
  if (ICM_WriteReg(ICM_REG_GYRO_CONFIG0, ICM_GYRO_CONFIG_2000DPS_1KHZ) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 4. 加速度量程 ±16g(000) + ODR 1kHz(0110) -> 0x06 */
  if (ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_CONFIG_16G_1KHZ) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 5. 电源管理: 陀螺 LN(11)、加速度 LN(11), TEMP 使能 -> 0x0F
   *    从 OFF 切到其它模式后 200us 内不要再写寄存器 */
  if (ICM_WriteReg(ICM_REG_PWR_MGMT0, ICM_PWR_MGMT0_ACCEL_GYRO_LN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(ICM_GYRO_STARTUP_TIME_MS);

  /* 清除复位/上电过程中残留的 R/C 状态位 */
  if (ICM_ReadReg(ICM_REG_INT_STATUS, &status) != HAL_OK)
  {
    return HAL_ERROR;
  }
  (void)status;

  /* 6. INT1 推挽、高有效、脉冲模式, 匹配 PA4 上升沿 EXTI */
  if (ICM_WriteReg(ICM_REG_INT_CONFIG, ICM_INT_CONFIG_INT1_PP_ACTIVE_HIGH) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 7. INT_CONFIG1: bit4 INT_ASYNC_RESET 需由默认 1 改为 0 (见数据手册 12.6) */
  if (ICM_WriteReg(ICM_REG_INT_CONFIG1, ICM_INT_CONFIG1_ASYNC_RESET_CLEAR) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 8. 数据就绪中断路由到 INT1: INT_SOURCE0.UI_DRDY_INT1_EN = bit3 */
  if (ICM_WriteReg(ICM_REG_INT_SOURCE0, ICM_INT_SOURCE0_DRDY_INT1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  读取 INT_STATUS
  * @note   INT_STATUS 为读清寄存器, 调用会清除已置位的中断状态
  */
HAL_StatusTypeDef ICM42688P_ReadStatus(uint8_t *status)
{
  if (status == NULL)
  {
    return HAL_ERROR;
  }

  return ICM_ReadReg(ICM_REG_INT_STATUS, status);
}

/**
  * @brief  查询当前是否有 UI 数据就绪事件
  * @note   读取 INT_STATUS 会清除 DATA_RDY_INT 位
  */
uint8_t ICM42688P_IsDataReady(void)
{
  uint8_t status = 0U;

  if (ICM42688P_ReadStatus(&status) != HAL_OK)
  {
    return 0U;
  }

  return ((status & ICM_INT_STATUS_DATA_RDY) != 0U) ? 1U : 0U;
}

/**
  * @brief  读取一帧原始数据 (温度 + 加速度 + 陀螺, 共 7x16bit)
  * @note   从 TEMP_DATA1(0x1D) 起突发读 14 字节, 大端存放(高字节在前)
  */
HAL_StatusTypeDef ICM42688P_ReadRaw(ICM42688P_RawData_t *raw)
{
  uint8_t buf[14];

  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  if (ICM_ReadRegs(ICM_REG_TEMP_DATA1, buf, sizeof(buf)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  raw->temp    = (int16_t)((buf[0]  << 8) | buf[1]);
  raw->accel_x = (int16_t)((buf[2]  << 8) | buf[3]);
  raw->accel_y = (int16_t)((buf[4]  << 8) | buf[5]);
  raw->accel_z = (int16_t)((buf[6]  << 8) | buf[7]);
  raw->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
  raw->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
  raw->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

  return HAL_OK;
}

/**
  * @brief  将原始数据换算为物理量 (g, °/s, °C)
  */
void ICM42688P_Scale(const ICM42688P_RawData_t *raw, ICM42688P_ScaledData_t *out)
{
  if ((raw == NULL) || (out == NULL))
  {
    return;
  }

  out->temp_c     = ((float)raw->temp / ICM_TEMP_SENSITIVITY) + ICM_TEMP_OFFSET;

  out->accel_g[0] = (float)raw->accel_x / ICM_ACCEL_SENS_16G;
  out->accel_g[1] = (float)raw->accel_y / ICM_ACCEL_SENS_16G;
  out->accel_g[2] = (float)raw->accel_z / ICM_ACCEL_SENS_16G;

  out->gyro_dps[0] = (float)raw->gyro_x / ICM_GYRO_SENS_2000DPS;
  out->gyro_dps[1] = (float)raw->gyro_y / ICM_GYRO_SENS_2000DPS;
  out->gyro_dps[2] = (float)raw->gyro_z / ICM_GYRO_SENS_2000DPS;
}

/* USER CODE END 1 */

