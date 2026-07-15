/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */

/* ---------------------------------------------------------------------------*/
/* ICM-42688-P 片选 (CS) 引脚: 根据接线 AP_CS 接到 PA9                         */
/* ---------------------------------------------------------------------------*/
#define ICM_CS_PORT              GPIOA
#define ICM_CS_PIN               GPIO_PIN_9

/* WHO_AM_I 期望值 */
#define ICM42688P_WHOAMI_VALUE   0x47U

/* User Bank 0 寄存器地址 */
#define ICM_REG_DEVICE_CONFIG    0x11U
#define ICM_REG_INT_CONFIG       0x14U
#define ICM_REG_TEMP_DATA1       0x1DU  /* 数据区起始: TEMP/ACCEL/GYRO 连续 14 字节 */
#define ICM_REG_INT_STATUS       0x2DU
#define ICM_REG_PWR_MGMT0        0x4EU
#define ICM_REG_GYRO_CONFIG0     0x4FU
#define ICM_REG_ACCEL_CONFIG0    0x50U
#define ICM_REG_INT_CONFIG1      0x64U
#define ICM_REG_INT_SOURCE0      0x65U
#define ICM_REG_WHO_AM_I         0x75U
#define ICM_REG_BANK_SEL         0x76U

/* 寄存器配置值: SPI Mode 0/3, ±2000dps, ±16g, 1kHz, Accel/Gyro LN */
#define ICM_DEVICE_CONFIG_RESET  0x01U
#define ICM_BANK_SEL_BANK0       0x00U
#define ICM_GYRO_CONFIG_2000DPS_1KHZ   0x06U
#define ICM_ACCEL_CONFIG_16G_1KHZ       0x06U
#define ICM_PWR_MGMT0_ACCEL_GYRO_LN    0x0FU

/* INT1: 推挽、高有效、脉冲模式。与 PA4 GPIO_MODE_IT_RISING 匹配 */
#define ICM_INT_CONFIG_INT1_PP_ACTIVE_HIGH 0x03U
#define ICM_INT_CONFIG1_ASYNC_RESET_CLEAR  0x00U
#define ICM_INT_SOURCE0_DRDY_INT1          0x08U

/* INT_STATUS 位 */
#define ICM_INT_STATUS_RESET_DONE 0x10U
#define ICM_INT_STATUS_DATA_RDY  0x08U

/* 超时/启动等待 */
#define ICM_RESET_DONE_TIMEOUT_MS 50U
#define ICM_GYRO_STARTUP_TIME_MS  45U

/* 灵敏度换算 (与初始化里选择的量程一致)
 *   陀螺   ±2000 dps -> 16.4 LSB/(°/s)
 *   加速度 ±16 g     -> 2048 LSB/g
 *   温度   degC = (raw / 132.48) + 25
 */
#define ICM_GYRO_SENS_2000DPS    16.4f
#define ICM_ACCEL_SENS_16G       2048.0f
#define ICM_TEMP_SENSITIVITY     132.48f
#define ICM_TEMP_OFFSET          25.0f

/* 原始数据 (16-bit, 二进制补码) */
typedef struct
{
  int16_t temp;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} ICM42688P_RawData_t;

/* 换算为物理量 */
typedef struct
{
  float temp_c;        /* °C   */
  float accel_g[3];    /* g    (x,y,z) */
  float gyro_dps[3];   /* °/s  (x,y,z) */
} ICM42688P_ScaledData_t;

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

HAL_StatusTypeDef ICM42688P_Init(void);
HAL_StatusTypeDef ICM42688P_ReadRaw(ICM42688P_RawData_t *raw);
HAL_StatusTypeDef ICM42688P_ReadStatus(uint8_t *status);
uint8_t           ICM42688P_IsDataReady(void);
void              ICM42688P_Scale(const ICM42688P_RawData_t *raw, ICM42688P_ScaledData_t *out);
uint8_t           ICM42688P_WhoAmI(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

