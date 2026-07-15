/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "spi.h"   /* 使用 ICM42688P_RawData_t */
/* USER CODE END Includes */

extern UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN Private defines */

/* 二进制数据帧:
 *   [0]      帧头 0xAA
 *   [1]      帧头 0x55
 *   [2..15]  7 个 int16 (大端): temp, ax, ay, az, gx, gy, gz
 *   [16]     校验和 = [2..15] 字节按和取低 8 位
 * 共 17 字节
 */
#define ICM_FRAME_HEAD0   0xAAU
#define ICM_FRAME_HEAD1   0x55U
#define ICM_FRAME_LEN     17U

/* VG6328A-MS 默认串口参数: 115200, 8N1
 * BLE 透传默认服务:
 *   Write  : FFE0 / FFE1
 *   Notify : FFE0 / FFE2
 */
#define VG6328A_AT_TIMEOUT_MS    300U
#define VG6328A_BOOT_DELAY_MS    300U
#define VG6328A_BLE_NOTIFY_MTU   20U

/* USER CODE END Private defines */

void MX_LPUART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

HAL_StatusTypeDef VG6328A_BLE_Init(void);
HAL_StatusTypeDef VG6328A_SendData(const uint8_t *data, uint16_t len);
HAL_StatusTypeDef VG6328A_SendImuRaw(const ICM42688P_RawData_t *raw);
void LPUART1_SendFrame(const ICM42688P_RawData_t *raw);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

