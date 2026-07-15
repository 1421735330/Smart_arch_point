/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stddef.h>
#include <string.h>

static uint8_t VG6328A_ResponseContains(const uint8_t *buf, uint16_t len, const char *text)
{
  uint16_t i;
  uint16_t j;
  uint16_t text_len;

  if ((buf == NULL) || (text == NULL))
  {
    return 0U;
  }

  text_len = (uint16_t)strlen(text);
  if ((text_len == 0U) || (len < text_len))
  {
    return 0U;
  }

  for (i = 0U; i <= (uint16_t)(len - text_len); i++)
  {
    for (j = 0U; j < text_len; j++)
    {
      if (buf[i + j] != (uint8_t)text[j])
      {
        break;
      }
    }

    if (j == text_len)
    {
      return 1U;
    }
  }

  return 0U;
}

static void VG6328A_FlushRx(void)
{
  uint8_t dump;

  while (HAL_UART_Receive(&hlpuart1, &dump, 1U, 1U) == HAL_OK)
  {
  }
}

static HAL_StatusTypeDef VG6328A_ReadResponse(uint8_t *buf,
                                              uint16_t buf_len,
                                              uint16_t *out_len,
                                              const char *expect,
                                              uint32_t timeout_ms)
{
  uint32_t start_ms;
  uint16_t len = 0U;
  uint8_t byte;

  if ((buf == NULL) || (buf_len == 0U))
  {
    return HAL_ERROR;
  }

  start_ms = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms)
  {
    if (HAL_UART_Receive(&hlpuart1, &byte, 1U, 5U) == HAL_OK)
    {
      if (len < buf_len)
      {
        buf[len] = byte;
        len++;
      }

      if ((expect != NULL) && VG6328A_ResponseContains(buf, len, expect))
      {
        if (out_len != NULL)
        {
          *out_len = len;
        }
        return HAL_OK;
      }
    }
  }

  if (out_len != NULL)
  {
    *out_len = len;
  }

  return ((expect == NULL) ? HAL_OK : HAL_TIMEOUT);
}

static HAL_StatusTypeDef VG6328A_SendAT(const char *cmd, const char *expect, uint32_t timeout_ms)
{
  static const uint8_t suffix[3] = {'\r', '\n', 0x00U};
  uint8_t rx[96];

  if (cmd == NULL)
  {
    return HAL_ERROR;
  }

  VG6328A_FlushRx();

  if (HAL_UART_Transmit(&hlpuart1, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_UART_Transmit(&hlpuart1, (uint8_t *)suffix, sizeof(suffix), 100U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return VG6328A_ReadResponse(rx, sizeof(rx), NULL, expect, timeout_ms);
}

static HAL_StatusTypeDef VG6328A_QueryContains(const char *cmd, const char *expect)
{
  return VG6328A_SendAT(cmd, expect, VG6328A_AT_TIMEOUT_MS);
}

static HAL_StatusTypeDef VG6328A_EnsureSetting(const char *query_cmd,
                                               const char *query_expect,
                                               const char *set_cmd,
                                               uint8_t *changed)
{
  if (VG6328A_QueryContains(query_cmd, query_expect) == HAL_OK)
  {
    return HAL_OK;
  }

  if (VG6328A_SendAT(set_cmd, "+OK", VG6328A_AT_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (changed != NULL)
  {
    *changed = 1U;
  }

  return HAL_OK;
}

/* USER CODE END 0 */

UART_HandleTypeDef hlpuart1;

/* LPUART1 init function */

void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspInit 0 */

  /* USER CODE END LPUART1_MspInit 0 */
    /* LPUART1 clock enable */
    __HAL_RCC_LPUART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**LPUART1 GPIO Configuration
    PA2     ------> LPUART1_TX
    PA3     ------> LPUART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_LPUART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN LPUART1_MspInit 1 */

  /* USER CODE END LPUART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspDeInit 0 */

  /* USER CODE END LPUART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPUART1_CLK_DISABLE();

    /**LPUART1 GPIO Configuration
    PA2     ------> LPUART1_TX
    PA3     ------> LPUART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN LPUART1_MspDeInit 1 */

  /* USER CODE END LPUART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  初始化 VG6328A-MS 为 BLE 从机透传模式
  * @note   手册默认 BLE 透传服务: FFE0/FFE1(write), FFE0/FFE2(notify)
  *         设置项掉电保存, 因此先查询, 不一致时才写入。
  */
HAL_StatusTypeDef VG6328A_BLE_Init(void)
{
  uint8_t changed = 0U;

  HAL_Delay(VG6328A_BOOT_DELAY_MS);

  if (VG6328A_SendAT("AT", "+OK", VG6328A_AT_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (VG6328A_EnsureSetting("AT+BTM?", "+BTM:0", "AT+BTM=0", &changed) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (VG6328A_EnsureSetting("AT+ADE?", "+ADE:1", "AT+ADE=1", &changed) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (VG6328A_EnsureSetting("AT+ULEN?", "+ULEN:16", "AT+ULEN=16", &changed) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (changed != 0U)
  {
    (void)VG6328A_SendAT("AT+RST=1", "+OK", VG6328A_AT_TIMEOUT_MS);
    HAL_Delay(VG6328A_BOOT_DELAY_MS);
  }

  return HAL_OK;
}

/**
  * @brief  通过 VG6328A-MS BLE 透传发送数据
  * @note   手机侧订阅 FFE0/FFE2 notify 后即可接收。
  *         默认按 20 字节分包, 兼容未协商大 MTU 的 BLE 连接。
  */
HAL_StatusTypeDef VG6328A_SendData(const uint8_t *data, uint16_t len)
{
  uint16_t offset = 0U;
  uint16_t chunk_len;

  if ((data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  while (offset < len)
  {
    chunk_len = (uint16_t)(len - offset);
    if (chunk_len > VG6328A_BLE_NOTIFY_MTU)
    {
      chunk_len = VG6328A_BLE_NOTIFY_MTU;
    }

    if (HAL_UART_Transmit(&hlpuart1, (uint8_t *)&data[offset], chunk_len, 100U) != HAL_OK)
    {
      return HAL_ERROR;
    }

    offset = (uint16_t)(offset + chunk_len);
  }

  return HAL_OK;
}

/**
  * @brief  打包一帧 IMU 原始数据并通过 BLE notify 透传发送
  * @note   帧格式: 0xAA 0x55 + 7个int16(大端) + 校验和, 共 17 字节
  */
HAL_StatusTypeDef VG6328A_SendImuRaw(const ICM42688P_RawData_t *raw)
{
  uint8_t frame[ICM_FRAME_LEN];
  int16_t values[7];
  uint8_t sum = 0;
  uint8_t i;

  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  values[0] = raw->temp;
  values[1] = raw->accel_x;
  values[2] = raw->accel_y;
  values[3] = raw->accel_z;
  values[4] = raw->gyro_x;
  values[5] = raw->gyro_y;
  values[6] = raw->gyro_z;

  frame[0] = ICM_FRAME_HEAD0;
  frame[1] = ICM_FRAME_HEAD1;

  /* 7 个 int16, 大端填充 */
  for (i = 0; i < 7; i++)
  {
    frame[2 + i * 2] = (uint8_t)(((uint16_t)values[i] >> 8) & 0xFFU);
    frame[3 + i * 2] = (uint8_t)((uint16_t)values[i] & 0xFFU);
  }

  /* 校验和: 数据区 [2..15] 累加取低 8 位 */
  for (i = 2; i < (ICM_FRAME_LEN - 1); i++)
  {
    sum = (uint8_t)(sum + frame[i]);
  }
  frame[ICM_FRAME_LEN - 1] = sum;

  return VG6328A_SendData(frame, ICM_FRAME_LEN);
}

/**
  * @brief  兼容旧调用名: 实际通过 VG6328A BLE 透传发送
  */
void LPUART1_SendFrame(const ICM42688P_RawData_t *raw)
{
  (void)VG6328A_SendImuRaw(raw);
}

/* USER CODE END 1 */

