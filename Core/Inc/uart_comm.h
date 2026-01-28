#ifndef UART_COMM_H
#define UART_COMM_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  float v_linear;
  float v_angular;
  uint32_t last_ms;
  bool valid;
} CmdVel;

void UARTComm_Init(UART_HandleTypeDef *huart);
void UARTComm_OnUartIrq(UART_HandleTypeDef *huart);
void UARTComm_Process(void);

/* Optional helper for sending data (DMA TX, one-shot). */
HAL_StatusTypeDef UARTComm_TransmitDMA(const uint8_t *data, uint16_t size);

/* HAL callbacks implemented in uart_comm.c (DMA RX/TX completion). */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void UARTComm_SendFloat(float value);

CmdVel UARTComm_GetLatestCmd(void);

float read_linear_input(void);
float read_angular_input(void);

#endif
