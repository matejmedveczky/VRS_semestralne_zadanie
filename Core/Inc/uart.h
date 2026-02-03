/*
 * uart.h
 *
 *  Created on: Jan 27, 2026
 *      Author: spear
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <string.h>


typedef struct {
  float v_linear;
  float v_angular;
  uint32_t last_ms;
  bool valid;
} CmdVel;

#define SOF1 0xA5
#define SOF2 0x5A
#define PAYLOAD_LEN 8
#define FRAME_LEN (3 + PAYLOAD_LEN + 2)

#define SOF1_F32 0xA1
#define SOF2_F32 0x1A

#define F32_PAYLOAD_LEN 4
#define F32_FRAME_LEN (3 + F32_PAYLOAD_LEN + 2)

#define RX_BUFFER_SIZE 64

void uart_start(UART_HandleTypeDef *huart);

uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len);


bool read_circ_buffer(uint8_t *processBuffer, uint16_t sizeBuffer);

bool drop_circ_buffer( uint16_t sizeBuffer);

void process_received_data(uint8_t *buffer);

void USART2_Interupt_Funk(void);

void uart_check(void);

void uart_write_float_output(float value);

float uart_read_linear_input(void);
float uart_read_angular_input(void);




#endif /* INC_UART_H_ */
