#include "uart.h"

/* Frame:
 * [0]  0xA5
 * [1]  0x5A
 * [2]  payload_len = 8
 * [3..6]  float v_linear  (LE)
 * [7..10] float v_angular (LE)
 * [11] CRC LSB (CRC-CCITT-FALSE over bytes 0..10)
 * [12] CRC MSB
 */

UART_HandleTypeDef *s_huart;

/* DMA RX circular buffer */
uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxBufferPos;
volatile uint16_t rxBufferPosNew;

/* Latest command */
static CmdVel s_latest = {0};

/* ================= CRC ================= */

uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

/* ================= UART INIT ================= */

void uart_start(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    s_huart = huart;

    rxBufferPos = 0;
    rxBufferPosNew = 0;

    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);


    __HAL_UART_CLEAR_IDLEFLAG(s_huart);
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);


    HAL_UART_Receive_DMA(s_huart, rxBuffer, RX_BUFFER_SIZE);

}

/* ================= CIRCULAR BUFFER ================= */

static uint16_t circ_available(void)
{
    if (rxBufferPosNew >= rxBufferPos)
        return rxBufferPosNew - rxBufferPos;
    else
        return RX_BUFFER_SIZE - rxBufferPos + rxBufferPosNew;
}

static bool circ_peek(uint8_t *dst, uint16_t len)
{
    if (circ_available() < len) return false;

    for (uint16_t i = 0; i < len; i++) {
        dst[i] = rxBuffer[(rxBufferPos + i) % RX_BUFFER_SIZE];
    }
    return true;
}

static void circ_drop(uint16_t len)
{
    rxBufferPos = (rxBufferPos + len) % RX_BUFFER_SIZE;
}

/* ================= FRAME PROCESS ================= */

static void process_frame(const uint8_t *buf)
{
    if (buf[0] != SOF1 || buf[1] != SOF2) return;
    if (buf[2] != PAYLOAD_LEN) return;

    uint16_t rx_crc =
        (uint16_t)buf[11] |
        ((uint16_t)buf[12] << 8);

    uint16_t calc =
        crc16_ccitt_false(buf, 3 + PAYLOAD_LEN);

    if (rx_crc != calc) return;

    memcpy(&s_latest.v_linear,  &buf[3], 4);
    memcpy(&s_latest.v_angular, &buf[7], 4);

    s_latest.last_ms = HAL_GetTick();
    s_latest.valid   = true;
}

/* ================= IRQ HANDLER ================= */

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(s_huart);

    if (!__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE))
        return;

    __HAL_UART_CLEAR_IDLEFLAG(s_huart);

    rxBufferPosNew =
        RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(s_huart->hdmarx);

    uint8_t frame[FRAME_LEN];

    /* Parse ALL available frames */
    while (circ_available() >= FRAME_LEN) {

        /* Look for SOF1 */
        if (!circ_peek(frame, 1))
            break;

        if (frame[0] != SOF1) {
            circ_drop(1);
            continue;
        }

        /* Full frame available? */
        if (!circ_peek(frame, FRAME_LEN))
            break;

        /* Validate SOF2 early */
        if (frame[1] != SOF2) {
            circ_drop(1);
            continue;
        }

        circ_drop(FRAME_LEN);
        process_frame(frame);
    }
}

/* ================= TIMEOUT CHECK ================= */

void uart_check(void)
{
    if (!s_latest.valid) return;

    if ((HAL_GetTick() - s_latest.last_ms) > 500U) {
        s_latest.valid = false;
        s_latest.v_linear  = 0.0f;
        s_latest.v_angular = 0.0f;
    }
}

/* ================= TX FLOAT FRAME ================= */

void uart_write_float_output(float value)
{

    if (s_huart->gState != HAL_UART_STATE_READY)
        return;

    uint8_t frame[F32_FRAME_LEN];

    frame[0] = SOF1_F32;
    frame[1] = SOF2_F32;
    frame[2] = F32_PAYLOAD_LEN;

    memcpy(&frame[3], &value, 4);

    uint16_t crc =
        crc16_ccitt_false(frame, 3 + F32_PAYLOAD_LEN);

    frame[7] = (uint8_t)(crc & 0xFF);
    frame[8] = (uint8_t)(crc >> 8);

    HAL_UART_Transmit(s_huart, frame, F32_FRAME_LEN, 100);
}

/* ================= READ API ================= */

float uart_read_linear_input(void)
{
    return s_latest.valid ? s_latest.v_linear : 0.0f;
}

float uart_read_angular_input(void)
{
    return s_latest.valid ? s_latest.v_angular : 0.0f;
}
