#include "uart_comm.h"
#include <string.h>

/* Frame:
 * [0] 0xA5
 * [1] 0x5A
 * [2] payload_len = 8
 * [3..6]  float v_linear  (LE)
 * [7..10] float v_angular (LE)
 * [11] CRC LSB (CRC-CCITT-FALSE over bytes 0..10)
 * [12] CRC MSB
 */
#define SOF1 0xA5
#define SOF2 0x5A
#define PAYLOAD_LEN 8
#define FRAME_LEN (3 + PAYLOAD_LEN + 2)

#define RX_DMA_BUF_SZ 128
#define RB_SZ 256

/* TX uses a small staging buffer so caller can pass a temporary pointer. */
#define TX_DMA_BUF_SZ 64

static UART_HandleTypeDef *s_huart = NULL;

static uint8_t s_rx_dma[RX_DMA_BUF_SZ];
static volatile uint16_t s_dma_old_pos = 0;

static uint8_t s_tx_dma[TX_DMA_BUF_SZ];
static volatile uint8_t s_tx_busy = 0;

static uint8_t s_rb[RB_SZ];
static volatile uint16_t s_rb_w = 0;
static volatile uint16_t s_rb_r = 0;

static CmdVel s_latest = {0};

/* Forward declarations (used from DMA callbacks). */
static bool parse_one_frame(void);

/* Debug markers (watch in debugger) */
volatile int uart_init_step = 0;
volatile HAL_StatusTypeDef uart_init_st = HAL_OK;
volatile uint16_t uart_rx_last_chunk = 0; /* bytes received since last DMA event */

static uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void rb_push(uint8_t b)
{
  uint16_t next = (uint16_t)((s_rb_w + 1U) % RB_SZ);
  if (next == s_rb_r) {
    s_rb_r = (uint16_t)((s_rb_r + 1U) % RB_SZ);
  }
  s_rb[s_rb_w] = b;
  s_rb_w = next;
}

static bool rb_peek(uint16_t idx_from_r, uint8_t *out)
{
  uint16_t size = (s_rb_w >= s_rb_r) ? (uint16_t)(s_rb_w - s_rb_r)
                                     : (uint16_t)(RB_SZ - s_rb_r + s_rb_w);
  if (idx_from_r >= size) return false;
  uint16_t pos = (uint16_t)((s_rb_r + idx_from_r) % RB_SZ);
  *out = s_rb[pos];
  return true;
}

static bool rb_drop(uint16_t n)
{
  uint16_t size = (s_rb_w >= s_rb_r) ? (uint16_t)(s_rb_w - s_rb_r)
                                     : (uint16_t)(RB_SZ - s_rb_r + s_rb_w);
  if (n > size) return false;
  s_rb_r = (uint16_t)((s_rb_r + n) % RB_SZ);
  return true;
}

/* Drain NEW bytes from DMA circular buffer to ring buffer.
 * Returns how many bytes were drained.
 */
static uint16_t uartcomm_drain_dma_to_rb(UART_HandleTypeDef *huart)
{
  if (huart == NULL) return 0U;
  if (huart != s_huart) return 0U;
  if (s_huart == NULL || s_huart->hdmarx == NULL) return 0U;

  /* Where DMA is currently writing:
   * dma_pos is in range [0..RX_DMA_BUF_SZ-1]
   */
  uint16_t dma_pos = (uint16_t)(RX_DMA_BUF_SZ - __HAL_DMA_GET_COUNTER(s_huart->hdmarx));
  if (dma_pos == s_dma_old_pos) return 0U;

  uint16_t drained = 0U;

  if (dma_pos > s_dma_old_pos) {
    /* Linear region [old..pos) */
    for (uint16_t i = s_dma_old_pos; i < dma_pos; i++) {
      rb_push(s_rx_dma[i]);
      drained++;
    }
  } else {
    /* Wrapped: [old..end) + [0..pos) */
    for (uint16_t i = s_dma_old_pos; i < RX_DMA_BUF_SZ; i++) {
      rb_push(s_rx_dma[i]);
      drained++;
    }
    for (uint16_t i = 0; i < dma_pos; i++) {
      rb_push(s_rx_dma[i]);
      drained++;
    }
  }

  /* Remember where next bytes will arrive (circular) */
  s_dma_old_pos = dma_pos;

  return drained;
}

/* Handle a DMA RX event (half or full complete):
 * - figure out how many NEW bytes arrived since last processing
 * - push them to the ring buffer
 * - parse as many complete frames as possible
 */
static void uartcomm_on_dma_rx_event(UART_HandleTypeDef *huart)
{
  uart_rx_last_chunk = uartcomm_drain_dma_to_rb(huart);
  while (parse_one_frame()) {
    /* keep parsing */
  }
}

void UARTComm_Init(UART_HandleTypeDef *huart)
{
  uart_init_step = 1;
  s_huart = huart;

  uart_init_step = 2;
  s_dma_old_pos = 0;
  s_rb_w = s_rb_r = 0;
  memset(&s_latest, 0, sizeof(s_latest));
  s_tx_busy = 0;

  uart_init_step = 4;
  __HAL_UART_CLEAR_OREFLAG(s_huart);
  __HAL_UART_CLEAR_NEFLAG(s_huart);
  __HAL_UART_CLEAR_FEFLAG(s_huart);
  __HAL_UART_CLEAR_PEFLAG(s_huart);

  uart_init_step = 5;
  if (s_huart->hdmarx == NULL) {
    uart_init_step = -100;  /* DMA RX not linked */
    s_latest.valid = false;
    return;
  }

  /* Start RX DMA into circular buffer */
  uart_init_step = 6;
  uart_init_st = HAL_UART_Receive_DMA(s_huart, s_rx_dma, RX_DMA_BUF_SZ);

  uart_init_step = 7;
  if (uart_init_st != HAL_OK) {
    uart_init_step = -200; /* Receive_DMA fail */
    s_latest.valid = false;
    return;
  }

  uart_init_step = 8;
  /* We want callbacks on both HT and TC, because RX DMA is circular. */
  __HAL_DMA_ENABLE_IT(s_huart->hdmarx, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(s_huart->hdmarx, DMA_IT_TC);

  uart_init_step = 9;
}

HAL_StatusTypeDef UARTComm_TransmitDMA(const uint8_t *data, uint16_t size)
{
  if (s_huart == NULL || data == NULL) return HAL_ERROR;
  if (size == 0U) return HAL_OK;

  if (s_tx_busy) return HAL_BUSY;
  if (size > TX_DMA_BUF_SZ) return HAL_ERROR;

  memcpy(s_tx_dma, data, size);
  s_tx_busy = 1;
  return HAL_UART_Transmit_DMA(s_huart, s_tx_dma, size);
}

/* HAL callback: TX done */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == s_huart) {
    s_tx_busy = 0;
  }
}

/* HAL callback: RX half-transfer complete (circular DMA).
 * This fires when DMA reaches RX_DMA_BUF_SZ/2.
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_huart) return;
  uartcomm_on_dma_rx_event(huart);
}

/* HAL callback: RX transfer complete (circular DMA).
 * This fires when DMA reaches RX_DMA_BUF_SZ and wraps back to 0.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_huart) return;
  uartcomm_on_dma_rx_event(huart);
}

void UARTComm_OnUartIrq(UART_HandleTypeDef *huart)
{
  (void)huart;
  /* Not used right now. Reception is handled via DMA callbacks
   * (HAL_UART_RxHalfCpltCallback / HAL_UART_RxCpltCallback).
   */
}

static bool parse_one_frame(void)
{
  while (1) {
    uint8_t b0;
    if (!rb_peek(0, &b0)) return false;
    if (b0 == SOF1) break;
    (void)rb_drop(1);
  }

  uint8_t buf[FRAME_LEN];
  for (uint16_t i = 0; i < FRAME_LEN; i++) {
    if (!rb_peek(i, &buf[i])) return false;
  }

  if (buf[0] != SOF1 || buf[1] != SOF2) { (void)rb_drop(1); return true; }
  if (buf[2] != PAYLOAD_LEN)            { (void)rb_drop(2); return true; }

  uint16_t rx_crc = (uint16_t)buf[11] | ((uint16_t)buf[12] << 8);
  uint16_t calc   = crc16_ccitt_false(buf, 11);
  if (rx_crc != calc) { (void)rb_drop(1); return true; }

  float vlin = 0.0f, vang = 0.0f;
  memcpy(&vlin, &buf[3], 4);
  memcpy(&vang, &buf[7], 4);

  s_latest.v_linear  = vlin;
  s_latest.v_angular = vang;
  s_latest.last_ms   = HAL_GetTick();
  s_latest.valid     = true;

  (void)rb_drop(FRAME_LEN);
  return true;
}

void UARTComm_Process(void)
{
  /* If for any reason a callback was missed, we still drain here. */
  (void)uartcomm_drain_dma_to_rb(s_huart);
  while (parse_one_frame()) {
    /* keep parsing */
  }

  if (s_latest.valid) {
    uint32_t now = HAL_GetTick();
    if ((now - s_latest.last_ms) > 500U) {
      s_latest.valid = false;
      s_latest.v_linear = 0.0f;
      s_latest.v_angular = 0.0f;
    }
  }
}

CmdVel UARTComm_GetLatestCmd(void) { return s_latest; }

float read_linear_input(void)  { return s_latest.valid ? s_latest.v_linear : 0.0f; }
float read_angular_input(void) { return s_latest.valid ? s_latest.v_angular : 0.0f; }
