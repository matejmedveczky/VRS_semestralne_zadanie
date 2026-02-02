/*
 * battery_monitor.c
 *
 *  Created on: Jan 29, 2026
 *      Author: benko
 */

#include "battery_monitor.h"
#include <string.h>

/* -------- HW konfigurácia -------- */

// odporový delič
#define R1_OHMS           (100000.0f)
#define R2_OHMS           (33000.0f)

// ADC
#define ADC_MAX_COUNTS    (4095.0f)     // 12-bit ADC
#define VDDA_VOLTS        (3.3f)        // neskôr sa dá nahradiť VREFINT

/* -------- 3S Li-Po napätia --------
   3S: full = 12.6V (4.2V/cell), safe empty ~9.6V (3.2V/cell)
*/
#define VBAT_FULL_VOLTS   (12.6f)
#define VBAT_EMPTY_VOLTS  (9.6f)

/* -------- Prahy batérie (praktické) --------
   WARN  ~3.5V/cell  => 10.5V
   CRIT  ~3.3V/cell  => 9.9V
   SHDN  ~3.2V/cell  => 9.6V
*/
#define VBAT_WARN_ON      (10.5f)
#define VBAT_WARN_OFF     (10.7f)

#define VBAT_CRIT_ON      (9.9f)
#define VBAT_CRIT_OFF     (10.1f)

#define VBAT_SHDN_ON      (9.6f)
#define VBAT_SHDN_OFF     (9.8f)

/* -------- Filter -------- */

#define VBAT_EMA_ALPHA    (0.05f)
#define VBAT_SAMPLES	  (30)

/* -------- Private state -------- */

static ADC_HandleTypeDef *s_hadc = NULL;
static float s_vbat_ema = 0.0f;
static bool  s_ema_init = false;

/* -------- Private helpers -------- */

static inline float DividerScale(void)
{
  return (R1_OHMS + R2_OHMS) / R2_OHMS;   // (47k+15k)/15k = 4.133333...
}

static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint8_t VBAT_ToPercent_Linear(float vbat)
{
  float p = (vbat - VBAT_EMPTY_VOLTS) / (VBAT_FULL_VOLTS - VBAT_EMPTY_VOLTS) * 100.0f;
  p = clampf(p, 0.0f, 100.0f);
  return (uint8_t)(p + 0.5f);
}

static bool ADC_ReadOnce_U16(uint16_t *out)
{
  if (s_hadc == NULL) return false;

  if (HAL_ADC_Start(s_hadc) != HAL_OK) return false;

  if (HAL_ADC_PollForConversion(s_hadc, 10) != HAL_OK) {
    (void)HAL_ADC_Stop(s_hadc);
    return false;
  }

  uint32_t val = HAL_ADC_GetValue(s_hadc);
  (void)HAL_ADC_Stop(s_hadc);

  if (val > 0xFFFFu) val = 0xFFFFu;
  *out = (uint16_t)val;
  return true;
}

static bool ADC_ReadAverage_U16(uint16_t *out, uint8_t n)
{
  if (n == 0) n = 1;

  uint32_t acc = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint16_t s = 0;
    if (!ADC_ReadOnce_U16(&s)) return false;
    acc += s;
    HAL_Delay(1);
  }
  *out = (uint16_t)(acc / n);
  return true;
}

static float VBAT_FromAdcCounts(uint16_t adc_counts)
{
  float v_adc = ((float)adc_counts / ADC_MAX_COUNTS) * VDDA_VOLTS;
  return v_adc * DividerScale();
}

static float VBAT_FilterEma(float vbat)
{
  if (!s_ema_init) {
    s_vbat_ema = vbat;
    s_ema_init = true;
  } else {
    s_vbat_ema = s_vbat_ema + VBAT_EMA_ALPHA * (vbat - s_vbat_ema);
  }
  return s_vbat_ema;
}

static vbat_state_t VBAT_UpdateState(float vbat_f)
{
  static vbat_state_t st = VBAT_OK;

  switch (st)
  {
    case VBAT_OK:
      if (vbat_f < VBAT_WARN_ON) st = VBAT_WARN;
      break;

    case VBAT_WARN:
      if (vbat_f < VBAT_CRIT_ON) st = VBAT_CRIT;
      else if (vbat_f > VBAT_WARN_OFF) st = VBAT_OK;
      break;

    case VBAT_CRIT:
      if (vbat_f < VBAT_SHDN_ON) st = VBAT_SHUTDOWN;
      else if (vbat_f > VBAT_CRIT_OFF) st = VBAT_WARN;
      break;

    case VBAT_SHUTDOWN:
      if (vbat_f > VBAT_SHDN_OFF) st = VBAT_CRIT;
      break;
  }

  return st;
}

static robot_limits_t Robot_LimitsForBattery(vbat_state_t st)
{
  robot_limits_t L;

  switch (st)
  {
    default:
    case VBAT_OK:
      L.max_speed = 1.0f;
      L.max_accel = 1.0f;
      L.max_pwm   = 1.0f;
      break;

    case VBAT_WARN:
      L.max_speed = 0.7f;
      L.max_accel = 0.6f;
      L.max_pwm   = 0.7f;
      break;

    case VBAT_CRIT:
      L.max_speed = 0.4f;
      L.max_accel = 0.3f;
      L.max_pwm   = 0.4f;
      break;

    case VBAT_SHUTDOWN:
      L.max_speed = 0.0f;
      L.max_accel = 0.0f;
      L.max_pwm   = 0.0f;
      break;
  }

  return L;
}

/* -------- Public API -------- */

void BatteryMon_Init(ADC_HandleTypeDef *hadc)
{
  s_hadc = hadc;
  s_vbat_ema = 0.0f;
  s_ema_init = false;
}

BatteryStatus BatteryMon_Update(uint8_t samples)
{
 (void)samples;

  BatteryStatus st;
  memset(&st, 0, sizeof(st));

  uint16_t adc = 0;
  if (!ADC_ReadAverage_U16(&adc, VBAT_SAMPLES)) {
    st.state = VBAT_OK;
    st.limits = Robot_LimitsForBattery(st.state);
    st.percent = 0; // fallback
    return st;
  }

  st.adc_raw = adc;
  st.vbat_raw = VBAT_FromAdcCounts(adc);
  st.vbat_filtered = VBAT_FilterEma(st.vbat_raw);
  st.state = VBAT_UpdateState(st.vbat_filtered);
  st.limits = Robot_LimitsForBattery(st.state);

  // Percentá z filtrovaného napätia (stabilnejšie)
  st.percent = VBAT_ToPercent_Linear(st.vbat_filtered);

  return st;
}

