/*
 * battery_monitor.h
 *
 *  Created on: Jan 29, 2026
 *      Author: benko
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

typedef enum {
  VBAT_OK = 0,
  VBAT_WARN,
  VBAT_CRIT,
  VBAT_SHUTDOWN
} vbat_state_t;

typedef struct {
  float max_speed;      // 0..1
  float max_accel;      // 0..1
  float max_pwm;        // 0..1
  //bool  leds_enabled;
} robot_limits_t;

typedef struct {
  uint16_t adc_raw;       // raw ADC hodnota
  float vbat_raw;         // prepočítané napätie batérie
  float vbat_filtered;    // filtrované napätie
  vbat_state_t state;     // stav batérie
  robot_limits_t limits;  // limity pre robota
  uint8_t percent;
} BatteryStatus;

void BatteryMon_Init(ADC_HandleTypeDef *hadc);
BatteryStatus BatteryMon_Update(uint8_t samples);

#endif /* BATTERY_MONITOR_H */
