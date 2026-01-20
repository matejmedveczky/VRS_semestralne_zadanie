/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include <stdbool.h>
#include <stdio.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

#define R1_OHMS           (47000.0f)
#define R2_OHMS           (15000.0f)
#define ADC_MAX_COUNTS    (4095.0f)     // 12-bit
#define VDDA_VOLTS        (3.3f)        // ak chceš presnejšie, neskôr spravíme VREFINT

#define VBAT_WARN_ON      (10.5f)
#define VBAT_WARN_OFF     (10.7f)

#define VBAT_CRIT_ON      (9.9f)
#define VBAT_CRIT_OFF     (10.1f)

#define VBAT_SHDN_ON      (9.6f)
#define VBAT_SHDN_OFF     (9.8f)

#define VBAT_EMA_ALPHA    (0.05f)       // filtrácia (nižšie = viac filtruje)

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
  bool  leds_enabled;
} robot_limits_t;

static float vbat_ema = 0.0f;
static bool  vbat_ema_init = false;

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);


/* USER CODE BEGIN PFP */

static inline float DividerScale(void);
static bool ADC_ReadOnce_U16(ADC_HandleTypeDef *hadc, uint16_t *out);
static bool ADC_ReadAverage_U16(ADC_HandleTypeDef *hadc, uint16_t *out, uint8_t n);
static float VBAT_FromAdcCounts(uint16_t adc_counts);

static float VBAT_FilterEma(float vbat);
static vbat_state_t VBAT_UpdateState(float vbat_filtered);
static robot_limits_t Robot_LimitsForBattery(vbat_state_t st);

// Tu si neskôr napojíš svoj reálny motor driver / motion control
static void Robot_ApplyLimits(robot_limits_t lim);

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline float DividerScale(void)
{
  return (R1_OHMS + R2_OHMS) / R2_OHMS; // 62k/15k = 4.133333...
}

static bool ADC_ReadOnce_U16(ADC_HandleTypeDef *hadc, uint16_t *out)
{
  if (HAL_ADC_Start(hadc) != HAL_OK) return false;

  if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
    (void)HAL_ADC_Stop(hadc);
    return false;
  }

  uint32_t val = HAL_ADC_GetValue(hadc);
  (void)HAL_ADC_Stop(hadc);

  if (val > 0xFFFFu) val = 0xFFFFu;
  *out = (uint16_t)val;
  return true;
}

static bool ADC_ReadAverage_U16(ADC_HandleTypeDef *hadc, uint16_t *out, uint8_t n)
{
  if (n == 0) n = 1;

  uint32_t acc = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint16_t s = 0;
    if (!ADC_ReadOnce_U16(hadc, &s)) return false;
    acc += s;
    HAL_Delay(1);
  }
  *out = (uint16_t)(acc / n);
  return true;
}

static float VBAT_FromAdcCounts(uint16_t adc_counts)
{
  float v_adc = ((float)adc_counts / ADC_MAX_COUNTS) * VDDA_VOLTS; // napätie na PA3
  return v_adc * DividerScale();                                   // prepočet na batériu
}

static float VBAT_FilterEma(float vbat)
{
  if (!vbat_ema_init) {
    vbat_ema = vbat;
    vbat_ema_init = true;
  } else {
    vbat_ema = vbat_ema + VBAT_EMA_ALPHA * (vbat - vbat_ema);
  }
  return vbat_ema;
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
      L.leds_enabled = true;
      break;

    case VBAT_WARN:
      L.max_speed = 0.7f;
      L.max_accel = 0.6f;
      L.max_pwm   = 0.7f;
      L.leds_enabled = false;
      break;

    case VBAT_CRIT:
      L.max_speed = 0.4f;
      L.max_accel = 0.3f;
      L.max_pwm   = 0.4f;
      L.leds_enabled = false;
      break;

    case VBAT_SHUTDOWN:
      L.max_speed = 0.0f;
      L.max_accel = 0.0f;
      L.max_pwm   = 0.0f;
      L.leds_enabled = false;
      break;
  }

  return L;
}

/* Zatiaľ len „stub“ – aby to buildlo.
   Neskôr sem napojíš Motor_SetPwmLimit(), Motion_SetSpeedLimit(), atď. */
static void Robot_ApplyLimits(robot_limits_t lim)
{
  (void)lim;
  // TODO: Motor_SetPwmLimit(lim.max_pwm);
  // TODO: Motion_SetSpeedLimit(lim.max_speed);
  // TODO: Motion_SetAccelLimit(lim.max_accel);
  // TODO: LEDs_Enable(lim.leds_enabled);
}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint16_t adc = 0;

    if (ADC_ReadAverage_U16(&hadc1, &adc, 16))
    {
      float vbat_raw = VBAT_FromAdcCounts(adc);
      float vbat_f   = VBAT_FilterEma(vbat_raw);

      vbat_state_t st = VBAT_UpdateState(vbat_f);
      robot_limits_t lim = Robot_LimitsForBattery(st);

      Robot_ApplyLimits(lim);

      // Voliteľné: keď máš UART a printf presmerované, môžeš vypísať
      // printf("ADC=%u  VBAT=%.2fV (f=%.2fV)  state=%d\r\n", adc, vbat_raw, vbat_f, (int)st);
    }

    HAL_Delay(200);
  }
  /* USER CODE END WHILE */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
