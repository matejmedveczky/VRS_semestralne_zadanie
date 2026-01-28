#include "mpu6500.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MPU_ADDR_7BIT 0x68
#define MPU_ADDR (MPU_ADDR_7BIT << 1)

/* registers */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define REG_ACCEL_XOUT_H 0x3B

static I2C_HandleTypeDef *s_hi2c = NULL;

/* raw offsets */
static float s_ax_off = 0.0f;
static float s_gz_off = 0.0f;

static bool i2c_wr(uint8_t reg, uint8_t val) {
  return (HAL_I2C_Mem_Write(s_hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            &val, 1, 100) == HAL_OK);
}

static bool i2c_rd(uint8_t reg, uint8_t *buf, uint16_t len) {
  return (HAL_I2C_Mem_Read(s_hi2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           buf, len, 100) == HAL_OK);
}

bool MPU6500_Init(I2C_HandleTypeDef *hi2c) {
  s_hi2c = hi2c;
  if (s_hi2c == NULL) return false;

  /* optional whoami read */
  uint8_t who = 0;
  (void)i2c_rd(REG_WHO_AM_I, &who, 1);

  /* wake up */
  if (!i2c_wr(REG_PWR_MGMT_1, 0x00)) return false;
  HAL_Delay(50);

  /* basic config: sample rate + DLPF */
  (void)i2c_wr(REG_SMPLRT_DIV, 0x04); /* ~200 Hz */
  (void)i2c_wr(REG_CONFIG, 0x05);     /* DLPF */

  /* gyro ±250 dps */
  (void)i2c_wr(REG_GYRO_CONFIG, 0x08);

  /* accel ±2g */
  (void)i2c_wr(REG_ACCEL_CONFIG, 0x18);

  /* calibration (robot must be stationary) */
  const int N = 300;
  int32_t sum_ax = 0;
  int32_t sum_gz = 0;

  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!MPU6500_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz)) return false;
    sum_ax += ax;
    sum_gz += gz;
    HAL_Delay(5);
  }

  s_ax_off = (float)sum_ax / (float)N;
  s_gz_off = (float)sum_gz / (float)N;

  return true;
}

bool MPU6500_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz) {
  if (s_hi2c == NULL) return false;

  uint8_t buf[14];
  if (!i2c_rd(REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

  if (ax) *ax = (int16_t)((buf[0] << 8) | buf[1]);
  if (ay) *ay = (int16_t)((buf[2] << 8) | buf[3]);
  if (az) *az = (int16_t)((buf[4] << 8) | buf[5]);
  /* buf[6..7] = temp */
  if (gx) *gx = (int16_t)((buf[8] << 8) | buf[9]);
  if (gy) *gy = (int16_t)((buf[10] << 8) | buf[11]);
  if (gz) *gz = (int16_t)((buf[12] << 8) | buf[13]);

  return true;
}

float read_gyroscope_z(void) {
  /* ±250 dps => 131 LSB/(deg/s) -> rad/s */
  const float lsb_per_dps = 131.0f;
  const float dps_to_rads = (float)(M_PI / 180.0);

  int16_t ax, ay, az, gx, gy, gz;
  if (!MPU6500_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz)) return 0.0f;

  float gz_corr = (float)gz - s_gz_off;
  float dps = gz_corr / lsb_per_dps;
  return dps * dps_to_rads;
}

float read_accelerometer_x(void) {
  /* ±2g => 16384 LSB/g -> m/s^2 */
  const float lsb_per_g = 16384.0f;
  const float g_to_ms2 = 9.80665f;

  int16_t ax, ay, az, gx, gy, gz;
  if (!MPU6500_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz)) return 0.0f;

  float ax_corr = (float)ax - s_ax_off;
  float g = ax_corr / lsb_per_g;
  return g * g_to_ms2;
}
