/**
 * @file    imu.cpp
 * @brief   Reads the MPU6050 IMU and writes IMU log records.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * The MPU6050 is polled at the 100 Hz fast-log rate. If it stops answering,
 * IMU records stop and init is retried every IMU_RETRY_PERIOD_MS.
 */
#include "imu.h"

#include <Wire.h>

#include "pins.h"
#include "can_bytes.h"
#include "main.h"

bool initImu();
bool readImuSample();
bool writeImuRegister(uint8_t reg, uint8_t value);

static constexpr uint8_t MPU6050_I2C_ADDR = 0x68;
static constexpr uint32_t IMU_I2C_CLOCK_HZ = 400000;
static constexpr uint8_t MPU6050_REG_SMPLRT_DIV   = 0x19;
static constexpr uint8_t MPU6050_REG_CONFIG       = 0x1A;
static constexpr uint8_t MPU6050_REG_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t MPU6050_REG_INT_PIN_CFG  = 0x37;
static constexpr uint8_t MPU6050_REG_INT_ENABLE   = 0x38;
static constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t MPU6050_REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t MPU6050_REG_WHO_AM_I     = 0x75;
static constexpr uint8_t MPU6050_WHO_AM_I_VALUE   = 0x68;
// ±4 g accel (8192 LSB/g) and ±500 dps gyro (65.5 LSB/dps); raw counts are
// converted and logged directly as floating-point g, deg/s and degC values.
static constexpr uint8_t MPU6050_ACCEL_FS_4G      = 0x08;
static constexpr uint8_t MPU6050_GYRO_FS_500DPS   = 0x08;
static constexpr uint8_t MPU6050_DLPF_44HZ        = 0x03;
static constexpr uint32_t IMU_RETRY_PERIOD_MS = 5000;

static constexpr uint8_t LOG_RECORD_IMU    = 0x04;

// IMU state; imuPresent reflects the last init/read outcome so a loose
// connector shows up as missing records, never as stale repeated data.
bool imuPresent = false;
uint32_t imuLastInitAttemptMs = 0;
int16_t imuAccelRaw[3] = {0, 0, 0};
int16_t imuGyroRaw[3] = {0, 0, 0};
int16_t imuTempRaw = 0;

// ---------------- IMU (MPU6050) ----------------
void setupImu() {
  Wire.begin();
  Wire.setClock(IMU_I2C_CLOCK_HZ);
  pinMode(IMU_INT_PIN, INPUT);

  imuPresent = initImu();
  imuLastInitAttemptMs = millis();
  Serial.println(imuPresent
                     ? "[IMU] MPU6050 detected; logging accel/gyro at 100 Hz"
                     : "[IMU] MPU6050 not responding; IMU records disabled");
}

bool initImu() {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(MPU6050_REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom((int)MPU6050_I2C_ADDR, 1) != 1) {
    return false;
  }
  uint8_t whoAmI = Wire.read();
  if (whoAmI != MPU6050_WHO_AM_I_VALUE) {
    Serial.print("[IMU] Unexpected WHO_AM_I 0x");
    printCanHexByte(whoAmI);
    Serial.println();
    return false;
  }

  // Wake from sleep with the gyro X PLL as clock, then 1 kHz sampling behind
  // the 44/42 Hz DLPF so every 100 Hz log read sees fresh, filtered data.
  return writeImuRegister(MPU6050_REG_PWR_MGMT_1, 0x01) &&
         writeImuRegister(MPU6050_REG_SMPLRT_DIV, 0) &&
         writeImuRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_44HZ) &&
         writeImuRegister(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500DPS) &&
         writeImuRegister(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G) &&
         writeImuRegister(MPU6050_REG_INT_PIN_CFG, 0x10) &&  // clear INT on any read
         writeImuRegister(MPU6050_REG_INT_ENABLE, 0x01);     // data-ready out on pin 33
}

bool writeImuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// Burst-reads accel, temp and gyro (registers 0x3B-0x48, big-endian pairs).
bool readImuSample() {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom((int)MPU6050_I2C_ADDR, 14) != 14) {
    return false;
  }

  uint8_t buf[14];
  for (uint8_t i = 0; i < sizeof(buf); i++) {
    buf[i] = (uint8_t)Wire.read();
  }

  for (uint8_t axis = 0; axis < 3; axis++) {
    imuAccelRaw[axis] = (int16_t)u16Be(buf[axis * 2], buf[axis * 2 + 1]);
    imuGyroRaw[axis] = (int16_t)u16Be(buf[8 + axis * 2], buf[9 + axis * 2]);
  }
  imuTempRaw = (int16_t)u16Be(buf[6], buf[7]);
  return true;
}

void writeImuLogRecord(uint32_t nowMs) {
  if (!imuPresent) {
    // Retry init occasionally so a sensor that comes back mid-session
    // (loose connector, late power) resumes logging without a reboot.
    if (nowMs - imuLastInitAttemptMs >= IMU_RETRY_PERIOD_MS) {
      imuLastInitAttemptMs = nowMs;
      imuPresent = initImu();
      if (imuPresent) {
        Serial.println("[IMU] MPU6050 detected; logging accel/gyro at 100 Hz");
      }
    }
    if (!imuPresent) {
      return;
    }
  }

  if (!readImuSample()) {
    Serial.println("[IMU] Read failed; will retry init");
    imuPresent = false;
    imuLastInitAttemptMs = nowMs;
    return;
  }

  MeasuredImuRecord measured;
  measured.type = (float)LOG_RECORD_IMU;
  measured.timeSeconds = (float)nowMs / 1000.0f;
  measured.accelXG = (float)imuAccelRaw[0] / 8192.0f;
  measured.accelYG = (float)imuAccelRaw[1] / 8192.0f;
  measured.accelZG = (float)imuAccelRaw[2] / 8192.0f;
  measured.gyroXDps = (float)imuGyroRaw[0] / 65.5f;
  measured.gyroYDps = (float)imuGyroRaw[1] / 65.5f;
  measured.gyroZDps = (float)imuGyroRaw[2] / 65.5f;
  measured.temperatureC = (float)imuTempRaw / 340.0f + 36.53f;
  appendSdLog(sdMeasuredLogFile, true, &measured, sizeof(measured));
}
