/**
 * @file    main.h
 * @brief   TEMPORARY: globals still defined in main.cpp that moved modules use.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software team (VCU_BERT26_TTT-D1_2.ino)
 * Worked on by: BERT27 restructure
 *
 * Each line goes away when the module that owns the variable moves out of
 * main.cpp. This file is deleted once main.cpp holds only setup() and loop().
 */
#ifndef VCU_MAIN_H
#define VCU_MAIN_H

#include <Arduino.h>
#include <SD.h>

// Owned by inverter (V12). Used by fans.
extern int16_t inverterTempTenthsC;
extern int16_t pm100MotorTempTenthsC;
extern bool haveInverterTemp;
extern bool havePm100MotorTemp;

// Owned by data_log (V21). Used by imu.
struct __attribute__((packed)) MeasuredImuRecord {
  float type, timeSeconds;
  float accelXG, accelYG, accelZG;
  float gyroXDps, gyroYDps, gyroZDps, temperatureC;
};
extern File sdMeasuredLogFile;
void appendSdLog(File &file, bool measuredLog, const void *data, size_t len);

// Owned by can_trace (V22). Used by imu.
void printCanHexByte(uint8_t value);

// Owned by dashboard (V26). Used by energy_meter.
extern bool screenDirty;

#endif  // VCU_MAIN_H
