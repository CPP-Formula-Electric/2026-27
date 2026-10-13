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

// Full type is in FlexCAN_T4.h; declared here so this header stays light.
struct CAN_message_t;

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

// Owned by dashboard (V26). Used by energy_meter, imd.
extern bool screenDirty;
void showFaultBanner(const char *text);

// Owned by vehicle_can (V13). Used by imd.
static constexpr uint32_t IMD_INFO_GENERAL_ID = 0x37;
static constexpr uint32_t IMD_RESPONSE_ID     = 0x23;
void sendCanFrame(CAN_message_t &msg);

// Owned by can_trace_imd (V23). Used by imd.
static constexpr uint8_t IMD_SNV_BYTE         = 0xFF;

// Owned by faults (V14). Used by imd.
bool bmsImdFaultDetectionArmed();

// Owned by tssi (V20). Used by imd.
bool tssiFaultFromWarnBits(uint16_t warnBits);
void printTssiWarnBits(uint16_t bits);

// Owned by fault_text (V25). Used by imd.
const char *firstTssiFaultName(uint16_t bits);

#endif  // VCU_MAIN_H
