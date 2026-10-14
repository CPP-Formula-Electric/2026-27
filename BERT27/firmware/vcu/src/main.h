/**
 * @file    main.h
 * @brief   TEMPORARY: globals still defined in main.cpp that moved modules use.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
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

// Owned by vehicle_can (V13). Used by imd, bms_link, inverter.
static constexpr uint32_t IMD_INFO_GENERAL_ID = 0x37;
static constexpr uint32_t IMD_RESPONSE_ID     = 0x23;
void sendCanFrame(CAN_message_t &msg);

// Owned by faults (V14). Used by imd, bms_link.
bool bmsImdFaultDetectionArmed();
bool anyFaultActive();

// Owned by regen (V16). Used by bms_link.
bool pushRegenVoltageSample(uint32_t nowMs);

// Owned by rtd (V19). Used by bms_link, inverter.
const char *rtdDropCauseText();
void dropReadyToDrive(const char *reason);
extern bool readyToDriveLatched;

// Owned by tssi (V20). Used by imd.
bool tssiFaultFromWarnBits(uint16_t warnBits);
void printTssiWarnBits(uint16_t bits);

// Owned by data_log (V21). Used by imu, bms_link.
struct __attribute__((packed)) MeasuredImuRecord {
  float type, timeSeconds;
  float accelXG, accelYG, accelZG;
  float gyroXDps, gyroYDps, gyroZDps, temperatureC;
};
extern File sdMeasuredLogFile;
void appendSdLog(File &file, bool measuredLog, const void *data, size_t len);
extern bool sdLogActive;

// Owned by can_trace (V22). Used by imu, bms_link.
void printCanHexByte(uint8_t value);
static constexpr uint8_t CELL_DATA_REQUEST_TEMPERATURES = 0x02;
static constexpr uint8_t CELL_DATA_REQUEST_VOLTAGES     = 0x01;

// Owned by can_trace_imd (V23). Used by imd.
static constexpr uint8_t IMD_SNV_BYTE         = 0xFF;

// Owned by fault_text (V25). Used by imd, bms_link, inverter.
const char *firstTssiFaultName(uint16_t bits);
const char *bmsTemperatureFaultCodeName(uint8_t faultCode);
const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults);

// Owned by dashboard (V26). Used by energy_meter, imd, bms_link, inverter.
extern bool screenDirty;
void showFaultBanner(const char *text);

#endif  // VCU_MAIN_H
