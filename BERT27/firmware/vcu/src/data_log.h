/**
 * @file    data_log.h
 * @brief   Writes the measured, derived and cell log files to the SD card and lists or dumps them over serial.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_DATA_LOG_H
#define VCU_DATA_LOG_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SD.h>

struct __attribute__((packed)) MeasuredImuRecord {
  float type, timeSeconds;
  float accelXG, accelYG, accelZG;
  float gyroXDps, gyroYDps, gyroZDps, temperatureC;
};

extern File sdMeasuredLogFile;
extern bool sdCardStarted;
extern bool sdLogActive;
extern uint32_t logSessionNumber;

void setupDataLogging();
void serviceDataLogging();
void appendSdLog(File &file, bool measuredLog, const void *data, size_t len);
void listLogFiles();
void dumpLogFile(uint32_t session);
void writeCellDataResponseLog(const CAN_message_t &msg);

#endif  // VCU_DATA_LOG_H
