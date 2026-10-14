/**
 * @file    imu.h
 * @brief   Reads the MPU6050 IMU and writes IMU log records.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_IMU_H
#define VCU_IMU_H

#include <Arduino.h>

void writeImuLogRecord(uint32_t nowMs);
void setupImu();

#endif  // VCU_IMU_H
