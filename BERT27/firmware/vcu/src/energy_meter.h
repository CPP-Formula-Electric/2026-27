/**
 * @file    energy_meter.h
 * @brief   Decodes the FSAE Energy Meter frames for the dashboard.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_ENERGY_METER_H
#define VCU_ENERGY_METER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Decoded Energy Meter values the dashboard and log read.
extern float emVoltageV;
extern bool haveEmMeasurement;
extern bool haveEmStatus;
extern uint8_t emNumSensors;
extern uint8_t emMinTempRaw;
extern uint8_t emMaxTempRaw;
extern bool haveEmTempSummary;
extern bool haveEmTemps;
extern bool emTimedOut;

void handleEmMeasurementFrame(const CAN_message_t &msg);
void handleEmStatusFrame(const CAN_message_t &msg);
void handleEmTemperatureFrame(const CAN_message_t &msg);
void serviceEnergyMeterTimeout(uint32_t now);
const char *emStatusSummaryText(uint16_t &colorOut);

#endif  // VCU_ENERGY_METER_H
