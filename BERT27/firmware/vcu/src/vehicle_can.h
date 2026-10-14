/**
 * @file    vehicle_can.h
 * @brief   Sets up CAN3, sends frames and dispatches received frames to their module.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_VEHICLE_CAN_H
#define VCU_VEHICLE_CAN_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

static constexpr uint32_t PM100_TEMPERATURE_1_FRAME_ID = 0x0A0;
static constexpr uint32_t PM100_TEMPERATURE_3_FRAME_ID = 0x0A2;
static constexpr uint32_t PM100_RPM_FRAME_ID = 0x0A5;
static constexpr uint32_t PM100_CURRENT_INFO_FRAME_ID = 0x0A6;
static constexpr uint32_t PM100_VOLTAGE_INFO_FRAME_ID = 0x0A7;
static constexpr uint32_t PM100_INTERNAL_VOLTAGES_FRAME_ID = 0x0A9;
static constexpr uint32_t PM100_FAULT_CODES_FRAME_ID = 0x0AB;
static constexpr uint32_t PM100_TORQUE_CAPABILITY_FRAME_ID = 0x0B1;
// BERT26 vehicle CAN definitions for human-readable log translations.
static constexpr uint32_t BMS_STATUS_2026_ID        = 0x310;
static constexpr uint32_t BMS_TEMPERATURE_2026_ID   = 0x311;
static constexpr uint32_t BMS_CELL_RESPONSE_2026_ID = 0x313;
static constexpr uint32_t IMD_INFO_GENERAL_ID = 0x37;
static constexpr uint32_t IMD_RESPONSE_ID     = 0x23;

extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> VehicleCan;

void setupVehicleCan();
void sendCanFrame(CAN_message_t &msg);
void receiveCanFrames();
void logCanFrame(const char *direction, const CAN_message_t &msg);

#endif  // VCU_VEHICLE_CAN_H
