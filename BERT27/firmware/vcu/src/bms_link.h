/**
 * @file    bms_link.h
 * @brief   BMS status and temperature frames, cell data requests, BMS state.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_BMS_LINK_H
#define VCU_BMS_LINK_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

static constexpr uint32_t BMS_CELL_REQUEST_2026_ID  = 0x312;
static constexpr int16_t CELL_TEMP_UNKNOWN_C = -1000;

extern bool regenBmsPairValid;
extern float packVoltage;
extern float packCurrent;
extern uint8_t stateOfChargePct;
extern uint8_t batteryTempC;
extern uint16_t minCellVoltageMv;
extern uint16_t maxCellVoltageMv;
extern uint16_t avgCellVoltageMv;
extern bool haveCellVoltageStats;
extern int16_t minCellTempC;
extern int16_t maxCellTempC;
extern int16_t avgCellTempC;
extern bool haveCellTempStats;
extern bool bmsActive;
extern bool bmsFault;
extern bool bmsTimedOut;
extern bool haveBmsStatus;
extern uint8_t bmsFlags;
extern uint8_t bms2026FaultCode;
extern bool haveBms2026FaultCode;
extern uint32_t lastBmsRxMs;
extern bool haveBmsCurrent;
extern uint32_t lastBmsCellVoltageRxMs;
extern uint32_t lastBmsCurrentRxMs;

void serviceCellDataRequests(uint32_t nowMs);
bool contactorsOpen();
void handleBms2026StatusFrame(const CAN_message_t &msg);
void handleBms2026TempFrame(const CAN_message_t &msg);

#endif  // VCU_BMS_LINK_H
