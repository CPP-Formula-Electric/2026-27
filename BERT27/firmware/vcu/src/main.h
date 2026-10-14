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

static constexpr uint8_t IMD_SNV_BYTE         = 0xFF;
static constexpr uint16_t IMD_SNV_WORD        = 0xFFFF;
void printImdRequestTranslation(const CAN_message_t &msg);
void printImdResponseTranslation(const CAN_message_t &msg);
void printImdInfoGeneralTranslation(const CAN_message_t &msg);
void printImdInfoIsolationDetailTranslation(const CAN_message_t &msg);
void printImdInfoVoltageTranslation(const CAN_message_t &msg);
void printImdInfoItSystemTranslation(const CAN_message_t &msg);

const char *firstTssiFaultName(uint16_t bits);
const char *bmsTemperatureFaultCodeName(uint8_t faultCode);
const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults);
const char *pm100PostFaultName(uint8_t bit);
const char *pm100RunFaultName(uint8_t bit);

extern bool screenDirty;
void showFaultBanner(const char *text);

#endif  // VCU_MAIN_H
