/**
 * @file    can_trace_imd.h
 * @brief   Prints Bender iso175 IMD CAN frames as readable text on the serial console.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_CAN_TRACE_IMD_H
#define VCU_CAN_TRACE_IMD_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

static constexpr uint8_t IMD_SNV_BYTE         = 0xFF;
static constexpr uint16_t IMD_SNV_WORD        = 0xFFFF;

void printImdRequestTranslation(const CAN_message_t &msg);
void printImdResponseTranslation(const CAN_message_t &msg);
void printImdInfoGeneralTranslation(const CAN_message_t &msg);
void printImdInfoIsolationDetailTranslation(const CAN_message_t &msg);
void printImdInfoVoltageTranslation(const CAN_message_t &msg);
void printImdInfoItSystemTranslation(const CAN_message_t &msg);

#endif  // VCU_CAN_TRACE_IMD_H
