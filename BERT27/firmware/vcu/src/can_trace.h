/**
 * @file    can_trace.h
 * @brief   Prints CAN frames as readable text on the serial console.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_CAN_TRACE_H
#define VCU_CAN_TRACE_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

static constexpr uint8_t CELL_DATA_REQUEST_TEMPERATURES = 0x02;
static constexpr uint8_t CELL_DATA_REQUEST_VOLTAGES     = 0x01;

extern uint8_t lastCellDataRequestType;

void updateCanTranslationState(const char *direction, const CAN_message_t &msg);
void logCanTranslationToSerial(const char *direction, const CAN_message_t &msg);
void printRawPayloadFrom(const CAN_message_t &msg, uint8_t startIndex);
void printCanHexByte(uint8_t value);

#endif  // VCU_CAN_TRACE_H
