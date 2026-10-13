/**
 * @file    imd.h
 * @brief   Bender iso175 IMD frames and the warning/alarm poll.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software team (VCU_BERT26_TTT-D1_2.ino)
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.6.1.7
 */
#ifndef VCU_IMD_H
#define VCU_IMD_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

static constexpr uint32_t IMD_REQUEST_ID      = 0x22;
static constexpr uint8_t IDX_WARN_ALARMS      = 0x6C;

// IMD state decoded from 0x37 and 0x23. imdFault drives the TSSI and RTD.
extern bool imdFault;
extern uint16_t imdWarnAlarms;
extern uint16_t imdRIsoKohm;
extern uint8_t imdRIsoStatus;
extern uint32_t lastImdRxMs;
extern bool imdCanTimedOut;

void handleImdFrame(const CAN_message_t &msg);
void requestImdWarnings();

#endif  // VCU_IMD_H
