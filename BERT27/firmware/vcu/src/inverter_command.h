/**
 * @file    inverter_command.h
 * @brief   Sends the PM100DX torque command every 10 ms from a timer interrupt.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_INVERTER_COMMAND_H
#define VCU_INVERTER_COMMAND_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// Cascadia PM100DX command frame from BERT25.
static constexpr uint32_t PM100_CMD_ID       = 0x0C0;

extern float commandedTorqueNm;
extern float displayedCommandTorqueNm;

void setupPm100CommandTimer();
void updatePm100CommandState();
void servicePm100TxLogging();

#endif  // VCU_INVERTER_COMMAND_H
