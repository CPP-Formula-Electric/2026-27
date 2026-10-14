/**
 * @file    rtd.h
 * @brief   Runs the ready-to-drive state machine: RTD button, entry conditions, drive mode and drop causes.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_RTD_H
#define VCU_RTD_H

#include <Arduino.h>

enum RtdState {
  RTD_STATE_FAULT_BLOCKED,
  RTD_STATE_READY_FOR_BUTTON,
  RTD_STATE_READY_TO_DRIVE
};

extern char driveMode;
extern RtdState rtdState;
extern bool readyToDriveLatched;
extern bool readyButtonLast;

void serviceReadyToDrive();
bool readReadyButtonPressed();
void refreshRtdStateFromInputs();
void dropReadyToDrive(const char *reason);
const char *rtdDropCauseText();
void setDriveMode(char newDriveMode);

#endif  // VCU_RTD_H
