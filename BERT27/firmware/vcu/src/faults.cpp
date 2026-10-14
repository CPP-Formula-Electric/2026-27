/**
 * @file    faults.cpp
 * @brief   Decides whether a BMS or IMD fault is active, after the startup delay.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "faults.h"

#include "safety_limits.h"
#include "imd.h"
#include "bms_link.h"

uint32_t startupFaultDelayStartMs = 0;

bool bmsImdFaultDetectionArmed() {
  return (millis() - startupFaultDelayStartMs) >= STARTUP_BMS_IMD_FAULT_DELAY_MS;
}

bool anyFaultActive() {
  if (!bmsImdFaultDetectionArmed()) {
    return false;
  }

  return imdFault || bmsFault;
}
