/**
 * @file    faults.h
 * @brief   Decides whether a BMS or IMD fault is active, after the startup delay.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_FAULTS_H
#define VCU_FAULTS_H

#include <Arduino.h>

extern uint32_t startupFaultDelayStartMs;

bool bmsImdFaultDetectionArmed();
bool anyFaultActive();

#endif  // VCU_FAULTS_H
