/**
 * @file    fans.h
 * @brief   Sets the cooling fan PWM from motor/inverter temperature.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software team (VCU_BERT26_TTT-D1_2.ino)
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_FANS_H
#define VCU_FANS_H

#include <Arduino.h>

// Fan duty in percent. The serial console can set these by hand
// (fanManualOverride = true) instead of the temperature curve.
extern int fan1DutyPercent;
extern int fan2DutyPercent;
extern bool fanManualOverride;

void setupFans();
void updateFanDutyFromTemps();
void serviceFans();

#endif  // VCU_FANS_H
