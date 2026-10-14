/**
 * @file    pedals.h
 * @brief   Reads the accelerator and brake pedals and runs the APPS plausibility and BSPC checks.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_PEDALS_H
#define VCU_PEDALS_H

#include <Arduino.h>

static const int ADC_RESOLUTION_BITS = 12;

extern float apps1Pct;
extern float apps2Pct;
extern float pedalPct;
extern float desiredTorqueNm;
extern bool appsPlausible;
extern const char *appsFaultText;
extern bool bspcActive;
extern int rawApps1Last;
extern int rawApps2Last;
extern int rawBse1Last;
extern int rawBse2Last;
extern float brakePressurePct;
extern bool braking;
extern bool hardBraking;

void updatePedal();
void logAppsTelemetry();
void updateBrakePressureSensors();

#endif  // VCU_PEDALS_H
