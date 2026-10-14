/**
 * @file    regen.h
 * @brief   Calculates regen braking torque within the pack voltage and charge-current limits.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_REGEN_H
#define VCU_REGEN_H

#include <Arduino.h>

extern float regenTorqueNm;

bool pushRegenVoltageSample(uint32_t nowMs);
float calculateRegenTorqueNm();

#endif  // VCU_REGEN_H
