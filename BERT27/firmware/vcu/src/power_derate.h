/**
 * @file    power_derate.h
 * @brief   Caps torque so battery power stays under the cell-voltage and temperature limit.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_POWER_DERATE_H
#define VCU_POWER_DERATE_H

#include <Arduino.h>

static const float    TORQUE_PER_KW_PER_RPM      = 9549.0f;  // 60000 / (2*pi)

extern bool powerDerateActive;
extern float powerDerateKw;

float powerDerateTorqueCapNm(int16_t rpm);
void servicePowerDerate(uint32_t nowMs);

#endif  // VCU_POWER_DERATE_H
