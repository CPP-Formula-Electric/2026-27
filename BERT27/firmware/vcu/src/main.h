/**
 * @file    main.h
 * @brief   TEMPORARY: globals still defined in main.cpp that moved modules use.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software team (VCU_BERT26_TTT-D1_2.ino)
 * Worked on by: BERT27 restructure
 *
 * Each line goes away when the module that owns the variable moves out of
 * main.cpp. This file is deleted once main.cpp holds only setup() and loop().
 */
#ifndef VCU_MAIN_H
#define VCU_MAIN_H

#include <Arduino.h>

// Owned by inverter (V12). Used by fans.
extern int16_t inverterTempTenthsC;
extern int16_t pm100MotorTempTenthsC;
extern bool haveInverterTemp;
extern bool havePm100MotorTemp;

#endif  // VCU_MAIN_H
