/**
 * @file    tssi.h
 * @brief   Drives the TSSI lights: green when safe, red flashing on an IMD or BMS fault.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_TSSI_H
#define VCU_TSSI_H

#include <Arduino.h>

bool tssiFaultFromWarnBits(uint16_t warnBits);
void setupTssi();
void serviceTssi();
void printTssiWarnBits(uint16_t bits);

#endif  // VCU_TSSI_H
