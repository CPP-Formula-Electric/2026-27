/**
 * @file    serial_console.h
 * @brief   Reads commands typed on the USB serial console: manual fan duty, log listing and log dumps.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_SERIAL_CONSOLE_H
#define VCU_SERIAL_CONSOLE_H

#include <Arduino.h>

void handleSerialConsoleInput();

#endif  // VCU_SERIAL_CONSOLE_H
