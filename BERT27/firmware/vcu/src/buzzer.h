/**
 * @file    buzzer.h
 * @brief   Plays tone patterns on the buzzer, such as the ready-to-drive sound.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.10.7
 */
#ifndef VCU_BUZZER_H
#define VCU_BUZZER_H

#include <Arduino.h>

// ---------------- Buzzer sequencer ----------------
struct ToneStep {
  uint16_t frequencyHz;
  uint16_t onMs;
  uint16_t offMs;
};

void playTonePattern(const ToneStep *pattern, uint8_t length, bool repeats);
void serviceBuzzer();

#endif  // VCU_BUZZER_H
