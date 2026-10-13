/**
 * @file    can_bytes.h
 * @brief   Little/big-endian byte helpers for unpacking CAN payloads.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software team (VCU_BERT26_TTT-D1_2.ino)
 * Worked on by: BERT27 restructure
 *
 * BMS frames are big-endian (u16Be); PM100, IMD and Energy Meter frames are
 * little-endian (u16Le, u32Le, s16Le, f32Le).
 */
#ifndef VCU_CAN_BYTES_H
#define VCU_CAN_BYTES_H

#include <Arduino.h>
#include <string.h>

static inline uint16_t u16Le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static inline uint16_t u16Be(uint8_t hi, uint8_t lo) {
  return ((uint16_t)hi << 8) | (uint16_t)lo;
}

static inline uint32_t u32Le(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  return (uint32_t)b0 | ((uint32_t)b1 << 8) |
         ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
}

static inline int16_t s16Le(uint8_t lo, uint8_t hi) {
  return (int16_t)u16Le(lo, hi);
}

// IEEE-754 32-bit float, little-endian on the wire. The Teensy 4.1 is also
// little-endian, so the four bytes copy straight into a float. Used for the
// Energy Meter Current / Voltage / Energy signals.
static inline float f32Le(const uint8_t *b) {
  float value;
  memcpy(&value, b, sizeof(value));
  return value;
}

#endif  // VCU_CAN_BYTES_H
