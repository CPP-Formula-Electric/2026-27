/**
 * @file    vehicle_speed.cpp
 * @brief   Tracks cumulative distance from vehicle speed.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software team (VCU_BERT26_TTT-D1_2.ino)
 * Worked on by: BERT27 restructure
 */
#include "vehicle_speed.h"

float speedMph = 0.0f;
float speedKmh = 0.0f;
float cumulativeDistanceKm = 0.0f;
uint32_t lastDistanceUpdateMs = 0;

// Integrate the RPM-derived road speed. This is intentionally approximate:
// MPH_PER_RPM already embodies the vehicle's gearing and tire circumference.
void updateCumulativeDistance(uint32_t nowMs) {
  if (lastDistanceUpdateMs != 0) {
    float elapsedHours = (float)(nowMs - lastDistanceUpdateMs) / 3600000.0f;
    cumulativeDistanceKm += speedKmh * elapsedHours;
  }
  lastDistanceUpdateMs = nowMs;
}
