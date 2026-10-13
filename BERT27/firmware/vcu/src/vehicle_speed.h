/**
 * @file    vehicle_speed.h
 * @brief   Tracks cumulative distance from vehicle speed.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software team (VCU_BERT26_TTT-D1_2.ino)
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_VEHICLE_SPEED_H
#define VCU_VEHICLE_SPEED_H

#include <Arduino.h>

// Road speed from motor rpm (set in loop()), and the distance driven this power-up.
extern float speedMph;
extern float speedKmh;
extern float cumulativeDistanceKm;

void updateCumulativeDistance(uint32_t nowMs);

#endif  // VCU_VEHICLE_SPEED_H
