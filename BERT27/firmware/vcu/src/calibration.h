/**
 * @file    calibration.h
 * @brief   Values tuned on the car: pedal ranges, regen, brakes, fans, speed scaling.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 *
 * Calibration changes are allowed after tech inspection (IN.16.2.3).
 * Rule and safety limits are in safety_limits.h, not here.
 */
#ifndef VCU_CALIBRATION_H
#define VCU_CALIBRATION_H

#include <Arduino.h>

// ---------------- Fans ----------------
// Fan duty tracks the hotter of motor/inverter temp: 20% at room temp,
// ramping linearly to 100% by 40 C (system max is 50 C).
static const float FAN_TEMP_MIN_C = 25.0f;
static const float FAN_TEMP_MAX_C = 40.0f;
static const int FAN_DUTY_MIN_PERCENT = 20;
static const int FAN_DUTY_MAX_PERCENT = 100;

// ---------------- Accelerator pedal ----------------
// Raw ADC calibration ranges from the BERT26 pedal test sketch.
static const int APPS_1_RAW_MIN = 325;
static const int APPS_1_RAW_MAX = 2400;
static const int APPS_2_RAW_MIN = 2130;
static const int APPS_2_RAW_MAX = 4013;
static const float APPS_EMA_ALPHA          = 0.35f;
static const float PEDAL_ZERO_DEADBAND_PCT = 7.0f;

// ---- Regenerative braking -----------------------------------------------
// This single switch makes regen easy to disable without changing any logic.
static constexpr bool REGEN_ENABLED = true;
static const float REGEN_MAX_TORQUE_NM = 50.0f;
// Regen reaches its maximum at the end of the light-braking region, then
// tapers back to zero through the heavy-braking region to reduce lockup risk.
static const float REGEN_LIGHT_BRAKE_END_PCT = 25.0f;
static const float REGEN_HEAVY_BRAKE_END_PCT = 40.0f;
// Reduce charging power as SOC approaches full to avoid pack over-voltage.
static const float REGEN_SOC_DERATE_START_PCT = 90.0f;
static const float REGEN_SOC_DISABLE_PCT = 95.0f;
static const float REGEN_MAX_POWER_KW = 20.0f;
// Equivalent DC resistance of one complete 4P cell bank. Pack current flows
// through every series bank, so no series-cell multiplier belongs here.
static const float REGEN_CELL_BANK_RESISTANCE_OHM = 0.00285f;
static const float REGEN_PACK_CURRENT_SENSOR_MAX_A = 500.0f;

// ---------------- Brake pressure sensors ----------------
// Calculated from voltage divider at input. R1 = 5.6k, R2 = 15k, Gain = 0.728
// This comment was written by a human, me
static const float BSE_SENSOR_MIN_VOLTAGE  = 0.364f;
static const float BSE_SENSOR_MAX_VOLTAGE  = 3.277f;
static const int BSE_1_BRAKING_RAW_THRESHOLD = 600;
static const int BSE_2_BRAKING_RAW_THRESHOLD = 800;
static const int BSE_1_HARD_BRAKING_RAW_THRESHOLD = 2000;
static const int BSE_2_HARD_BRAKING_RAW_THRESHOLD = 2100;

// ---------------- Speed and GLV ----------------
static constexpr float MPH_PER_RPM           = 0.014005f;
static constexpr float KMH_PER_MPH           = 1.609344f;
// GLV battery sits around 12-13 V; show the readout red once it sags below this.
static constexpr float GLV_LOW_VOLTAGE_WARN_V = 11.5f;

#endif  // VCU_CALIBRATION_H
