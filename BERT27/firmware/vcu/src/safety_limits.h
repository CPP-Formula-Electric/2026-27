/**
 * @file    safety_limits.h
 * @brief   Rule and safety limits. Changes need a second reviewer.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 T.4.2.4, T.4.2.5, EV.4.7, EV.6.1.7, EV.10.6
 *
 * Not calibration: these values are not changed after tech inspection (IN.16.2.3).
 */
#ifndef VCU_SAFETY_LIMITS_H
#define VCU_SAFETY_LIMITS_H

#include <Arduino.h>

// ---------------- APPS, torque and BSPC ----------------
static const float APPS_IMPLAUSIBLE_PCT    = 10.0f;
// T.4.2.5 allows up to 100 ms of >10% channel deviation before it counts as
// an implausibility, so brief sensor/connector noise no longer drops RTD.
static const uint32_t APPS_IMPLAUSIBILITY_PERSIST_MS = 100;
// A raw count this far outside a channel's calibrated span means a wiring or
// sensor fault (short/open), not pedal travel.
static const int APPS_RAW_FAULT_MARGIN = 150;
static const float MAX_TORQUE_NM           = 170.0f;
static const float BSPC_APPS_THRESHOLD_PCT = 25.0f;
// Per EV.4.7/T.4.7 the BSPC latch only releases once the accelerator returns
// below 5% travel; releasing the brake alone must NOT restore torque.
static const float BSPC_APPS_RESET_PCT     = 5.0f;

// ---------------- Regen limits ----------------
// Keep regen below the BMS's 4.200 V/cell hard fault. Regulate toward 4.160 V,
// then latch regen off for 10 seconds if a terminal measurement reaches
// 4.180 V. The two margins prevent normal regulation from repeatedly hitting
// the stop threshold and leave room for quantization and control-loop delay.
static const float REGEN_CELL_OV_LIMIT_V = 4.200f;
static const float REGEN_CELL_CURRENT_TARGET_MARGIN_V = 0.040f;
static const float REGEN_CELL_TERMINAL_STOP_MARGIN_V = 0.020f;
static const float REGEN_CELL_CURRENT_TARGET_V =
    REGEN_CELL_OV_LIMIT_V - REGEN_CELL_CURRENT_TARGET_MARGIN_V;
static const float REGEN_CELL_TERMINAL_STOP_V =
    REGEN_CELL_OV_LIMIT_V - REGEN_CELL_TERMINAL_STOP_MARGIN_V;
// 0x310 and 0x311 are broadcast every 500 ms. Allow three missed broadcasts,
// then fail safe to zero regen rather than calculating from stale data.
static constexpr uint32_t REGEN_BMS_DATA_TIMEOUT_MS = 1500;
static constexpr uint32_t REGEN_BMS_PAIR_MAX_SKEW_MS = 100;
// Rules cutoff: no regen at or below 5 km/h; ramp to full strength by 15 km/h.
static const float REGEN_CUTOFF_SPEED_KMH = 5.0f;
static const float REGEN_FULL_SPEED_KMH = 15.0f;

// ---- Power derate -------------------------------------------------------
// When any protection condition trips, commanded torque is capped so output
// power stays at or below the active power limit (kW). The cap is the torque
// that produces that power at the current shaft speed:
//   T[Nm] = 9549 * kW / rpm (mechanical power form; ignores drivetrain/inverter
// efficiency, so the electrical bus power will be slightly higher than this).
//
// Low cell voltage derates *linearly* on the rolling-average minimum cell:
// at POWER_DERATE_CELL_FULL_MV the limit is POWER_DERATE_CELL_MAX_KW, falling
// to 0 kW at POWER_DERATE_CELL_ZERO_MV. Motor/battery-temperature trips use the
// fixed POWER_DERATE_KW cap. When more than one condition is active the lowest
// (most restrictive) limit wins.
static const float    POWER_DERATE_KW           = 5.0f;
// Linear low-voltage derate endpoints (rolling-average minimum cell):
//   >= CELL_FULL_MV -> no voltage derate; == CELL_FULL_MV -> CELL_MAX_KW;
//   <= CELL_ZERO_MV -> 0 kW; linear in between.
static const uint16_t POWER_DERATE_CELL_FULL_MV  = 3600;   // 60 kW point
static const uint16_t POWER_DERATE_CELL_ZERO_MV  = 2500;   // 0 kW point
static const float    POWER_DERATE_CELL_MAX_KW   = 60.0f;
// Derate trip points. Battery temperature is evaluated on a 15 s rolling
// average; motor temperature is instantaneous.
static const float    POWER_DERATE_MOTOR_TEMP_C  = 105.0f; // >  trips (instantaneous)
static const int16_t  POWER_DERATE_BATT_TEMP_C   = 50;     // >  trips (rolling avg)

// ---------------- Ready to drive, inverter fault clear, timeouts, TSSI ----------------
// Set false to bypass brake-press requirement when bench testing without pedals.
static constexpr bool REQUIRE_BRAKE_FOR_RTD = true;
static constexpr float PM100_FAULT_CLEAR_MIN_BUS_V = 240.0f;
static constexpr uint32_t PM100_FAULT_CLEAR_RETRY_MS = 2000;
static constexpr uint8_t PM100_FAULT_CLEAR_MAX_ATTEMPTS = 3;
static constexpr uint32_t BMS_TIMEOUT_MS = 10000;
static constexpr uint32_t STARTUP_BMS_IMD_FAULT_DELAY_MS = 12000;
static constexpr uint32_t TSSI_BLINK_PERIOD_MS    = 500;  // 2 Hz
static constexpr uint32_t TSSI_BLINK_ON_MS        = 250;
static constexpr uint32_t IMD_HEARTBEAT_TIMEOUT_MS = 5000;
static constexpr uint16_t IMD_FAULT_BITS_MASK = (1u << 0) | (1u << 1) | (1u << 2) |
                                                (1u << 3) | (1u << 4) | (1u << 7) |
                                                (1u << 8) | (1u << 9) | (1u << 10);

#endif  // VCU_SAFETY_LIMITS_H
