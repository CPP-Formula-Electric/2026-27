/**
 * @file    pins.h
 * @brief   Every Teensy 4.1 pin the VCU uses, in one place.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_PINS_H
#define VCU_PINS_H

// ---------------- Display / touch pins ----------------
static const int TFT_MISO = 12;
static const int TFT_MOSI = 11;
static const int TFT_SCK  = 13;
static const int TFT_DC   = 8;
static const int TFT_CS   = 10;
static const int TFT_RST  = 9;
static const int TFT_LED  = 29;

static const int TOUCH_CS   = 7;
static const int TOUCH_IRQ  = 6;

// ---------------- Vehicle IO ----------------
static const int READY_BUTTON_PIN = 23;
static const int BUZZER_PIN       = 22;    // Change here if the buzzer is wired elsewhere.
static const int BRAKE_LIGHT_PIN  = 5;     // HIGH = brake light on
static const int FAN_1_PIN        = 1;
static const int FAN_2_PIN        = 2;

// Shutdown-circuit status inputs: HIGH = signal OK (green), LOW = tripped (red).
static const int BSPD_STATUS_PIN  = 24;
static const int IMD_STATUS_PIN   = 25;
static const int BMS_STATUS_PIN   = 28;

// HV battery current sensor analog input; sampled into the fast data log.
static const int HV_CURRENT_SENSE_PIN = 14;

// MPU6050 IMU on the default Wire pins (18=SDA, 19=SCL). The sensor's INT pin
// is wired to 33 and configured as data-ready, but samples are polled at the
// fast-log rate so the interrupt is informational only.
static const int IMU_INT_PIN = 33;

// EURO-XPD analog input pins from VCU_Pedal_Read_Test.
static const int APPS_1_PIN = 38;         // Inverted channel
static const int APPS_2_PIN = 41;         // Normal channel
static const int BSE_1_PIN  = 17;
static const int BSE_2_PIN  = 16;

// TSSI LED pins: low-side switches, HIGH = transistor on = LED on.
static const int TSSI_LED_RED   = 3;
static const int TSSI_LED_GREEN = 4;

#endif  // VCU_PINS_H
