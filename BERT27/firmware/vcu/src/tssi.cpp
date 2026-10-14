/**
 * @file    tssi.cpp
 * @brief   Drives the TSSI lights: green when safe, red flashing on an IMD or BMS fault.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "tssi.h"

#include "pins.h"
#include "safety_limits.h"
#include "imd.h"
#include "bms_link.h"
#include "faults.h"
#include "rtd.h"
#include "main.h"
#include "dashboard.h"

void updateTssiLed(bool faultActive, uint32_t now);
void setTssiGreenOn();
void setTssiRedOn();
void setTssiRedOff();
void setTssiAllOff();

static constexpr uint32_t IMD_GET_REQ_PERIOD_MS    = 500;

uint32_t tssiBlinkTimerMs = 0;
bool tssiRedOn = false;
// Once the TSSI has blinked red for any fault it stays latched in the red-blink
// state until the board is power-cycled (this flag clears on reset).
bool tssiFaultLatched = false;
uint32_t lastImdGetReqMs = 0;

bool tssiFaultFromWarnBits(uint16_t warnBits) {
  return (warnBits & IMD_FAULT_BITS_MASK) != 0;
}

void setupTssi() {
  pinMode(TSSI_LED_RED, OUTPUT);
  pinMode(TSSI_LED_GREEN, OUTPUT);
  setTssiAllOff();

  imdFault = true;
  imdCanTimedOut = true;
  lastImdRxMs = millis();
  tssiBlinkTimerMs = millis();
  lastImdGetReqMs = 0;
}

void serviceTssi() {
  uint32_t now = millis();
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (faultDetectionArmed && now - lastImdRxMs > IMD_HEARTBEAT_TIMEOUT_MS) {
    if (!imdCanTimedOut) {
      Serial.println("[TSSI] IMD heartbeat lost, forcing FAULT");
      showFaultBanner("IMD Timeout");
    }
    imdCanTimedOut = true;
    imdFault = true;
  }

  if ((anyFaultActive() || contactorsOpen()) && readyToDriveLatched) {
    dropReadyToDrive(rtdDropCauseText());
  }

  if (now - lastImdGetReqMs >= IMD_GET_REQ_PERIOD_MS) {
    lastImdGetReqMs = now;
    requestImdWarnings();
  }

  // Latch the red blink the first time a fault is seen; once latched the TSSI
  // stays red until power cycle, even if the underlying fault clears.
  if (faultDetectionArmed && anyFaultActive()) {
    tssiFaultLatched = true;
  }

  if (tssiFaultLatched) {
    updateTssiLed(true, now);
  } else if (!faultDetectionArmed) {
    setTssiGreenOn();
  } else {
    updateTssiLed(anyFaultActive(), now);
  }
}

void updateTssiLed(bool faultActive, uint32_t now) {
  if (faultActive) {
    uint32_t elapsed = now - tssiBlinkTimerMs;
    if (elapsed >= TSSI_BLINK_PERIOD_MS) {
      tssiBlinkTimerMs += TSSI_BLINK_PERIOD_MS;
      elapsed = now - tssiBlinkTimerMs;
    }

    if (elapsed < TSSI_BLINK_ON_MS) {
      if (!tssiRedOn) {
        setTssiRedOn();
      }
    } else if (tssiRedOn) {
      setTssiRedOff();
    }
  } else {
    setTssiGreenOn();
  }
}

void setTssiGreenOn() {
  digitalWrite(TSSI_LED_RED, LOW);
  digitalWrite(TSSI_LED_GREEN, HIGH);
  tssiRedOn = false;
}

void setTssiRedOn() {
  digitalWrite(TSSI_LED_GREEN, LOW);
  digitalWrite(TSSI_LED_RED, HIGH);
  tssiRedOn = true;
}

void setTssiRedOff() {
  digitalWrite(TSSI_LED_RED, LOW);
  tssiRedOn = false;
}

void setTssiAllOff() {
  digitalWrite(TSSI_LED_GREEN, LOW);
  digitalWrite(TSSI_LED_RED, LOW);
  tssiRedOn = false;
}

void printTssiWarnBits(uint16_t bits) {
  if (bits & (1u << 0)) Serial.print(" [DeviceError]");
  if (bits & (1u << 1)) Serial.print(" [HV+ConnFail]");
  if (bits & (1u << 2)) Serial.print(" [HV-ConnFail]");
  if (bits & (1u << 3)) Serial.print(" [EarthConnFail]");
  if (bits & (1u << 4)) Serial.print(" [IsoAlarm]");
  if (bits & (1u << 5)) Serial.print(" [IsoWarning]");
  if (bits & (1u << 6)) Serial.print(" [IsoOutdated]");
  if (bits & (1u << 7)) Serial.print(" [UnbalanceAlarm]");
  if (bits & (1u << 8)) Serial.print(" [Undervoltage]");
  if (bits & (1u << 9)) Serial.print(" [UnsafeToStart]");
  if (bits & (1u << 10)) Serial.print(" [EarthliftOpen]");
}
