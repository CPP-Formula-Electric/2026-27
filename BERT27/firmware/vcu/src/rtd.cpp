/**
 * @file    rtd.cpp
 * @brief   Runs the ready-to-drive state machine: RTD button, entry conditions, drive mode and drop causes.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "rtd.h"

#include "pins.h"
#include "safety_limits.h"
#include "buzzer.h"
#include "imd.h"
#include "bms_link.h"
#include "faults.h"
#include "pedals.h"
#include "main.h"
#include "can_trace.h"

bool rtdInputsReady();
bool readyForRtdButton();
void logRtdBlockedReason();
void handleRtdButtonPressed();
void setRtdState(RtdState newState);
const char *rtdStateName(RtdState state);

static constexpr uint32_t RTD_BLOCKED_LOG_PERIOD_MS = 1000;

// ---------------- Runtime state ----------------
char driveMode = 'N';  // RTD state machine sets D only while ready to drive.

RtdState rtdState = RTD_STATE_FAULT_BLOCKED;
// Compatibility flag used by the display and PM100 gate; kept in sync with rtdState.
bool readyToDriveLatched = false;

bool readyButtonLast = false;

// EV.10.2: ready-to-drive sound, 1-3 seconds. Solid continuous tone for the
// first 1.25 s (satisfies the >=1 s continuous requirement), then a short
// playful flourish. Step on+off times sum to 2000 ms total.
static const ToneStep RTD_PATTERN[] = {
  {2093, 1250,  0},  // C7 - continuous main sound (1.25 s)
  {2637,  120, 40},  // E7
  {3136,  120, 40},  // G7
  {2637,  120, 40},  // E7
  {3136,  270,  0}   // G7 - final flourish
};

// ---------------- Ready to drive state machine ----------------
void serviceReadyToDrive() {
  refreshRtdStateFromInputs();

  bool pressed = readReadyButtonPressed();

  if (pressed && !readyButtonLast) {
    handleRtdButtonPressed();
    screenDirty = true;
  }

  readyButtonLast = pressed;
}

bool readReadyButtonPressed() {
  return digitalRead(READY_BUTTON_PIN) == HIGH;
}

bool rtdInputsReady() {
  if (!bmsImdFaultDetectionArmed()) {
    return false;
  }

  // APPS implausibility intentionally does not gate RTD; it zeroes the torque
  // command instead (see updatePedal/updatePm100CommandState).
  return !anyFaultActive() && !contactorsOpen();
}

bool readyForRtdButton() {
  return rtdState == RTD_STATE_READY_FOR_BUTTON;
}

void refreshRtdStateFromInputs() {
  if (rtdState == RTD_STATE_READY_TO_DRIVE) {
    if (!rtdInputsReady()) {
      dropReadyToDrive(rtdDropCauseText());
    }
    return;
  }

  bool inputsReady = rtdInputsReady();
  if (!inputsReady) {
    logRtdBlockedReason();
  }

  setRtdState(inputsReady ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);
}

// Periodically prints why RTD_STATE_FAULT_BLOCKED hasn't cleared, e.g. so a
// stuck contactor/fault/IMD flag after an HV power cycle shows up in the
// serial log instead of just leaving the dashboard stuck on "N".
void logRtdBlockedReason() {
  static uint32_t lastLogMs = 0;
  uint32_t now = millis();
  if (now - lastLogMs < RTD_BLOCKED_LOG_PERIOD_MS) {
    return;
  }
  lastLogMs = now;

  if (!bmsImdFaultDetectionArmed()) {
    Serial.println("[RTD] Blocked: startup fault-detection delay active");
    return;
  }

  Serial.print("[RTD] Blocked:");
  if (imdFault)            Serial.print(" imdFault");
  if (bmsFault)            Serial.print(" bmsFault");
  if (contactorsOpen())    Serial.print(" contactorsOpen");
  if (imdCanTimedOut)      Serial.print(" imdCanTimedOut");
  if (bmsTimedOut)         Serial.print(" bmsTimedOut");
  Serial.print(" bmsFlags=0x");
  printCanHexByte(bmsFlags);
  Serial.print(" imdWarnAlarms=0x");
  Serial.print(imdWarnAlarms, HEX);
  Serial.println();
}

void handleRtdButtonPressed() {
  Serial.print("[RTD] Button pressed, state=");
  Serial.println(rtdStateName(rtdState));

  switch (rtdState) {
    case RTD_STATE_READY_FOR_BUTTON:
      if (REQUIRE_BRAKE_FOR_RTD && !braking) {
        Serial.println("[RTD] Not latched; brake pedal must be pressed");
        break;
      }
      setRtdState(RTD_STATE_READY_TO_DRIVE);
      playTonePattern(RTD_PATTERN, sizeof(RTD_PATTERN) / sizeof(RTD_PATTERN[0]), false);
      Serial.println("[RTD] Ready to drive latched");
      break;

    case RTD_STATE_READY_TO_DRIVE:
      dropReadyToDrive("RTD button pressed");
      break;

    case RTD_STATE_FAULT_BLOCKED:
    default:
      setRtdState(RTD_STATE_FAULT_BLOCKED);
      Serial.println(anyFaultActive() ? "[RTD] Not latched; anyFault is active"
                                       : "[RTD] Not latched; contactors open");
      break;
  }
}

void setRtdState(RtdState newState) {
  char newDriveMode = (newState == RTD_STATE_READY_TO_DRIVE) ? 'D' : 'N';

  if (rtdState == newState) {
    readyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
    if (driveMode != newDriveMode) {
      setDriveMode(newDriveMode);
      screenDirty = true;
    }
    return;
  }

  rtdState = newState;
  readyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
  setDriveMode(newDriveMode);
  screenDirty = true;
}

void dropReadyToDrive(const char *reason) {
  bool wasLatched = (rtdState == RTD_STATE_READY_TO_DRIVE) || readyToDriveLatched;

  setRtdState(rtdInputsReady() ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);

  if (wasLatched && reason != nullptr) {
    Serial.print("[RTD] Dropped; ");
    Serial.println(reason);
    if (strcmp(reason, "RTD button pressed") != 0) {
      showFaultBanner(reason);
    }
  }
}

// Names the specific condition currently blocking RTD, for serial logs and
// the dashboard fault banner.
const char *rtdDropCauseText() {
  if (bmsTimedOut && haveBmsStatus) return "BMS Timeout";
  if (imdCanTimedOut) return "IMD Timeout";
  if (bmsFault) {
    return (haveBms2026FaultCode && bms2026FaultCode != 0)
               ? bmsTemperatureFaultCodeName(bms2026FaultCode)
               : "BMS Fault";
  }
  if (imdFault) return firstTssiFaultName(imdWarnAlarms);
  if (contactorsOpen()) return "Contactors Open";
  return "Fault";
}

const char *rtdStateName(RtdState state) {
  switch (state) {
    case RTD_STATE_FAULT_BLOCKED:
      return "FAULT_BLOCKED";
    case RTD_STATE_READY_FOR_BUTTON:
      return "READY_FOR_BUTTON";
    case RTD_STATE_READY_TO_DRIVE:
      return "READY_TO_DRIVE";
    default:
      return "UNKNOWN";
  }
}

void setDriveMode(char newDriveMode) {
  if (newDriveMode != 'D' && newDriveMode != 'N') {
    return;
  }

  if (driveMode == newDriveMode) {
    return;
  }

  driveMode = newDriveMode;
  screenDirty = true;
}
