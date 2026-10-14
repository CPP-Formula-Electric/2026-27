/**
 * @file    imd.cpp
 * @brief   Bender iso175 IMD frames and the warning/alarm poll.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 * Rules:   FSAE 2027 V1 EV.6.1.7
 */
#include "imd.h"

#include "can_bytes.h"
#include "main.h"
#include "vehicle_can.h"
#include "faults.h"

bool imdFault = true;
uint16_t imdWarnAlarms = 0;
uint16_t imdRIsoKohm = 0xFFFF;
uint8_t imdRIsoStatus = 0;
uint8_t imdDeviceActivity = 0;
uint32_t lastImdRxMs = 0;
bool imdCanTimedOut = true;

void handleImdFrame(const CAN_message_t &msg) {
  bool wasTimedOut = imdCanTimedOut;
  uint16_t previousWarnings = imdWarnAlarms;
  bool previousFault = imdFault;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (msg.id == IMD_INFO_GENERAL_ID && msg.len >= 5) {
    imdRIsoKohm = u16Le(msg.buf[0], msg.buf[1]);
    imdRIsoStatus = msg.buf[2];
    imdWarnAlarms = (imdWarnAlarms & 0xFF00u) | msg.buf[4];
    imdDeviceActivity = (msg.len >= 7) ? msg.buf[6] : IMD_SNV_BYTE;
    lastImdRxMs = millis();
    imdCanTimedOut = false;
    imdFault = tssiFaultFromWarnBits(imdWarnAlarms);
  } else if (msg.id == IMD_RESPONSE_ID && msg.len >= 3 && msg.buf[0] == IDX_WARN_ALARMS) {
    imdWarnAlarms = u16Le(msg.buf[1], msg.buf[2]);
    lastImdRxMs = millis();
    imdCanTimedOut = false;
    imdFault = tssiFaultFromWarnBits(imdWarnAlarms);
  } else {
    return;
  }

  if (wasTimedOut) {
    Serial.println("[TSSI] IMD CAN restored");
  }

  if (faultDetectionArmed && (imdWarnAlarms != previousWarnings || imdFault != previousFault)) {
    Serial.print("[TSSI] IMD warn=0x");
    Serial.print(imdWarnAlarms, HEX);
    printTssiWarnBits(imdWarnAlarms);
    Serial.print(" => ");
    Serial.println(imdFault ? "FAULT, red blink" : "OK, green on");
    if (imdFault && !previousFault) {
      showFaultBanner(firstTssiFaultName(imdWarnAlarms));
    }
  }
}

void requestImdWarnings() {
  CAN_message_t tx = {};
  tx.id = IMD_REQUEST_ID;
  tx.len = 8;
  tx.flags.extended = 0;
  tx.buf[0] = IDX_WARN_ALARMS;
  for (int i = 1; i < 8; i++) {
    tx.buf[i] = 0xFF;
  }
  sendCanFrame(tx);
}
