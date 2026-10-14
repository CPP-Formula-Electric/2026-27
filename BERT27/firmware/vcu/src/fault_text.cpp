/**
 * @file    fault_text.cpp
 * @brief   Builds the fault names shown on the dashboard from the TSSI, BMS and PM100 fault bits.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#include "fault_text.h"

#include "imd.h"
#include "bms_link.h"
#include "inverter.h"
#include "faults.h"
#include "pedals.h"

static constexpr uint32_t FAULT_DISPLAY_CYCLE_MS = 1500;

static constexpr uint8_t DASHBOARD_MAX_FAULT_TEXTS = 48;

struct DashboardFaultList {
  const char *texts[DASHBOARD_MAX_FAULT_TEXTS];
  uint8_t count;
};

void collectDashboardFaults(DashboardFaultList &faults);
void appendDashboardFault(DashboardFaultList &faults, const char *text);
void appendTssiFaultNames(DashboardFaultList &faults, uint16_t bits);
void appendPm100FaultNames(DashboardFaultList &faults, uint32_t postFaults, uint32_t runFaults);

const char *dashboardFaultText() {
  DashboardFaultList faults;
  collectDashboardFaults(faults);

  if (faults.count == 0) {
    if (!bmsImdFaultDetectionArmed()) {
      return "Startup Wait";
    }
    return "No Fault";
  }

  if (faults.count == 1) {
    return faults.texts[0];
  }

  uint8_t faultIndex = (millis() / FAULT_DISPLAY_CYCLE_MS) % faults.count;
  return faults.texts[faultIndex];
}

void collectDashboardFaults(DashboardFaultList &faults) {
  faults.count = 0;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (faultDetectionArmed && bmsTimedOut) {
    appendDashboardFault(faults, "BMS Timeout");
  }

  if (faultDetectionArmed && imdCanTimedOut) {
    appendDashboardFault(faults, "IMD Timeout");
  }

  if (faultDetectionArmed) {
    if (haveBms2026FaultCode && bms2026FaultCode != 0) {
      appendDashboardFault(faults, bmsTemperatureFaultCodeName(bms2026FaultCode));
    } else if (bmsFault && haveBmsStatus && !haveBms2026FaultCode) {
      appendDashboardFault(faults, "BMS Fault");
    }
  }

  if (faultDetectionArmed && imdFault && (imdWarnAlarms != 0 || !imdCanTimedOut)) {
    uint8_t beforeCount = faults.count;
    appendTssiFaultNames(faults, imdWarnAlarms);
    if (faults.count == beforeCount) {
      appendDashboardFault(faults, "IMD Fault");
    }
  }

  if (havePm100FaultStatus && pm100Fault) {
    uint8_t beforeCount = faults.count;
    appendPm100FaultNames(faults, pm100PostFaults, pm100RunFaults);
    if (faults.count == beforeCount) {
      appendDashboardFault(faults, "Inv Fault");
    }
  }

  if (!appsPlausible) {
    appendDashboardFault(faults, appsFaultText);
  }

  if (bspcActive) {
    appendDashboardFault(faults, "BSPC Zero Trq");
  }
}

void appendDashboardFault(DashboardFaultList &faults, const char *text) {
  if (faults.count >= DASHBOARD_MAX_FAULT_TEXTS || text == nullptr || text[0] == '\0') {
    return;
  }

  faults.texts[faults.count++] = text;
}

void appendTssiFaultNames(DashboardFaultList &faults, uint16_t bits) {
  if (bits & (1u << 0)) appendDashboardFault(faults, "DeviceError");
  if (bits & (1u << 1)) appendDashboardFault(faults, "HV+ConnFail");
  if (bits & (1u << 2)) appendDashboardFault(faults, "HV-ConnFail");
  if (bits & (1u << 3)) appendDashboardFault(faults, "EarthConnFail");
  if (bits & (1u << 4)) appendDashboardFault(faults, "IsoAlarm");
  if (bits & (1u << 7)) appendDashboardFault(faults, "Unbalance");
  if (bits & (1u << 8)) appendDashboardFault(faults, "Undervolt");
  if (bits & (1u << 9)) appendDashboardFault(faults, "UnsafeStart");
  if (bits & (1u << 10)) appendDashboardFault(faults, "EarthliftOpen");
  if (bits & (1u << 5)) appendDashboardFault(faults, "IsoWarning");
  if (bits & (1u << 6)) appendDashboardFault(faults, "IsoOutdated");
}

void appendPm100FaultNames(DashboardFaultList &faults, uint32_t postFaults, uint32_t runFaults) {
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) {
      appendDashboardFault(faults, pm100PostFaultName(bit));
    }
  }

  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) {
      appendDashboardFault(faults, pm100RunFaultName(bit));
    }
  }
}

const char *firstTssiFaultName(uint16_t bits) {
  if (bits & (1u << 0)) return "DeviceError";
  if (bits & (1u << 1)) return "HV+ConnFail";
  if (bits & (1u << 2)) return "HV-ConnFail";
  if (bits & (1u << 3)) return "EarthConnFail";
  if (bits & (1u << 4)) return "IsoAlarm";
  if (bits & (1u << 7)) return "Unbalance";
  if (bits & (1u << 8)) return "Undervolt";
  if (bits & (1u << 9)) return "UnsafeStart";
  if (bits & (1u << 10)) return "EarthliftOpen";
  if (bits & (1u << 5)) return "IsoWarning";
  if (bits & (1u << 6)) return "IsoOutdated";
  return "Fault";
}

const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults) {
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) return pm100PostFaultName(bit);
  }
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) return pm100RunFaultName(bit);
  }
  return "No Fault";
}

const char *bmsTemperatureFaultCodeName(uint8_t faultCode) {
  switch (faultCode) {
    case 0:  return "No Fault";
    case 1:  return "Cell UV";
    case 2:  return "Cell OV";
    case 3:  return "Pack UV";
    case 4:  return "Pack OV";
    case 5:  return "Over Temp";
    case 6:  return "Cell Spread";
    case 7:  return "Prchg Timeout";
    case 8:  return "Prchg Fast";
    case 9:  return "Pack V Sense";
    case 10: return "Chgr Comms";
    case 11: return "BQ Init Fail";
    case 12: return "BQ Comms Fail";
    default: return "BMS Fault";
  }
}

const char *pm100PostFaultName(uint8_t bit) {
  switch (bit) {
    case 0: return "POST GateDesat";
    case 1: return "POST OverCur";
    case 2: return "POST AccelShrt";
    case 3: return "POST AccelOpen";
    case 4: return "POST CurSensLo";
    case 5: return "POST CurSensHi";
    case 6: return "POST ModTempLo";
    case 7: return "POST ModTempHi";
    case 8: return "POST CtrlTmpLo";
    case 9: return "POST CtrlTmpHi";
    case 10: return "POST GateTmpLo";
    case 11: return "POST GateTmpHi";
    case 12: return "POST 5V Low";
    case 13: return "POST 5V High";
    case 14: return "POST 12V Low";
    case 15: return "POST 12V High";
    case 16: return "POST 2.5V Lo";
    case 17: return "POST 2.5V Hi";
    case 18: return "POST 1.5V Lo";
    case 19: return "POST 1.5V Hi";
    case 20: return "POST DCBusHi";
    case 21: return "POST DCBusLo";
    case 22: return "POST PrechTmo";
    case 23: return "POST PrechVolt";
    case 24: return "POST EEPROMSum";
    case 25: return "POST EEPROMRg";
    case 26: return "POST EEPROMUpd";
    case 27: return "POST HWBusOV";
    case 28: return "POST GateInit";
    case 29: return "POST Reserved";
    case 30: return "POST BrakeShrt";
    case 31: return "POST BrakeOpen";
    default: return "POST Fault";
  }
}

const char *pm100RunFaultName(uint8_t bit) {
  switch (bit) {
    case 0: return "Mtr Overspeed";
    case 1: return "Inv Overcur";
    case 2: return "Inv Overvolt";
    case 3: return "Inv Overtemp";
    case 4: return "Accel Short";
    case 5: return "Accel Open";
    case 6: return "Dir Cmd Fault";
    case 7: return "Inv Resp Tmo";
    case 8: return "Gate/Desat";
    case 9: return "HW Overcur";
    case 10: return "Undervolt";
    case 11: return "CAN Cmd Lost";
    case 12: return "Mtr Overtemp";
    case 13: return "Run Reserved45";
    case 14: return "Run Reserved46";
    case 15: return "Run Reserved47";
    case 16: return "Brake Short";
    case 17: return "Brake Open";
    case 18: return "ModA Overtemp";
    case 19: return "ModB Overtemp";
    case 20: return "ModC Overtemp";
    case 21: return "PCB Overtemp";
    case 22: return "Gate1 Overtemp";
    case 23: return "Gate2 Overtemp";
    case 24: return "Gate3 Overtemp";
    case 25: return "Cur Sensor";
    case 26: return "Gate Overvolt";
    case 27: return "HWBusOV Gen3";
    case 28: return "HWBusOV Gen5";
    case 29: return "Run Reserved61";
    case 30: return "Resolver Disc";
    case 31: return "Run Reserved63";
    default: return "Inv Fault";
  }
}
