/**
 * @file    inverter.h
 * @brief   PM100DX status frames, CAN broadcast enable and automatic fault clear.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_INVERTER_H
#define VCU_INVERTER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

extern float inverterDcBusVoltage;
extern float inverterDcBusCurrent;
extern bool haveInverterVoltageInfo;
extern bool haveInverterCurrentInfo;
extern float glvVoltage;
extern bool haveGlvVoltage;
extern bool havePm100Traffic;
extern uint32_t firstPm100FrameMs;
extern float powerKw;
extern int16_t inverterTempTenthsC;
extern int16_t pm100MotorTempTenthsC;
extern bool haveInverterTemp;
extern bool havePm100MotorTemp;
extern uint32_t pm100PostFaults;
extern uint32_t pm100RunFaults;
extern bool havePm100FaultStatus;
extern bool pm100Fault;
extern int16_t motorRpm;

void handlePm100Temperatures1Frame(const CAN_message_t &msg);
void handlePm100Temperatures3Frame(const CAN_message_t &msg);
void handlePm100RpmFrame(const CAN_message_t &msg);
void handlePm100CurrentInfoFrame(const CAN_message_t &msg);
void handlePm100VoltageInfoFrame(const CAN_message_t &msg);
void handlePm100InternalVoltagesFrame(const CAN_message_t &msg);
void handlePm100ParamResponseFrame(const CAN_message_t &msg);
void servicePm100BroadcastEnable();
void servicePm100FaultClear();
void handlePm100FaultCodesFrame(const CAN_message_t &msg);

#endif  // VCU_INVERTER_H
