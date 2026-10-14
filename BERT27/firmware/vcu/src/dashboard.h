/**
 * @file    dashboard.h
 * @brief   Draws the ST7796 touchscreen dashboard and handles touch input.
 *
 * Board:   VCU (Teensy 4.1)
 * Original Creator: BERT26 software
 * Worked on by: BERT27 restructure
 */
#ifndef VCU_DASHBOARD_H
#define VCU_DASHBOARD_H

#include <Arduino.h>

static constexpr uint32_t SCREEN_PERIOD_MS = 100;
static constexpr uint32_t DASHBOARD_FULL_REDRAW_MS = 120000;  // periodic anti-glitch repaint

extern bool screenDirty;
extern bool dashboardNeedsFullRedraw;

void setupDisplay();
void handleTouch();
void drawDashboard();
void showFaultBanner(const char *text);
void serviceFaultBanner();

#endif  // VCU_DASHBOARD_H
