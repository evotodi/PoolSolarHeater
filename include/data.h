#pragma once

#include "defines.h"
#include "globals.h"
#include "utils.h"
#include "display.h"

#include <Homie.h>
#include <SolarCalculator.h>
#include <TimeLib.h>

void getSolar(Solar * pSolar);
void getDaylight(Daylight * pDaylight);

void addPoolTemp();
void addAirTemp();

bool checkTempSensors();

void setRunStatus(RunStatus rs, bool force = false);
const char * getRunStatusStr();

void setupOwSensors();
int16_t mcpReadCallback(uint8_t channel);
