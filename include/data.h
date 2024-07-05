#pragma once

#include "defines.h"
#include "globals.h"
#include "utils.h"
#include "display.h"

#include <Homie.h>
#include <SolarCalculator.h>
#include <TimeLib.h>

// Debugging Defines >>>
// The FAKE_TEMP_XXX values are for example 92.5 = 925
#define FAKE_TEMP_POOL 925
#define FAKE_TEMP_TIN 925
#define FAKE_TEMP_TOUT 973
#define FAKE_TEMP_AIR 834
#define FAKE_LIGHT 4500
// <<< Debugging Defines

void getSolar(Solar * pSolar);
void getDaylight(Daylight * pDaylight);

void addPoolTemp();
void addAirTemp();
void addTInTemp();
void addTOutSolarTemp();
void addTOutHeatTemp();
void addLight();

bool checkTempSensors();

bool wantsSolar();
bool wantsHeatAux();

void setRunStatus(RunStatus rs, bool force = false);
const char * getRunStatusStr();

void setupOwSensors();
int16_t mcpReadCallback(uint8_t channel);
