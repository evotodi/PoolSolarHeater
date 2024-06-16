#pragma once

#include "defines.h"
#include "globals.h"
#include "utils.h"
#include "display.h"
#include "data.h"

#include <Homie.h>

void doProcess();
bool heatOn();
bool heatOff();
void setPumpOn();
void setPumpOff();
void setAuxHeatOn();
void setAuxHeatOff();
void setAuxOn();
void setAuxOff();

bool envAllowHeat();

