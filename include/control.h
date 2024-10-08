#pragma once

#include "defines.h"
#include "globals.h"
#include "utils.h"
#include "display.h"
#include "data.h"

#include <Homie.h>

void doProcess();

void setAllOff();
void setPumpOn();
void setPumpOff();
void setHeatAuxOn();
void setHeatAuxOff();
void setAuxOn();
void setAuxOff();
bool getEnable();
bool getForceOn();