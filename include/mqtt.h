#pragma once

#include "defines.h"
#include "globals.h"
#include "config.h"
#include "control.h"
#include <Homie.h>

bool configNodeInputHandler(const HomieRange &range, const String &property, const String &value);
//bool mqttHeatOnHandler(const HomieRange &range, const String &value);
//void toggleOverrideEnv();
//void toggleManualHeatingEnable();
//void toggleManualHeating();
//void toggleEnvNoCheckSolar();
//void toggleEnvNoCheckAir();
//void toggleEnvNoCheckCloud();
//void toggleEnvNoCheckTDiff();
//void toggleEnvNoCheckAuxHeatDiff();
