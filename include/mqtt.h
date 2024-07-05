#pragma once

#include "defines.h"
#include "globals.h"
#include "config.h"
#include "control.h"
#include <Homie.h>

bool configNodeInputHandler(const HomieRange &range, const String &property, const String &value);
void toggleForceOn(int8_t setTo = -1);
bool mqttForceOnHandler(const HomieRange &range, const String &value);
