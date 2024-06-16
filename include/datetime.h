#pragma once

#include "defines.h"
#include "globals.h"
#include <Homie.h>
#include <TimeLib.h>

time_t getNtpTime();
int getTimeOffset(const time_t *t, bool asHours = false);
std::string getTimestamp(bool withOffset = false, bool human = false, time_t ts = 0);
