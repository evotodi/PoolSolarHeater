#pragma once

#include "defines.h"
#include "globals.h"
#include <Homie.h>
#include "PoolValidation.h"

bool configLoad();
void configRead();
void configWrite();
void configSetPoolSettings(JsonObject settingsObject, std::vector<PoolInternals::IPoolSetting *> *settings);
void configLogSettings(const char *name, std::vector<PoolInternals::IPoolSetting *> *settings);
void configUpdateStructs();
