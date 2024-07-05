#pragma once

#include <Arduino.h>
#include <DallasTemperature.h>
#include <Homie.h>
#include "structures.h"

float ItoF(int val);
String ItoS(int val);
int FtoI(float val);
void strToAddress(const String &addr, DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);
float calcWatts(float tempIn, float tempOut, float gpm);
void parseDTSettings(DTSetting *pDTSetting, const char *settings, double offset);
bool strToBool(const char * str);