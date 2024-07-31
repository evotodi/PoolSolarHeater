#pragma once

#include "defines.h"
#include "globals.h"
#include "structures.h"
#include "enums.h"
#include "utils.h"
#include "display.h"
#include "buttons.h"
#include "config.h"
#include "datetime.h"
#include "mqtt.h"
#include "control.h"
#include "data.h"

#include <vector>
#include <Arduino.h>
#include <Esp.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EasyStringStream.h>
#include "MCP_ADC.h"
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <Oversampling.h>
#include <SolarCalculator.h>
#include <Smoothed.h>
#include <PoolSetting.hpp>
#include <PoolValidation.h>
#include <SPI.h>
#include "InterruptButton.h"
#include "Adafruit_GFX.h"
#include <Adafruit_ILI9341.h>

//>> Function Prototypes
void setupHandler();
void loopHandler();

void loopGatherData();
void loopGatherProcess();
void loopControl();
void loopPublishData();
void loopPublishConfig();
void loopUpdateDaylight();
void loopHeartbeat();

#ifdef LOG_TO_TELNET
void handleTelnet();
void printHelp();
#endif
void onHomieEvent(const HomieEvent& event);
void processBtnSyncEvents();
void setupConfigPageItems();

//<< Function Prototypes