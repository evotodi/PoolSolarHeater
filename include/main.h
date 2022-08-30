#pragma once

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EasyStringStream.h>
#include "MCP_ADC.h"
#include <Thermistor.h>
#include <ArduinoJson.h>

#define DEBUG 1
// #define TESTING 1
#define NO_ENV_HOUR_CHECK 1
//#define NO_ENV_SP_CHECK 1

#define boolToStr(x) ((x)?"Yes":"No")

#define DS_TEMP_PRECISION 12

#define ONE_WIRE_BUS   D2
#define MCP_DIN        D6
#define MCP_DOUT       D7
#define MCP_CLK        D5
#define MCP_CS         D8
#define RLY_PIN        D1
#define LED_PIN        D0

#define ADC_LIGHT      0
#define ADC_AMBIANT    1
#define ADC_FRAME1     2
#define ADC_FRAME2     3


#define DARK_DEFAULT   400
#define GPM_DEFAULT    5.0
#define SP_DEFAULT     95.0

#define WATTS_MIN      200.0

#define LOOP_DAT_DLY      3*1E3
#define LOOP_PROC_DLY     5*1E3
#define LOOP_PUB_DLY      15*1E3
#define LOOP_CIRC_ON_DLY  10*1E3
#define LOOP_CIRC_OFF_DLY 60*1E3

#define LOOP_SLEEP_DLY   30*1E3

#define TIME_OFFSET_DST  -14400
#define TIME_OFFSET_ST  -18000
#define DST_BEGIN_DAY    13
#define DST_BEGIN_MONTH  3
#define DST_END_DAY      6
#define DST_END_MONTH    11
#define STAY_DARK_CNT    3
#define DAY_START_HOUR   9  // GMT - TIME_OFFSET
#define DAY_END_HOUR     20  // GMT - TIME_OFFSET

#define FRAME_TO_AIR_MIN_DIFF 5

//>> Structures
//<< Structures

//>> Function Prototypes
time_t getNtpTime();
int getTimeOffset(time_t t);
const char* getTimestamp();
void setupHandler();
void loopHandler();
void strToAddress(const String& addr, DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);
void setupOwSensors();
int16_t mcpReadCallback(uint8_t channel);
ThermistorSettings parseNTCSettings(const char * json, const char * name);
float calcWatts(float tempIn, float tempOut);
bool envAllowPump();
void doProcess();
bool turnPumpOn();
bool turnPumpOff();
#ifdef TESTING
void testing();
#endif
//<< Function Prototypes
