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
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>

#define DEBUG 1
//#define TESTING 1
#define LOG_TO_TELNET 1
//#define NO_ENV_HOUR_CHECK 1
//#define NO_ENV_SP_CHECK 1

#define boolToStr(x) ((x)?"Yes":"No")

#define TELNET_PORT 23
#define DS_TEMP_PRECISION 12

#define ONE_WIRE_BUS   D2
#define MCP_DIN        D6
#define MCP_DOUT       D7
#define MCP_CLK        D5
#define MCP_CS         D8
#define RLY_PIN        D1

#define ADC_LIGHT      0
#define ADC_AMBIANT    1
#define ADC_FRAME1     2
#define ADC_FRAME2     3


#define DARK_DEFAULT      400
#define GPM_DEFAULT       5.0
#define SP_DEFAULT        95.0
#define TOFS_DEFAULT      0.0
#define AIR_DIFF_DEFAULT  5
#define WATTS_MIN_DEFAULT 200.0

#define LOOP_DAT_DLY      5*1E3
#define LOOP_PROC_DLY     5*1E3
#define LOOP_PUB_DLY      15*1E3
#define LOOP_CIRC_DLY     10*1E3

#define LOOP_SLEEP_DLY   60*1E3

#define TIME_OFFSET_DST  -14400
#define TIME_OFFSET_ST  -18000
#define DST_BEGIN_DAY    13
#define DST_BEGIN_MONTH  3
#define DST_END_DAY      6
#define DST_END_MONTH    11
#define STAY_DARK_CNT    3
#define DAY_START_HOUR   9  // GMT - TIME_OFFSET
#define DAY_END_HOUR     20  // GMT - TIME_OFFSET

#define SP_HYSTERESIS 2.0 // Degrees under setpoint before turning pump on
#define CIRC_LOOP_OFF_CNT 59 // Min of 2; (This * LOOP_CIRC_DLY) + LOOP_CIRC_DLY; 3 * 10 + 10 = 40 seconds
#define CIRC_LOOP_ON_CNT 4 //Min of 2; (This * LOOP_CIRC_DLY) - LOOP_CIRC_DLY; 2 * 10 - 10 = 10 seconds

//>> Structures
struct PSHConfig
{
    int16_t dark;
    float gpm;
    float setpoint;
    int airDiff;
    float minWatts;
};

struct DTSetting
{
    char addr[18];
    float offset;
};
//<< Structures

//>> Function Prototypes
time_t getNtpTime();
int getTimeOffset(time_t t);
const char* getTimestamp();
void setupHandler();
void loopHandler();
#ifdef LOG_TO_TELNET
void handleTelnet();
void printHelp();
void onHomieEvent(const HomieEvent& event);
#endif
void strToAddress(const String& addr, DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);
void setupOwSensors();
int16_t mcpReadCallback(uint8_t channel);
ThermistorSettings parseNTCSettings(const char * settings, const char * name);
PSHConfig parsePSHSettings(const char * settings, const char * name);
DTSetting parseDTSettings(const char * settings, const char * name);
float calcWatts(float tempIn, float tempOut);
bool envAllowPump(bool overrideEnv = false);
bool envAllowGather();
void doProcess();
void doCirculate();
bool turnPumpOn(bool overrideEnv = false);
bool turnPumpOff();
#ifdef TESTING
void testing();
#endif
//<< Function Prototypes
