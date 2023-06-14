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
#include <Oversampling.h>
#include <SolarCalculator.h>

#define DEBUG 1
//#define TESTING 1
// #define LOG_TO_TELNET 1
//#define NO_ENV_HOUR_CHECK 1
//#define NO_ENV_SP_CHECK 1

#define LOOP_DAT_DLY      5*1E3
#define LOOP_PROC_DLY     5*1E3
#define LOOP_PUB_DLY      15*1E3
#define LOOP_SLEEP_DLY   60*1E3

#define boolToStr(x) ((x)?"Yes":"No")

#define LATITUDE 38.502539
#define LONGITUDE -86.163691

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

#define DARK_DEFAULT      400
#define SP_DEFAULT        95.0
#define TOFS_DEFAULT      0.0

#define STAY_DARK_CNT    3
#define DAY_START_HOUR   9  // GMT - TIME_OFFSET
#define DAY_END_HOUR     20  // GMT - TIME_OFFSET

//>> Structures
struct PSHConfig
{
    int16_t dark;
    float setpoint;
    int airDiff;
};

struct DTSetting
{
    char addr[18];
    float offset;
};

struct Solar
{
    double azimuth;
    double elevation;
};
//<< Structures

//>> Function Prototypes
void setupHandler();
void loopHandler();
time_t getNtpTime();
const char* getTimestamp();
void strToAddress(const String& addr, DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);
void setupOwSensors();
int16_t mcpReadCallback(uint8_t channel);
int16_t adcReadCallback(uint8_t channel);
void parseNTCSettings(const char * settings, const char * name, ThermistorSettings * ts);
PSHConfig parsePSHSettings(const char * settings, const char * name);
DTSetting parseDTSettings(const char * settings, const char * name);
#ifdef LOG_TO_TELNET
void handleTelnet();
void printHelp();
void onHomieEvent(const HomieEvent& event);
#endif
void toggleOverrideEnv();
Solar getSolar();
//<< Function Prototypes