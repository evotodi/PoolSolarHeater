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
#include <Smoothed.h>
#include <EasyButton.h>

// Debugging Defines >>>
#define DEBUG 1
#define LOG_TO_TELNET 1
//#define NO_ENV_SOLAR_CHECK 1
//#define NO_ENV_AIR_CHECK 1
//#define NO_ENV_CLOUD_CHECK 1
//#define NO_ENV_IN_OUT_DIFF_CHECK 1
//#define DEBUG_FORCE_TIME 1687217416
// <<< Debugging Defines

#define LOOP_DAT_DLY          5*1E3
#define LOOP_PROC_DLY         5*1E3
#define LOOP_PUB_DLY         15*1E3
#define LOOP_HB_DLY           5*1E3
#define LOOP_DAYLIGHT_DLY   1800*1E3

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
#define CAL_PIN        3

#define ADC_LIGHT      0
#define ADC_AMBIANT    1
#define ADC_POOL       2

#define CLOUDY_DEFAULT      2800
#define OVERCAST_CNT    3

#define SP_DEFAULT        95.0
#define SP_HYSTERESIS_DEFAULT 2.0 // Degrees under setpoint before turning heat on
#define ELV_AM_DEFAULT 30.0
#define ELV_PM_DEFAULT 30.0
#define AIR_DIFF_DEFAULT 10.0
#define POOL_TEMP_IN_DEFAULT 1 // See struct PSHConfig
#define TIN_DIFF_MAX_DEFAULT 4.0

#define TIME_OFFSET_DST_HOURS -4
#define TIME_OFFSET_ST_HOURS -5
#define DST_BEGIN_DAY    13
#define DST_BEGIN_MONTH  3
#define DST_END_DAY      6
#define DST_END_MONTH    11

#define CONFIG_PATH "/pool/config.json"

//>> Structures
struct PSHConfig
{
    int16_t cloudy;
    float setpoint;
    float swing;
    float airDiff;
    float elevationMinAM;
    float elevationMinPM;
    uint16_t poolTempIn; // 0 = tin 1 = ntc
    float tinDiffMax;
};

struct DTSetting
{
    char addr[18];
    float offset; // This is stored in the eeprom
};

struct Solar
{
    double azimuth;
    double elevation;
};

struct Daylight
{
    double sunrise;
    double sunset;
    double transit;
    time_t midday;
};
//<< Structures

//>> Function Prototypes
void setupHandler();
void loopHandler();
time_t getNtpTime();
int getTimeOffset(const time_t * t, bool asHours = false);
const char* getTimestamp(bool withOffset = false);
void strToAddress(const String& addr, DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);
void setupOwSensors();
int16_t mcpReadCallback(uint8_t channel);
void parseNTCSettings(const char * settings, const char * name, ThermistorSettings * ts);
void parsePSHSettings(PSHConfig * pPSHConfig, const char * settings, const char * name);
void parseDTSettings(DTSetting * pDTSetting, const char * settings, const char * name);
#ifdef LOG_TO_TELNET
void handleTelnet();
void printHelp();
void onHomieEvent(const HomieEvent& event);
#endif
void toggleOverrideEnv();
void toggleManualHeatingEnable();
void toggleManualHeating();
void toggleEnvNoCheckSolar();
void toggleEnvNoCheckAir();
void toggleEnvNoCheckCloud();
void toggleEnvNoCheckTDiff();
void getSolar(Solar * pSolar);
void getDaylight(Daylight * pDaylight);
void doProcess();
bool turnHeatOn();
bool turnHeatOff();
bool envAllowHeat();
void calBtnISR();
void calibratePoolTemps();
void calibrationReset();
void readConfig();
void writeConfig();
float ItoF(int val);
String ItoS(int val);
int FtoI(float val);
void addPoolTemp();
//<< Function Prototypes