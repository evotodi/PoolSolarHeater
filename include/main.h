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
#include "PoolSetting.h"
#include "PoolValidation.h"

// Debugging Defines >>>
#define DEBUG 1
//#define LOG_TO_TELNET 1
//#define NO_ENV_SOLAR_CHECK 1
//#define NO_ENV_AIR_CHECK 1
//#define NO_ENV_CLOUD_CHECK 1
//#define NO_ENV_IN_OUT_DIFF_CHECK 1
//#define DEBUG_FORCE_TIME 1687217416
#define PRINT_POOL_CONFIG_ON_READ_WRITE 1
// <<< Debugging Defines

#define LOOP_DAT_DLY          (5*1E3)
#define LOOP_PROC_DLY         (5*1E3)
#define LOOP_PUB_DLY         (15*1E3)
#define LOOP_PUB_CFG_DLY     (60*1E3)
#define LOOP_HB_DLY           (5*1E3)
#define LOOP_DAYLIGHT_DLY   (1800*1E3)

#define boolToStr(x) ((x)?"Yes":"No")

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
#define ADC_AIR        1
#define ADC_POOL       2

#define MAX_JSON_CONFIG_ARDUINOJSON_BUFFER_SIZE 1024

#define CONFIG_PATH "/pool/config.json"

//>> Structures
struct DTSetting
{
    char addr[18];
    float offset; // This is stored in spiffs pool/config.json
};

struct NTCSetting
{
    uint8_t pin;
    float offset; // This is stored in spiffs pool/config.json
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
void parseNTCSettings(ThermistorSettings * ts, const char * settings, const char * name);
void parseDTSettings(DTSetting * pDTSetting, const char * settings, double offset, const char * name);
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
bool configLoad();
void configRead();
void configSetPoolSettings(JsonObject settingsObject, std::vector<PoolInternals::IPoolSetting*> * settings);
void configLogSettings(const char * name, std::vector<PoolInternals::IPoolSetting*> * settings);
void configWrite();
void configUpdateStructs();
float ItoF(int val);
String ItoS(int val);
int FtoI(float val);
void addPoolTemp();
void addAirTemp();
bool mqttHeatOnHandler(const HomieRange& range, const String& value);
bool configNodeInputHandler(const HomieRange& range, const String& property, const String& value);
//<< Function Prototypes