#pragma once

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
#include <Thermistor.h>
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

#define VERSION "2.1.63"

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

// ESP32 Usable Pins
#ifndef LED_BUILTIN
#  define LED_BUILTIN     2
#endif
#define PIN_AUX_4_SP    4 // SPARE Strapping
#define MCP_CS          5
#define ONE_WIRE_BUS    13
#define BTN2_PIN        14 // Button 2
#define TFT_CS          15
#define TFT_RST         16
#define PIN_AUX_17      17
#define SPI_CLK         18 // MCP_CLK, DISP_SCK
#define SPI_MISO        19 // MCP_DOUT, DISP_MISO
#define LED_PIN         21
#define PIN_AUX_22      22 // SPARE
#define SPI_MOSI        23 // MCP_DIN, DISP_MOSI
#define PUMP_RLY_PIN    25
#define HEAT_RLY_PIN    26
#define AUX_RLY_PIN     27
#define TFT_LED         32
#define TFT_DC          33
#define PIN_AUX_34      34
#define BTN1_PIN        35
#define PIN_AUX_36_I    36 // SPARE Input Only
#define PIN_AUX_39_I    39 // SPARE Input Only

// MCP3204 ADC Ports
#define ADC_LIGHT      0
#define ADC_AIR        1
#define ADC_POOL       2
#define ADC_AUX        3

#define MAX_JSON_CONFIG_ARDUINOJSON_BUFFER_SIZE 1024
#define CONFIG_PATH "/pool/config.json"
#define LOOP_DAT_DLY          (5*1E3)

// Colors
#define TFT_LT_BLUE 0xBDFF

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

//>> Enums
/**
 * Update the DISP_LAST_NUM_OF_PAGES to equal the total number of pages in the enum
 */
enum DisplayPage {
    DISP_MAIN = 0,
    DISP_INFO = 1,
    DISP_CONFIG = 2,
    DISP_LAST_NUM_OF_PAGES = 3,
};

/**
 * Update the MENU_LAST_NUM_OF_PAGES to equal the total number of pages in the enum
 */
enum MenuPage {
    MENU_MAIN = 0,
    MENU_CONFIG = 1,
    MENU_LAST_NUM_OF_PAGES = 2,
};
//<< Enums

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
#endif
void onHomieEvent(const HomieEvent& event);
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
bool heatOn();
bool heatOff();
void setPumpOn();
void setPumpOff();
void setPropaneOn();
void setPropaneOff();
void setAuxOn();
void setAuxOff();
float calcWatts(float tempIn, float tempOut);
bool envAllowHeat();
void calibratePoolTemps();
void calibrationReset();

float ItoF(int val);
String ItoS(int val);
int FtoI(float val);

void addPoolTemp();
void addAirTemp();
bool mqttHeatOnHandler(const HomieRange& range, const String& value);
bool configNodeInputHandler(const HomieRange& range, const String& property, const String& value);

bool configLoad();
void configRead();
void configWrite();
void configSetPoolSettings(JsonObject settingsObject, std::vector<PoolInternals::IPoolSetting*> * settings);
void configLogSettings(const char * name, std::vector<PoolInternals::IPoolSetting*> * settings);
void configUpdateStructs();

void setupButtons();
void menuMainBtn1KeyPress(void);
void menuMainBtn1DblClick(void);
void menuConfigBtn1KeyPress(void);
void menuConfigBtn1LongPress(void);
void menuConfigBtn1DblClick(void);

void displayCenterMessage(const char * str);
void displayCenterMessage(std::string str);
void displayPageMain();
void displayPageMainUpdate();
void displayPageInfo();
void displayPageConfig();
//<< Function Prototypes