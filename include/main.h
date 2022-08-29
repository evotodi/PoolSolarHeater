#ifndef POOL_SOLAR_HEATER_H
#define POOL_SOLAR_HEATER_H

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

#define DEBUG 1
// #define TESTING 1

#define DS_TEMP_PRECISION 12

#define ONE_WIRE_BUS   D2
#define MCP_DIN        D6
#define MCP_DOUT       D7
#define MCP_CLK        D5
#define MCP_CS         D8

#define ADC_LIGHT      0
#define ADC_FRAME1     1
#define ADC_FRAME2     2
#define ADC_AMBIANT    3

#define DARK_DEFAULT   400
#define GPM_DEFAULT    5

#define WATTS_MIN      100.0
#define LOOP_RUN_DLY     3*1E3
#define LOOP_SLEEP_DLY   600*1E3

//>> Structures
struct NTCSettings
{
    float vcc = 4.55; 
    float adcRef = 4.55; 
    int serRes = 10000; 
    int ntcRes = 10000;
    int tempNom = 25; 
    int bc = 3950; 
    int samples = 5; 
    int sampleDly = 20;
};
//<< Structures

//>> Function Prototypes
time_t getNtpTime();
const char* getTimestamp();
void setupHandler();
void loopHandler();
void strToAddress(const String addr, DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);
void setupOwSensors();
int16_t mcpReadCallback(uint8_t channel);
void validateHomieSettings();
NTCSettings parseNTCSettings(const char * settings, const char * name);
float calcWatts(float tempIn, float tempOut);
#ifdef TESTING
void testing();
#endif
//<< Function Prototypes


#endif