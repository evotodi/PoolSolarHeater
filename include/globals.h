#pragma once

#include <WiFiUdp.h>
#include <WiFiServer.h>
#include "defines.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "NTPClient.h"
#include "MCP_ADC.h"
#include "Oversampling.h"
#include "InterruptButton.h"
#include "Adafruit_ILI9341.h"
#include "structures.h"
#include "Thermistor.h"
#include "HomieNode.hpp"
#include "PoolSetting.hpp"
#include "Smoothed.h"
#include "EasyStringStream.h"
#include "enums.h"

extern OneWire oneWire;
extern DallasTemperature sensors;
extern DeviceAddress tempDeviceAddress;
// 289D6F49F67A3CF8
extern DeviceAddress tempSensorIn; // = { 0x28, 0x9D, 0x6F, 0x49, 0xF6, 0x7A, 0x3C, 0xF8 };
// 283E4749F6C13C1A
extern DeviceAddress tempSensorOut; // = { 0x28, 0x3E, 0x47, 0x49, 0xF6, 0xC1, 0x3C, 0x1A };

extern WiFiUDP ntpUDP;
extern NTPClient timeClient;

#ifdef LOG_TO_TELNET
extern WiFiServer TelnetServer;
extern WiFiClient Telnet;
#endif

extern MCP3204 mcp;
extern Oversampling adc;

extern InterruptButton button1;
//extern InterruptButton button2;

extern Adafruit_ILI9341 tft;

extern DTSetting tinSettings;
extern DTSetting toutSettings;

extern ThermistorSettings thermistorAirSettings;
extern Thermistor thermistorAir;
extern NTCSetting ntcAirSetting;
extern ThermistorSettings thermistorPoolSettings;
extern Thermistor thermistorPool;
extern NTCSetting ntcPoolSetting;

extern HomieNode statusNode;
extern HomieNode valuesNode;
extern HomieNode configNode;

extern DynamicJsonDocument poolJsonDoc;

extern PoolSetting<uint16_t> poolConfigCloudySetting;
extern PoolSetting<uint16_t> poolConfigOvercastCntSetting;
extern PoolSetting<double> poolConfigSunMinElvAMSetting;
extern PoolSetting<double> poolConfigSunMinElvPMSetting;
extern PoolSetting<double> poolConfigSetPointSetting;
extern PoolSetting<double> poolConfigSetPointSwingSetting;
extern PoolSetting<double> poolConfigAuxHeatDiffSetting;
extern PoolSetting<double> poolConfigAirPoolDiffSetting;
extern PoolSetting<uint16_t> poolConfigPoolTempInSetting;
extern PoolSetting<double> poolConfigPumpGpmSetting;
extern PoolSetting<double> poolAirOffsetSetting;
extern PoolSetting<double> poolPoolOffsetSetting;
extern PoolSetting<double> poolTinOffsetSetting;
extern PoolSetting<double> poolToutOffsetSetting;
extern PoolSetting<const char *> poolAirNtcSetting;
extern PoolSetting<const char *> poolPoolNtcSetting;
extern PoolSetting<const char *> poolTinDtSetting;
extern PoolSetting<const char *> poolToutDtSetting;
extern PoolSetting<int16_t> poolDstOffsetSetting;
extern PoolSetting<int16_t> poolStOffsetSetting;
extern PoolSetting<uint16_t> poolDstBeginDaySetting;
extern PoolSetting<uint16_t> poolDstBeginMonthSetting;
extern PoolSetting<uint16_t> poolDstEndDaySetting;
extern PoolSetting<uint16_t> poolDstEndMonthSetting;
extern PoolSetting<double> poolLatitudeSetting;
extern PoolSetting<double> poolLongitudeSetting;

extern unsigned long currentMillis;
extern unsigned long intervalData;
extern unsigned long previousMillisData;
extern unsigned long intervalProc;
extern unsigned long previousMillisProc;
extern unsigned long intervalPub;
extern unsigned long previousMillisPub;
extern unsigned long intervalPubCfg;
extern unsigned long previousMillisPubCfg;
extern unsigned long intervalHB;
extern unsigned long previousMillisHB;
extern unsigned long intervalDayLight;
extern unsigned long previousMillisDaylight;

extern bool isCloudy;
extern bool isOvercast;
extern bool atSetpoint;
extern bool isHeating;
extern bool overrideEnv;
extern bool manualHeatingEnable;
extern bool manualHeating;
extern bool envCheckNoSolar;
extern bool envCheckNoAir;
extern bool envCheckNoCloud;
extern bool envCheckNoTDiff;
extern bool envCheckNoAuxHeatDiff;
extern bool otaInProgress;
extern bool firstRun;

extern int telnetBuffer;
extern Smoothed<int16_t> light;
extern uint16_t cloudyCnt;
extern Smoothed<int> tin;
extern bool tinOk;
extern Smoothed<int> tout;
extern bool toutOk;
extern Smoothed<int> air;
extern bool airOk;
extern Smoothed<int> pool;
extern bool poolOk;
extern Smoothed<int> watts;
extern Solar solar;
extern Daylight daylight;
extern char statusBuffer[1024];
extern EasyStringStream status;
extern DisplayPage displayPage;
extern char ip[17];
extern RunStatus runStatus;
extern bool pumpOn;
extern bool propaneOn;
extern bool auxOn;
extern time_t lastPublishData;