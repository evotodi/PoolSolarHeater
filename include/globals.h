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
#include "HomieNode.hpp"
#include "PoolSetting.hpp"
#include "Smoothed.h"
#include "EasyStringStream.h"
#include "enums.h"

extern OneWire oneWire;
extern DallasTemperature sensors;
extern DeviceAddress tempDeviceAddress;

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

extern DTSetting dtSettingTin;
extern DTSetting dtSettingToutSolar;
extern DTSetting dtSettingToutHeat;
extern DTSetting dtSettingPool;
extern DTSetting dtSettingAir;

extern HomieNode statusNode;
extern HomieNode valuesNode;
extern HomieNode configNode;

extern DynamicJsonDocument poolJsonDoc;

// IPoolSetting::settingsConfig
extern PoolSetting<uint16_t> settingCloudy;
extern PoolSetting<uint16_t> settingOvercastCnt;
extern PoolSetting<double> settingSunMinElvAM;
extern PoolSetting<double> settingSunMinElvPM;
extern PoolSetting<double> settingPoolSP;
extern PoolSetting<double> settingHeatAuxSP;
extern PoolSetting<double> settingSPHyst;
extern PoolSetting<double> settingPumpGpm;
extern PoolSetting<bool> settingHeatAuxEnable;
// IPoolSetting::settingsTime
extern PoolSetting<int16_t> settingTimeDstOffset;
extern PoolSetting<int16_t> settingTimeStOffset;
extern PoolSetting<uint16_t> settingTimeDstBeginDay;
extern PoolSetting<uint16_t> settingTimeDstBeginMonth;
extern PoolSetting<uint16_t> settingTimeDstEndDay;
extern PoolSetting<uint16_t> settingTimeDstEndMonth;
// IPoolSetting::settingsLocation
extern PoolSetting<double> settingLatitude;
extern PoolSetting<double> settingLongitude;
// IPoolSetting::settingsProbeOffset
extern PoolSetting<double> settingAirOffset;
extern PoolSetting<double> settingPoolOffset;
extern PoolSetting<double> settingTinOffset;
extern PoolSetting<double> settingToutSolarOffset;
extern PoolSetting<double> settingToutHeatOffset;
// IPoolSetting::settingsProbe
extern PoolSetting<const char *> settingAirProbeCfg;
extern PoolSetting<const char *> settingPoolProbeCfg;
extern PoolSetting<const char *> settingTinProbeCfg;
extern PoolSetting<const char *> settingToutSolarProbeCfg;
extern PoolSetting<const char *> settingToutHeatProbeCfg;

extern unsigned long currentMillis;
// Loop Gather
extern unsigned long intervalGather;
extern unsigned long prevMillisGather;
// Loop Control
extern unsigned long intervalControl;
extern unsigned long prevMillisControl;
// Loop MQTT Publish
extern unsigned long intervalPub;
extern unsigned long prevMillisPub;
// Loop MQTT Pub Config
extern unsigned long intervalPubCfg;
extern unsigned long prevMillisPubCfg;
// Loop Heartbeat
extern unsigned long intervalHB;
extern unsigned long prevMillisHB;
// Loop Update Daylight
extern unsigned long intervalDayLight;
extern unsigned long prevMillisDaylight;

extern bool isCloudy;
extern bool isOvercast;
extern bool otaInProgress;
extern bool firstRun;

extern int telnetBuffer;
extern Smoothed<int16_t> light;
extern uint16_t cloudyCnt;
extern Smoothed<int> tin;
extern bool tinOk;
extern Smoothed<int> toutSolar;
extern bool toutSolarOk;
extern Smoothed<int> toutHeat;
extern bool toutHeatOk;
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
extern bool rlyPump;
extern bool rlyHeatAux;
extern bool rlyAux;
extern time_t lastPublishData;