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

#define DS_TEMP_PRECISION 9

#define ONE_WIRE_BUS   D2
#define MCP_DIN        D6
#define MCP_DOUT       D7
#define MCP_CLK        D5
#define MCP_CS         D8

#define ADC_LIGHT      0
#define ADC_FRAME1     1
#define ADC_FRAME2     2

#define DARK_DEFAULT   400
#define FSR_DEFAULT    10000
#define FTR_DEFAULT    10000
#define FTN_DEFAULT    25
#define FBC_DEFAULT    3950
#define FVCC_DEFAULT   4.55
#define FREF_DEFAULT   4.55

#define LOOP_RUN_DLY     3*1E3
#define LOOP_SLEEP_DLY   600*1E3

////> Function Prototypes
time_t getNtpTime();
const char* getTimestamp();
void setupHandler();
void loopHandler();
void strToAddress(const String addr, DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);
void setupOwSensors();
int16_t mcpReadCallback(uint8_t channel);
void validateHomieSettings();
void testing();
////< Function Prototypes

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
// 289D6F49F67A3CF8
DeviceAddress tempSensorIn; // = { 0x28, 0x9D, 0x6F, 0x49, 0xF6, 0x7A, 0x3C, 0xF8 };
// 283E4749F6C13C1A
DeviceAddress tempSensorOut; // = { 0x28, 0x3E, 0x47, 0x49, 0xF6, 0xC1, 0x3C, 0x1A };

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

MCP3204 mcp(MCP_DIN, MCP_DOUT, MCP_CLK);

char timestampStrBuf[1024];
EasyStringStream timestampStream(timestampStrBuf, 1024);

Thermistor frame1(mcpReadCallback, 5.0, 5.0, 4096, 10000, 10000, 25, 3950, 5, 30);
Thermistor frame2(mcpReadCallback, 5.0, 5.0, 4096, 10000, 10000, 25, 3950, 5, 30);

unsigned long currentMillis = 0;
unsigned long intervalData = LOOP_RUN_DLY;
unsigned long previousMillisData = 0;
unsigned long intervalHB = 1*1E3;
unsigned long previousMillisHB = 0;

bool isDark = true;
int16_t light = 0;
double tin = -127.0;
double tout = -127.0;
double f1 = -127.0;
double f2 = -127.0;

HomieNode infoNode("info", "Info", "string");
HomieNode tempNode("temp", "Temps", "float");
HomieNode lightNode("light", "Light", "float");

HomieSetting<long> darkSetting("dark", "minimum light val");

HomieSetting<double> frameSrSetting("framesr", "frames series res");
HomieSetting<double> frameTrSetting("frametr", "frame ntc res");
HomieSetting<double> frameBcSetting("framebc", "frame bCoef");
HomieSetting<double> frameVccSetting("framevcc", "frame vcc");

void testing()
{
  delay(200);  
}

void setup() {
  pinMode(LED_BUILTIN_AUX, OUTPUT);

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // testing();
  // delay(60000);
  // delay(60000);
  // delay(60000);
  
  validateHomieSettings();

  setupOwSensors();

  mcp.begin(MCP_CS);
  Serial.print("ADC channels = "); Serial.println(mcp.channels());
  Serial.print("ADC spi speed = "); Serial.print(mcp.getSPIspeed()); Serial.println(" Hz");
  Serial.print("ADC max value = "); Serial.println(mcp.maxValue());

  Homie_setFirmware("bare-minimum", "1.0.0");
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
  Homie_setBrand("PoolHeater");

  tempNode.advertise("tin").setName("TempIn").setDatatype("float").setUnit("F");
  tempNode.advertise("tout").setName("TempOut").setDatatype("float").setUnit("F"); 
  tempNode.advertise("f1").setName("Frame1").setDatatype("float").setUnit("F");
  tempNode.advertise("f2").setName("Frame2").setDatatype("float").setUnit("F");                                    
  lightNode.advertise("light").setName("LightLvl").setDatatype("float").setUnit("%");   
  lightNode.advertise("dark").setName("IsDark").setDatatype("boolean");
  infoNode.advertise("timestamp").setName("Timestamp").setDatatype("string").setUnit("");                                                                          

  Homie.setup();
}

void loop() {
  Homie.loop();
}

void setupHandler() {
  // Code which should run AFTER the WiFi is connected.
  timeClient.begin();
  timeClient.forceUpdate();

  setSyncProvider(getNtpTime);
  setSyncInterval(600);

  frame1.setSeriesResistor(frameSrSetting.get());
  frame1.setThermistorNominal(frameTrSetting.get());
  frame1.setBCoef(frameBcSetting.get());
  frame1.setVcc(frameVccSetting.get());
  frame1.setAnalogReference(frameVccSetting.get());

  frame2.setSeriesResistor(frameSrSetting.get());
  frame2.setThermistorNominal(frameTrSetting.get());
  frame2.setBCoef(frameBcSetting.get());
  frame2.setVcc(frameVccSetting.get());
  frame2.setAnalogReference(frameVccSetting.get());

}

void loopHandler()
{
  currentMillis = millis();

  // Process data
  if(currentMillis - previousMillisData > intervalData || previousMillisData == 0) {
    if (year() == 1970) {
      Serial.println("Force update NTP");
      timeClient.forceUpdate();
      time_t tt;
      tt = getNtpTime();
      setTime(tt);
      yield();
    }
    
    sensors.requestTemperatures();
    delay(100);
    
    tin = sensors.getTempF(tempSensorIn);
    Homie.getLogger() << "Tin = " << tin << " °F" << endl;
    yield();

    tout = sensors.getTempF(tempSensorOut);
    Homie.getLogger() << "Tout = " << tout << " °F" << endl;
    yield();

    f1 = frame1.readTempF(ADC_FRAME1);
    Homie.getLogger() << "Frame 1 = " << f1 << " °F" << endl;
    yield();

    f2 = frame1.readTempF(ADC_FRAME2);
    Homie.getLogger() << "Frame 2 = " << f2 << " °F" << endl;
    yield();

    light = mcp.analogRead(ADC_LIGHT);
    Homie.getLogger() << "Light level = " << light << endl;
    yield();

    Homie.getLogger() << "Dark setting = " << darkSetting.get() << endl;
    yield();

    if(light <= int16_t(darkSetting.get())) {
      isDark = true;
      Homie.getLogger() << "Dark out = Yes" << endl;
    }else{
      isDark = false;
      Homie.getLogger() << "Dark out = No" << endl;
    }
    
    infoNode.setProperty("timestamp").send(getTimestamp());
    lightNode.setProperty("light").send(String(light));
    lightNode.setProperty("dark").send(isDark ? "true" : "false");
    tempNode.setProperty("tin").send(String(tin));
    tempNode.setProperty("tout").send(String(tout));
    tempNode.setProperty("f1").send(String(f1));
    tempNode.setProperty("f2").send(String(f2));

    yield();

    Homie.getLogger() << endl;
    previousMillisData = currentMillis;
  }

  yield();

  // Blink the heartbeat led
  if(currentMillis - previousMillisHB > intervalHB || previousMillisHB == 0){
    digitalWrite(LED_BUILTIN_AUX, !digitalRead(LED_BUILTIN_AUX));
    previousMillisHB = currentMillis;
  }

}

time_t getNtpTime()
{
  timeClient.forceUpdate();
  Serial.print("NTP Updated. Current time is ");
  Serial.println(timeClient.getEpochTime());
  return time_t(timeClient.getEpochTime());
}

const char* getTimestamp()
{
  timestampStream.reset();
  timestampStream << String(year()) << "-";

  if (month() < 10) {
    timestampStream << "0" << String(month());
  }else{
    timestampStream << String(month());
  }

  timestampStream << "-";

  if (day() < 10) {
    timestampStream << "0" << String(day());
  }else{
    timestampStream << String(day());
  }

  timestampStream << "T";

  if (hour() < 10) {
    timestampStream << "0" << String(hour());
  }else{
    timestampStream << String(hour());
  }

  timestampStream << ":";
  
  if (minute() < 10) {
    timestampStream << "0" << String(minute());
  }else{
    timestampStream << String(minute());
  }
  
  timestampStream << ":";

  if (second() < 10)
  {
    timestampStream << "0" << String(second());
  }else{
    timestampStream << String(second());
  }

  timestampStream << ".00+00:00";
  // Serial.println("TIMESTAMP = " + String(timestampStream.get()));
  return timestampStream.get();
}

void strToAddress(const String addr, DeviceAddress deviceAddress)
{
  char byt[3] = {0, 0, 0};
  unsigned int number = 0;

  for (size_t i = 0; i < addr.length(); i+=2)
  {
    byt[0] = addr[i];
    byt[1] = addr[i + 1];

    number = (int)strtol(byt, NULL, 16);
    deviceAddress[i / 2] = number;
  }
  
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void setupOwSensors()
{
  Serial.print("Locating devices...");
  Homie.getLogger() << "THIS IS HERE IN SETUP!" << endl;
  
  sensors.begin();
  sensors.setWaitForConversion(true);
  
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  for (size_t i = 0; i < sensors.getDeviceCount(); i++)
  {
    if (sensors.getAddress(tempDeviceAddress, i))
    {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();

      Serial.print("Setting resolution to ");
      Serial.println(DS_TEMP_PRECISION, DEC);

      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, DS_TEMP_PRECISION);

      Serial.print("Resolution actually set to: ");
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
  
  if (sensors.isConnected(tempSensorIn)){
    Serial.print("Temp sensor IN Address: "); printAddress(tempSensorIn); Serial.println();
    Serial.print("Temp sensor IN resolution: "); Serial.print(sensors.getResolution(tempSensorIn), DEC); Serial.println();
  }else{
    Serial.println("Temp sensor IN is not connected !");
  }

  if (sensors.isConnected(tempSensorOut)){
    Serial.print("Temp sensor OUT Address: "); printAddress(tempSensorOut); Serial.println();
    Serial.print("Temp sensor OUT resolution: "); Serial.print(sensors.getResolution(tempSensorOut), DEC); Serial.println();
  }else{
    Serial.println("Temp sensor OUT is not connected !");
  }  
}

int16_t mcpReadCallback(uint8_t channel)
{
  return mcp.analogRead(channel);
}

void validateHomieSettings()
{
  darkSetting.setDefaultValue(DARK_DEFAULT).setValidator([] (long candidate) {
    return candidate > 0;
  });

  frameSrSetting.setDefaultValue(FSR_DEFAULT).setValidator([] (double candidate) {
    return candidate > 0;
  });

  frameTrSetting.setDefaultValue(FTR_DEFAULT).setValidator([] (double candidate) {
    return candidate > 0;
  });

  frameBcSetting.setDefaultValue(FBC_DEFAULT).setValidator([] (double candidate) {
    return candidate > 0;
  });

  frameVccSetting.setDefaultValue(FVCC_DEFAULT).setValidator([] (double candidate) {
    return candidate > 0;
  });
}

