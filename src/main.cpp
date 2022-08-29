#include "main.h"

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

Thermistor ntcFrame1(mcpReadCallback, 5.0, 5.0, 4096, 10000, 10000, 25, 3950, 5, 30);
Thermistor ntcFrame2(mcpReadCallback, 5.0, 5.0, 4096, 10000, 10000, 25, 3950, 5, 30);
Thermistor ntcAmbiant(mcpReadCallback, 5.0, 5.0, 4096, 100500, 100000, 25, 3950, 5, 30);

NTCSettings frame1Settings;
NTCSettings frame2Settings;
NTCSettings ambiantSettings;

unsigned long currentMillis = 0;
unsigned long intervalData = LOOP_DAT_DLY;
unsigned long previousMillisData = 0;
unsigned long intervalProc = LOOP_PROC_DLY;
unsigned long previousMillisProc = 0;
unsigned long intervalPub = LOOP_PUB_DLY;
unsigned long previousMillisPub = 0;
unsigned long intervalHB = 1*1E3;
unsigned long previousMillisHB = 0;

bool isDark = true;
bool pumpOn = false;
int16_t light = 0;
float tin = -127.0;
float tout = -127.0;
float f1 = -127.0;
float f2 = -127.0;
float air = -127.0;
float wattsT = 0.0;
float wattsF1 = 0.0;
float wattsF2 = 0.0;

HomieNode infoNode("info", "Info", "string");
HomieNode tempNode("temp", "Temps", "float");
HomieNode lightNode("light", "Light", "float");

HomieSetting<long> darkSetting("dark", "minimum light val");
HomieSetting<double> gpmSetting("gpm", "flow in gpm");
HomieSetting<double> setpointSetting("setpoint", "set point");

HomieSetting<const char *> frame1Setting("frame1", "frame 1 settings");
HomieSetting<const char *> frame2Setting("frame2", "frame 2 settings");
HomieSetting<const char *> ambiantSetting("ambiant", "ambiant settings");

HomieSetting<const char *> tinAddrSetting("tin_addr", "tin addr");
HomieSetting<const char *> toutAddrSetting("tout_addr", "tout addr");

#ifdef TESTING
void testing()
{
  delay(200);  
  Serial.println("Testing!!!");

  String frameData = "vcc=4.55,adcRef=4.55,serRes=10000,ntcRes=10000,tempNom=25,bc=3950,samples=5,sampleDly=20";
  NTCSettings fs;

  sscanf(frameData.c_str(), "vcc=%f,adcRef=%f,serRes=%d,ntcRes=%d,tempNom=%d,bc=%d,samples=%d,sampleDly=%d",
      &fs.vcc, &fs.adcRef, &fs.serRes, &fs.ntcRes, &fs.tempNom, &fs.bc, &fs.samples, &fs.sampleDly
      );

  Serial.print("VCC = "); Serial.println(fs.vcc);
  Serial.print("ADC Ref = "); Serial.println(fs.adcRef);
  Serial.print("Ser Res = "); Serial.println(fs.serRes);
  Serial.print("NTC Res = "); Serial.println(fs.ntcRes);
  Serial.print("Temp Nom = "); Serial.println(fs.tempNom);
  Serial.print("bCoef = "); Serial.println(fs.bc);
  Serial.print("Samples = "); Serial.println(fs.samples);
  Serial.print("Sample Delay = "); Serial.println(fs.sampleDly);
  
}
#endif

void setup() {
  pinMode(LED_BUILTIN_AUX, OUTPUT);
  pinMode(RLY_PIN, OUTPUT);

  #ifdef DEBUG
  Serial.begin(115200);
  #else
  Homie.disableLogging();
  #endif

  WiFi.mode(WIFI_STA);

  #ifdef TESTING
  testing();
  while(true){
    delay(60000);
  }
  #endif
    
  validateHomieSettings();

  mcp.begin(MCP_CS);
  #ifdef DEBUG
  Serial.print("ADC channels = "); Serial.println(mcp.channels());
  Serial.print("ADC spi speed = "); Serial.print(mcp.getSPIspeed()); Serial.println(" Hz");
  Serial.print("ADC max value = "); Serial.println(mcp.maxValue());
  #endif

  Homie_setFirmware("bare-minimum", "1.0.0");
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
  Homie_setBrand("PoolHeater");

  tempNode.advertise("tin").setName("TempIn").setDatatype("float").setUnit("F");
  tempNode.advertise("tout").setName("TempOut").setDatatype("float").setUnit("F"); 
  tempNode.advertise("f1").setName("Frame1").setDatatype("float").setUnit("F");
  tempNode.advertise("f2").setName("Frame2").setDatatype("float").setUnit("F");
  tempNode.advertise("air").setName("Air").setDatatype("float").setUnit("F");
  lightNode.advertise("light").setName("LightLvl").setDatatype("float").setUnit("%");   
  lightNode.advertise("dark").setName("IsDark").setDatatype("boolean");
  infoNode.advertise("timestamp").setName("Timestamp").setDatatype("string").setUnit("");                                                                          
  infoNode.advertise("watts_t").setName("Watts Total").setDatatype("float").setUnit("W");
  infoNode.advertise("watts_f1").setName("Watts F1").setDatatype("float").setUnit("W");
  infoNode.advertise("watts_f2").setName("Watts F2").setDatatype("float").setUnit("W");
  infoNode.advertise("pump").setName("PumpOn").setDatatype("boolean");

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

  frame1Settings = parseNTCSettings(frame1Setting.get(), "Frame 1");
  frame2Settings = parseNTCSettings(frame2Setting.get(), "Frame 2");
  ambiantSettings = parseNTCSettings(ambiantSetting.get(), "Ambiant");

  ntcFrame1.setSeriesResistor(frame1Settings.serRes);
  ntcFrame1.setThermistorNominal(frame1Settings.ntcRes);
  ntcFrame1.setBCoef(frame1Settings.bc);
  ntcFrame1.setTemperatureNominal(frame1Settings.tempNom);
  ntcFrame1.setVcc(frame1Settings.vcc);
  ntcFrame1.setAnalogReference(frame1Settings.adcRef);
  
  ntcFrame2.setSeriesResistor(frame2Settings.serRes);
  ntcFrame2.setThermistorNominal(frame2Settings.ntcRes);
  ntcFrame2.setBCoef(frame2Settings.bc);
  ntcFrame2.setTemperatureNominal(frame2Settings.tempNom);
  ntcFrame2.setVcc(frame2Settings.vcc);
  ntcFrame2.setAnalogReference(frame2Settings.adcRef);

  ntcAmbiant.setSeriesResistor(ambiantSettings.serRes);
  ntcAmbiant.setThermistorNominal(ambiantSettings.ntcRes);
  ntcAmbiant.setBCoef(ambiantSettings.bc);
  ntcAmbiant.setTemperatureNominal(ambiantSettings.tempNom);
  ntcAmbiant.setVcc(ambiantSettings.vcc);
  ntcAmbiant.setAnalogReference(ambiantSettings.adcRef);

  setupOwSensors();

}

void loopHandler()
{
  currentMillis = millis();

  // Gather data
  if(currentMillis - previousMillisData > intervalData || previousMillisData == 0) {
    if (year() == 1970) {
      #ifdef DEBUG
      Serial.println("Force update NTP");
      #endif
      timeClient.forceUpdate();
      time_t tt;
      tt = getNtpTime();
      setTime(tt);
      yield();
    }
    
    Homie.getLogger() << "Setpoint = " << setpointSetting.get() << " °F" << endl;

    sensors.requestTemperatures();
    delay(100);
    
    tin = sensors.getTempF(tempSensorIn);
    yield();
    tout = sensors.getTempF(tempSensorOut);
    yield();
    wattsT = calcWatts(tin, tout);
    Homie.getLogger() << "Tin = " << tin << " °F " << "Tout = " << tout << " °F " << " Watts = " << wattsT << endl;
    yield();

    air = ntcAmbiant.readTempF(ADC_AMBIANT);
    Homie.getLogger() << "Air = " << air << " °F" << endl;
    yield();

    f1 = ntcFrame1.readTempF(ADC_FRAME1);
    yield();
    wattsF1 = calcWatts(air, f1);
    Homie.getLogger() << "Frame 1 = " << f1 << " °F " << " Watts = " << wattsF1 << endl;
    yield();

    f2 = ntcFrame1.readTempF(ADC_FRAME2);
    yield();
    wattsF2 = calcWatts(air, f2);
    Homie.getLogger() << "Frame 2 = " << f2 << " °F "  << " Watts = " << wattsF2 << endl;
    yield();

    light = mcp.analogRead(ADC_LIGHT);
    if(light <= int16_t(darkSetting.get())) {
      isDark = true;
    }else{
      isDark = false;
    }
    Homie.getLogger() << "Light level = " << light << "  Dark setting = " << darkSetting.get() << " Dark out = " << boolToStr(isDark) << endl;
    yield();
    
    if(isDark){
      intervalProc = LOOP_SLEEP_DLY;
      intervalPub = LOOP_SLEEP_DLY;
      previousMillisProc = 0;
    }else{
      intervalProc = LOOP_PROC_DLY;
      intervalPub = LOOP_PUB_DLY;
    }
    yield();

    Homie.getLogger() << "Pump On = " << boolToStr(pumpOn) << endl;

    Homie.getLogger() << endl;
    previousMillisData = currentMillis;
  }
  yield();

  // Process data
  if((currentMillis - previousMillisProc > intervalProc || previousMillisProc == 0) && !isDark)
  {
    Serial.println("PROCESS");
    Serial.printf("Watts T = %d\n", int(wattsT));

    if(int(wattsT) >= 1){
      Serial.println("PUMP ON");
      digitalWrite(RLY_PIN, HIGH);
      pumpOn = true;
    }else{
      Serial.println("PUMP OFF");
      digitalWrite(RLY_PIN, LOW);
      pumpOn = false;
    }


    previousMillisProc = currentMillis;
  }
  else if((currentMillis - previousMillisProc > intervalProc || previousMillisProc == 0) && isDark)
  {
    Serial.println("PUMP OFF ELSE");
    digitalWrite(RLY_PIN, LOW);
    pumpOn = false;
    
    previousMillisProc = currentMillis;
  }
  yield();

  // Publish data
  if(currentMillis - previousMillisPub > intervalPub || previousMillisPub == 0) {
    infoNode.setProperty("timestamp").send(getTimestamp());
    infoNode.setProperty("watts_t").send(String(wattsT));
    infoNode.setProperty("watts_f1").send(String(wattsF1));
    infoNode.setProperty("watts_f2").send(String(wattsF2));
    infoNode.setProperty("pump").send(pumpOn ? "true" : "false");
    lightNode.setProperty("light").send(String(light));
    lightNode.setProperty("dark").send(isDark ? "true" : "false");
    tempNode.setProperty("tin").send(String(tin));
    tempNode.setProperty("tout").send(String(tout));
    tempNode.setProperty("f1").send(String(f1));
    tempNode.setProperty("f2").send(String(f2));
    tempNode.setProperty("air").send(String(air));

    previousMillisPub = currentMillis;
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
  Homie.getLogger() << "NTP Updated. Current time is " << timeClient.getEpochTime() << endl << endl;
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
  #ifdef DEBUG
  Serial.print("Locating devices...");
  #endif
  sensors.begin();
  sensors.setWaitForConversion(true);
  
  #ifdef DEBUG
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  #endif
  for (size_t i = 0; i < sensors.getDeviceCount(); i++)
  {
    if (sensors.getAddress(tempDeviceAddress, i))
    {
      #ifdef DEBUG
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();

      Serial.print("Setting resolution to ");
      Serial.println(DS_TEMP_PRECISION, DEC);
      #endif
      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, DS_TEMP_PRECISION);

      #ifdef DEBUG
      Serial.print("Resolution actually set to: ");
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
      Serial.println();
      #endif
    } else {
      #ifdef DEBUG
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
      #endif
    }
  }
  
  strToAddress(tinAddrSetting.get(), tempSensorIn);
  strToAddress(toutAddrSetting.get(), tempSensorOut);

  #ifdef DEBUG
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
  #endif
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

  gpmSetting.setDefaultValue(GPM_DEFAULT).setValidator([] (double candidate) {
    return candidate > 0;
  });

  setpointSetting.setDefaultValue(SP_DEFAULT).setValidator([] (double candidate) {
    return candidate > 0;
  });
}

NTCSettings parseNTCSettings(const char * settings, const char * name)
{
  NTCSettings fs;

  sscanf(settings, "vcc=%f,adcRef=%f,serRes=%d,ntcRes=%d,tempNom=%d,bc=%d,samples=%d,sampleDly=%d",
      &fs.vcc, &fs.adcRef, &fs.serRes, &fs.ntcRes, &fs.tempNom, &fs.bc, &fs.samples, &fs.sampleDly
      );

  Homie.getLogger() << "Parsed " << name << " settings:" << endl;
  Homie.getLogger() << "VCC = " << fs.vcc << endl;
  Homie.getLogger() << "ADC Ref = " << fs.adcRef << endl;
  Homie.getLogger() << "Ser Res = " << fs.serRes << endl;
  Homie.getLogger() << "NTC Res = " << fs.ntcRes << endl;
  Homie.getLogger() << "Temp Nom = " << fs.tempNom << endl;
  Homie.getLogger() << "bCoef = " << fs.bc << endl;
  Homie.getLogger() << "Samples = " << fs.samples << endl;
  Homie.getLogger() << "Sample Delay = " << fs.sampleDly << endl;
  Homie.getLogger() << endl;

  return fs;
}

float calcWatts(float tempIn, float tempOut)
{
  const float dt = fabs(tempIn - tempOut);
  const float gpm = float(gpmSetting.get());
  const float c = float(0.00682);
  float rtn;
  rtn = (gpm * dt) / c;
  if(rtn < WATTS_MIN){
    return 0.0;
  }
  return rtn;
}








