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

ThermistorSettings frame1Settings = {5.0, 5.0, 10000, 10000, 25, 4096, 3950, 5, 20};
ThermistorSettings frame2Settings = {5.0, 5.0, 10000, 10000, 25, 4096, 3950, 5, 20};
ThermistorSettings ambiantSettings = {5.0, 5.0, 100000, 100000, 25, 4096, 3950, 5, 20};

Thermistor ntcFrame1(mcpReadCallback, frame1Settings);
Thermistor ntcFrame2(mcpReadCallback, frame2Settings);
Thermistor ntcAmbiant(mcpReadCallback, ambiantSettings);

DynamicJsonDocument doc(256);
DeserializationError jsonError;

unsigned long currentMillis = 0;
unsigned long intervalData = LOOP_DAT_DLY;
unsigned long previousMillisData = 0;
unsigned long intervalProc = LOOP_PROC_DLY;
unsigned long previousMillisProc = 0;
unsigned long intervalPub = LOOP_PUB_DLY;
unsigned long previousMillisPub = 0;
unsigned long intervalCircOn = LOOP_CIRC_ON_DLY;
unsigned long previousMillisCircOn = 0;
unsigned long intervalCircOff = LOOP_CIRC_OFF_DLY;
unsigned long previousMillisCircOff = 0;

unsigned long intervalHB = 1 * 1E3;
unsigned long previousMillisHB = 0;

char stayDarkCnt = 0;
int timeOffset = 0;

float setpoint = float(SP_DEFAULT);
int16_t dark = int16_t(DARK_DEFAULT);
float gpm = float(GPM_DEFAULT);

bool isDark = false;
bool realDark = false;
bool pumpOn = false;
bool tempCheckPumpOn = false;
int16_t light = 0;
String tinAddr = "0000000000000000";
float tin = -127.0;
float tinOffset = 0;
String toutAddr = "0000000000000000";
float tout = -127.0;
float toutOffset = 0;
float f1 = -127.0;
float f2 = -127.0;
float air = -127.0;
float wattsT = 0.0;
float wattsF1 = 0.0;
float wattsF2 = 0.0;

HomieNode infoNode("info", "Info", "string");
HomieNode tempNode("temp", "Temps", "float");
HomieNode lightNode("light", "Light", "float");

HomieSetting<const char *> configSetting("config", "config jon");
HomieSetting<const char *> frame1Setting("frame1", "frame 1 json");
HomieSetting<const char *> frame2Setting("frame2", "frame 2 json");
HomieSetting<const char *> ambiantSetting("ambiant", "ambiant json");
HomieSetting<const char *> tinSetting("tin", "tin json");
HomieSetting<const char *> toutSetting("tout", "tout json");

#ifdef TESTING
void testing()
{
  delay(200);  
  Serial.println("Testing!!!");

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

    mcp.begin(MCP_CS);
#ifdef DEBUG
    Serial.print("ADC channels = ");
    Serial.println(mcp.channels());
    Serial.print("ADC spi speed = ");
    Serial.print(mcp.getSPIspeed());
    Serial.println(" Hz");
    Serial.print("ADC max value = ");
    Serial.println(mcp.maxValue());
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
    // Code which should run AFTER the Wi-Fi is connected.
    timeClient.begin();
    yield();
    timeClient.forceUpdate();
    yield();

    if(timeClient.isTimeSet()){
        timeOffset = getTimeOffset(now());
        yield();
        Homie.getLogger() << "Time offset = " << timeOffset << endl;
        timeClient = NTPClient(ntpUDP, long(timeOffset));
        timeClient.begin();
        timeClient.forceUpdate();
        yield();
    }

    setSyncProvider(getNtpTime);
    setSyncInterval(600);
    yield();

    frame1Settings = parseNTCSettings(frame1Setting.get(), "Frame 1");
    frame2Settings = parseNTCSettings(frame2Setting.get(), "Frame 2");
    ambiantSettings = parseNTCSettings(ambiantSetting.get(), "Ambiant");
    yield();

    ntcFrame1.setSeriesResistor(&frame1Settings.seriesResistor);
    ntcFrame1.setThermistorNominal(&frame1Settings.thermistorNominal);
    ntcFrame1.setBCoef(&frame1Settings.bCoef);
    ntcFrame1.setTemperatureNominal(&frame1Settings.temperatureNominal);
    ntcFrame1.setVcc(&frame1Settings.vcc);
    ntcFrame1.setAnalogReference(&frame1Settings.analogReference);
    Homie.getLogger() << "Frame 1 settings = " << ntcFrame1.dumpSettings() << endl;
    yield();

    ntcFrame2.setSeriesResistor(&frame2Settings.seriesResistor);
    ntcFrame2.setThermistorNominal(&frame2Settings.thermistorNominal);
    ntcFrame2.setBCoef(&frame2Settings.bCoef);
    ntcFrame2.setTemperatureNominal(&frame2Settings.temperatureNominal);
    ntcFrame2.setVcc(&frame2Settings.vcc);
    ntcFrame2.setAnalogReference(&frame2Settings.analogReference);
    Homie.getLogger() << "Frame 2 settings = " << ntcFrame2.dumpSettings() << endl;
    yield();

    ntcAmbiant.setSeriesResistor(&ambiantSettings.seriesResistor);
    ntcAmbiant.setThermistorNominal(&ambiantSettings.thermistorNominal);
    ntcAmbiant.setBCoef(&ambiantSettings.bCoef);
    ntcAmbiant.setTemperatureNominal(&ambiantSettings.temperatureNominal);
    ntcAmbiant.setVcc(&ambiantSettings.vcc);
    ntcAmbiant.setAnalogReference(&ambiantSettings.analogReference);
    Homie.getLogger() << "Ambiant settings = " << ntcAmbiant.dumpSettings() << endl;
    yield();

    // deserialize config key
    jsonError = deserializeJson(doc, configSetting.get());
    if (jsonError) {
        Homie.getLogger() << "Config: 'config' deserialize failed: " << jsonError.c_str() << endl;
    }else{
        dark = doc["dark"].as<int16_t>();
        gpm = doc["gpm"].as<float>();
        setpoint = doc["setpoint"].as<float>();
        Homie.getLogger() << "Dark = " << dark << " GPM = " << gpm << " Setpoint = " << setpoint << endl;
    }
    doc.clear();
    yield();

    // deserialize tin key
    jsonError = deserializeJson(doc, tinSetting.get());
    if (jsonError) {
        Homie.getLogger() << "Config: 'tin' deserialize failed: " << jsonError.c_str() << endl;
    }else{
        tinAddr = doc["addr"].as<String>();
        tinOffset = doc["offset"].as<float>();
        strToAddress(tinAddr, tempSensorIn);
        Homie.getLogger() << "Tin: addr = " << tinAddr << " offset = " << tinOffset << endl;
    }
    doc.clear();
    yield();

    // deserialize tin key
    jsonError = deserializeJson(doc, toutSetting.get());
    if (jsonError) {
        Homie.getLogger() << "Config: 'tout' deserialize failed: " << jsonError.c_str() << endl;
    }else{
        toutAddr = doc["addr"].as<String>();
        toutOffset = doc["offset"].as<float>();
        strToAddress(toutAddr, tempSensorOut);
        Homie.getLogger() << "Tout: addr = " << toutAddr << " offset = " << toutOffset << endl;
    }
    doc.clear();
    yield();

    setupOwSensors();
}

void loopHandler() {
    currentMillis = millis();

    // Gather data
    if (currentMillis - previousMillisData > intervalData || previousMillisData == 0) {
        Homie.getLogger() << "GATHER" << endl;
        if (year() == 1970) {
            Homie.getLogger() << "Force update NTP" << endl;
            timeClient.forceUpdate();
            time_t tt;
            tt = getNtpTime();
            setTime(tt);
            yield();
        }

        sensors.requestTemperatures();
        delay(100);

        tin = sensors.getTempF(tempSensorIn) + tinOffset;
        yield();
        tout = sensors.getTempF(tempSensorOut) + toutOffset;
        yield();
        wattsT = calcWatts(tin, tout);
        Homie.getLogger() << "Tin = " << tin << " °F " << "Tout = " << tout << " °F " << " Watts = " << wattsT << endl;
        yield();

        air = ntcAmbiant.readTempF(ADC_AMBIANT); // NOLINT(cppcoreguidelines-narrowing-conversions)
        Homie.getLogger() << "Air = " << air << " °F" << endl;
        yield();

        f1 = ntcFrame1.readTempF(ADC_FRAME1); // NOLINT(cppcoreguidelines-narrowing-conversions)
        yield();
        wattsF1 = calcWatts(air, f1);
        Homie.getLogger() << "Frame 1 = " << f1 << " °F " << " Watts = " << wattsF1 << endl;
        yield();

        f2 = ntcFrame2.readTempF(ADC_FRAME2); // NOLINT(cppcoreguidelines-narrowing-conversions)
        yield();
        wattsF2 = calcWatts(air, f2);
        Homie.getLogger() << "Frame 2 = " << f2 << " °F " << " Watts = " << wattsF2 << endl;
        yield();

        light = mcp.analogRead(ADC_LIGHT);
        if (light <= dark) {
            isDark = true;
        } else {
            isDark = false;
        }
        Homie.getLogger() << "Light level = " << light << " Dark out = " << boolToStr(isDark) << endl;
        yield();

        if (isDark) {
            intervalProc = LOOP_SLEEP_DLY;
            intervalPub = LOOP_SLEEP_DLY;
            previousMillisProc = 0;
            stayDarkCnt++;
            if(stayDarkCnt >= STAY_DARK_CNT){
                realDark = true;
            }
            yield();
        } else {
            intervalProc = LOOP_PROC_DLY;
            intervalPub = LOOP_PUB_DLY;
            realDark = false;
            stayDarkCnt = 0;
        }
        yield();

        Homie.getLogger() << "Pump On = " << boolToStr(pumpOn) << endl;

        Homie.getLogger() << endl;
        previousMillisData = currentMillis;
    }
    yield();

    // Process data
    if (currentMillis - previousMillisProc > intervalProc || previousMillisProc == 0) {
        Homie.getLogger() << "PROCESS" << endl;

        doProcess();

        previousMillisProc = currentMillis;
    }
    yield();

    // Circulate On
    if ((currentMillis - previousMillisCircOn > intervalCircOn || previousMillisCircOn == 0) && !tempCheckPumpOn) {
        Homie.getLogger() << "Temp check begin" << endl;
        if(turnPumpOn()){
            tempCheckPumpOn = true;

            if(previousMillisCircOff == 0){
                previousMillisCircOff = currentMillis;
            }
        }

        previousMillisCircOn = currentMillis;
    }
    yield();

    // Circulate Off
    if ((currentMillis - previousMillisCircOff > intervalCircOff) && tempCheckPumpOn) {
        Homie.getLogger() << "Temp check end" << endl;
        tempCheckPumpOn = false;
        turnPumpOff();

        previousMillisCircOff = currentMillis;
    }
    yield();

    // Publish data
    if (currentMillis - previousMillisPub > intervalPub || previousMillisPub == 0) {
        Homie.getLogger() << "PUBLISH" << endl;
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
    if (currentMillis - previousMillisHB > intervalHB || previousMillisHB == 0) {
        digitalWrite(LED_BUILTIN_AUX, !digitalRead(LED_BUILTIN_AUX));
        previousMillisHB = currentMillis;
    }

}

void doProcess()
{
    if(tin >= setpoint && !tempCheckPumpOn) {
        Homie.getLogger() << "Set point reached" << endl;
        turnPumpOff();
        return;
    }

    if(tempCheckPumpOn){
        Homie.getLogger() << "Temp check running" << endl;
        return;
    }

    turnPumpOn();
}

time_t getNtpTime() {
    timeClient.forceUpdate();
    Homie.getLogger() << "NTP Updated. Current time is " << timeClient.getEpochTime() << endl << endl;
    return time_t(timeClient.getEpochTime());
}

int getTimeOffset(time_t t)
{
    if(month(t) == DST_BEGIN_MONTH && day(t) >= DST_BEGIN_DAY){
        return TIME_OFFSET_DST;
    }

    if(month(t) == DST_END_MONTH && day(t) <= DST_END_DAY){
        return TIME_OFFSET_ST;
    }

    if(month(t) > DST_BEGIN_MONTH && month(t) < DST_END_MONTH){
        return TIME_OFFSET_DST;
    }

    if(month(t) > DST_END_MONTH){
        return TIME_OFFSET_ST;
    }

    return TIME_OFFSET_DST;
}

const char *getTimestamp() {
    timestampStream.reset();
    timestampStream << String(year()) << "-";

    if (month() < 10) {
        timestampStream << "0" << String(month());
    } else {
        timestampStream << String(month());
    }

    timestampStream << "-";

    if (day() < 10) {
        timestampStream << "0" << String(day());
    } else {
        timestampStream << String(day());
    }

    timestampStream << "T";

    if (hour() < 10) {
        timestampStream << "0" << String(hour());
    } else {
        timestampStream << String(hour());
    }

    timestampStream << ":";

    if (minute() < 10) {
        timestampStream << "0" << String(minute());
    } else {
        timestampStream << String(minute());
    }

    timestampStream << ":";

    if (second() < 10) {
        timestampStream << "0" << String(second());
    } else {
        timestampStream << String(second());
    }

    timestampStream << ".00+00:00";
    // Serial.println("TIMESTAMP = " + String(timestampStream.get()));
    return timestampStream.get();
}

void strToAddress(const String &addr, DeviceAddress deviceAddress) {
    char byt[3] = {0, 0, 0};
    unsigned int number;

    for (size_t i = 0; i < addr.length(); i += 2) {
        byt[0] = addr[i];
        byt[1] = addr[i + 1];

        number = (int) strtol(byt, nullptr, 16);
        deviceAddress[i / 2] = number;
    }

}

void printAddress(DeviceAddress deviceAddress) {
    for (uint8_t i = 0; i < 8; i++) {
        if (deviceAddress[i] < 16) Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
    }
}

void setupOwSensors() {
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
    for (size_t i = 0; i < sensors.getDeviceCount(); i++) {
        if (sensors.getAddress(tempDeviceAddress, i)) {
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

#ifdef DEBUG
    if (sensors.isConnected(tempSensorIn)) {
        Serial.print("Temp sensor IN Address: ");
        printAddress(tempSensorIn);
        Serial.println();
        Serial.print("Temp sensor IN resolution: ");
        Serial.print(sensors.getResolution(tempSensorIn), DEC);
        Serial.println();
    } else {
        Serial.println("Temp sensor IN is not connected !");
    }

    if (sensors.isConnected(tempSensorOut)) {
        Serial.print("Temp sensor OUT Address: ");
        printAddress(tempSensorOut);
        Serial.println();
        Serial.print("Temp sensor OUT resolution: ");
        Serial.print(sensors.getResolution(tempSensorOut), DEC);
        Serial.println();
    } else {
        Serial.println("Temp sensor OUT is not connected !");
    }
#endif
}

int16_t mcpReadCallback(uint8_t channel) {
    return mcp.analogRead(channel);
}

ThermistorSettings parseNTCSettings(const char * json, const char * name)
{
    ThermistorSettings fs{};

//    sscanf(settings, "vcc=%lf,adcRef=%lf,serRes=%lf,ntcRes=%lf,tempNom=%lf,bc=%lf,samples=%i,sampleDly=%i", // NOLINT(cert-err34-c)
//      &fs.vcc, &fs.analogReference, &fs.seriesResistor, &fs.thermistorNominal, &fs.temperatureNominal, &fs.bCoef, &fs.samples, &fs.sampleDelay
//      );

    jsonError = deserializeJson(doc, json);

    if (jsonError) {
        Homie.getLogger() << "Config: '" << name << "' deserialize failed: " << jsonError.c_str() << endl;
    }else{
        fs.vcc = doc["vcc"].as<float>();
        fs.analogReference = doc["adcRef"].as<float>();
        fs.seriesResistor = doc["serRes"].as<double>();
        fs.thermistorNominal = doc["ntcRes"].as<double>();
        fs.temperatureNominal = doc["tempNom"].as<double>();
        fs.bCoef = doc["bc"].as<double>();
        fs.samples = doc["samples"].as<int>();
        fs.sampleDelay = doc["sampleDly"].as<int>();
    }
    doc.clear();


    Homie.getLogger() << "Parsed " << name << " settings:" << endl;
    Homie.getLogger() << "VCC = " << fs.vcc << endl;
    Homie.getLogger() << "ADC Ref = " << fs.analogReference << endl;
    Homie.getLogger() << "Ser Res = " << fs.seriesResistor << endl;
    Homie.getLogger() << "NTC Res = " << fs.thermistorNominal << endl;
    Homie.getLogger() << "Temp Nom = " << fs.temperatureNominal << endl;
    Homie.getLogger() << "bCoef = " << fs.bCoef << endl;
    Homie.getLogger() << "Samples = " << fs.samples << endl;
    Homie.getLogger() << "Sample Delay = " << fs.sampleDelay << endl;
    Homie.getLogger() << endl;

    return fs;
}

float calcWatts(float tempIn, float tempOut) {
    const float dt = fabs(tempIn - tempOut); // NOLINT(cppcoreguidelines-narrowing-conversions,performance-type-promotion-in-math-fn)
    const auto c = float(0.00682);
    float rtn;
    rtn = (gpm * dt) / c;
    if (rtn < WATTS_MIN) {
        return 0.0;
    }
    return rtn;
}

bool envAllowPump()
{
#ifndef NO_ENV_HOUR_CHECK
    if(hour() < DAY_START_HOUR || hour() >= DAY_END_HOUR) {
        Homie.getLogger() << "Env: Now hour " << hour() << " not between " << DAY_START_HOUR << " and " << DAY_END_HOUR << endl;
        return false;
    }
#endif

    if(realDark){
        Homie.getLogger() << "Env: Real dark" << endl;
        return false;
    }

#ifndef NO_ENV_SP_CHECK
    if(air < setpoint){
        if(((int(f1) + int(f2)) / 2) < (int(air) + FRAME_TO_AIR_MIN_DIFF)){
            Homie.getLogger() << "Env: Frame to air not enough diff" << endl;
            return false;
        }
    }
#endif

    return true;
}

bool turnPumpOn()
{
    if(!envAllowPump()){
        Homie.getLogger() << "Env not allow pump on" << endl;
        turnPumpOff();
        return false;
    }

    if(!pumpOn){
        Homie.getLogger() << "Pump turned on" << endl;
    }
    digitalWrite(RLY_PIN, HIGH);
    pumpOn = true;

    return true;
}

bool turnPumpOff()
{
    if(pumpOn){
        Homie.getLogger() << "Pump turned off" << endl;
    }
    digitalWrite(RLY_PIN, LOW);
    pumpOn = false;

    return true;
}



