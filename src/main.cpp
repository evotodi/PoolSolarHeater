#include "main.h"

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
// 289D6F49F67A3CF8
DeviceAddress tempSensorIn; // = { 0x28, 0x9D, 0x6F, 0x49, 0xF6, 0x7A, 0x3C, 0xF8 };
// 283E4749F6C13C1A
DeviceAddress tempSensorOut; // = { 0x28, 0x3E, 0x47, 0x49, 0xF6, 0xC1, 0x3C, 0x1A };

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "192.168.14.254", 0);

#ifdef LOG_TO_TELNET
WiFiServer TelnetServer(TELNET_PORT);
WiFiClient Telnet;
#endif

MCP3204 mcp(MCP_DIN, MCP_DOUT, MCP_CLK);

char timestampStrBuf[1024];
EasyStringStream timestampStream(timestampStrBuf, 1024);

PSHConfig pshConfigs = {int16_t(CLOUDY_DEFAULT), float(SP_DEFAULT), float(SP_HYSTERESIS_DEFAULT), float(AIR_DIFF_DEFAULT), float(ELV_AM_DEFAULT), float(ELV_PM_DEFAULT)};
DTSetting tinSettings = {"0000000000000000", float(0)};
DTSetting toutSettings = {"0000000000000000", float(0)};

ThermistorSettings ambiantSettings = {5.0, 5.0, 100000, 100000, 25, 4096, 3950, 5, 20};
Thermistor ntcAmbiant(mcpReadCallback, ambiantSettings);

#ifndef USE_TIN_AS_POOL
ThermistorSettings poolSettings = {5.0, 5.0, 10000, 10000, 25, 4096, 3950, 5, 20};
Thermistor ntcPool(mcpReadCallback, poolSettings); // Labeled F1 in box
#endif

Oversampling adc(10, 12, 2);

EasyButton calBtn(CAL_PIN);

HomieSetting<const char *> configSetting("config", "config kv");
HomieSetting<const char *> ambiantSetting("ambiant", "ambiant kv");
HomieSetting<const char *> tinSetting("tin", "tin json");
HomieSetting<const char *> toutSetting("tout", "tout json");
#ifndef USE_TIN_AS_POOL
HomieSetting<const char *> poolSetting("pool", "pool kv");
#endif

unsigned long currentMillis = 0;
unsigned long intervalData = LOOP_DAT_DLY;
unsigned long previousMillisData = 0;
unsigned long intervalProc = LOOP_PROC_DLY;
unsigned long previousMillisProc = 0;
unsigned long intervalPub = LOOP_PUB_DLY;
unsigned long previousMillisPub = 0;
unsigned long intervalHB = LOOP_HB_DLY;
unsigned long previousMillisHB = 0;
unsigned long intervalDayLight = LOOP_DAYLIGHT_DLY;
unsigned long previousMillisDaylight = 0;

bool isCloudy = false;
bool isOvercast = false;
bool atSetpoint = false;
bool overrideEnv = false;
bool isHeating = false;

int telnetBuffer;
Smoothed<int16_t> light;
char cloudyCnt = 0;
Smoothed<float> tin;
Smoothed<float> tout;
Smoothed<float> air;
Smoothed<float> pool;
auto solar = Solar{};
auto daylight = Daylight{};

void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#else
    Homie.disableLogging();
#endif
    light.begin(SMOOTHED_AVERAGE, 3);
    tin.begin(SMOOTHED_AVERAGE, 5);
    tout.begin(SMOOTHED_AVERAGE, 5);
    pool.begin(SMOOTHED_AVERAGE, 5);
    air.begin(SMOOTHED_AVERAGE, 3);

    WiFi.mode(WIFI_STA);

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

    Homie_setFirmware("bare-minimum", "1.0.0")
    Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
    Homie_setBrand("PoolHeater")

#ifdef LOG_TO_TELNET
    Homie.setLoggingPrinter(&Telnet);
    Homie.onEvent(onHomieEvent);
#endif

    Homie.setup();
}

void loop() 
{
    Homie.loop();
#ifdef LOG_TO_TELNET
    handleTelnet();
#endif
}

void setupHandler() 
{
    // Code which should run AFTER the Wi-Fi is connected.
#ifdef LOG_TO_TELNET
    TelnetServer.begin();
    Serial.print("Starting telnet server on port "); Serial.println(TELNET_PORT);
#endif
    pinMode(LED_BUILTIN_AUX, OUTPUT);

    timeClient.begin();
    yield();
    timeClient.forceUpdate();
    yield();

    setSyncProvider(getNtpTime);
    setSyncInterval(600);
    yield();

    parsePSHSettings(&pshConfigs, configSetting.get(), "Config");

    parseDTSettings(&tinSettings, tinSetting.get(), "Tin");
    strToAddress(tinSettings.addr, tempSensorIn);
    parseDTSettings(&toutSettings, toutSetting.get(), "Tout");
    strToAddress(toutSettings.addr, tempSensorOut);
    yield();

    parseNTCSettings(ambiantSetting.get(), "Ambiant", &ambiantSettings);
    ntcAmbiant.setSeriesResistor(&ambiantSettings.seriesResistor);
    ntcAmbiant.setThermistorNominal(&ambiantSettings.thermistorNominal);
    ntcAmbiant.setBCoef(&ambiantSettings.bCoef);
    ntcAmbiant.setTemperatureNominal(&ambiantSettings.temperatureNominal);
    ntcAmbiant.setVcc(&ambiantSettings.vcc);
    ntcAmbiant.setAnalogReference(&ambiantSettings.analogReference);
    Homie.getLogger() << "Ambiant settings = " << ntcAmbiant.dumpSettings() << endl;
    yield();

#ifndef USE_TIN_AS_POOL
    parseNTCSettings(poolSetting.get(), "Pool", &poolSettings);
    ntcPool.setSeriesResistor(&poolSettings.seriesResistor);
    ntcPool.setThermistorNominal(&poolSettings.thermistorNominal);
    ntcPool.setBCoef(&poolSettings.bCoef);
    ntcPool.setTemperatureNominal(&poolSettings.temperatureNominal);
    ntcPool.setVcc(&poolSettings.vcc);
    ntcPool.setAnalogReference(&poolSettings.analogReference);
    Homie.getLogger() << "Pool settings = " << ntcPool.dumpSettings() << endl;
    yield();
#endif

    setupOwSensors();

    readConfig();

    calBtn.begin();
    calBtn.onPressedFor(2000, calibratePoolTemps);
    calBtn.onSequence(2, 1500, calibrationReset);
    calBtn.enableInterrupt(calBtnISR);

    digitalWrite(LED_BUILTIN_AUX, HIGH);
}

void loopHandler() 
{
    currentMillis = millis();

    // Gather data
    if (currentMillis - previousMillisData > intervalData || previousMillisData == 0) {
        Homie.getLogger() << "GATHER:" << endl;
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

        if (isCloudy) {
            intervalProc = LOOP_SLEEP_DLY;
            intervalPub = LOOP_SLEEP_DLY;
            previousMillisProc = 0;
            cloudyCnt++;
            if(cloudyCnt >= OVERCAST_CNT){
                isOvercast = true;
            }
            yield();
        } else {
            intervalProc = LOOP_PROC_DLY;
            intervalPub = LOOP_PUB_DLY;
            isOvercast = false;
            cloudyCnt = 0;
        }
        if(isOvercast){
            cloudyCnt = OVERCAST_CNT;
        }
        yield();

        tin.add(sensors.getTempF(tempSensorIn) + tinSettings.offset);

        yield();
        tout.add(sensors.getTempF(tempSensorOut) + toutSettings.offset);
        yield();

        Homie.getLogger() << "Tin = " << tin.getLast() << " °F  Tin Smooth = " << tin.get() << " °F " << endl;
        Homie.getLogger() << "Tout = " << tout.getLast() << " °F  Tout Smooth = " << tout.get() << " °F " << endl;
        yield();

        air.add(float(ntcAmbiant.readTempF(ADC_AMBIANT)));
        Homie.getLogger() << "Air = " << air.getLast() << " °F  Air Smooth = " << air.get() << " °F" << endl;
        yield();

#ifdef USE_TIN_AS_POOL
        pool.add(tin.getLast());
#else
        pool.add(float(ntcPool.readTempF(ADC_POOL)));
#endif
        Homie.getLogger() << "Pool = " << pool.getLast() << " °F  Pool Smooth = " << pool.get() << " °F" << endl;
        yield();

        light.add(mcp.analogRead(ADC_LIGHT));
        if (light.get() <= pshConfigs.cloudy) {
            isCloudy = true;
        } else {
            isCloudy = false;
        }
        Homie.getLogger() << "Light level = " << light.getLast() << " Light smoothed = " << light.get() << " Cloudy = " << boolToStr(isCloudy) << " Overcast = " << boolToStr(isOvercast) << endl;
        yield();

        getSolar(&solar);
        Homie.getLogger() << "Solar Azimuth = " << solar.azimuth << "° Elevation = " << solar.elevation << "°" << endl;
        yield();

        Homie.getLogger() << "At setpoint = " << boolToStr(atSetpoint) << "  Heating = " << boolToStr(isHeating) << endl;
        yield();

        Homie.getLogger() << endl;
        previousMillisData = currentMillis;
    }
    yield();
    calBtn.update();

    // Process data
    if (currentMillis - previousMillisProc > intervalProc || previousMillisProc == 0) {
        Homie.getLogger() << "PROCESS:" << endl;

        doProcess();

        Homie.getLogger() << endl;
        previousMillisProc = currentMillis;
    }
    yield();
    calBtn.update();

    // Update Daylight
    if (currentMillis - previousMillisDaylight > intervalDayLight || previousMillisDaylight == 0) {
        getDaylight(&daylight);
        previousMillisDaylight = currentMillis;
    }
    yield();

    // Blink the heartbeat led
    if (currentMillis - previousMillisHB > intervalHB || previousMillisHB == 0) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(75);
        yield();
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        yield();
        digitalWrite(LED_BUILTIN, LOW);
        delay(75);
        yield();
        digitalWrite(LED_BUILTIN, HIGH);
        previousMillisHB = currentMillis;
    }

    calBtn.update();
}

#ifdef LOG_TO_TELNET
void handleTelnet(){
    if (TelnetServer.hasClient()){
        // client is connected
        if (!Telnet || !Telnet.connected()){
            if(Telnet) Telnet.stop();          // client disconnected
            Telnet = TelnetServer.available(); // ready for new client
        } else {
            TelnetServer.available().stop();  // have client, block new conections
        }
    }

    if (Telnet && Telnet.connected() && Telnet.available()){
        // client input processing
        while(Telnet.available()) {
            telnetBuffer = Telnet.read();

            switch (telnetBuffer) {
                case 'h':
                case '?':
                    printHelp();
                    break;
                case 'r':
                    Homie.reboot();
                    break;
                case 'X':
                    HomieInternals::HomieClass::reset();
                    break;
                case 'o':
                    toggleOverrideEnv();
                    break;
                default:
                    Serial.write(telnetBuffer);
            }
        }
    }
}

void printHelp()
{
    Homie.getLogger() << "Help:" << endl;
    Homie.getLogger() << "h or ? - This help" << endl;
    Homie.getLogger() << "r - Reboot" << endl;
    Homie.getLogger() << "X - Reset config to default!" << endl;
    Homie.getLogger() << "o - Toggle override environment (current: " << boolToStr(overrideEnv) << ")" << endl;

    delay(2000);
}

void onHomieEvent(const HomieEvent& event)
{
    if(event.type == HomieEventType::OTA_STARTED){
        if (TelnetServer.hasClient()) {
            TelnetServer.stop();
        }
        TelnetServer.close();
    }
}
#endif

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
void readConfig()
{
    if(!SPIFFS.begin()){
        Homie.getLogger() << F("✖ Failed to initialize SPIFFS for pool config read") << endl;
        return;
    }

    if (!SPIFFS.exists(CONFIG_PATH)) {
        Homie.getLogger() << F("✖ ") << CONFIG_PATH << F(" doesn't exist") << endl;
        return;
    }

    File configFile = SPIFFS.open(CONFIG_PATH, "r");

    if (!configFile) {
        Homie.getLogger() << F("✖ Cannot open pool config file") << endl;
        return;
    }

    size_t configSize = configFile.size();

    if (configSize >= HomieInternals::MAX_JSON_CONFIG_FILE_SIZE) {
        Homie.getLogger() << F("✖ Pool config file too big") << endl;
        return;
    }

    char buf[HomieInternals::MAX_JSON_CONFIG_FILE_SIZE];
    configFile.readBytes(buf, configSize);
    configFile.close();
    buf[configSize] = '\0';

    StaticJsonDocument<HomieInternals::MAX_JSON_CONFIG_ARDUINOJSON_BUFFER_SIZE> jsonDoc;
    if (deserializeJson(jsonDoc, buf) != DeserializationError::Ok || !jsonDoc.is<JsonObject>()) {
        Homie.getLogger() << F("✖ Invalid JSON in the pool config file") << endl;
        return;
    }

    JsonObject parsedJson = jsonDoc.as<JsonObject>();

    tinSettings.offset = parsedJson["tinOffset"].as<float>();
    toutSettings.offset = parsedJson["toutOffset"].as<float>();

    Homie.getLogger() << endl << F("{} Stored Pool configuration") << endl;
    Homie.getLogger() << F("  • TIN Offset: ") << tinSettings.offset << endl;
    Homie.getLogger() << F("  • TOUT Offset: ") << toutSettings.offset << endl;
    Homie.getLogger() << endl;

}
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
void writeConfig()
{
    if(!SPIFFS.begin()){
        Homie.getLogger() << F("✖ Failed to initialize SPIFFS for pool config write") << endl;
        return;
    }

    SPIFFS.remove(CONFIG_PATH);

    File configFile = SPIFFS.open(CONFIG_PATH, "w");
    if (!configFile) {
        Homie.getLogger() << F("✖ Cannot open pool config file") << endl;
        return;
    }

    StaticJsonDocument<HomieInternals::MAX_JSON_CONFIG_ARDUINOJSON_BUFFER_SIZE> jsonDoc;
    jsonDoc["tinOffset"] = tinSettings.offset;
    jsonDoc["toutOffset"] = toutSettings.offset;

    serializeJson(jsonDoc, configFile);
    configFile.close();

    Homie.getLogger() << F("✔ Pool config file written") << endl << endl;
}
#pragma clang diagnostic pop

time_t getNtpTime() 
{
    timeClient.forceUpdate();
    Homie.getLogger() << "NTP Updated. Current time is [ Unix: " << timeClient.getEpochTime() << " Human: " << timeClient.getFormattedTime() << " ]" << endl;
#ifdef DEBUG_FORCE_TIME
    return time_t (DEBUG_FORCE_TIME);
#else
    return time_t(timeClient.getEpochTime());
#endif

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

const char *getTimestamp() 
{
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

void strToAddress(const String &addr, DeviceAddress deviceAddress) 
{
    char byt[3] = {0, 0, 0};
    unsigned int number;

    for (size_t i = 0; i < addr.length(); i += 2) {
        byt[0] = addr[i];
        byt[1] = addr[i + 1];

        number = (int) strtol(byt, nullptr, 16);
        deviceAddress[i / 2] = number;
    }

}

void printAddress(DeviceAddress deviceAddress) 
{
    for (uint8_t i = 0; i < 8; i++) {
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

int16_t mcpReadCallback(uint8_t channel) 
{
    return mcp.analogRead(channel);
}

int16_t adcReadCallback(uint8_t channel) 
{
    return int16_t(adc.read(channel));
}

void parseNTCSettings(const char * settings, const char * name, ThermistorSettings * ts)
{
    sscanf(settings, "vcc=%lf;adcRef=%lf;serRes=%lf", // NOLINT(cert-err34-c)
           &ts->vcc, &ts->analogReference, &ts->seriesResistor
    );

    Homie.getLogger() << "Parsed " << name << " settings = >>>" << settings << "<<<" << endl;
    Homie.getLogger() << "VCC = " << ts->vcc << endl;
    Homie.getLogger() << "ADC Ref = " << ts->analogReference << endl;
    Homie.getLogger() << "Ser Res = " << ts->seriesResistor << endl;
    Homie.getLogger() << endl;
}

void parsePSHSettings(PSHConfig * pPSHConfig, const char * settings, const char * name)
{
    sscanf(settings, "cloudy=%hi;setpoint=%f;swing=%f;airDiff=%f;sunMinElvAM=%f;sunMinElvPM=%f", // NOLINT(cert-err34-c)
           &pPSHConfig->cloudy, &pPSHConfig->setpoint, &pPSHConfig->swing, &pPSHConfig->airDiff, &pPSHConfig->elevationMinAM, &pPSHConfig->elevationMinPM
    );

    Homie.getLogger() << "Parsed " << name << " settings = >>>" << settings << "<<<" << endl;
    Homie.getLogger() << "Cloudy = " << pPSHConfig->cloudy << endl;
    Homie.getLogger() << "Setpoint = " << pPSHConfig->setpoint << endl;
    Homie.getLogger() << "Swing = " << pPSHConfig->swing << endl;
    Homie.getLogger() << "Min Air->Water Diff = " << pPSHConfig->airDiff << endl;
    Homie.getLogger() << "Min Sun Elevation AM = " << pPSHConfig->elevationMinAM << endl;
    Homie.getLogger() << "Min Sun Elevation PM = " << pPSHConfig->elevationMinPM << endl;
    Homie.getLogger() << endl;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat"
void parseDTSettings(DTSetting * pDTSetting, const char * settings, const char * name)
{
    sscanf(settings, "addr=%[0-9a-fA-F]", &pDTSetting->addr); // NOLINT(cert-err34-c)

    Homie.getLogger() << "Parsed " << name << " settings = >>>" << settings << "<<<" << endl;
    Homie.getLogger() << "Address = " << pDTSetting->addr << endl;
    Homie.getLogger() << "Offset = " << pDTSetting->offset << endl;
    Homie.getLogger() << endl;
}
#pragma clang diagnostic pop

void toggleOverrideEnv()
{
    overrideEnv = !overrideEnv;
}

void getSolar(Solar * pSolar)
{
    time_t utc = now();
    // Calculate the solar position, in degrees
    calcHorizontalCoordinates(utc, LATITUDE, LONGITUDE, pSolar->azimuth, pSolar->elevation);

    // // Print results
//#ifdef DEBUG
//    Serial.print("Solar Time: ");
//    Serial.println(utc);
//    Serial.print(F("Solar Az: "));
//    Serial.print(_solar->azimuth);
//    Serial.print(F("°  El: "));
//    Serial.print(_solar->elevation);
//    Serial.println(F("°"));
//#endif
}

void getDaylight(Daylight * pDaylight)
{
    double md;
    time_t utc = now();
    calcCivilDawnDusk(utc, LATITUDE, LONGITUDE, pDaylight->transit, pDaylight->sunrise, pDaylight->sunset);
    md = ((pDaylight->sunset - pDaylight->sunrise) / 2) + pDaylight->sunrise;

    tmElements_t tm;
    tm.Year = year(utc) - 1970;
    tm.Month = month(utc);
    tm.Day = day(utc);
    tm.Hour = 0;
    tm.Minute = 0;
    tm.Second = 0;
    time_t t = makeTime(tm);
    pDaylight->midday = t + long(round(60 * 60 * md));

//#ifdef DEBUG
//    Serial.print("Daylight Time: ");
//    Serial.println(utc);
//    Serial.print(F("Sun Rise: "));
//    Serial.print(pDaylight->sunrise);
//    Serial.print(F("  Sun Set: "));
//    Serial.print(pDaylight->sunset);
//    Serial.print(F("  Transit: "));
//    Serial.println(pDaylight->transit);
//    Serial.print(F("Mid: "));
//    Serial.println(pDaylight->midday);
//#endif
}

void doProcess()
{
    if(pool.get() >= pshConfigs.setpoint) {
        Homie.getLogger() << "Set point reached" << endl;
        turnHeatOff();
        atSetpoint = true;
        return;
    }

    if(pool.get() < (pshConfigs.setpoint - pshConfigs.swing)){
        atSetpoint = false;
    }

    turnHeatOn();
}

bool turnHeatOn()
{
    if(!envAllowHeat()){
        Homie.getLogger() << "Env not allow heat on" << endl;
        turnHeatOff();
        return false;
    }

    if(!isHeating){
        Homie.getLogger() << "Heat turned on" << endl;
    }
    digitalWrite(RLY_PIN, HIGH);
    isHeating = true;
    return true;
}

bool turnHeatOff()
{
    if(isHeating){
        Homie.getLogger() << "Heat turned off" << endl;
    }
    digitalWrite(RLY_PIN, LOW);
    isHeating = false;

    return true;
}

bool envAllowHeat()
{
    bool rtn = true;

    if(overrideEnv){
        return true;
    }

#ifndef NO_ENV_SOLAR_CHECK
    time_t t = now();
    if(t < daylight.midday && solar.elevation <= pshConfigs.elevationMinAM) {
        Homie.getLogger() << "Env: Elevation " << solar.elevation << "° below morning minimum " << pshConfigs.elevationMinAM << "°" << endl;
        rtn = false;
    }
    if(t >= daylight.midday && solar.elevation <= pshConfigs.elevationMinPM) {
        Homie.getLogger() << "Env: Elevation " << solar.elevation << "° below evening minimum " << pshConfigs.elevationMinPM << "°" << endl;
        rtn = false;
    }
#else
    Homie.getLogger() << "Env: Skip solar check" << endl;
#endif

#ifndef NO_ENV_CLOUD_CHECK
    if(isOvercast && air.get() < pshConfigs.setpoint){
        Homie.getLogger() << "Env: Overcast and too cool outside" << endl;
        rtn = false;
    }
#else
    Homie.getLogger() << "Env: Skip cloud check" << endl;
#endif

#ifndef NO_ENV_SP_CHECK
    if (air < pshConfigs.setpoint) {
        if ((int(tin)) < (int(air) + pshConfigs.airDiff)) {
            Homie.getLogger() << "Env: Temp in to air not enough diff" << endl;
            rtn = false;
        }
    }
#else
    Homie.getLogger() << "Env: Skip set-point check" << endl;
#endif

    if(int(tout.get()) < int(tin.get())) {
        Homie.getLogger() << "Env: Temp out less than temp in" << endl;
        rtn = false;
    }

    return rtn;
}

void calBtnISR()
{
    calBtn.read();
}

void calibratePoolTemps()
{
    Homie.getLogger() << "Calibration Started!" << endl;
    digitalWrite(LED_BUILTIN_AUX, LOW);
    delay(2000);

    for (int i = 0; i < 10; i++) {
        digitalWrite(LED_BUILTIN_AUX, !digitalRead(LED_BUILTIN_AUX));
        sensors.requestTemperatures();
        delay(100);

        tin.add(sensors.getTempF(tempSensorIn) + tinSettings.offset);
        tout.add(sensors.getTempF(tempSensorOut) + toutSettings.offset);

        Homie.getLogger() << "Tin = " << tin.getLast() << " °F  Tin Smooth = " << tin.get() << " °F " << endl;
        Homie.getLogger() << "Tout = " << tout.getLast() << " °F  Tout Smooth = " << tout.get() << " °F " << endl;

#ifdef USE_TIN_AS_POOL
        pool.add(tin.getLast());
#else
        pool.add(float(ntcPool.readTempF(ADC_POOL)));
        Homie.getLogger() << "Pool = " << pool.getLast() << " °F  Pool Smooth = " << pool.get() << " °F" << endl;
#endif
        yield();
    }

    digitalWrite(LED_BUILTIN_AUX, LOW);

#ifdef USE_TIN_AS_POOL
    toutSettings.offset = tin.get() - tout.get();
#else
    tinSettings.offset = pool.get() - tin.get();
    toutSettings.offset = pool.get() - tout.get();
#endif

    Homie.getLogger() << "Offsets: tin = " << tinSettings.offset << " °F  tout = " << toutSettings.offset << " °F" << endl;

    writeConfig();

    Homie.getLogger() << "Calibration Completed!" << endl;
    digitalWrite(LED_BUILTIN_AUX, HIGH);

}

void calibrationReset()
{
    Homie.getLogger() << "Calibration Reset!" << endl;
    digitalWrite(LED_BUILTIN_AUX, LOW);
    tinSettings.offset = float(0);
    toutSettings.offset = float(0);
    writeConfig();
    delay(500);
    digitalWrite(LED_BUILTIN_AUX, HIGH);
}
