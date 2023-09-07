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
Oversampling adc(10, 12, 2);
EasyButton calBtn(CAL_PIN);

/* >>> These struct values will be changed when loading the spiffs config file */
DTSetting tinSettings = {"0000000000000000", float(0)};
DTSetting toutSettings = {"0000000000000000", float(0)};
ThermistorSettings thermistorAirSettings = {5.0, 5.0, 100000, 100000, 25, 4096, 3950, 5, 20};
Thermistor thermistorAir(mcpReadCallback, thermistorAirSettings);
NTCSetting ntcAirSetting = {ADC_AIR, 0.0};
ThermistorSettings thermistorPoolSettings = {5.0, 5.0, 10000, 10000, 25, 4096, 3950, 5, 20};
Thermistor thermistorPool(mcpReadCallback, thermistorPoolSettings); // Labeled F1 in box
NTCSetting ntcPoolSetting = {ADC_POOL, 0.0};
/* <<< */

HomieNode statusNode("status", "Status", "string");
HomieNode valuesNode("values", "Values", "string");
HomieNode configNode("config", "Config", "string", false, 0, 0, configNodeInputHandler);

DynamicJsonDocument poolJsonDoc(MAX_JSON_CONFIG_ARDUINOJSON_BUFFER_SIZE);

std::vector<PoolInternals::IPoolSetting*> __attribute__((init_priority(101))) poolConfigSettings;
PoolSetting<uint16_t> poolConfigCloudySetting("cloudy", "Cloudy", &poolConfigSettings);
PoolSetting<uint16_t> poolConfigOvercastCntSetting("overcastCnt", "Overcast Count", &poolConfigSettings);
PoolSetting<double> poolConfigSunMinElvAMSetting("sunMinElvAM", "Sun Min Elv AM", &poolConfigSettings);
PoolSetting<double> poolConfigSunMinElvPMSetting("sunMinElvPM", "Sun Min Elv PM", &poolConfigSettings);
PoolSetting<double> poolConfigSetPointSetting("setPoint", "Set Point", &poolConfigSettings);
PoolSetting<double> poolConfigSetPointSwingSetting("setPointSwing", "Set Point Swing", &poolConfigSettings);
PoolSetting<double> poolConfigAirPoolDiffSetting("airPoolDiff", "Air Pool Diff", &poolConfigSettings);
PoolSetting<uint16_t> poolConfigPoolTempInSetting("poolTempIn", "Pool Temp Input", &poolConfigSettings); // 0 = tin 1 = ntc
PoolSetting<double> poolConfigPumpGpmSetting("pumpGpm", "Pump GPM", &poolConfigSettings);

std::vector<PoolInternals::IPoolSetting*> __attribute__((init_priority(101))) poolProbeOffsetSettings;
PoolSetting<double> poolAirOffsetSetting("air", "air offset", &poolProbeOffsetSettings);
PoolSetting<double> poolPoolOffsetSetting("pool", "pool offset", &poolProbeOffsetSettings);
PoolSetting<double> poolTinOffsetSetting("tin", "tin offset", &poolProbeOffsetSettings);
PoolSetting<double> poolToutOffsetSetting("tout", "tout offset", &poolProbeOffsetSettings);

std::vector<PoolInternals::IPoolSetting*> __attribute__((init_priority(101))) poolProbeSettings;
PoolSetting<const char *> poolAirNtcSetting("air", "air kv", &poolProbeSettings);
PoolSetting<const char *> poolPoolNtcSetting("pool", "pool kv", &poolProbeSettings);
PoolSetting<const char *> poolTinDtSetting("tin", "tin kv", &poolProbeSettings);
PoolSetting<const char *> poolToutDtSetting("tout", "tout kv", &poolProbeSettings);

std::vector<PoolInternals::IPoolSetting*> __attribute__((init_priority(101))) poolTimeSettings;
PoolSetting<int16_t> poolDstOffsetSetting("dstOffset", "DST Offset Hours", &poolTimeSettings);
PoolSetting<int16_t> poolStOffsetSetting("stOffset", "ST Offset Hours", &poolTimeSettings);
PoolSetting<uint16_t> poolDstBeginDaySetting("dstBeginDay", "DST begin day", &poolTimeSettings);
PoolSetting<uint16_t> poolDstBeginMonthSetting("dstBeginMonth", "DST begin month", &poolTimeSettings);
PoolSetting<uint16_t> poolDstEndDaySetting("dstEndDay", "DST end day", &poolTimeSettings);
PoolSetting<uint16_t> poolDstEndMonthSetting("dstEndMonth", "DST end month", &poolTimeSettings);

std::vector<PoolInternals::IPoolSetting*> __attribute__((init_priority(101))) poolLocationSettings;
PoolSetting<double> poolLatitudeSetting("latitude", "Latitude", &poolLocationSettings);
PoolSetting<double> poolLongitudeSetting("longitude", "Longitude", &poolLocationSettings);

unsigned long currentMillis = 0;
unsigned long intervalData = LOOP_DAT_DLY;
unsigned long previousMillisData = 0;
unsigned long intervalProc = LOOP_PROC_DLY;
unsigned long previousMillisProc = 0;
unsigned long intervalPub = LOOP_PUB_DLY;
unsigned long previousMillisPub = 0;
unsigned long intervalPubCfg = LOOP_PUB_CFG_DLY;
unsigned long previousMillisPubCfg = 0;
unsigned long intervalHB = LOOP_HB_DLY;
unsigned long previousMillisHB = 0;
unsigned long intervalDayLight = LOOP_DAYLIGHT_DLY;
unsigned long previousMillisDaylight = 0;

bool isCloudy = false;
bool isOvercast = false;
bool atSetpoint = false;
bool isHeating = false;
bool overrideEnv = false;
bool manualHeatingEnable = false;
bool manualHeating = false;
bool envCheckNoSolar = false;
bool envCheckNoAir = false;
bool envCheckNoCloud = false;
bool envCheckNoTDiff = false;
bool otaInProgress = false;
bool firstRun = true;

int telnetBuffer;
Smoothed<int16_t> light;
uint16_t cloudyCnt = 0;
Smoothed<int> tin;
Smoothed<int> tout;
Smoothed<int> air;
Smoothed<int> pool;
Smoothed<int> watts;
auto solar = Solar{};
auto daylight = Daylight{};
char statusBuffer[1024];
EasyStringStream status(statusBuffer, 1024);

void setup()
{
    pinMode(LED_BUILTIN_AUX, OUTPUT);
    pinMode(RLY_PIN, OUTPUT);

#ifdef DEBUG
    Serial.begin(115200);
#else
    Homie.disableLogging();
#endif
    light.begin(SMOOTHED_AVERAGE, 3);
    tin.begin(SMOOTHED_AVERAGE, 10);
    tout.begin(SMOOTHED_AVERAGE, 10);
    pool.begin(SMOOTHED_AVERAGE, 10);
    air.begin(SMOOTHED_AVERAGE, 5);
    watts.begin(SMOOTHED_AVERAGE, 5);

    status.reset();

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

    Homie_setFirmware("bare-minimum", "1.0.1")
    Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
    Homie_setBrand("PoolHeater")

    valuesNode.advertise("tin").setName("TempIn").setDatatype("float").setUnit("°F");
    valuesNode.advertise("tout").setName("TempOut").setDatatype("float").setUnit("°F");
    valuesNode.advertise("air").setName("Air").setDatatype("float").setUnit("°F");
    valuesNode.advertise("pool").setName("Pool").setDatatype("float").setUnit("°F");
    valuesNode.advertise("light").setName("LightLvl").setDatatype("float").setUnit("");
    valuesNode.advertise("azimuth").setName("Azimuth").setDatatype("float").setUnit("°");
    valuesNode.advertise("elevation").setName("Elevation").setDatatype("float").setUnit("°");
    valuesNode.advertise("watts").setName("Watts").setDatatype("float").setUnit("W");

    statusNode.advertise("timestamp").setName("Timestamp").setDatatype("string").setUnit("");
    statusNode.advertise("heating").setName("Heating").setDatatype("boolean").settable(mqttHeatOnHandler);
    statusNode.advertise("at_setpoint").setName("At Setpoint").setDatatype("boolean");
    statusNode.advertise("cloudy").setName("Cloudy").setDatatype("boolean");
    statusNode.advertise("overcast").setName("Overcast").setDatatype("boolean");
    statusNode.advertise("env_override").setName("EnvOverride").setDatatype("boolean");
    statusNode.advertise("man_heat").setName("ManualHeat").setDatatype("boolean");
    statusNode.advertise("status").setName("Status").setDatatype("string");

    configNode.advertise("cloudy").setName("Cloudy").setDatatype("integer").setUnit("").settable();
    configNode.advertise("overcastCnt").setName("Overcast Cnt").setDatatype("integer").setUnit("").settable();
    configNode.advertise("sunMinElvAM").setName("Sun Min Elv AM").setDatatype("float").setUnit("°").settable();
    configNode.advertise("sunMinElvPM").setName("Sun Min Elv PM").setDatatype("float").setUnit("°").settable();
    configNode.advertise("setPoint").setName("Set Point").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("setPointSwing").setName("Set Point Swing").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("airPoolDiff").setName("Air Pool Diff").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("poolTempIn").setName("Pool Temp In").setDatatype("integer").setUnit("°F").settable();
    configNode.advertise("airOffset").setName("Air Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("poolOffset").setName("Pool Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("tinOffset").setName("TIN Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("toutOffset").setName("TOUT Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("pumpGpm").setName("Pump GPM").setDatatype("float").setUnit("GPM").settable();

#ifdef LOG_TO_TELNET
    Homie.setLoggingPrinter(&Telnet);
    Homie.onEvent(onHomieEvent);
#endif

    if (!configLoad()) {
        Homie.getLogger() << F("Pool configuration invalid.") << endl;
        delay(5000);
        ESP.restart();
    }
    configUpdateStructs();

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

    timeClient.begin();
    yield();
    timeClient.forceUpdate();
    yield();

    setSyncProvider(getNtpTime);
    setSyncInterval(600);
    yield();

    parseDTSettings(&tinSettings, poolTinDtSetting.get(), poolTinOffsetSetting.get(), "Tin");
    strToAddress(tinSettings.addr, tempSensorIn);

    parseDTSettings(&toutSettings, poolToutDtSetting.get(), poolToutOffsetSetting.get(), "Tout");
    strToAddress(toutSettings.addr, tempSensorOut);
    yield();

    parseNTCSettings(&thermistorAirSettings, poolAirNtcSetting.get(), "Air");
    thermistorAir.setSeriesResistor(&thermistorAirSettings.seriesResistor);
    thermistorAir.setThermistorNominal(&thermistorAirSettings.thermistorNominal);
    thermistorAir.setBCoef(&thermistorAirSettings.bCoef);
    thermistorAir.setTemperatureNominal(&thermistorAirSettings.temperatureNominal);
    thermistorAir.setVcc(&thermistorAirSettings.vcc);
    thermistorAir.setAnalogReference(&thermistorAirSettings.analogReference);
    Homie.getLogger() << "Air settings = " << thermistorAir.dumpSettings() << endl;
    yield();

    parseNTCSettings(&thermistorPoolSettings, poolPoolNtcSetting.get(), "Pool");
    thermistorPool.setSeriesResistor(&thermistorPoolSettings.seriesResistor);
    thermistorPool.setThermistorNominal(&thermistorPoolSettings.thermistorNominal);
    thermistorPool.setBCoef(&thermistorPoolSettings.bCoef);
    thermistorPool.setTemperatureNominal(&thermistorPoolSettings.temperatureNominal);
    thermistorPool.setVcc(&thermistorPoolSettings.vcc);
    thermistorPool.setAnalogReference(&thermistorPoolSettings.analogReference);
    Homie.getLogger() << "Pool settings = " << thermistorPool.dumpSettings() << endl;
    yield();

    setupOwSensors();

    calBtn.begin();
    calBtn.onPressedFor(2000, calibratePoolTemps);
    calBtn.onSequence(2, 1500, calibrationReset);
    calBtn.enableInterrupt(calBtnISR);

    digitalWrite(LED_BUILTIN_AUX, HIGH);
}

void loopHandler() 
{
    currentMillis = millis();

    if (!otaInProgress) {
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

            tin.add(FtoI(sensors.getTempF(tempSensorIn) + tinSettings.offset));

            yield();
            tout.add(FtoI(sensors.getTempF(tempSensorOut) + toutSettings.offset));
            yield();

            Homie.getLogger() << "Tin = " << ItoF(tin.getLast()) << " °F  Tin Smooth = " << ItoF(tin.get()) << " °F " << endl;
            Homie.getLogger() << "Tout = " << ItoF(tout.getLast()) << " °F  Tout Smooth = " << ItoF(tout.get()) << " °F " << endl;
            yield();

            if (isHeating) {
                watts.add(FtoI(calcWatts(ItoF(tin.getLast()), ItoF(tout.getLast()))));
                Homie.getLogger() << "Watts = " << ItoF(watts.getLast()) << " W  Watts Smooth = " << ItoF(watts.get()) << " W " << endl;
            } else {
                watts.clear();
                watts.add(0);
            }

            addAirTemp();
            Homie.getLogger() << "Air = " << ItoF(air.getLast()) << " °F  Air Smooth = " << ItoF(air.get()) << " °F" << endl;
            yield();

            addPoolTemp();
            Homie.getLogger() << "Pool = " << ItoF(pool.getLast()) << " °F  Pool Smooth = " << ItoF(pool.get()) << " °F" << endl;
            yield();

            light.add(mcp.analogRead(ADC_LIGHT));
            if (light.get() <= poolConfigCloudySetting.get()) {
                isCloudy = true;
                cloudyCnt++;
                if (cloudyCnt >= poolConfigOvercastCntSetting.get()) {
                    isOvercast = true;
                }
            } else {
                isCloudy = false;
                isOvercast = false;
                cloudyCnt = 0;
            }

            if (isOvercast) {
                cloudyCnt = poolConfigOvercastCntSetting.get();
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

        // Publish data
        if (currentMillis - previousMillisPub > intervalPub || previousMillisPub == 0) {
            Homie.getLogger() << "PUBLISH:" << endl;
            delay(100); //This is needed to allow time to publish
            statusNode.setProperty("timestamp").send(getTimestamp(true));
            statusNode.setProperty("heating").send(isHeating ? "true" : "false");
            statusNode.setProperty("at_setpoint").send(atSetpoint ? "true" : "false");
            statusNode.setProperty("cloudy").send(isCloudy ? "true" : "false");
            statusNode.setProperty("overcast").send(isOvercast ? "true" : "false");
            statusNode.setProperty("env_override").send(overrideEnv ? "true" : "false");
            statusNode.setProperty("man_heat").send(manualHeating ? "true" : "false");
            statusNode.setProperty("status").send(status.get());
            yield();

            delay(100); //This is needed to allow time to publish
            valuesNode.setProperty("tin").send(ItoS(tin.get()));
            valuesNode.setProperty("tout").send(ItoS(tout.get()));
            valuesNode.setProperty("air").send(ItoS(air.get()));
            valuesNode.setProperty("pool").send(ItoS(pool.get()));
            valuesNode.setProperty("light").send(String(light.get()));
            valuesNode.setProperty("azimuth").send(String(solar.azimuth));
            valuesNode.setProperty("elevation").send(String(solar.elevation));
            valuesNode.setProperty("watts").send(ItoS(watts.get()));
            yield();

            Homie.getLogger() << "Published data" << endl;
            Homie.getLogger() << endl;
            previousMillisPub = currentMillis;
        }
        yield();
        calBtn.update();

        // Publish Config
        if (currentMillis - previousMillisPubCfg > intervalPubCfg || previousMillisPubCfg == 0) {
            Homie.getLogger() << "PUBLISH CONFIG:" << endl;
            delay(100); //This is needed to allow time to publish

            configNode.setProperty("cloudy").send(String(poolConfigCloudySetting.get()));
            configNode.setProperty("overcastCnt").send(String(poolConfigOvercastCntSetting.get()));
            configNode.setProperty("sunMinElvAM").send(String(poolConfigSunMinElvAMSetting.get()));
            configNode.setProperty("sunMinElvPM").send(String(poolConfigSunMinElvPMSetting.get()));
            configNode.setProperty("setPoint").send(String(poolConfigSetPointSetting.get()));
            configNode.setProperty("setPointSwing").send(String(poolConfigSetPointSwingSetting.get()));
            configNode.setProperty("airPoolDiff").send(String(poolConfigAirPoolDiffSetting.get()));
            configNode.setProperty("poolTempIn").send(String(poolConfigPoolTempInSetting.get()));
            configNode.setProperty("airOffset").send(String(poolAirOffsetSetting.get()));
            configNode.setProperty("poolOffset").send(String(poolPoolOffsetSetting.get()));
            configNode.setProperty("tinOffset").send(String(poolTinOffsetSetting.get()));
            configNode.setProperty("toutOffset").send(String(poolToutOffsetSetting.get()));
            configNode.setProperty("pumpGpm").send(String(poolConfigPumpGpmSetting.get()));

            Homie.getLogger() << "Published config" << endl;
            Homie.getLogger() << endl;
            previousMillisPubCfg = currentMillis;
        }
        yield();
        calBtn.update();
    }
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
    firstRun = false;
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
                case 'm':
                    toggleManualHeatingEnable();
                    break;
                case 'h':
                    toggleManualHeating();
                    break;
                case 'u':
                    calibratePoolTemps();
                    break;
                case 'i':
                    calibrationReset();
                    break;
                case 'p':
                    previousMillisPub = 0;
                    break;
                case 's':
                    toggleEnvNoCheckSolar();
                    break;
                case 'a':
                    toggleEnvNoCheckAir();
                    break;
                case 'c':
                    toggleEnvNoCheckCloud();
                    break;
                case 'd':
                    toggleEnvNoCheckTDiff();
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
    Homie.getLogger() << "? - This help" << endl;
    Homie.getLogger() << endl;
    Homie.getLogger() << "r - Reboot" << endl;
    Homie.getLogger() << "X - Reset config to default! (capital x)" << endl;
    Homie.getLogger() << endl;
    Homie.getLogger() << "u - Calibrate tin and tout to pool" << endl;
    Homie.getLogger() << "i - Calibration Reset" << endl;
    Homie.getLogger() << endl;
    Homie.getLogger() << "p - Publish homie now" << endl;
    Homie.getLogger() << endl;
    Homie.getLogger() << "o - Toggle override environment (current: " << boolToStr(overrideEnv) << ")" << endl;
    Homie.getLogger() << "m - Toggle manual heating enable (current: " << boolToStr(manualHeatingEnable) << ")" << endl;
    Homie.getLogger() << "h - Toggle manual heating (current: " << boolToStr(manualHeating) << ")" << endl;
    Homie.getLogger() << endl;
    Homie.getLogger() << "s - Toggle environment don't check solar (current: " << boolToStr(envCheckNoSolar) << ")" << endl;
    Homie.getLogger() << "a - Toggle environment don't check air (current: " << boolToStr(envCheckNoAir) << ")" << endl;
    Homie.getLogger() << "c - Toggle environment don't check cloudy (current: " << boolToStr(envCheckNoCloud) << ")" << endl;
    Homie.getLogger() << "d - Toggle environment don't check tin tout diff (current: " << boolToStr(envCheckNoTDiff) << ")" << endl;

    delay(3000);
}
#endif

void onHomieEvent(const HomieEvent& event)
{
    if(event.type == HomieEventType::OTA_STARTED) {
#ifdef LOG_TO_TELNET
        if (TelnetServer.hasClient()) {
            TelnetServer.stop();
        }
        TelnetServer.close();
#endif
        otaInProgress = true;
    }
    else if(event.type == HomieEventType::WIFI_CONNECTED) {
        Serial.print("Wi-Fi connected, IP: ");
        Serial.println(event.ip);
    }
    else if(event.type == HomieEventType::OTA_FAILED || event.type == HomieEventType::OTA_SUCCESSFUL) {
        otaInProgress = false;
    }
    else if(event.type == HomieEventType::WIFI_DISCONNECTED) {
        Homie.getLogger() << F("✖ Failed to connect to wifi. Rebooting...") << endl;
        ESP.reset();
    }
}

bool configLoad()
{
    configRead();
    JsonObject parsedJson = poolJsonDoc.as<JsonObject>();

    PoolInternals::ConfigValidationResult configValidationResult = PoolInternals::Validation::validateConfig(parsedJson);
    if (!configValidationResult.valid) {
        Homie.getLogger() << F("✖ Pool config file is not valid, reason: ") << configValidationResult.reason << endl;

        return false;
    }

    configSetPoolSettings(parsedJson["config"].as<JsonObject>(), &poolConfigSettings);
    configSetPoolSettings(parsedJson["probeOffsets"].as<JsonObject>(), &poolProbeOffsetSettings);
    configSetPoolSettings(parsedJson["probeSettings"].as<JsonObject>(), &poolProbeSettings);
    configSetPoolSettings(parsedJson["time"].as<JsonObject>(), &poolTimeSettings);
    configSetPoolSettings(parsedJson["location"].as<JsonObject>(), &poolLocationSettings);

    Homie.getLogger() << endl << F("{} Stored Pool configuration") << endl;
    configLogSettings("Config", &poolConfigSettings);
    configLogSettings("Probe Offsets", &poolProbeOffsetSettings);
    configLogSettings("Probe Settings", &poolProbeSettings);
    configLogSettings("Time", &poolTimeSettings);
    configLogSettings("Location", &poolLocationSettings);

    return true;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
void configRead()
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
#ifdef PRINT_POOL_CONFIG_ON_READ_WRITE
    configFile.seek(0);
    Homie.getLogger() << F("Read Pool Config File Content >>>") << endl;
    while(configFile.available()){
        Homie.getLogger().write(configFile.read());
    }
    Homie.getLogger() << "<<<" << endl;
#endif
    configFile.close();
    buf[configSize] = '\0';


    poolJsonDoc.clear();
    if (deserializeJson(poolJsonDoc, buf) != DeserializationError::Ok || !poolJsonDoc.is<JsonObject>()) {
        Homie.getLogger() << F("✖ Invalid JSON in the pool config file") << endl;
        return;
    }
}
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-use-auto"
#pragma ide diagnostic ignored "cppcoreguidelines-pro-type-static-cast-downcast"
void configSetPoolSettings(JsonObject settingsObject, std::vector<PoolInternals::IPoolSetting*> * settings)
{
    for (auto &iSetting : *settings) {
        JsonVariant reqSetting = settingsObject[iSetting->getName()];

        if (!reqSetting.isNull()) {
            if (iSetting->isBool()) {
                PoolSetting<bool>* setting = static_cast<PoolSetting<bool>*>(iSetting);
                setting->set(reqSetting.as<bool>());
            } else if (iSetting->isLong()) {
                PoolSetting<long>* setting = static_cast<PoolSetting<long>*>(iSetting);
                setting->set(reqSetting.as<long>());
            } else if (iSetting->isShort()) {
                PoolSetting<int16_t>* setting = static_cast<PoolSetting<int16_t>*>(iSetting);
                setting->set(reqSetting.as<int16_t>());
            } else if (iSetting->isUShort()) {
                PoolSetting<uint16_t>* setting = static_cast<PoolSetting<uint16_t>*>(iSetting);
                setting->set(reqSetting.as<uint16_t>());
            } else if (iSetting->isDouble()) {
                PoolSetting<double>* setting = static_cast<PoolSetting<double>*>(iSetting);
                setting->set(reqSetting.as<double>());
            } else if (iSetting->isConstChar()) {
                PoolSetting<const char*>* setting = static_cast<PoolSetting<const char*>*>(iSetting);
                setting->set(strdup(reqSetting.as<const char*>()));
            }
        }
    }
}
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-use-auto"
#pragma ide diagnostic ignored "cppcoreguidelines-pro-type-static-cast-downcast"
void configLogSettings(const char * name, std::vector<PoolInternals::IPoolSetting*> * settings)
{
    if (!settings->empty()) {
        Homie.getLogger() << F("  • ") << name << F(" settings: ") << endl;
        for (auto &iSetting : *settings) {
            Homie.getLogger() << F("    ◦ ");
            if (iSetting->isBool()) {
                PoolSetting<bool>* setting = static_cast<PoolSetting<bool>*>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isLong()) {
                PoolSetting<long>* setting = static_cast<PoolSetting<long>*>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isShort()) {
                PoolSetting<int16_t>* setting = static_cast<PoolSetting<int16_t>*>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isUShort()) {
                PoolSetting<uint16_t>* setting = static_cast<PoolSetting<uint16_t>*>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isDouble()) {
                PoolSetting<double>* setting = static_cast<PoolSetting<double>*>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isConstChar()) {
                PoolSetting<const char*>* setting = static_cast<PoolSetting<const char*>*>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            }

            Homie.getLogger() << endl;
        }
    }
}
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
void configWrite()
{
    poolJsonDoc.clear();

    JsonObject config = poolJsonDoc.createNestedObject("config");
    config["cloudy"] = poolConfigCloudySetting.get();
    config["overcastCnt"] = poolConfigOvercastCntSetting.get();
    config["sunMinElvAM"] = poolConfigSunMinElvAMSetting.get();
    config["sunMinElvPM"] = poolConfigSunMinElvPMSetting.get();
    config["setPoint"] = poolConfigSetPointSetting.get();
    config["setPointSwing"] = poolConfigSetPointSwingSetting.get();
    config["airPoolDiff"] = poolConfigAirPoolDiffSetting.get();
    config["poolTempIn"] = poolConfigPoolTempInSetting.get();
    config["pumpGpm"] = poolConfigPumpGpmSetting.get();

    JsonObject probeOffsets = poolJsonDoc.createNestedObject("probeOffsets");
    probeOffsets["air"] = poolAirOffsetSetting.get();
    probeOffsets["pool"] = poolPoolOffsetSetting.get();
    probeOffsets["tin"] = poolTinOffsetSetting.get();
    probeOffsets["tout"] = poolToutOffsetSetting.get();

    JsonObject probeSettings = poolJsonDoc.createNestedObject("probeSettings");
    probeSettings["air"] = poolAirNtcSetting.get();
    probeSettings["pool"] = poolPoolNtcSetting.get();
    probeSettings["tin"] = poolTinDtSetting.get();
    probeSettings["tout"] = poolToutDtSetting.get();

    JsonObject time = poolJsonDoc.createNestedObject("time");
    time["dstOffset"] = poolDstOffsetSetting.get();
    time["stOffset"] = poolStOffsetSetting.get();
    time["dstBeginDay"] = poolDstBeginDaySetting.get();
    time["dstBeginMonth"] = poolDstBeginMonthSetting.get();
    time["dstEndDay"] = poolDstEndDaySetting.get();
    time["dstEndMonth"] = poolDstEndMonthSetting.get();

    JsonObject location = poolJsonDoc.createNestedObject("location");
    location["latitude"] = poolLatitudeSetting.get();
    location["longitude"] = poolLongitudeSetting.get();

    SPIFFS.remove(CONFIG_PATH);

    File configFile = SPIFFS.open(CONFIG_PATH, "w");
    if (!configFile) {
        Homie.getLogger() << F("✖ Cannot open pool config file") << endl;
        return;
    }

#ifdef PRINT_POOL_CONFIG_ON_READ_WRITE
    Homie.getLogger() << endl << F("{} New Stored Pool configuration") << endl;
    serializeJsonPretty(poolJsonDoc, Serial);
    Homie.getLogger() << endl;
#endif

    serializeJsonPretty(poolJsonDoc, configFile);
    configFile.close();

    configUpdateStructs();

    Homie.getLogger() << F("✔ Pool config file written") << endl << endl;
}
#pragma clang diagnostic pop

void configUpdateStructs()
{
    ntcAirSetting.offset = float(poolAirOffsetSetting.get());
    ntcPoolSetting.offset = float(poolPoolOffsetSetting.get());
    tinSettings.offset = float(poolTinOffsetSetting.get());
    toutSettings.offset = float(poolToutOffsetSetting.get());
}

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

int getTimeOffset(const time_t * t, bool asHours)
{
    if(month(*t) == poolDstBeginMonthSetting.get() && day(*t) >= poolDstBeginDaySetting.get()){
        if(asHours) {
            return poolDstOffsetSetting.get();
        }else{
            return poolDstOffsetSetting.get() * 60 * 60;
        }
    }

    if(month(*t) == poolDstEndMonthSetting.get() && day(*t) <= poolDstEndDaySetting.get()){
        if(asHours) {
            return poolStOffsetSetting.get();
        }else{
            return poolStOffsetSetting.get() * 60 * 60;
        }
    }

    if(month(*t) > poolDstBeginMonthSetting.get() && month(*t) < poolDstEndMonthSetting.get()){
        if(asHours) {
            return poolDstOffsetSetting.get();
        }else{
            return poolDstOffsetSetting.get() * 60 * 60;
        }
    }

    if(month(*t) > poolDstEndMonthSetting.get()){
        if(asHours) {
            return poolStOffsetSetting.get();
        }else{
            return poolStOffsetSetting.get() * 60 * 60;
        }
    }

    if(asHours) {
        return poolDstOffsetSetting.get();
    }else{
        return poolDstOffsetSetting.get() * 60 * 60;
    }
}

const char *getTimestamp(bool withOffset)
{
    time_t tNow = now();
    int offsetHours = getTimeOffset(&tNow, true);

    if(withOffset){
        tNow = tNow + (offsetHours * 60 * 60);
    }

    timestampStream.reset();
    timestampStream << String(year(tNow)) << "-";

    if (month() < 10) {
        timestampStream << "0" << String(month(tNow));
    } else {
        timestampStream << String(month(tNow));
    }

    timestampStream << "-";

    if (day() < 10) {
        timestampStream << "0" << String(day(tNow));
    } else {
        timestampStream << String(day(tNow));
    }

    timestampStream << "T";

    if (hour() < 10) {
        timestampStream << "0" << String(hour(tNow));
    } else {
        timestampStream << String(hour(tNow));
    }

    timestampStream << ":";

    if (minute() < 10) {
        timestampStream << "0" << String(minute(tNow));
    } else {
        timestampStream << String(minute(tNow));
    }

    timestampStream << ":";

    if (second() < 10) {
        timestampStream << "0" << String(second(tNow));
    } else {
        timestampStream << String(second(tNow));
    }

    if(withOffset){
#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantConditionsOC"
        if(offsetHours < 0){
            timestampStream << ".00-0" << abs(offsetHours) << ":00";
        }else{
            timestampStream << ".00+0" << abs(offsetHours) << ":00";
        }
#pragma clang diagnostic pop

    }else{
        timestampStream << ".00+00:00";
    }

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
        if (deviceAddress[i] < 16) Homie.getLogger().print("0");
        Homie.getLogger().print(deviceAddress[i], HEX);
    }
}

void setupOwSensors() 
{
    Homie.getLogger() << "Locating one wire devices..." << endl;

    sensors.begin();
    sensors.setWaitForConversion(true);


    Homie.getLogger() << "Found ";
    Homie.getLogger().print(sensors.getDeviceCount(), DEC);
    Homie.getLogger() << " devices." << endl;

    for (size_t i = 0; i < sensors.getDeviceCount(); i++) {
        if (sensors.getAddress(tempDeviceAddress, i)) {
            Homie.getLogger() << "Found device ";
            Homie.getLogger().print(i, DEC);
            Homie.getLogger() << " with address: ";
            printAddress(tempDeviceAddress);
            Homie.getLogger() << endl;
            Homie.getLogger() << "Setting resolution to ";
            Homie.getLogger().print(DS_TEMP_PRECISION, DEC);
            Homie.getLogger() << endl;

            // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
            sensors.setResolution(tempDeviceAddress, DS_TEMP_PRECISION);

            Homie.getLogger() << "Resolution actually set to: ";
            Homie.getLogger().print(sensors.getResolution(tempDeviceAddress), DEC);
            Homie.getLogger() << endl;
        } else {
            Homie.getLogger() << "Found ghost device at ";
            Homie.getLogger().print(i, DEC);
            Homie.getLogger() << " but could not detect address. Check power and cabling" << endl;
        }
    }

    if (sensors.isConnected(tempSensorIn)) {
        Homie.getLogger() << "Temp sensor IN Address: ";
        printAddress(tempSensorIn);
        Homie.getLogger() << endl;
        Homie.getLogger() << "Temp sensor IN resolution: ";
        Homie.getLogger().print(sensors.getResolution(tempSensorIn), DEC);
        Homie.getLogger() << endl;
    } else {
        Homie.getLogger() << "Temp sensor IN is not connected !" << endl;
    }

    if (sensors.isConnected(tempSensorOut)) {
        Homie.getLogger() << "Temp sensor OUT Address: ";
        printAddress(tempSensorOut);
        Homie.getLogger() << endl;
        Homie.getLogger() << "Temp sensor OUT resolution: ";
        Homie.getLogger().print(sensors.getResolution(tempSensorOut), DEC);
        Homie.getLogger() << endl;
    } else {
        Homie.getLogger() << "Temp sensor OUT is not connected !" << endl;
    }
}

int16_t mcpReadCallback(uint8_t channel) 
{
    return mcp.analogRead(channel);
}

void parseNTCSettings(ThermistorSettings * ts, const char * settings, const char * name)
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

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat"
void parseDTSettings(DTSetting * pDTSetting, const char * settings, double offset, const char * name)
{
    sscanf(settings, "addr=%[0-9a-fA-F]", &pDTSetting->addr); // NOLINT(cert-err34-c)
    pDTSetting->offset = float(offset);

    Homie.getLogger() << "Parsed " << name << " settings = >>>" << settings << "<<<" << endl;
    Homie.getLogger() << "Address = " << pDTSetting->addr << endl;
    Homie.getLogger() << "Offset = " << pDTSetting->offset << endl;
    Homie.getLogger() << endl;
}
#pragma clang diagnostic pop

void toggleOverrideEnv()
{
    overrideEnv = !overrideEnv;
    if(overrideEnv) {
        Homie.getLogger() << "Environment Override Enabled !" << endl;
    }else{
        Homie.getLogger() << "Environment Override Disabled !" << endl;
        manualHeating = false;
        manualHeatingEnable = false;
        envCheckNoSolar = false;
        envCheckNoAir = false;
        envCheckNoCloud = false;
        envCheckNoTDiff = false;
    }
}

void toggleManualHeatingEnable()
{
    if(!overrideEnv){
        Homie.getLogger() << F("✖ Must override the environment for manual heating") << endl;
        manualHeatingEnable = false;
        return;
    }

    manualHeatingEnable = !manualHeatingEnable;

    if(manualHeatingEnable) {
        Homie.getLogger() << "Environment Manual Heating Enabled !" << endl;
        manualHeating = isHeating;
    }else{
        Homie.getLogger() << "Environment Manual Heating Disabled !" << endl;
        manualHeating = false;
        turnHeatOff();
    }

}

void toggleManualHeating()
{
    if(!overrideEnv){
        Homie.getLogger() << F("✖ Must override the environment for manual heating") << endl;
        manualHeating = false;
        return;
    }

    if(!manualHeatingEnable){
        Homie.getLogger() << F("✖ Must enable manual heating first") << endl;
        manualHeating = false;
        return;
    }

    manualHeating = !manualHeating;

    if(manualHeating) {
        turnHeatOn();
    }else{
        turnHeatOff();
    }
}

void toggleEnvNoCheckSolar()
{
    envCheckNoSolar = !envCheckNoSolar;

    if(envCheckNoSolar) {
        Homie.getLogger() << "Environment don't check solar" << endl;
    }else{
        Homie.getLogger() << "Environment check solar" << endl;
    }
}

void toggleEnvNoCheckAir()
{
    envCheckNoAir = !envCheckNoAir;

    if(envCheckNoAir) {
        Homie.getLogger() << "Environment don't check air" << endl;
    }else{
        Homie.getLogger() << "Environment check air" << endl;
    }
}

void toggleEnvNoCheckCloud()
{
    envCheckNoCloud = !envCheckNoCloud;

    if(envCheckNoCloud) {
        Homie.getLogger() << "Environment don't check cloudy" << endl;
    }else{
        Homie.getLogger() << "Environment check cloudy" << endl;
    }
}

void toggleEnvNoCheckTDiff()
{
    envCheckNoTDiff = !envCheckNoTDiff;

    if(envCheckNoTDiff) {
        Homie.getLogger() << "Environment don't check tin tout diff" << endl;
    }else{
        Homie.getLogger() << "Environment check tin tout diff" << endl;
    }
}

void getSolar(Solar * pSolar)
{
    time_t utc = now();
    // Calculate the solar position, in degrees
    calcHorizontalCoordinates(utc, poolLatitudeSetting.get(), poolLongitudeSetting.get(), pSolar->azimuth, pSolar->elevation);

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
    calcCivilDawnDusk(utc, poolLatitudeSetting.get(), poolLongitudeSetting.get(), pDaylight->transit, pDaylight->sunrise, pDaylight->sunset);
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
    if (firstRun) {
        return;
    }

    if(manualHeatingEnable) {
        return;
    }

    if(ItoF(pool.get()) >= poolConfigSetPointSetting.get()) {
        Homie.getLogger() << "Set point reached" << endl;
        turnHeatOff();
        atSetpoint = true;
        return;
    }

    if(ItoF(pool.get()) < (poolConfigSetPointSetting.get() - poolConfigSetPointSwingSetting.get())){
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

float calcWatts(float tempIn, float tempOut) {
    const float dt = fabsf(tempIn - tempOut);
    const auto c = float(0.00682);
    float rtn;
    rtn = (float(poolConfigPumpGpmSetting.get()) * dt) / c;
    return rtn;
}

bool envAllowHeat()
{
    bool rtn = true;
    status.reset();

    if(overrideEnv){
        Homie.getLogger() << "Env: Override enabled !" << endl;
        status << "ENV: Override EN\n";
        return true;
    }

#ifndef NO_ENV_SOLAR_CHECK
    if(!envCheckNoSolar) {
        time_t t = now();
        if (t < daylight.midday && solar.elevation <= poolConfigSunMinElvAMSetting.get()) {
            Homie.getLogger() << "Env: Elevation " << solar.elevation << "° below morning minimum " << poolConfigSunMinElvAMSetting.get() << "°" << endl;
            status << "ENV: Below AM Elv\n";
            rtn = false;
        }
        if (t >= daylight.midday && solar.elevation <= poolConfigSunMinElvPMSetting.get()) {
            Homie.getLogger() << "Env: Elevation " << solar.elevation << "° below evening minimum " << poolConfigSunMinElvPMSetting.get() << "°" << endl;
            status << "ENV: Below PM Elv\n";
            rtn = false;
        }
    }else{
        Homie.getLogger() << "Env: Skip solar check" << endl;
    }
#else
    Homie.getLogger() << "Env: Skip solar check by define" << endl;
#endif

#ifndef NO_ENV_CLOUD_CHECK
    if(!envCheckNoCloud) {
        if (isOvercast && ItoF(air.get()) < poolConfigSetPointSetting.get()) {
            Homie.getLogger() << "Env: Overcast and too cool outside" << endl;
            status << "ENV: Overcast and Cold\n";
            rtn = false;
        }
    }else {
        Homie.getLogger() << "Env: Skip cloud check" << endl;
    }
#else
    Homie.getLogger() << "Env: Skip cloud check by define" << endl;
#endif

#ifndef NO_ENV_AIR_CHECK
    if(!envCheckNoAir) {
        if (ItoF(air.get()) < poolConfigSetPointSetting.get()) {
            if ((ItoF(air.get()) + poolConfigAirPoolDiffSetting.get()) < ItoF(tin.get())) {
                Homie.getLogger() << "Env: Temp in to air not enough diff" << endl;
                status << "ENV: TIN -> Air diff to small\n";
                rtn = false;
            }
        }
    } else {
        Homie.getLogger() << "Env: Skip set-point check" << endl;
    }
#else
    Homie.getLogger() << "Env: Skip set-point check by define" << endl;
#endif

    // todo-evo: This needs fixed tin and tout can only be check while running
//#ifndef NO_ENV_IN_OUT_DIFF_CHECK
//    if(!envCheckNoTDiff) {
//        if (ItoF(tout.get()) < (ItoF(tin.get()) - pshConfigs.tinDiffMax)) {
//            Homie.getLogger() << "Env: Temp out less than temp in" << endl;
//            status << "ENV: TOUT < TIN\n";
//            rtn = false;
//        }
//    }else {
//        Homie.getLogger() << "Env: Skip tin to tout diff check" << endl;
//    }
//#else
//    Homie.getLogger() << "Env: Skip tin to tout diff check by define" << endl;
//#endif

    if(status.getLength() == 0) {
        status << "ok";
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
    tin.clear();
    tout.clear();
    pool.clear();

    for (int i = 0; i < 10; i++) {
        digitalWrite(LED_BUILTIN_AUX, !digitalRead(LED_BUILTIN_AUX));
        sensors.requestTemperatures();
        delay(100);

        tin.add(FtoI(sensors.getTempF(tempSensorIn)));
        tout.add(FtoI(sensors.getTempF(tempSensorOut)));
        addPoolTemp();
        Homie.getLogger() << "Tin = " << ItoF(tin.getLast()) << " °F  Tin Smooth = " << ItoF(tin.get()) << " °F " << endl;
        Homie.getLogger() << "Tout = " << ItoF(tout.getLast()) << " °F  Tout Smooth = " << ItoF(tout.get()) << " °F " << endl;
        Homie.getLogger() << "Pool = " << ItoF(pool.getLast()) << " °F  Pool Smooth = " << ItoF(pool.get()) << " °F" << endl;

        yield();
    }

    digitalWrite(LED_BUILTIN_AUX, LOW);

    if(poolConfigPoolTempInSetting.get() == 0) {
        toutSettings.offset = ItoF(tin.get()) - ItoF(tout.get());
    } else if(poolConfigPoolTempInSetting.get() == 1) {
        tinSettings.offset = ItoF(pool.get()) - ItoF(tin.get());
        toutSettings.offset = ItoF(pool.get()) - ItoF(tout.get());
    }else{
        Homie.getLogger() << F("✖ Invalid poolTemp type: ") << poolConfigPoolTempInSetting.get() << endl;
    }

    Homie.getLogger() << "Offsets: tin = " << tinSettings.offset << " °F  tout = " << toutSettings.offset << " °F" << endl;

    configWrite();

    Homie.getLogger() << F("✔ Calibration Completed!") << endl;
    digitalWrite(LED_BUILTIN_AUX, HIGH);

}

void calibrationReset()
{
    Homie.getLogger() << F("✔ Calibration Reset!") << endl;
    digitalWrite(LED_BUILTIN_AUX, LOW);
    tinSettings.offset = float(0);
    toutSettings.offset = float(0);
    configWrite();
    delay(500);
    digitalWrite(LED_BUILTIN_AUX, HIGH);
}

float ItoF(const int val)
{
    return (float) val / 10;
}

String ItoS(int val)
{
    return String(ItoF(val));
}

int FtoI(const float val)
{
    return (int) roundf(val * 10);
}

void addPoolTemp()
{
    if(poolConfigPoolTempInSetting.get() == 0) {
        pool.add(tin.getLast());
    } else if(poolConfigPoolTempInSetting.get() == 1) {
        pool.add(FtoI(float(thermistorPool.readTempF(ntcPoolSetting.pin)) + ntcPoolSetting.offset));
    }else{
        Homie.getLogger() << F("✖ Invalid poolTemp type: ") << poolConfigPoolTempInSetting.get() << endl;
    }
}

void addAirTemp()
{
    air.add(FtoI(float(thermistorAir.readTempF(ntcAirSetting.pin)) + ntcAirSetting.offset));
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedParameter"
bool mqttHeatOnHandler(const HomieRange& range, const String& value)
{
    if (value != "true" && value != "false") return false;
    bool on = (value == "true");

    if(on){
        overrideEnv = true;
        manualHeatingEnable = true;
        manualHeating = true;
        turnHeatOn();
    }else{
        overrideEnv = false;
        manualHeatingEnable = false;
        manualHeating = false;
        turnHeatOff();
    }
    statusNode.setProperty("heating").send(value);
    Homie.getLogger() << "MQTT Pump is forced " << (on ? "on" : "off") << endl;

    return true;
}
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedParameter"
bool configNodeInputHandler(const HomieRange& range, const String& property, const String& value)
{
    Homie.getLogger() << "Config Node Input >> Property: " << property << " Value: " << value << endl;

    char *end = nullptr;
    uint16_t tempUint16;
    float tempFloat;
    bool doWrite = false;

    if (strcmp(property.c_str(), "cloudy") == 0) {
        tempUint16 = strtoul(value.c_str(), &end, 10);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigCloudySetting.set(tempUint16);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "overcastCnt") == 0) {
        tempUint16 = strtoul(value.c_str(), &end, 10);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigOvercastCntSetting.set(tempUint16);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "sunMinElvAM") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigSunMinElvAMSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "sunMinElvPM") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigSunMinElvPMSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "setPoint") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigSetPointSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "setPointSwing") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigSetPointSwingSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "airPoolDiff") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigAirPoolDiffSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "poolTempIn") == 0) {
        tempUint16 = strtoul(value.c_str(), &end, 10);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigPoolTempInSetting.set(tempUint16);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "airOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolAirOffsetSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "poolOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolPoolOffsetSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "tinOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolTinOffsetSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "toutOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolToutOffsetSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "pumpGpm") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if(end == value.c_str()){
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigPumpGpmSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }

    if (doWrite) {
        configWrite();
        previousMillisPubCfg = 0;
    }

    return true;
}
#pragma clang diagnostic pop
