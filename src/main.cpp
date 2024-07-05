#include "main.h"

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress tempDeviceAddress;
//DeviceAddress tinAddress;
//DeviceAddress toutSolarAddress;
//DeviceAddress toutHeatAddress;
//DeviceAddress toutPoolAddress;
//DeviceAddress toutAirAddress;


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "192.168.14.254", 0);

#ifdef LOG_TO_TELNET
WiFiServer TelnetServer(TELNET_PORT);
WiFiClient Telnet;
#endif

MCP3204 mcp(&SPI);
Oversampling adc(10, 12, 2);

InterruptButton button1(BTN1_PIN, LOW);
// InterruptButton button2(BTN2_PIN, LOW);

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

/* >>> These struct values will be changed when loading the spiffs config file */
DTSetting dtSettingTin = {"0000000000000000", float(0)};
DTSetting dtSettingToutSolar = {"0000000000000000", float(0)};
DTSetting dtSettingToutHeat = {"0000000000000000", float(0)};
DTSetting dtSettingPool = {"0000000000000000", float(0)};
DTSetting dtSettingAir = {"0000000000000000", float(0)};

/* <<< */

HomieNode statusNode("status", "Status", "string");
HomieNode valuesNode("values", "Values", "string");
HomieNode configNode("config", "Config", "string", false, 0, 0, configNodeInputHandler);

DynamicJsonDocument poolJsonDoc(MAX_JSON_CONFIG_ARDUINOJSON_BUFFER_SIZE_POOL);

// IPoolSetting::settingsConfig
PoolSetting<uint16_t> settingCloudy("cloudy", "Cloudy", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<uint16_t> settingOvercastCnt("overcastCnt", "Overcast Count", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> settingSunMinElvAM("sunMinElvAM", "Sun Min Elv AM", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> settingSunMinElvPM("sunMinElvPM", "Sun Min Elv PM", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> settingPoolSP("poolSP", "Pool Set Point", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> settingHeatAuxSP("heatAuxSP", "Heat Aux Set Point", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> settingSPHyst("spHyst", "Set Point Swing", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> settingPumpGpm("pumpGpm", "Pump GPM", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<bool> settingHeatAuxEnable("heatAuxEnable", "Heat Aux Enable", &PoolInternals::IPoolSetting::settingsConfig);
// IPoolSetting::settingsTime
PoolSetting<int16_t> settingTimeDstOffset("dstOffset", "DST Offset Hours", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<int16_t> settingTimeStOffset("stOffset", "ST Offset Hours", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<uint16_t> settingTimeDstBeginDay("dstBeginDay", "DST begin day", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<uint16_t> settingTimeDstBeginMonth("dstBeginMonth", "DST begin month", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<uint16_t> settingTimeDstEndDay("dstEndDay", "DST end day", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<uint16_t> settingTimeDstEndMonth("dstEndMonth", "DST end month", &PoolInternals::IPoolSetting::settingsTime);
// IPoolSetting::settingsLocation
PoolSetting<double> settingLatitude("latitude", "Latitude", &PoolInternals::IPoolSetting::settingsLocation);
PoolSetting<double> settingLongitude("longitude", "Longitude", &PoolInternals::IPoolSetting::settingsLocation);
// IPoolSetting::settingsProbeOffset
PoolSetting<double> settingAirOffset("air", "air offset", &PoolInternals::IPoolSetting::settingsProbeOffset);
PoolSetting<double> settingPoolOffset("pool", "pool offset", &PoolInternals::IPoolSetting::settingsProbeOffset);
PoolSetting<double> settingTinOffset("tin", "tin offset", &PoolInternals::IPoolSetting::settingsProbeOffset);
PoolSetting<double> settingToutSolarOffset("toutSolar", "toutSolar offset", &PoolInternals::IPoolSetting::settingsProbeOffset);
PoolSetting<double> settingToutHeatOffset("toutHeat", "toutHeat offset", &PoolInternals::IPoolSetting::settingsProbeOffset);
// IPoolSetting::settingsProbe
PoolSetting<const char *> settingAirProbeCfg("air", "air kv", &PoolInternals::IPoolSetting::settingsProbe);
PoolSetting<const char *> settingPoolProbeCfg("pool", "pool kv", &PoolInternals::IPoolSetting::settingsProbe);
PoolSetting<const char *> settingTinProbeCfg("tin", "tin kv", &PoolInternals::IPoolSetting::settingsProbe);
PoolSetting<const char *> settingToutSolarProbeCfg("toutSolar", "tout Solar kv", &PoolInternals::IPoolSetting::settingsProbe);
PoolSetting<const char *> settingToutHeatProbeCfg("toutHeat", "tout Heat kv", &PoolInternals::IPoolSetting::settingsProbe);

unsigned long currentMillis = 0;
// Loop Gather
unsigned long intervalGather = LOOP_GATHER_DLY;
unsigned long prevMillisGather = 0;
// Loop Control
unsigned long intervalControl = LOOP_CONTROL_DLY;
unsigned long prevMillisControl = 0;
// Loop MQTT Publish
unsigned long intervalPub = LOOP_PUB_DLY;
unsigned long prevMillisPub = 0;
// Loop MQTT Pub Config
unsigned long intervalPubCfg = LOOP_PUB_CFG_DLY;
unsigned long prevMillisPubCfg = 0;
// Loop Heartbeat
unsigned long intervalHB = LOOP_HB_DLY;
unsigned long prevMillisHB = 0;
// Loop Update Daylight
unsigned long intervalDayLight = LOOP_DAYLIGHT_DLY;
unsigned long prevMillisDaylight = 0;

bool isCloudy = false;
bool isOvercast = false;
bool atSetpoint = false;
bool otaInProgress = false;
bool firstRun = true;

int telnetBuffer;
Smoothed<int16_t> light;
uint16_t cloudyCnt = 0;
Smoothed<int> tin;
bool tinOk = true;
Smoothed<int> toutSolar;
bool toutSolarOk = true;
Smoothed<int> toutHeat;
bool toutHeatOk = true;
Smoothed<int> air;
bool airOk = true;
Smoothed<int> pool;
bool poolOk = true;
Smoothed<int> watts;
Solar solar = Solar{};
Daylight daylight = Daylight{};
char statusBuffer[1024];
EasyStringStream status(statusBuffer, 1024);
DisplayPage displayPage = DisplayPage::DISP_MAIN;
char ip[17] = "\0";
RunStatus runStatus = RunStatus::OFF;
bool rlyPump = false;
bool rlyHeatAux = false;
bool rlyAux = false;
time_t lastPublishData = 0;

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(PUMP_RLY_PIN, OUTPUT);
    pinMode(AUX_HEAT_RLY_PIN, OUTPUT);
    pinMode(AUX_RLY_PIN, OUTPUT);
    pinMode(BTN1_PIN, INPUT);
    // pinMode(BTN2_PIN, INPUT);
    pinMode(TFT_LED, OUTPUT);
    pinMode(TFT_CS, OUTPUT);

#ifdef DEBUG
    Serial.begin(115200);
#else
    Homie.disableLogging();
#endif

    digitalWrite(TFT_LED, HIGH);
    digitalWrite(TFT_CS, HIGH);
    tft.begin();
#ifdef DEBUG
    // read diagnostics (optional but can help debug problems)
    uint8_t x = tft.readcommand8(ILI9341_RDMODE);
    Serial.print("TFT Power Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDMADCTL);
    Serial.print("TFT MADCTL Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDPIXFMT);
    Serial.print("TFT Pixel Format: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDIMGFMT);
    Serial.print("TFT Image Format: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDSELFDIAG);
    Serial.print("TFT Self Diagnostic: 0x"); Serial.println(x, HEX);
#endif

    displayCenterMessage("Booting...");

    light.begin(SMOOTHED_AVERAGE, 3);
    tin.begin(SMOOTHED_AVERAGE, 10);
    toutSolar.begin(SMOOTHED_AVERAGE, 10);
    toutHeat.begin(SMOOTHED_AVERAGE, 10);
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

    Homie_setFirmware("bare-minimum", VERSION)
    Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
    Homie_setBrand("PoolHeater")

    valuesNode.advertise("tin").setName("TempIn").setDatatype("float").setUnit("°F");
    valuesNode.advertise("toutSolar").setName("TempOut Solar").setDatatype("float").setUnit("°F");
    valuesNode.advertise("toutHeat").setName("TempOut Heat").setDatatype("float").setUnit("°F");
    valuesNode.advertise("air").setName("Air").setDatatype("float").setUnit("°F");
    valuesNode.advertise("pool").setName("Pool").setDatatype("float").setUnit("°F");
    valuesNode.advertise("light").setName("LightLvl").setDatatype("float").setUnit("");
    valuesNode.advertise("azimuth").setName("Azimuth").setDatatype("float").setUnit("°");
    valuesNode.advertise("elevation").setName("Elevation").setDatatype("float").setUnit("°");
    valuesNode.advertise("watts").setName("Watts").setDatatype("float").setUnit("W");

    statusNode.advertise("timestamp").setName("Timestamp").setDatatype("string").setUnit("");
    statusNode.advertise("cloudy").setName("Cloudy").setDatatype("boolean");
    statusNode.advertise("overcast").setName("Overcast").setDatatype("boolean");
    statusNode.advertise("status").setName("Status").setDatatype("string");

    configNode.advertise("cloudy").setName("Cloudy").setDatatype("integer").setUnit("").settable();
    configNode.advertise("overcastCnt").setName("Overcast Cnt").setDatatype("integer").setUnit("").settable();
    configNode.advertise("sunMinElvAM").setName("Sun Min Elv AM").setDatatype("float").setUnit("°").settable();
    configNode.advertise("sunMinElvPM").setName("Sun Min Elv PM").setDatatype("float").setUnit("°").settable();
    configNode.advertise("poolSP").setName("Pool Set Point").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("heatAuxSP").setName("Heat Aux Set Point").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("spHyst").setName("Set Point Swing").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("airOffset").setName("Air Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("poolOffset").setName("Pool Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("tinOffset").setName("TIN Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("toutSolarOffset").setName("TOUT Solar Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("toutHeatOffset").setName("TOUT Heat Aux Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("pumpGpm").setName("Pump GPM").setDatatype("float").setUnit("GPM").settable();
    configNode.advertise("heatAuxEnable").setName("Heat Aux Enable").setDatatype("boolean").settable();

#ifdef LOG_TO_TELNET
    Homie.setLoggingPrinter(&Telnet);
#endif

    Homie.onEvent(onHomieEvent);

    displayCenterMessage("Config load...");
    if (!configLoad()) {
        Homie.getLogger() << F("Pool configuration invalid.") << endl;
        delay(5000);
        ESP.restart();
    }
    configUpdateStructs();

    displayCenterMessage("Homie\nStartup...");
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

    displayCenterMessage("NTP...");
    timeClient.begin();
    yield();
    timeClient.forceUpdate();
    yield();

    setSyncProvider(getNtpTime);
    setSyncInterval(600);
    yield();

    displayCenterMessage("Pool Config...");
    Homie.getLogger() << "settingTinProbeCfg: " << settingTinProbeCfg.get() << endl;
    Homie.getLogger() << "settingTinOffset: " << settingTinOffset.get() << endl;
    Homie.getLogger() << "settingToutSolarProbeCfg: " << settingToutSolarProbeCfg.get() << endl;
    Homie.getLogger() << "settingToutSolarOffset: " << settingToutSolarOffset.get() << endl;
    Homie.getLogger() << "settingPoolProbeCfg: " << settingPoolProbeCfg.get() << endl;
    Homie.getLogger() << "settingPoolOffset: " << settingPoolOffset.get() << endl;
    Homie.getLogger() << "settingAirProbeCfg: " << settingAirProbeCfg.get() << endl;
    Homie.getLogger() << "settingAirOffset: " << settingAirOffset.get() << endl;


    parseDTSettings(&dtSettingTin, settingTinProbeCfg.get(), settingTinOffset.get(), "Tin");
    yield();

    parseDTSettings(&dtSettingToutSolar, settingToutSolarProbeCfg.get(), settingToutSolarOffset.get(), "ToutSolar");
    yield();

    parseDTSettings(&dtSettingToutHeat, settingToutHeatProbeCfg.get(), settingToutHeatOffset.get(), "ToutHeat");
    yield();

    parseDTSettings(&dtSettingPool, settingPoolProbeCfg.get(), settingPoolOffset.get(), "Pool");
    yield();

    parseDTSettings(&dtSettingAir, settingAirProbeCfg.get(), settingAirOffset.get(), "Tout");
    yield();

    setupOwSensors();
    setupButtons();
    displayPageMain();
    digitalWrite(LED_PIN, HIGH);
}

void loopHandler() 
{
    currentMillis = millis();
    button1.processSyncEvents();

    if (!otaInProgress) {
        // Gather data
//        if (currentMillis - previousMillisData > intervalData || previousMillisData == 0) {
//            Homie.getLogger() << "GATHER:" << endl;
//            if (year() == 1970) {
//                Homie.getLogger() << "Force update NTP" << endl;
//                timeClient.forceUpdate();
//                time_t tt;
//                tt = getNtpTime();
//                setTime(tt);
//                yield();
//            }
//
//            sensors.requestTemperatures();
//            button1.processSyncEvents();
//            delay(100);
//            addTInTemp();
//            yield();
//            addTOutTemp();
//            yield();
//
//            Homie.getLogger() << "Tin = " << ItoF(tin.getLast()) << " °F  Tin Smooth = " << ItoF(tin.get()) << " °F " << endl;
//            Homie.getLogger() << "Tout Solar = " << ItoF(toutSolar.getLast()) << " °F  Tout Solar Smooth = " << ItoF(toutSolar.get()) << " °F " << endl;
//            Homie.getLogger() << "Tout Heat = " << ItoF(toutHeat.getLast()) << " °F  Tout Heat Smooth = " << ItoF(toutHeat.get()) << " °F " << endl;
//            yield();
//
//            if (isHeating) {
//                watts.add(FtoI(calcWatts(ItoF(tin.getLast()), ItoF(tout.getLast()), float(poolConfigPumpGpmSetting.get()))));
//                Homie.getLogger() << "Watts = " << ItoF(watts.getLast()) << " W  Watts Smooth = " << ItoF(watts.get()) << " W " << endl;
//            } else {
//                watts.clear();
//                watts.add(0);
//            }
//
//            addAirTemp();
//            Homie.getLogger() << "Air = " << ItoF(air.getLast()) << " °F  Air Smooth = " << ItoF(air.get()) << " °F" << endl;
//            yield();
//
//            addPoolTemp();
//            Homie.getLogger() << "Pool = " << ItoF(pool.getLast()) << " °F  Pool Smooth = " << ItoF(pool.get()) << " °F" << endl;
//            yield();
//
//            addLight();
//            if (light.get() <= poolConfigCloudySetting.get()) {
//                isCloudy = true;
//                cloudyCnt++;
//                if (cloudyCnt >= poolConfigOvercastCntSetting.get()) {
//                    isOvercast = true;
//                }
//            } else {
//                isCloudy = false;
//                isOvercast = false;
//                cloudyCnt = 0;
//            }
//
//            if (isOvercast) {
//                cloudyCnt = poolConfigOvercastCntSetting.get();
//            }
//
//            Homie.getLogger() << "Light level = " << light.getLast() << " Light smoothed = " << light.get() << endl;
//            Homie.getLogger() << "Cloudy = " << boolToStr(isCloudy) << " Overcast = " << boolToStr(isOvercast) << " Cloudy Count = " << cloudyCnt << endl;
//            yield();
//
//            getSolar(&solar);
//            Homie.getLogger() << "Solar Azimuth = " << solar.azimuth << "° Elevation = " << solar.elevation << "°" << endl;
//            yield();
//
//            Homie.getLogger() << "At setpoint = " << boolToStr(atSetpoint) << "  Heating = " << boolToStr(isHeating) << " Heat Type = " << getRunStatusStr() << endl;
//            yield();
//
//            Homie.getLogger() << endl;
//
//            checkTempSensors();
//
//            button1.processSyncEvents();
//            if (displayPage == DisplayPage::DISP_MAIN) {
//                displayPageMainUpdate();
//            } else if (displayPage == DisplayPage::DISP_INFO) {
//                displayPageInfo();
//            }
//
//            previousMillisData = currentMillis;
//        }
        yield();
        button1.processSyncEvents();

        // Process data
//        if (currentMillis - previousMillisProc > intervalProc || previousMillisProc == 0) {
//            Homie.getLogger() << "PROCESS:" << endl;
//
//            button1.processSyncEvents();
//            doProcess();
//
//            Homie.getLogger() << endl;
//            previousMillisProc = currentMillis;
//        }
        yield();
        button1.processSyncEvents();

        // Publish data
//        if (currentMillis - previousMillisPub > intervalPub || previousMillisPub == 0) {
//            Homie.getLogger() << "PUBLISH:" << endl;
//            delay(100); //This is needed to allow time to publish
//            statusNode.setProperty("timestamp").send(getTimestamp(true).c_str());
//            statusNode.setProperty("heating").send(isHeating ? "true" : "false");
//            statusNode.setProperty("at_setpoint").send(atSetpoint ? "true" : "false");
//            statusNode.setProperty("cloudy").send(isCloudy ? "true" : "false");
//            statusNode.setProperty("overcast").send(isOvercast ? "true" : "false");
//            statusNode.setProperty("env_override").send(overrideEnv ? "true" : "false");
//            statusNode.setProperty("man_heat").send(manualHeating ? "true" : "false");
//            statusNode.setProperty("status").send(status.get());
//            Homie.getLogger() << "Status: " << status.get() << endl;
//            yield();
//            button1.processSyncEvents();
//
//            delay(100); //This is needed to allow time to publish
//            valuesNode.setProperty("tin").send(ItoS(tin.get()));
//            valuesNode.setProperty("tout").send(ItoS(tout.get()));
//            valuesNode.setProperty("air").send(ItoS(air.get()));
//            valuesNode.setProperty("pool").send(ItoS(pool.get()));
//            valuesNode.setProperty("light").send(String(light.get()));
//            valuesNode.setProperty("azimuth").send(String(solar.azimuth));
//            valuesNode.setProperty("elevation").send(String(solar.elevation));
//            valuesNode.setProperty("watts").send(ItoS(watts.get()));
//            yield();
//
//            Homie.getLogger() << "Published data" << endl;
//            Homie.getLogger() << endl;
//
//            lastPublishData = now();
//            previousMillisPub = currentMillis;
//        }
        yield();
        button1.processSyncEvents();

        // Publish Config
//        if (currentMillis - previousMillisPubCfg > intervalPubCfg || previousMillisPubCfg == 0) {
//            Homie.getLogger() << "PUBLISH CONFIG:" << endl;
//            button1.processSyncEvents();
//            delay(100); //This is needed to allow time to publish
//            button1.processSyncEvents();
//
//            configNode.setProperty("cloudy").send(String(poolConfigCloudySetting.get()));
//            configNode.setProperty("overcastCnt").send(String(poolConfigOvercastCntSetting.get()));
//            configNode.setProperty("sunMinElvAM").send(String(poolConfigSunMinElvAMSetting.get()));
//            configNode.setProperty("sunMinElvPM").send(String(poolConfigSunMinElvPMSetting.get()));
//            configNode.setProperty("setPoint").send(String(poolConfigSetPointSetting.get()));
//            configNode.setProperty("setPointSwing").send(String(poolConfigSetPointSwingSetting.get()));
//            configNode.setProperty("airPoolDiff").send(String(poolConfigAirPoolDiffSetting.get()));
//            configNode.setProperty("poolTempIn").send(String(poolConfigPoolTempInSetting.get()));
//            configNode.setProperty("airOffset").send(String(poolAirOffsetSetting.get()));
//            configNode.setProperty("poolOffset").send(String(poolPoolOffsetSetting.get()));
//            configNode.setProperty("tinOffset").send(String(poolTinOffsetSetting.get()));
//            configNode.setProperty("toutOffset").send(String(poolToutOffsetSetting.get()));
//            configNode.setProperty("pumpGpm").send(String(poolConfigPumpGpmSetting.get()));
//
//            Homie.getLogger() << "Published config" << endl;
//            Homie.getLogger() << endl;
//            previousMillisPubCfg = currentMillis;
//        }
        yield();
        button1.processSyncEvents();
    }

    // Loop Update Daylight
    if (currentMillis - prevMillisDaylight > intervalDayLight || prevMillisDaylight == 0) {
        getDaylight(&daylight);
        prevMillisDaylight = currentMillis;
    }
    yield();
    button1.processSyncEvents();

    // Loop Heartbeat
    if (currentMillis - prevMillisHB > intervalHB || prevMillisHB == 0) {
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
        prevMillisHB = currentMillis;
    }

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
        displayCenterMessage("OTA Updating...");
        otaInProgress = true;
    }
    else if(event.type == HomieEventType::WIFI_CONNECTED) {
        sprintf(ip, "%s", event.ip.toString().c_str());
        Serial.printf("Wi-Fi connected, IP: %s", ip);
        char s[32];
        sprintf(s, "IP Address\n%s", ip);
        displayCenterMessage(s);
        delay(1500);
    }
    else if(event.type == HomieEventType::OTA_FAILED || event.type == HomieEventType::OTA_SUCCESSFUL) {
        displayCenterMessage("OTA Update\nFAILED!");
        otaInProgress = false;
        delay(3000);
    }
    else if(event.type == HomieEventType::WIFI_DISCONNECTED) {
        Homie.getLogger() << F("✖ Failed to connect to wifi. Rebooting...") << endl;
        displayCenterMessage("WIFI\nDisconnected");
        esp_restart();
    }
}