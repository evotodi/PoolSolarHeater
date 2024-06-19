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

MCP3204 mcp(&SPI);
Oversampling adc(10, 12, 2);

InterruptButton button1(BTN1_PIN, LOW);
// InterruptButton button2(BTN2_PIN, LOW);

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

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

DynamicJsonDocument poolJsonDoc(MAX_JSON_CONFIG_ARDUINOJSON_BUFFER_SIZE_POOL);

PoolSetting<uint16_t> poolConfigCloudySetting("cloudy", "Cloudy", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<uint16_t> poolConfigOvercastCntSetting("overcastCnt", "Overcast Count", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> poolConfigSunMinElvAMSetting("sunMinElvAM", "Sun Min Elv AM", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> poolConfigSunMinElvPMSetting("sunMinElvPM", "Sun Min Elv PM", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> poolConfigSetPointSetting("setPoint", "Set Point", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> poolConfigSetPointSwingSetting("setPointSwing", "Set Point Swing", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> poolConfigAuxHeatDiffSetting("auxHeatTempDiff", "Aux Heat Diff", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<double> poolConfigAirPoolDiffSetting("airPoolDiff", "Air Pool Diff", &PoolInternals::IPoolSetting::settingsConfig);
PoolSetting<uint16_t> poolConfigPoolTempInSetting("poolTempIn", "Pool Temp Input", &PoolInternals::IPoolSetting::settingsConfig); // 0 = tin 1 = ntc
PoolSetting<double> poolConfigPumpGpmSetting("pumpGpm", "Pump GPM", &PoolInternals::IPoolSetting::settingsConfig);

PoolSetting<double> poolAirOffsetSetting("air", "air offset", &PoolInternals::IPoolSetting::settingsProbeOffset);
PoolSetting<double> poolPoolOffsetSetting("pool", "pool offset", &PoolInternals::IPoolSetting::settingsProbeOffset);
PoolSetting<double> poolTinOffsetSetting("tin", "tin offset", &PoolInternals::IPoolSetting::settingsProbeOffset);
PoolSetting<double> poolToutOffsetSetting("tout", "tout offset", &PoolInternals::IPoolSetting::settingsProbeOffset);

PoolSetting<const char *> poolAirNtcSetting("air", "air kv", &PoolInternals::IPoolSetting::settingsProbe);
PoolSetting<const char *> poolPoolNtcSetting("pool", "pool kv", &PoolInternals::IPoolSetting::settingsProbe);
PoolSetting<const char *> poolTinDtSetting("tin", "tin kv", &PoolInternals::IPoolSetting::settingsProbe);
PoolSetting<const char *> poolToutDtSetting("tout", "tout kv", &PoolInternals::IPoolSetting::settingsProbe);

PoolSetting<int16_t> poolDstOffsetSetting("dstOffset", "DST Offset Hours", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<int16_t> poolStOffsetSetting("stOffset", "ST Offset Hours", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<uint16_t> poolDstBeginDaySetting("dstBeginDay", "DST begin day", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<uint16_t> poolDstBeginMonthSetting("dstBeginMonth", "DST begin month", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<uint16_t> poolDstEndDaySetting("dstEndDay", "DST end day", &PoolInternals::IPoolSetting::settingsTime);
PoolSetting<uint16_t> poolDstEndMonthSetting("dstEndMonth", "DST end month", &PoolInternals::IPoolSetting::settingsTime);

PoolSetting<double> poolLatitudeSetting("latitude", "Latitude", &PoolInternals::IPoolSetting::settingsLocation);
PoolSetting<double> poolLongitudeSetting("longitude", "Longitude", &PoolInternals::IPoolSetting::settingsLocation);

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
bool envCheckNoAuxHeatDiff = false;
bool otaInProgress = false;
bool firstRun = true;

int telnetBuffer;
Smoothed<int16_t> light;
uint16_t cloudyCnt = 0;
Smoothed<int> tin;
bool tinOk = true;
Smoothed<int> tout;
bool toutOk = true;
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
bool pumpOn = false;
bool propaneOn = false;
bool auxOn = false;
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

    Homie_setFirmware("bare-minimum", VERSION)
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
    configNode.advertise("auxHeatTempDiff").setName("AuxHeat Diff").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("airPoolDiff").setName("Air Pool Diff").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("poolTempIn").setName("Pool Temp In").setDatatype("integer").setUnit("°F").settable();
    configNode.advertise("airOffset").setName("Air Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("poolOffset").setName("Pool Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("tinOffset").setName("TIN Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("toutOffset").setName("TOUT Offset").setDatatype("float").setUnit("°F").settable();
    configNode.advertise("pumpGpm").setName("Pump GPM").setDatatype("float").setUnit("GPM").settable();

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
    Homie.getLogger() << "poolTinDtSetting: " << poolTinDtSetting.get() << endl;
    Homie.getLogger() << "poolTinOffsetSetting: " << poolTinOffsetSetting.get() << endl;
    Homie.getLogger() << "poolToutDtSetting: " << poolToutDtSetting.get() << endl;
    Homie.getLogger() << "poolToutOffsetSetting: " << poolToutOffsetSetting.get() << endl;
    Homie.getLogger() << "poolAirNtcSetting: " << poolAirNtcSetting.get() << endl;
    Homie.getLogger() << "poolPoolNtcSetting: " << poolPoolNtcSetting.get() << endl;


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
            button1.processSyncEvents();
            delay(100);
            addTInTemp();
            yield();
            addTOutTemp();
            yield();

            Homie.getLogger() << "Tin = " << ItoF(tin.getLast()) << " °F  Tin Smooth = " << ItoF(tin.get()) << " °F " << endl;
            Homie.getLogger() << "Tout = " << ItoF(tout.getLast()) << " °F  Tout Smooth = " << ItoF(tout.get()) << " °F " << endl;
            yield();

            if (isHeating) {
                watts.add(FtoI(calcWatts(ItoF(tin.getLast()), ItoF(tout.getLast()), float(poolConfigPumpGpmSetting.get()))));
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

            addLight();
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

            Homie.getLogger() << "Light level = " << light.getLast() << " Light smoothed = " << light.get() << endl;
            Homie.getLogger() << "Cloudy = " << boolToStr(isCloudy) << " Overcast = " << boolToStr(isOvercast) << " Cloudy Count = " << cloudyCnt << endl;
            yield();

            getSolar(&solar);
            Homie.getLogger() << "Solar Azimuth = " << solar.azimuth << "° Elevation = " << solar.elevation << "°" << endl;
            yield();

            Homie.getLogger() << "At setpoint = " << boolToStr(atSetpoint) << "  Heating = " << boolToStr(isHeating) << " Heat Type = " << getRunStatusStr() << endl;
            yield();

            Homie.getLogger() << endl;

            checkTempSensors();

            button1.processSyncEvents();
            if (displayPage == DisplayPage::DISP_MAIN) {
                displayPageMainUpdate();
            } else if (displayPage == DisplayPage::DISP_INFO) {
                displayPageInfo();
            }

            previousMillisData = currentMillis;
        }
        yield();
        button1.processSyncEvents();

        // Process data
        if (currentMillis - previousMillisProc > intervalProc || previousMillisProc == 0) {
            Homie.getLogger() << "PROCESS:" << endl;

            button1.processSyncEvents();
            doProcess();

            Homie.getLogger() << endl;
            previousMillisProc = currentMillis;
        }
        yield();
        button1.processSyncEvents();

        // Publish data
        if (currentMillis - previousMillisPub > intervalPub || previousMillisPub == 0) {
            Homie.getLogger() << "PUBLISH:" << endl;
            delay(100); //This is needed to allow time to publish
            statusNode.setProperty("timestamp").send(getTimestamp(true).c_str());
            statusNode.setProperty("heating").send(isHeating ? "true" : "false");
            statusNode.setProperty("at_setpoint").send(atSetpoint ? "true" : "false");
            statusNode.setProperty("cloudy").send(isCloudy ? "true" : "false");
            statusNode.setProperty("overcast").send(isOvercast ? "true" : "false");
            statusNode.setProperty("env_override").send(overrideEnv ? "true" : "false");
            statusNode.setProperty("man_heat").send(manualHeating ? "true" : "false");
            statusNode.setProperty("status").send(status.get());
            Homie.getLogger() << "Status: " << status.get() << endl;
            yield();
            button1.processSyncEvents();

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

            lastPublishData = now();
            previousMillisPub = currentMillis;
        }
        yield();
        button1.processSyncEvents();

        // Publish Config
        if (currentMillis - previousMillisPubCfg > intervalPubCfg || previousMillisPubCfg == 0) {
            Homie.getLogger() << "PUBLISH CONFIG:" << endl;
            button1.processSyncEvents();
            delay(100); //This is needed to allow time to publish
            button1.processSyncEvents();

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
        button1.processSyncEvents();
    }
    // Update Daylight
    if (currentMillis - previousMillisDaylight > intervalDayLight || previousMillisDaylight == 0) {
        getDaylight(&daylight);
        previousMillisDaylight = currentMillis;
    }
    yield();
    button1.processSyncEvents();

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