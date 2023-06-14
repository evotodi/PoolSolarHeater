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

PSHConfig pshConfigs = {int16_t(DARK_DEFAULT), float(SP_DEFAULT)};
DTSetting tinSettings = {"0000000000000000", float(TOFS_DEFAULT)};
DTSetting toutSettings = {"0000000000000000", float(TOFS_DEFAULT)};

ThermistorSettings ambiantSettings = {5.0, 5.0, 100000, 100000, 25, 4096, 3950, 5, 20};
Thermistor ntcAmbiant(mcpReadCallback, ambiantSettings);

Oversampling adc(10, 12, 2);

HomieSetting<const char *> configSetting("config", "config kv");
HomieSetting<const char *> ambiantSetting("ambiant", "ambiant kv");
HomieSetting<const char *> tinSetting("tin", "tin json");
HomieSetting<const char *> toutSetting("tout", "tout json");

unsigned long currentMillis = 0;
unsigned long intervalData = LOOP_DAT_DLY;
unsigned long previousMillisData = 0;
unsigned long intervalProc = LOOP_PROC_DLY;
unsigned long previousMillisProc = 0;
unsigned long intervalPub = LOOP_PUB_DLY;
unsigned long previousMillisPub = 0;
unsigned long intervalHB = 1 * 1E3;
unsigned long previousMillisHB = 0;

bool isDark = false;
bool realDark = false;
bool atSetpoint = false;
bool overrideEnv = false;
int telnetBuffer;
int16_t light = 0;
char stayDarkCnt = 0;
float tin = -127.0;
float tout = -127.0;
float air = -127.0;

void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#else
    Homie.disableLogging();
#endif

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
    timeClient.begin();
    yield();
    timeClient.forceUpdate();
    yield();

    setSyncProvider(getNtpTime);
    setSyncInterval(600);
    yield();

    pshConfigs = parsePSHSettings(configSetting.get(), "Config");

    parseNTCSettings(ambiantSetting.get(), "Ambiant", &ambiantSettings);

    tinSettings = parseDTSettings(tinSetting.get(), "Tin");
    strToAddress(tinSettings.addr, tempSensorIn);
    toutSettings = parseDTSettings(toutSetting.get(), "Tout");
    strToAddress(toutSettings.addr, tempSensorOut);
    yield();

    ntcAmbiant.setSeriesResistor(&ambiantSettings.seriesResistor);
    ntcAmbiant.setThermistorNominal(&ambiantSettings.thermistorNominal);
    ntcAmbiant.setBCoef(&ambiantSettings.bCoef);
    ntcAmbiant.setTemperatureNominal(&ambiantSettings.temperatureNominal);
    ntcAmbiant.setVcc(&ambiantSettings.vcc);
    ntcAmbiant.setAnalogReference(&ambiantSettings.analogReference);
    Homie.getLogger() << "Ambiant settings = " << ntcAmbiant.dumpSettings() << endl;
    yield();

    setupOwSensors();
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
        if(realDark){
            stayDarkCnt = STAY_DARK_CNT;
        }
        yield();

        tin = sensors.getTempF(tempSensorIn) + tinSettings.offset;
        yield();
        tout = sensors.getTempF(tempSensorOut) + toutSettings.offset;
        yield();

        Homie.getLogger() << "Tin = " << tin << " °F " << "Tout = " << tout << " °F " << endl;
        yield();

        air = float(ntcAmbiant.readTempF(ADC_AMBIANT));
        Homie.getLogger() << "Air = " << air << " °F" << endl;
        yield();

        light = mcp.analogRead(ADC_LIGHT);
        if (light <= pshConfigs.dark) {
            isDark = true;
        } else {
            isDark = false;
        }
        Homie.getLogger() << "Light level = " << light << " Dark out = " << boolToStr(isDark) << " Real Dark = " << boolToStr(realDark) << endl;
        yield();

        Homie.getLogger() << "At setpoint = " << boolToStr(atSetpoint) << endl;
        yield();

        Homie.getLogger() << endl;
        previousMillisData = currentMillis;
    }
    yield();

    // Process data
    if (currentMillis - previousMillisProc > intervalProc || previousMillisProc == 0) {
        Homie.getLogger() << "PROCESS:" << endl;

//        doProcess();

        Homie.getLogger() << endl;
        previousMillisProc = currentMillis;
    }
    yield();

    // Blink the heartbeat led
    if (currentMillis - previousMillisHB > intervalHB || previousMillisHB == 0) {
        digitalWrite(LED_BUILTIN_AUX, !digitalRead(LED_BUILTIN_AUX));
        previousMillisHB = currentMillis;
    }
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

time_t getNtpTime() 
{
    timeClient.forceUpdate();
    Homie.getLogger() << "NTP Updated. Current time is [ Unix: " << timeClient.getEpochTime() << " Human: " << timeClient.getFormattedTime() << " ]" << endl;
    return time_t(timeClient.getEpochTime());
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

PSHConfig parsePSHSettings(const char * settings, const char * name)
{
    PSHConfig fs{};

    sscanf(settings, "dark=%hi;setpoint=%f;airDiff=%i", // NOLINT(cert-err34-c)
           &fs.dark, &fs.setpoint, &fs.airDiff
    );

    Homie.getLogger() << "Parsed " << name << " settings = >>>" << settings << "<<<" << endl;
    Homie.getLogger() << "Dark = " << fs.dark << endl;
    Homie.getLogger() << "Setpoint = " << fs.setpoint << endl;
    Homie.getLogger() << "Min Air->Frame Diff = " << fs.airDiff << endl;
    Homie.getLogger() << endl;

    return fs;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat"
DTSetting parseDTSettings(const char * settings, const char * name)
{
    DTSetting fs{};

    sscanf(settings, "addr=%[0-9a-fA-F];offset=%f", &fs.addr, &fs.offset ); // NOLINT(cert-err34-c)

    Homie.getLogger() << "Parsed " << name << " settings = >>>" << settings << "<<<" << endl;
    Homie.getLogger() << "Address = " << fs.addr << endl;
    Homie.getLogger() << "Offset = " << fs.offset << endl;
    Homie.getLogger() << endl;

    return fs;
}
#pragma clang diagnostic pop

void toggleOverrideEnv()
{
    overrideEnv = !overrideEnv;
}

Solar getSolar()
{
    time_t utc = now();
    Solar solar{};

    // Calculate the solar position, in degrees
    calcHorizontalCoordinates(utc, LATITUDE, LONGITUDE, solar.azimuth, solar.elevation);

    // // Print results
#ifdef DEBUG  
    Serial.print("Solar Time: ");
    Serial.println(utc);
    Serial.print(F("Solar Az: "));
    Serial.print(solar.azimuth);
    Serial.print(F("°  El: "));
    Serial.print(solar.elevation);
    Serial.println(F("°"));
#endif

    return solar;
}
