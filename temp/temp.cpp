#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <EasyStringStream.h>
#include <Homie.h>
#include <Oversampling.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include "config.h"

#define maxAdc 2911
#define maxPct 90.0
#define ONE_WIRE_BUS D2

#define rxPin D5
#define txPin D6                                    // TX Not used
SoftwareSerial vSerial;         // RX, TX Using Software Serial so we can use the hardware serial to check the ouput
char receivedChars[buffsize];                       // an array to store the received data
char tempChars[buffsize];                           // an array to manipulate the received data
char recv_label[num_keywords][label_bytes]  = {0};  // {0} tells the compiler to initalize it with 0. 
char recv_value[num_keywords][value_bytes]  = {0};  // That does not mean it is filled with 0's
char value[num_keywords][value_bytes]       = {0};  // The array that holds the verified data
static byte blockindex = 0;
bool new_data = false;
bool blockend = false;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensor;
bool tempComp = true;
float oTemp = -127.0;

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
Oversampling adc(10, 12, 2);
int adcValue = 0;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

unsigned long intervalData = 1800*1E3;
unsigned long previousMillisData = 0;
unsigned long intervalHB = 1*1E3;
unsigned long previousMillisHB = 0;
unsigned long intervalVE = 2*1E3;
unsigned long previousMillisVE = 0;
char timestampStrBuf[1024];
EasyStringStream timestampStream(timestampStrBuf, 1024);

HomieNode propaneLevelNode("level", "Level", "float");
HomieNode propaneTsNode("ts", "Timestamp", "string");
HomieNode propaneTempNode("temp", "Temperature", "float");

HomieSetting<long> percentageSetting("percentage", "minimum percentage");

void RecvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    Homie.getLogger() << "Receive VE Data Start\n";
    Homie.getLogger() << "New Data = " << new_data << "\n";
    Homie.getLogger() << "DATA: ";
    while (vSerial.available() > 0 && new_data == false) {
        rc = vSerial.read();
        Homie.getLogger() << rc;
        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= buffsize) {
                ndx = buffsize - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            new_data = true;
        }
        yield();
    }
    Homie.getLogger() << "\nReceive VE Data End\n";
}

void ParseData() {
    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(tempChars,"\t");      // get the first part - the label
    // The last field of a block is always the Checksum
    if (strcmp(strtokIndx, "Checksum") == 0) {
        blockend = true;
    }
    strcpy(recv_label[blockindex], strtokIndx); // copy it to label
    
    // Now get the value
    strtokIndx = strtok(NULL, "\r");    // This continues where the previous call left off until '/r'.
    if (strtokIndx != NULL) {           // We need to check here if we don't receive NULL.
        strcpy(recv_value[blockindex], strtokIndx);
    }
    blockindex++;

    if (blockend) {
        // We got a whole block into the received data.
        // Check if the data received is not corrupted.
        // Sum off all received bytes should be 0;
        byte checksum = 0;
        for (int x = 0; x < blockindex; x++) {
            // Loop over the labels and value gotten and add them.
            // Using a byte so the the % 256 is integrated. 
            char *v = recv_value[x];
            char *l = recv_label[x];
            while (*v) {
                checksum += *v;
                v++;
            }
            while (*l) {
                checksum+= *l;
                l++;
            }
            // Because we strip the new line(10), the carriage return(13) and 
            // the horizontal tab(9) we add them here again.  
            checksum += 32;
        }
        // Checksum should be 0, so if !0 we have correct data.
        if (!checksum) {
            // Since we are getting blocks that are part of a 
            // keyword chain, but are not certain where it starts
            // we look for the corresponding label. This loop has a trick
            // that will start searching for the next label at the start of the last
            // hit, which should optimize it. 
            int start = 0;
            for (int i = 0; i < blockindex; i++) {
              for (int j = start; (j - start) < num_keywords; j++) {
                if (strcmp(recv_label[i], keywords[j % num_keywords]) == 0) {
                  // found the label, copy it to the value array
                  strcpy(value[j], recv_value[i]);
                  start = (j + 1) % num_keywords; // start searching the next one at this hit +1
                  break;
                }
              }
            }
        }
        // Reset the block index, and make sure we clear blockend.
        blockindex = 0;
        blockend = false;
    }
}

void HandleNewData() {
    // We have gotten a field of data 
    if (new_data == true) {
        //Copy it to the temp array because parseData will alter it.
        Serial.println("VE New Data");
        strcpy(tempChars, receivedChars);
        ParseData();
        new_data = false;
    }
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}

void getTemperature(DeviceAddress deviceAddress)
{
  oTemp = sensors.getTempC(deviceAddress);
  if(oTemp == DEVICE_DISCONNECTED_C) 
  {
    Homie.getLogger() << "Error: Could not read temperature data\n";
    Homie.getLogger() << "Temp Value: " << oTemp << "\n";
    tempComp = false;
    return;
  }
  oTemp = DallasTemperature::toFahrenheit(oTemp);
  Homie.getLogger() << "Temp F: " << oTemp << "\n";
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

time_t getNtpTime()
{
  timeClient.forceUpdate();
  Serial.print("NTP Updated. Current time is ");
  Serial.println(timeClient.getEpochTime());
  return time_t(timeClient.getEpochTime());
}

double regressVol(double x) {
  double terms[] = {
    -1.0151275088294892e+005,
     2.5752581425469202e+002,
    -2.4852643047892017e-001,
     1.0535573031481093e-004,
    -1.3875714974931539e-008,
    -1.3887817516824368e-012,
     9.6479981812471241e-016,
    -2.0161148350930362e-018,
     1.1863146248712647e-021,
    -6.8121827116753780e-026,
    -1.4877447203358990e-028,
     5.5254475626628387e-032,
    -7.9402147316181344e-036,
     4.1552370260928951e-040
  };
  
  double t = 1;
  double r = 0;
  for (double c : terms) {
    r += c * t;
    t *= x;
  }
  if (r > 100.0) {
    r = 100.0;
  }
  if (r < 0.0) {
    r = 0.0;
  }
  if (x > maxAdc) {
    r = maxPct;
  }
  return r;
}

double regressTemp(double x) {
  double terms[] = {
     1.1304898207630996e+000,
    -2.4547873798194740e-003,
     5.0890564996290990e-006,
    -7.2920442224548634e-009
  };
  
  double t = 1;
  double r = 0;
  for (double c : terms) {
    r += c * t;
    t *= x;
  }
  return r;
}

void setupHandler() {
  // Code which should run AFTER the WiFi is connected.
  timeClient.begin();
  timeClient.forceUpdate();

  setSyncProvider(getNtpTime);
  setSyncInterval(600);
}

void loopHandler()
{
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillisData > intervalData || previousMillisData == 0) {
    if (year() == 1970) {
      Serial.println("Force update NTP");
      timeClient.forceUpdate();
      time_t tt;
      tt = getNtpTime();
      setTime(tt);
      yield();
    }

    Serial.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    Serial.println("DONE");
    getTemperature(tempSensor);

    yield();

    adcValue = adc.read(analogInPin);
    double pO;
    double pC;
    pO = regressVol(adcValue);
    pC = pO;
    if(tempComp){
      double c;
      c = regressTemp(oTemp);
      pC = pO * c; // Apply temp correction factor
    }

    yield();

    Homie.getLogger() << "Counts = " << adcValue << "; Orig Percent = " << pO << "; Comp Percent = " << pC << "\n";
    propaneLevelNode.setProperty("percent").send(String(pC));
    propaneTsNode.setProperty("timestamp").send(getTimestamp());
    propaneTempNode.setProperty("temperature").send(String(oTemp));
    if(tempComp){
      propaneTempNode.setProperty("comp").send("true");
    }else{
      propaneTempNode.setProperty("comp").send("false");
    }
    if(pC <= float(percentageSetting.get())){
      propaneLevelNode.setProperty("alarm").send("true");
    }else{
      propaneLevelNode.setProperty("alarm").send("false");
    }
    
    previousMillisData = currentMillis;
  }

  if(currentMillis - previousMillisVE > intervalVE || previousMillisVE == 0){
    RecvWithEndMarker();
    HandleNewData();
    for (int i = 0; i < num_keywords; i++){
        Serial.print(keywords[i]);
        Serial.print(": ##");
        Serial.print(value[i]);
        Serial.println("##");
    }
    previousMillisVE = currentMillis;
  }

  // Blink the heartbeat led
  if(currentMillis - previousMillisHB > intervalHB || previousMillisHB == 0){
    digitalWrite(LED_BUILTIN_AUX, !digitalRead(LED_BUILTIN_AUX));
    previousMillisHB = currentMillis;
  }
}

void setup()
{
  Serial.begin(115200);
  vSerial.begin(19200, SWSERIAL_8N1, rxPin, txPin, false, 512);

  pinMode(LED_BUILTIN_AUX, OUTPUT);

  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  if (!sensors.getAddress(tempSensor, 0)){
    Serial.println("Unable to find address for Device 0");
    tempComp = false;
  }

  Serial.print("Device 0 Address: ");
  printAddress(tempSensor);

  sensors.setResolution(tempSensor, 9);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(tempSensor), DEC); 
  Serial.println();

  Homie_setFirmware("bare-minimum", "1.0.0");
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
  Homie_setBrand("PropaneMon");

  propaneLevelNode.advertise("percent").setName("Percent")
                                      .setDatatype("float")
                                      .setUnit("%");
  propaneLevelNode.advertise("alarm").setName("Alarm")
                                      .setDatatype("string")
                                      .setUnit("");                                      
  propaneTsNode.advertise("timestamp").setName("Timestamp")
                                      .setDatatype("string")
                                      .setUnit("");                                      
  propaneTempNode.advertise("temperature").setName("Temperature")
                                      .setDatatype("float")
                                      .setUnit("°F");                                      
  propaneTempNode.advertise("comp").setName("Compensation")
                                      .setDatatype("string")
                                      .setUnit("");                                      

  percentageSetting.setDefaultValue(20).setValidator([] (long candidate) {
    return (candidate >= 0) && (candidate <= 100);
  });

  Homie.setup();
}

void loop() 
{
  Homie.loop();

}

