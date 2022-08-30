/*
  Thermistor.cpp - Universal Thermistor Library

  Original code by Paul Cowan <paul@monospacesoftware.com>
*/
#ifndef UNIVERSAL_THERMISTOR_H
#define UNIVERSAL_THERMISTOR_H

#include <math.h>
#include <arduino.h>

typedef int16_t (*CallBackPtr)(uint8_t);

struct ThermistorSettings
{
    double vcc;
    double analogReference;
    double seriesResistor;
    double thermistorNominal;
    double temperatureNominal;
    double adcMax;
    double bCoef;
    int samples;
    int sampleDelay;
};

class Thermistor {
  protected:
    const CallBackPtr _callback;
    ThermistorSettings _settings;
    char dumpStr[100]{};

  public:

    /*
    * arg 1: callback: Callback to get the adc value
    * arg 2: vcc: Input voltage
    * arg 3: analogReference: reference voltage. Typically the same as vcc, but not always (ie ESP8266=1.0)
    * arg 4: adcMax: The maximum analog-to-digital convert value returned by analogRead (1023 or 4095)
    * arg 5: seriesResistor: The ohms value of the fixed resistor (based on your hardware setup, usually 10k)
    * arg 6: thermistorNominal: Resistance at nominal temperature (will be documented with the thermistor, usually 10k)
    * arg 7: temperatureNominal: Temperature for nominal resistance in celcius (will be documented with the thermistor, assume 25 if not stated)
    * arg 8: bCoef: Beta coefficient (or constant) of the thermistor (will be documented with the thermistor, typically 3380, 3435, or 3950)
    * arg 9: samples: Number of analog samples to average (for smoothing)
    * arg 10: sampleDelay: Milliseconds between samples (for smoothing)
    */
    Thermistor(CallBackPtr callback, ThermistorSettings settings);

    // Smoothed ADC value
    double readADC(uint8_t channel) const;

    // Temperature in Kelvin
    double readTempK(uint8_t channel) const;

    // Temperature in Celsius
    double readTempC(uint8_t channel) const;

    // Temperature in Fahrenheit
    double readTempF(uint8_t channel) const;

    // convert ADC value to Kelvin
    double adcToK(double adc) const;

    // convert Kelvin to Celsius
    double kToC(double k) const;

    // convert Celsius to Fahrenheit
    double cToF(double c) const;

    void setSeriesResistor(const double * res);

    void setThermistorNominal(const double * val);

    void setTemperatureNominal(const double * val);

    void setBCoef(const double * val);

    void setVcc(const double * val);

    void setAnalogReference(const double * val);

    char * dumpSettings(const ThermistorSettings * ts = nullptr);
};

#endif
