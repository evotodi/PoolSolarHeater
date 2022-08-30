/*
  thermistor.cpp - Universal Thermistor Library

  Copyright (c) 2018 Paul Cowan <paul@monospacesoftware.com>
  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "Thermistor.h"

#define ABS_ZERO -273.15

Thermistor::Thermistor(
	CallBackPtr callback,
    ThermistorSettings settings):
		_callback(callback),
		_settings(settings)
		 {

}

double Thermistor::readADC(uint8_t channel) const {
	unsigned sum = 0;
	for(int i=0; i<_settings.samples-1; i++) {
		sum += _callback(channel);
		delay(_settings.sampleDelay);
	}
	sum += _callback(channel);
	return (1. * sum) / _settings.samples;
}

double Thermistor::readTempK(uint8_t channel) const {
	return adcToK(readADC(channel));
}

double Thermistor::readTempC(uint8_t channel) const {
	return kToC(readTempK(channel));
}

double Thermistor::readTempF(uint8_t channel) const {
	return cToF(readTempC(channel));
}

double Thermistor::adcToK(double adc) const {
	double resistance = -1.0 * (_settings.analogReference * _settings.seriesResistor * adc) / (_settings.analogReference * adc - _settings.vcc * _settings.adcMax);
	double steinhart = (1.0 / (_settings.temperatureNominal - ABS_ZERO)) + (1.0 / _settings.bCoef) * log(resistance / _settings.thermistorNominal);
	double kelvin = 1.0 / steinhart;
	return kelvin;
}

double Thermistor::kToC(double k) const {
	double c = k + ABS_ZERO;
	return c;
}

double Thermistor::cToF(double c) const {
	return (c * 1.8) + 32;
}

void Thermistor::setSeriesResistor(const double * res)
{
    _settings.seriesResistor = *res;
}

void Thermistor::setThermistorNominal(const double * val)
{
    _settings.thermistorNominal = *val;
}

void Thermistor::setTemperatureNominal(const double * val)
{
    _settings.temperatureNominal = *val;
}

void Thermistor::setBCoef(const double * val)
{
    _settings.bCoef = *val;
}

void Thermistor::setVcc(const double * val)
{
    _settings.vcc = *val;
}

void Thermistor::setAnalogReference(const double * val)
{
    _settings.analogReference = *val;
}

char * Thermistor::dumpSettings(const ThermistorSettings * ts) {
    // vcc=4.55,adcRef=4.55,serRes=120000,ntcRes=100000,tempNom=25,bc=3950,samples=5,sampleDly=20
    if(ts == nullptr){
        sprintf(dumpStr, "vcc=%.2f,adcRef=%.2f,serRes=%.0f,ntcRes=%.0f,tempNom=%.0f,bc=%.0f,samples=%i,sampleDly=%i",
                _settings.vcc, _settings.analogReference, _settings.seriesResistor, _settings.thermistorNominal,
                _settings.temperatureNominal, _settings.bCoef, _settings.samples, _settings.sampleDelay
        );
    }else{
        sprintf(dumpStr, "vcc=%.2f,adcRef=%.2f,serRes=%.0f,ntcRes=%.0f,tempNom=%.0f,bc=%.0f,samples=%i,sampleDly=%i",
                ts->vcc, ts->analogReference, ts->seriesResistor, ts->thermistorNominal,
                ts->temperatureNominal, ts->bCoef, ts->samples, ts->sampleDelay
        );
    }


    return dumpStr;
}
