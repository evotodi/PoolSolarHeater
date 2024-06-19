#include "data.h"

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

#ifdef DEBUG
    Serial.print("Daylight Time: ");
    Serial.println(utc);
    Serial.print(F("Sun Rise: "));
    Serial.print(pDaylight->sunrise);
    Serial.print(F("  Sun Set: "));
    Serial.print(pDaylight->sunset);
    Serial.print(F("  Transit: "));
    Serial.println(pDaylight->transit);
    Serial.print(F("Mid: "));
    Serial.println(pDaylight->midday);
#endif
}

void addPoolTemp()
{
#ifdef FAKE_TEMP_POOL
    pool.add(FAKE_TEMP_POOL + ntcPoolSetting.offset);
#else
    if(poolConfigPoolTempInSetting.get() == 0) {
        pool.add(tin.getLast());
    } else if(poolConfigPoolTempInSetting.get() == 1) {
        pool.add(FtoI(float(thermistorPool.readTempF(ntcPoolSetting.pin)) + ntcPoolSetting.offset));
    }else{
        Homie.getLogger() << F("✖ Invalid poolTemp type: ") << poolConfigPoolTempInSetting.get() << endl;
    }
#endif
}

void addAirTemp()
{
#ifdef FAKE_TEMP_AIR
    air.add(FAKE_TEMP_AIR + ntcAirSetting.offset);
#else
    air.add(FtoI(float(thermistorAir.readTempF(ntcAirSetting.pin)) + ntcAirSetting.offset));
#endif
}

void addTInTemp()
{
#ifdef FAKE_TEMP_TIN
    tin.add(FAKE_TEMP_TIN + tinSettings.offset);
#else
    tin.add(FtoI(sensors.getTempF(tempSensorIn) + tinSettings.offset));
#endif
}

void addTOutTemp()
{
#ifdef FAKE_TEMP_TOUT
    tout.add(FAKE_TEMP_TOUT + toutSettings.offset);
#else
    tout.add(FtoI(sensors.getTempF(tempSensorOut) + toutSettings.offset));
#endif
}

void addLight()
{
#ifdef FAKE_LIGHT
    light.add(FAKE_LIGHT);
#else
    light.add(mcp.read(ADC_LIGHT));
#endif
}

bool checkTempSensors() {
    poolOk = true;
    tinOk = true;
    toutOk = true;
    airOk = true;
#ifdef NO_CHECK_SENSORS_OK
    return true;
#endif
    if (pool.get() <= T_SENSOR_BAD) poolOk = false;
    if (tin.get() <= T_SENSOR_BAD) tinOk = false;
    if (tout.get() <= T_SENSOR_BAD) toutOk = false;
    if (air.get() <= T_SENSOR_BAD) airOk = false;

    if (poolOk && tinOk && toutOk && airOk) return true;

    setRunStatus(RunStatus::ERROR, true);
    return false;
}

void setRunStatus(RunStatus rs, bool force) {
    if (runStatus == RunStatus::ERROR) {
        if (force) {
            runStatus = rs;
        }
        return;
    }

    runStatus = rs;
}

const char * getRunStatusStr() {
    switch (runStatus) {
        case OFF:
            return "Off";
            break;
        case SOLAR:
            return "Solar";
            break;
        case PROPANE:
            return "Heat";
            break;
        case MANUAL_SOLAR:
            return "Manual Solar";
            break;
        case MANUAL_PROPANE:
            return "Manual Heat";
            break;
        case ERROR:
            return "ERROR";
            break;
    }

    return "ERROR";
}

void setupOwSensors()
{
    Homie.getLogger() << "Locating one wire devices..." << endl;
    displayCenterMessage("1Wire...");

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
    return mcp.read(channel);
}
