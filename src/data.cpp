#include "data.h"

void getSolar(Solar * pSolar)
{
    time_t utc = now();
    // Calculate the solar position, in degrees
    calcHorizontalCoordinates(utc, settingLatitude.get(), settingLongitude.get(), pSolar->azimuth, pSolar->elevation);

#ifdef DEBUG
    Serial.print("Solar Time: ");
    Serial.println(utc);
    Serial.print(F("Solar Az: "));
    Serial.print(pSolar->azimuth);
    Serial.print(F("°  El: "));
    Serial.print(pSolar->elevation);
    Serial.println(F("°"));
#endif
}

void getDaylight(Daylight * pDaylight)
{
    double md;
    time_t utc = now();
    calcCivilDawnDusk(utc, settingLatitude.get(), settingLongitude.get(), pDaylight->transit, pDaylight->sunrise, pDaylight->sunset);
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
    pool.add(FAKE_TEMP_POOL + dtSettingPool.offset);
#else
    pool.add(FtoI(sensors.getTempF(dtSettingPool.daddr) + dtSettingPool.offset));
#endif
}

void addAirTemp()
{
#ifdef FAKE_TEMP_AIR
    air.add(FAKE_TEMP_AIR + dtSettingAir.offset);
#else
    air.add(FtoI(sensors.getTempF(dtSettingAir.daddr) + dtSettingAir.offset));
#endif
}

void addTInTemp()
{
#ifdef FAKE_TEMP_TIN
    tin.add(FAKE_TEMP_TIN + dtSettingTin.offset);
#else
    tin.add(FtoI(sensors.getTempF(dtSettingTin.daddr) + dtSettingTin.offset));
#endif
}

void addTOutSolarTemp()
{
#ifdef FAKE_TEMP_TOUT
    toutSolar.add(FAKE_TEMP_TOUT + dtSettingToutSolar.offset);
#else
    toutSolar.add(FtoI(sensors.getTempF(dtSettingToutSolar.daddr) + dtSettingToutSolar.offset));
#endif
}

void addTOutHeatTemp()
{
#ifdef FAKE_TEMP_TOUT
    toutSolar.add(FAKE_TEMP_TOUT + dtSettingToutHeat.offset);
#else
    toutSolar.add(FtoI(sensors.getTempF(dtSettingToutHeat.daddr) + dtSettingToutHeat.offset));
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
    toutSolarOk = true;
    toutHeatOk = true;
    airOk = true;
#ifdef NO_CHECK_SENSORS_OK
    return true;
#endif
    if (pool.get() <= T_SENSOR_BAD) poolOk = false;
    if (tin.get() <= T_SENSOR_BAD) tinOk = false;
    if (toutSolar.get() <= T_SENSOR_BAD) toutSolarOk = false;
    if (toutHeat.get() <= T_SENSOR_BAD) toutHeatOk = false;
    if (air.get() <= T_SENSOR_BAD) airOk = false;

    if (poolOk && tinOk && toutSolarOk && toutHeatOk && airOk) return true;

    setRunStatus(RunStatus::ERROR, true);
    return false;
}

bool wantsSolar() {
    if (ItoF(pool.get()) <= (settingPoolSP.get() - settingSPHyst.get())) {
        return true;
    }

    return false;
}

bool wantsHeatAux() {
    if (ItoF(pool.get()) <= (settingHeatAuxSP.get() - settingSPHyst.get())) {
        return true;
    }

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
        case STANDBY:
            return "Standby";
            break;
        case ENV_STANDBY:
            return "Env Standby";
            break;
        case SOLAR:
            return "Solar";
            break;
        case HEAT_AUX:
            return "Heat Aux";
            break;
        case MANUAL_SOLAR:
            return "MAN Solar";
            break;
        case MANUAL_HEAT_AUX:
            return "MAN Heat Aux";
            break;
        case ERROR:
            return "ERROR";
            break;
    }

    return "ERROR";
}

void setupOwSensors() {
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

    DTSetting devices[] = {
            {dtSettingAir},
            {dtSettingPool},
            {dtSettingTin},
            {dtSettingToutSolar},
            {dtSettingToutHeat},
    };

    for (size_t i = 0; i < sizeof(devices); i++) {
        if (sensors.isConnected(devices[i].daddr)) {
            Homie.getLogger() << "Temp sensor " << devices[i].name << " address: ";
            printAddress(devices[i].daddr);
            Homie.getLogger() << endl;
            Homie.getLogger() << "Temp sensor " << devices[i].name << " resolution: ";
            Homie.getLogger().print(sensors.getResolution(devices[i].daddr), DEC);
            Homie.getLogger() << endl;
        } else {
            Homie.getLogger() << "Temp sensor " << devices[i].name << " is not connected !" << endl;
        }
    }
}

int16_t mcpReadCallback(uint8_t channel)
{
    return mcp.read(channel);
}
