#include "mqtt.h"

bool mqttHeatOnHandler(const HomieRange &range, const String &value) {
    if (value != "true" && value != "false") return false;
    bool on = (value == "true");

    if (on) {
        overrideEnv = true;
        manualHeatingEnable = true;
        manualHeating = true;
        heatOn();
    } else {
        overrideEnv = false;
        manualHeatingEnable = false;
        manualHeating = false;
        heatOff();
    }
    statusNode.setProperty("heating").send(value);
    Homie.getLogger() << "MQTT Pump is forced " << (on ? "on" : "off") << endl;

    return true;
}

bool configNodeInputHandler(const HomieRange &range, const String &property, const String &value) {
    Homie.getLogger() << "Config Node Input >> Property: " << property << " Value: " << value << endl;

    char *end = nullptr;
    uint16_t tempUint16;
    float tempFloat;
    bool doWrite = false;

    if (strcmp(property.c_str(), "cloudy") == 0) {
        tempUint16 = strtoul(value.c_str(), &end, 10);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigCloudySetting.set(tempUint16);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "overcastCnt") == 0) {
        tempUint16 = strtoul(value.c_str(), &end, 10);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigOvercastCntSetting.set(tempUint16);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "sunMinElvAM") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigSunMinElvAMSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "sunMinElvPM") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigSunMinElvPMSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "setPoint") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigSetPointSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "setPointSwing") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigSetPointSwingSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "airPoolDiff") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigAirPoolDiffSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "poolTempIn") == 0) {
        tempUint16 = strtoul(value.c_str(), &end, 10);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolConfigPoolTempInSetting.set(tempUint16);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "airOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolAirOffsetSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "poolOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolPoolOffsetSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "tinOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolTinOffsetSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "toutOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        poolToutOffsetSetting.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    } else if (strcmp(property.c_str(), "pumpGpm") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
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

void toggleOverrideEnv() {
    overrideEnv = !overrideEnv;
    if (overrideEnv) {
        Homie.getLogger() << "Environment Override Enabled !" << endl;
    } else {
        Homie.getLogger() << "Environment Override Disabled !" << endl;
        manualHeating = false;
        manualHeatingEnable = false;
        envCheckNoSolar = false;
        envCheckNoAir = false;
        envCheckNoCloud = false;
        envCheckNoTDiff = false;
    }
}

void toggleManualHeatingEnable() {
    if (!overrideEnv) {
        Homie.getLogger() << F("✖ Must override the environment for manual heating") << endl;
        manualHeatingEnable = false;
        return;
    }

    manualHeatingEnable = !manualHeatingEnable;

    if (manualHeatingEnable) {
        Homie.getLogger() << "Environment Manual Heating Enabled !" << endl;
        manualHeating = isHeating;
    } else {
        Homie.getLogger() << "Environment Manual Heating Disabled !" << endl;
        manualHeating = false;
        heatOff();
    }

}

void toggleManualHeating() {
    if (!overrideEnv) {
        Homie.getLogger() << F("✖ Must override the environment for manual heating") << endl;
        manualHeating = false;
        return;
    }

    if (!manualHeatingEnable) {
        Homie.getLogger() << F("✖ Must enable manual heating first") << endl;
        manualHeating = false;
        return;
    }

    manualHeating = !manualHeating;

    if (manualHeating) {
        heatOn();
    } else {
        heatOff();
    }
}

void toggleEnvNoCheckSolar() {
    envCheckNoSolar = !envCheckNoSolar;

    if (envCheckNoSolar) {
        Homie.getLogger() << "Environment don't check solar" << endl;
    } else {
        Homie.getLogger() << "Environment check solar" << endl;
    }
}

void toggleEnvNoCheckAir() {
    envCheckNoAir = !envCheckNoAir;

    if (envCheckNoAir) {
        Homie.getLogger() << "Environment don't check air" << endl;
    } else {
        Homie.getLogger() << "Environment check air" << endl;
    }
}

void toggleEnvNoCheckCloud() {
    envCheckNoCloud = !envCheckNoCloud;

    if (envCheckNoCloud) {
        Homie.getLogger() << "Environment don't check cloudy" << endl;
    } else {
        Homie.getLogger() << "Environment check cloudy" << endl;
    }
}

void toggleEnvNoCheckTDiff() {
    envCheckNoTDiff = !envCheckNoTDiff;

    if (envCheckNoTDiff) {
        Homie.getLogger() << "Environment don't check tin tout diff" << endl;
    } else {
        Homie.getLogger() << "Environment check tin tout diff" << endl;
    }
}

void toggleEnvNoCheckAuxHeatDiff() {
    envCheckNoAuxHeatDiff = !envCheckNoAuxHeatDiff;

    if (envCheckNoAuxHeatDiff) {
        Homie.getLogger() << "Environment don't check aux heat diff" << endl;
    } else {
        Homie.getLogger() << "Environment check aux heat diff" << endl;
    }
}