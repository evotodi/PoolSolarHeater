#include "mqtt.h"

bool configNodeInputHandler(const HomieRange &range, const String &property, const String &value) {
    Homie.getLogger() << "Config Node Input >> Property: " << property << " Value: " << value << endl;

    char *end = nullptr;
    uint16_t tempUint16;
    float tempFloat;
    bool tempBool;
    bool doWrite = false;

    if (strcmp(property.c_str(), "cloudy") == 0) {
        tempUint16 = strtoul(value.c_str(), &end, 10);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingCloudy.set(tempUint16);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "overcastCnt") == 0) {
        tempUint16 = strtoul(value.c_str(), &end, 10);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingOvercastCnt.set(tempUint16);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "sunMinElvAM") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingSunMinElvAM.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "sunMinElvPM") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingSunMinElvPM.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "poolSP") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        if (tempFloat < settingHeatAuxSP.get()) {
            Homie.getLogger() << "Pool SP is < Heat SP ! Pool SP changed to Heat SP" << endl;
            tempFloat = settingHeatAuxSP.get();
        }

        settingPoolSP.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "heatAuxSP") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }
        if (tempFloat > settingPoolSP.get()) {
            Homie.getLogger() << "Heat SP is > Pool SP ! Heat SP changed to Pool SP" << endl;
            tempFloat = settingPoolSP.get();
        }
        settingHeatAuxSP.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "spHyst") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingSPHyst.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "airOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingAirOffset.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "poolOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingPoolOffset.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "tinOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingTinOffset.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "toutSolarOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingToutSolarOffset.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "toutHeatOffset") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingToutHeatOffset.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "pumpGpm") == 0) {
        tempFloat = strtof(value.c_str(), &end);

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingPumpGpm.set(tempFloat);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }
    else if (strcmp(property.c_str(), "heatAuxEnable") == 0) {
        tempBool = strToBool(value.c_str());

        if (end == value.c_str()) {
            Homie.getLogger() << F("✖ Conversion error for ") << property << " with value of " << value << endl;
            return false;
        }

        settingHeatAuxEnable.set(tempBool);
        doWrite = true;
        Homie.getLogger() << F("✔ Property ") << property << " has been updated!" << endl;
    }

    if (doWrite) {
        configWrite();
        prevMillisPubCfg = 0;
    }

    return true;
}

//bool mqttHeatOnHandler(const HomieRange &range, const String &value) {
//    if (value != "true" && value != "false") return false;
//    bool on = (value == "true");
//
//    if (on) {
//        overrideEnv = true;
//        manualHeatingEnable = true;
//        manualHeating = true;
//        heatOn();
//    } else {
//        overrideEnv = false;
//        manualHeatingEnable = false;
//        manualHeating = false;
//        heatOff();
//    }
//    statusNode.setProperty("heating").send(value);
//    Homie.getLogger() << "MQTT Pump is forced " << (on ? "on" : "off") << endl;
//
//    return true;
//}

//void toggleOverrideEnv() {
//    overrideEnv = !overrideEnv;
//    if (overrideEnv) {
//        Homie.getLogger() << "Environment Override Enabled !" << endl;
//    } else {
//        Homie.getLogger() << "Environment Override Disabled !" << endl;
//        manualHeating = false;
//        manualHeatingEnable = false;
//        envCheckNoSolar = false;
//        envCheckNoAir = false;
//        envCheckNoCloud = false;
//        envCheckNoTDiff = false;
//    }
//}
//
//void toggleManualHeatingEnable() {
//    if (!overrideEnv) {
//        Homie.getLogger() << F("✖ Must override the environment for manual heating") << endl;
//        manualHeatingEnable = false;
//        return;
//    }
//
//    manualHeatingEnable = !manualHeatingEnable;
//
//    if (manualHeatingEnable) {
//        Homie.getLogger() << "Environment Manual Heating Enabled !" << endl;
//        manualHeating = isHeating;
//    } else {
//        Homie.getLogger() << "Environment Manual Heating Disabled !" << endl;
//        manualHeating = false;
//        heatOff();
//    }
//
//}
//
//void toggleManualHeating() {
//    if (!overrideEnv) {
//        Homie.getLogger() << F("✖ Must override the environment for manual heating") << endl;
//        manualHeating = false;
//        return;
//    }
//
//    if (!manualHeatingEnable) {
//        Homie.getLogger() << F("✖ Must enable manual heating first") << endl;
//        manualHeating = false;
//        return;
//    }
//
//    manualHeating = !manualHeating;
//
//    if (manualHeating) {
//        heatOn();
//    } else {
//        heatOff();
//    }
//}
//
//void toggleEnvNoCheckSolar() {
//    envCheckNoSolar = !envCheckNoSolar;
//
//    if (envCheckNoSolar) {
//        Homie.getLogger() << "Environment don't check solar" << endl;
//    } else {
//        Homie.getLogger() << "Environment check solar" << endl;
//    }
//}
//
//void toggleEnvNoCheckAir() {
//    envCheckNoAir = !envCheckNoAir;
//
//    if (envCheckNoAir) {
//        Homie.getLogger() << "Environment don't check air" << endl;
//    } else {
//        Homie.getLogger() << "Environment check air" << endl;
//    }
//}
//
//void toggleEnvNoCheckCloud() {
//    envCheckNoCloud = !envCheckNoCloud;
//
//    if (envCheckNoCloud) {
//        Homie.getLogger() << "Environment don't check cloudy" << endl;
//    } else {
//        Homie.getLogger() << "Environment check cloudy" << endl;
//    }
//}
//
//void toggleEnvNoCheckTDiff() {
//    envCheckNoTDiff = !envCheckNoTDiff;
//
//    if (envCheckNoTDiff) {
//        Homie.getLogger() << "Environment don't check tin tout diff" << endl;
//    } else {
//        Homie.getLogger() << "Environment check tin tout diff" << endl;
//    }
//}
//
//void toggleEnvNoCheckAuxHeatDiff() {
//    envCheckNoAuxHeatDiff = !envCheckNoAuxHeatDiff;
//
//    if (envCheckNoAuxHeatDiff) {
//        Homie.getLogger() << "Environment don't check aux heat diff" << endl;
//    } else {
//        Homie.getLogger() << "Environment check aux heat diff" << endl;
//    }
//}