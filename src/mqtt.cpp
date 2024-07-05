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

void toggleForceOn(int8_t setTo) {
    if (setTo == 0) {
        forceOn = false;
    }
    else if (setTo > 0) {
        forceOn = true;
    }
    else {
        forceOn = !forceOn;
    }

    if (forceOn) {
        Homie.getLogger() << "Force ON enabled" << endl;
    } else {
        Homie.getLogger() << "Force ON disabled" << endl;
    }
}

bool mqttForceOnHandler(const HomieRange &range, const String &value) {
    if (value != "true" && value != "false") return false;
    bool on = (value == "true");

    if (on) {
        toggleForceOn(1);
    } else {
        toggleForceOn(0);
    }
    statusNode.setProperty("forceOn").send(value);
    Homie.getLogger() << "MQTT Pump/Heating is forced " << (on ? "on" : "off") << endl;

    return true;
}
