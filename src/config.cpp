#include "config.h"


bool configLoad() {
    configRead();
    JsonObject parsedJson = poolJsonDoc.as<JsonObject>();

    PoolInternals::ConfigValidationResult configValidationResult = PoolInternals::Validation::validateConfig(parsedJson);
    if (!configValidationResult.valid) {
//        Homie.getLogger() << F("✖ Pool config file is not valid, reason: ") << configValidationResult.reason << endl;
        Serial.printf("✖ Pool config file is not valid, reason: %s\n", configValidationResult.reason.c_str());

        return false;
    }

    configSetPoolSettings(parsedJson["config"].as<JsonObject>(), &PoolInternals::IPoolSetting::settingsConfig);
    configSetPoolSettings(parsedJson["probeOffsets"].as<JsonObject>(), &PoolInternals::IPoolSetting::settingsProbeOffset);
    configSetPoolSettings(parsedJson["probeSettings"].as<JsonObject>(), &PoolInternals::IPoolSetting::settingsProbe);
    configSetPoolSettings(parsedJson["time"].as<JsonObject>(), &PoolInternals::IPoolSetting::settingsTime);
    configSetPoolSettings(parsedJson["location"].as<JsonObject>(), &PoolInternals::IPoolSetting::settingsLocation);

//    Homie.getLogger() << endl << F("{} Stored Pool configuration") << endl;
    configLogSettings("Config", &PoolInternals::IPoolSetting::settingsConfig);
    configLogSettings("Probe Offsets", &PoolInternals::IPoolSetting::settingsProbeOffset);
    configLogSettings("Probe Settings", &PoolInternals::IPoolSetting::settingsProbe);
    configLogSettings("Time", &PoolInternals::IPoolSetting::settingsTime);
    configLogSettings("Location", &PoolInternals::IPoolSetting::settingsLocation);

    return true;
}

void configRead() {
    if (!SPIFFS.begin()) {
        Homie.getLogger() << F("✖ Failed to initialize SPIFFS for pool config read") << endl;
        return;
    }

    if (!SPIFFS.exists(CONFIG_PATH)) {
        Homie.getLogger() << F("✖ ") << CONFIG_PATH << F(" doesn't exist") << endl;
        return;
    }

    File configFile = SPIFFS.open(CONFIG_PATH, "r");

    if (!configFile) {
        Homie.getLogger() << F("✖ Cannot open pool config file") << endl;
        return;
    }

    size_t configSize = configFile.size();

    if (configSize >= HomieInternals::MAX_JSON_CONFIG_FILE_SIZE) {
        Homie.getLogger() << F("✖ Pool config file too big") << endl;
        return;
    }

    poolJsonDoc.clear();
    if (deserializeJson(poolJsonDoc, configFile) != DeserializationError::Ok || !poolJsonDoc.is<JsonObject>()) {
        Homie.getLogger() << F("✖ Invalid JSON in the pool config file") << endl;
        configFile.close();
        return;
    }

#ifdef PRINT_POOL_CONFIG_ON_READ_WRITE
    serializeJsonPretty(poolJsonDoc, Serial);
#endif
    configFile.close();
}

void configSetPoolSettings(JsonObject settingsObject, std::vector<PoolInternals::IPoolSetting *> *settings) {
    for (auto &iSetting: *settings) {
        JsonVariant reqSetting = settingsObject[iSetting->getName()];

        if (!reqSetting.isNull()) {
            if (iSetting->isBool()) {
                PoolSetting<bool> *setting = static_cast<PoolSetting<bool> *>(iSetting);
                setting->set(reqSetting.as<bool>());
            } else if (iSetting->isLong()) {
                PoolSetting<long> *setting = static_cast<PoolSetting<long> *>(iSetting);
                setting->set(reqSetting.as<long>());
            } else if (iSetting->isShort()) {
                PoolSetting<int16_t> *setting = static_cast<PoolSetting<int16_t> *>(iSetting);
                setting->set(reqSetting.as<int16_t>());
            } else if (iSetting->isUShort()) {
                PoolSetting<uint16_t> *setting = static_cast<PoolSetting<uint16_t> *>(iSetting);
                setting->set(reqSetting.as<uint16_t>());
            } else if (iSetting->isDouble()) {
                PoolSetting<double> *setting = static_cast<PoolSetting<double> *>(iSetting);
                setting->set(reqSetting.as<double>());
            } else if (iSetting->isConstChar()) {
                PoolSetting<const char *> *setting = static_cast<PoolSetting<const char *> *>(iSetting);
                setting->set(reqSetting.as<const char *>());
            }
        }
    }
}

void configLogSettings(const char *name, std::vector<PoolInternals::IPoolSetting *> *settings) {
    if (!settings->empty()) {
        Homie.getLogger() << F("  • ") << name << F(" settings: ") << endl;
        for (auto &iSetting: *settings) {
            Homie.getLogger() << F("    ◦ ");
            if (iSetting->isBool()) {
                PoolSetting<bool> *setting = static_cast<PoolSetting<bool> *>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isLong()) {
                PoolSetting<long> *setting = static_cast<PoolSetting<long> *>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isShort()) {
                PoolSetting<int16_t> *setting = static_cast<PoolSetting<int16_t> *>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isUShort()) {
                PoolSetting<uint16_t> *setting = static_cast<PoolSetting<uint16_t> *>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isDouble()) {
                PoolSetting<double> *setting = static_cast<PoolSetting<double> *>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            } else if (iSetting->isConstChar()) {
                PoolSetting<const char *> *setting = static_cast<PoolSetting<const char *> *>(iSetting);
                Homie.getLogger() << setting->getDescription() << F(": ") << setting->get() << F(" (") << (setting->wasProvided() ? F("set") : F("default")) << F(")");
            }

            Homie.getLogger() << endl;
        }
    }
}

void configWrite() {
    poolJsonDoc.clear();

    JsonObject config = poolJsonDoc.createNestedObject("config");
    config["cloudy"] = poolConfigCloudySetting.get();
    config["overcastCnt"] = poolConfigOvercastCntSetting.get();
    config["sunMinElvAM"] = poolConfigSunMinElvAMSetting.get();
    config["sunMinElvPM"] = poolConfigSunMinElvPMSetting.get();
    config["setPoint"] = poolConfigSetPointSetting.get();
    config["setPointSwing"] = poolConfigSetPointSwingSetting.get();
    config["auxHeatTempDiff"] = poolConfigAuxHeatDiffSetting.get();
    config["airPoolDiff"] = poolConfigAirPoolDiffSetting.get();
    config["poolTempIn"] = poolConfigPoolTempInSetting.get();
    config["pumpGpm"] = poolConfigPumpGpmSetting.get();

    JsonObject probeOffsets = poolJsonDoc.createNestedObject("probeOffsets");
    probeOffsets["air"] = poolAirOffsetSetting.get();
    probeOffsets["pool"] = poolPoolOffsetSetting.get();
    probeOffsets["tin"] = poolTinOffsetSetting.get();
    probeOffsets["tout"] = poolToutOffsetSetting.get();

    JsonObject probeSettings = poolJsonDoc.createNestedObject("probeSettings");
    probeSettings["air"] = poolAirNtcSetting.get();
    probeSettings["pool"] = poolPoolNtcSetting.get();
    probeSettings["tin"] = poolTinDtSetting.get();
    probeSettings["tout"] = poolToutDtSetting.get();

    JsonObject time = poolJsonDoc.createNestedObject("time");
    time["dstOffset"] = poolDstOffsetSetting.get();
    time["stOffset"] = poolStOffsetSetting.get();
    time["dstBeginDay"] = poolDstBeginDaySetting.get();
    time["dstBeginMonth"] = poolDstBeginMonthSetting.get();
    time["dstEndDay"] = poolDstEndDaySetting.get();
    time["dstEndMonth"] = poolDstEndMonthSetting.get();

    JsonObject location = poolJsonDoc.createNestedObject("location");
    location["latitude"] = poolLatitudeSetting.get();
    location["longitude"] = poolLongitudeSetting.get();

    SPIFFS.remove(CONFIG_PATH);

    File configFile = SPIFFS.open(CONFIG_PATH, "w");
    if (!configFile) {
        Homie.getLogger() << F("✖ Cannot open pool config file") << endl;
        return;
    }

#ifdef PRINT_POOL_CONFIG_ON_READ_WRITE
    Homie.getLogger() << endl << F("{} New Stored Pool configuration") << endl;
    serializeJsonPretty(poolJsonDoc, Serial);
    Homie.getLogger() << endl;
#endif

    serializeJsonPretty(poolJsonDoc, configFile);
    configFile.close();

    configUpdateStructs();

    Homie.getLogger() << F("✔ Pool config file written") << endl << endl;
}

void configUpdateStructs() {
    ntcAirSetting.offset = float(poolAirOffsetSetting.get());
    ntcPoolSetting.offset = float(poolPoolOffsetSetting.get());
    tinSettings.offset = float(poolTinOffsetSetting.get());
    toutSettings.offset = float(poolToutOffsetSetting.get());
}