#include "PoolValidation.h"

using namespace PoolInternals;

ConfigValidationResult Validation::validateConfig(const JsonObject object) {
    ConfigValidationResult result;
    result = _validateConfig(object);
    if (!result.valid) return result;
//    result = _validateProbeOffsets(object);
//    if (!result.valid) return result;
    result = _validateProbeSettings(object);
    if (!result.valid) return result;
    result = _validateTime(object);
    if (!result.valid) return result;
    result = _validateLocation(object);
    if (!result.valid) return result;

    result.valid = true;
    return result;
}

ConfigValidationResult Validation::_validateConfig(const JsonObject object) {
    ConfigValidationResult result;
    result.valid = false;

    JsonVariant obj = object["config"];

    if (!obj.is<JsonObject>()) {
        result.reason = F("config is not an object");
        return result;
    }

    {
        JsonVariant cloudy = obj["cloudy"];

        if (cloudy.as<uint16_t>() == 0) {
            result.reason = F("config.cloudy must not be 0");
            return result;
        }
    }

    {
        JsonVariant overcastCnt = obj["overcastCnt"];

        if (overcastCnt.as<uint16_t>() == 0) {
            result.reason = F("config.overcastCnt must not be 0");
            return result;
        }
    }

    {
        JsonVariant sunMinElvAM = obj["sunMinElvAM"];

        if (sunMinElvAM.as<double>() <= 0) {
            result.reason = F("config.sunMinElvAM must be greater than 0");
            return result;
        }
    }

    {
        JsonVariant sunMinElvPM = obj["sunMinElvPM"];

        if (sunMinElvPM.as<double>() <= 0) {
            result.reason = F("config.sunMinElvPM must be greater than 0");
            return result;
        }
    }

    {
        JsonVariant setPoint = obj["setPoint"];

        if (setPoint.as<double>() <= 0) {
            result.reason = F("config.setPoint must be greater than 0");
            return result;
        }
    }

    {
        JsonVariant setPointSwing = obj["setPointSwing"];

        if (setPointSwing.as<double>() <= 0) {
            result.reason = F("config.setPointSwing must be greater than 0");
            return result;
        }
    }

    {
        JsonVariant airPoolDiff = obj["airPoolDiff"];

        if (airPoolDiff.as<double>() <= 0) {
            result.reason = F("config.airPoolDiff must be greater than 0");
            return result;
        }
    }

    {
        JsonVariant poolTempIn = obj["poolTempIn"];

        if (poolTempIn.as<uint16_t>() > 1) {
            result.reason = F("config.poolTempIn must be a 1 or 0");
            return result;
        }
    }

    {
        JsonVariant pumpGpm = obj["pumpGpm"];

        if (pumpGpm.as<double>() <= 0) {
            result.reason = F("config.pumpGpm must be greater than 0");
            return result;
        }
    }

    result.valid = true;
    return result;
}

ConfigValidationResult Validation::_validateProbeSettings(const JsonObject object) {
    ConfigValidationResult result;
    result.valid = false;

    JsonVariant obj = object["probeSettings"];

    if (!obj.is<JsonObject>()) {
        result.reason = F("probeSettings is not an object");
        return result;
    }

    {
        JsonVariant air = obj["air"];

        if (!air.as<const char*>()) {
            result.reason = F("probeSettings.air is not a string");
            return result;
        }
        if (strcmp_P(air.as<const char*>(), PSTR("")) == 0) {
            result.reason = F("probeSettings.air is empty");
            return result;
        }
    }

    {
        JsonVariant pool = obj["pool"];

        if (!pool.as<const char*>()) {
            result.reason = F("probeSettings.pool is not a string");
            return result;
        }
        if (strcmp_P(pool.as<const char*>(), PSTR("")) == 0) {
            result.reason = F("probeSettings.pool is empty");
            return result;
        }
    }

    {
        JsonVariant tin = obj["tin"];

        if (!tin.as<const char*>()) {
            result.reason = F("probeSettings.tin is not a string");
            return result;
        }
        if (strcmp_P(tin.as<const char*>(), PSTR("")) == 0) {
            result.reason = F("probeSettings.tin is empty");
            return result;
        }
    }

    {
        JsonVariant tout = obj["tout"];

        if (!tout.as<const char*>()) {
            result.reason = F("probeSettings.tout is not a string");
            return result;
        }
        if (strcmp_P(tout.as<const char*>(), PSTR("")) == 0) {
            result.reason = F("probeSettings.tout is empty");
            return result;
        }
    }

    result.valid = true;
    return result;
}

ConfigValidationResult Validation::_validateTime(const JsonObject object)
{
    ConfigValidationResult result;
    result.valid = false;

    JsonVariant obj = object["time"];

    if (!obj.is<JsonObject>()) {
        result.reason = F("time is not an object");
        return result;
    }

    {
        JsonVariant dstOffset = obj["dstOffset"];

        if (dstOffset <= -13 || dstOffset >= 13) {
            result.reason = F("time.dstOffset must be between -12 and 12");
            return result;
        }
    }

    {
        JsonVariant stOffset = obj["stOffset"];

        if (stOffset <= -13 || stOffset >= 13) {
            result.reason = F("time.stOffset must be between -12 and 12");
            return result;
        }
    }

    {
        JsonVariant dstBeginDay = obj["dstBeginDay"];

        if (dstBeginDay <= 0 || dstBeginDay >= 32) {
            result.reason = F("time.dstBeginDay must be between 1 and 31");
            return result;
        }
    }

    {
        JsonVariant dstBeginMonth = obj["dstBeginMonth"];

        if (dstBeginMonth <= 0 || dstBeginMonth >= 13) {
            result.reason = F("time.dstBeginMonth must be between 1 and 12");
            return result;
        }
    }

    {
        JsonVariant dstEndDay = obj["dstEndDay"];

        if (dstEndDay <= 0 || dstEndDay >= 32) {
            result.reason = F("time.dstEndDay must be between 1 and 31");
            return result;
        }
    }

    {
        JsonVariant dstEndMonth = obj["dstEndMonth"];

        if (dstEndMonth <= 0 || dstEndMonth >= 13) {
            result.reason = F("time.dstEndMonth must be between 1 and 12");
            return result;
        }
    }

    result.valid = true;
    return result;
}

ConfigValidationResult Validation::_validateLocation(const JsonObject object)
{
    ConfigValidationResult result;
    result.valid = false;

    JsonVariant obj = object["location"];

    if (!obj.is<JsonObject>()) {
        result.reason = F("location is not an object");
        return result;
    }

    {
        JsonVariant latitude = obj["latitude"];

        if (latitude <= -91 || latitude >= 91) {
            result.reason = F("location.latitude must be between -90 and 90");
            return result;
        }
    }

    {
        JsonVariant longitude = obj["longitude"];

        if (longitude <= -181 || longitude >= 181) {
            result.reason = F("location.longitude must be between -180 and 180");
            return result;
        }
    }

    result.valid = true;
    return result;
}