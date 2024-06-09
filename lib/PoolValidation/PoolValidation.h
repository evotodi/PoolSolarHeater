#pragma once

#include "Arduino.h"
#include <ArduinoJson.h>

namespace PoolInternals {
    struct ConfigValidationResult {
        bool valid;
        String reason;
    };

    class Validation {
    public:
        static ConfigValidationResult validateConfig(JsonObject object);

    private:
        static ConfigValidationResult _validateConfig(JsonObject object);
//        static ConfigValidationResult _validateProbeOffsets(const JsonObject object);
        static ConfigValidationResult _validateProbeSettings(JsonObject object);
        static ConfigValidationResult _validateTime(JsonObject object);
        static ConfigValidationResult _validateLocation(JsonObject object);
    };
}  // namespace HomieInternals
