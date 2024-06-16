#pragma once

struct DTSetting {
    char addr[18];
    float offset; // This is stored in spiffs pool/config.json
};

struct NTCSetting {
    uint8_t pin;
    float offset; // This is stored in spiffs pool/config.json
};

struct Solar {
    double azimuth;
    double elevation;
};

struct Daylight {
    double sunrise;
    double sunset;
    double transit;
    time_t midday;
};
