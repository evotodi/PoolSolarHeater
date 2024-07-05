#pragma once

struct DTSetting {
    char addrStr[18];
    float offset; // This is stored in spiffs pool/config.json
    char name[16];
    DeviceAddress daddr;
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
