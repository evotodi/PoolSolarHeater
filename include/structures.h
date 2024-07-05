#pragma once

struct DTSetting {
    char addrStr[18];
    float offset; // This is stored in spiffs pool/config.json
    DeviceAddress daddr;
    char name[16];
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
