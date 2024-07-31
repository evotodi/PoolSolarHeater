#pragma once

#include "enums.h"
#include "PoolSetting.hpp"
#include <Adafruit_ILI9341.h>

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

struct CfgPageItemData {
    ConfigPageItem item;
    VariableType varType;
    const char *name;
    PoolInternals::IPoolSetting *setting;

public:
    std::basic_string<char> getSettingValueString(uint8_t pad = 1) {
        char data[8] = {0};

        if (setting->isBool()) {
            PoolSetting<bool>* dataSetting = static_cast<PoolSetting<bool>*>(setting);
            sprintf(data, "%s", dataSetting->get() ? "1" : "0");
        } else if (setting->isLong()) {
            PoolSetting<long>* dataSetting = static_cast<PoolSetting<long >*>(setting);
            sprintf(data, "%0*ld", pad, dataSetting->get());
        } else if (setting->isShort()) {
            PoolSetting<int16_t>* dataSetting = static_cast<PoolSetting<int16_t>*>(setting);
            sprintf(data, "%0*hd", pad, dataSetting->get());
        } else if (setting->isUShort()) {
            PoolSetting<uint16_t>* dataSetting = static_cast<PoolSetting<uint16_t>*>(setting);
            sprintf(data, "%0*hu", pad, dataSetting->get());
        } else if (setting->isDouble()) {
            PoolSetting<double>* dataSetting = static_cast<PoolSetting<double>*>(setting);
            sprintf(data, "%0*.1f", pad, dataSetting->get());
        } else if (setting->isConstChar()) {
            PoolSetting<const char *>* dataSetting = static_cast<PoolSetting<const char *>*>(setting);
            sprintf(data, "%*s", pad, dataSetting->get());
        }

        return data;
    }
};

struct CursorPos {
    int16_t x = 0;
    int16_t y = 0;
};

struct Rectangle {
    int16_t x = 0;
    int16_t y = 0;
    int16_t w = 0;
    int16_t h = 0;
    uint16_t color = 0;
};

struct TextSizeData {
    int16_t width = 6;
    int16_t height = 8;
    uint8_t num = 1;
};

struct TextBounds {
    int16_t x = 0;
    int16_t y = 0;
    int16_t x1 = 0;
    int16_t y1 = 0;
    uint16_t w = 0;
    uint16_t h = 0;
    TextSizeData size;
};

struct TextSize {
    TextSizeData small = TextSizeData();
    TextSizeData med = TextSizeData();
    TextSizeData large = TextSizeData();
    TextSizeData xl = TextSizeData();

public:
    void init(Adafruit_ILI9341 *tft) {
        int16_t x1 = 0;
        int16_t y1 = 0;
        uint16_t w = 0;
        uint16_t h = 0;

        tft->setTextSize(1);
        tft->getTextBounds("0", 0, 0, &x1, &y1, &w, &h); // Just to get character size
        small.num = 1;
        small.height = h;
        small.width = w;

        tft->setTextSize(2);
        tft->getTextBounds("0", 0, 0, &x1, &y1, &w, &h); // Just to get character size
        med.num = 2;
        med.height = h;
        med.width = w;

        tft->setTextSize(3);
        tft->getTextBounds("0", 0, 0, &x1, &y1, &w, &h); // Just to get character size
        large.num = 3;
        large.height = h;
        large.width = w;

        tft->setTextSize(4);
        tft->getTextBounds("0", 0, 0, &x1, &y1, &w, &h); // Just to get character size
        xl.num = 4;
        xl.height = h;
        xl.width = w;
    }
};