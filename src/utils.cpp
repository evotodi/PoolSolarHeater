#include "utils.h"

float ItoF(const int val) {
    return (float) val / 10;
}

String ItoS(int val) {
    return String(ItoF(val));
}

int FtoI(const float val) {
    return (int) roundf(val * 10);
}

void strToAddress(const String &addr, DeviceAddress deviceAddress) {
    char byt[3] = {0, 0, 0};
    unsigned int number;

    for (size_t i = 0; i < addr.length(); i += 2) {
        byt[0] = addr[i];
        byt[1] = addr[i + 1];

        number = (int) strtol(byt, nullptr, 16);
        deviceAddress[i / 2] = number;
    }

}

void printAddress(DeviceAddress deviceAddress) {
    for (uint8_t i = 0; i < 8; i++) {
        if (deviceAddress[i] < 16) Homie.getLogger().print("0");
        Homie.getLogger().print(deviceAddress[i], HEX);
    }
}

float calcWatts(float tempIn, float tempOut, float gpm) {
    const float dt = fabsf(tempIn - tempOut);
    const auto c = float(0.00682);
    float rtn;
    rtn = (gpm * dt) / c;
    return rtn;
}

void parseNTCSettings(ThermistorSettings *ts, const char *settings, const char *name) {
    sscanf(settings, "vcc=%lf;adcRef=%lf;serRes=%lf", // NOLINT(cert-err34-c)
           &ts->vcc, &ts->analogReference, &ts->seriesResistor
    );

    Homie.getLogger() << "Parsed " << name << " settings = >>>" << settings << "<<<" << endl;
    Homie.getLogger() << "VCC = " << ts->vcc << endl;
    Homie.getLogger() << "ADC Ref = " << ts->analogReference << endl;
    Homie.getLogger() << "Ser Res = " << ts->seriesResistor << endl;
    Homie.getLogger() << endl;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat"

void parseDTSettings(DTSetting *pDTSetting, const char *settings, double offset, const char *name) {
    sscanf(settings, "addr=%[0-9a-fA-F]", &pDTSetting->addr); // NOLINT(cert-err34-c)
    pDTSetting->offset = float(offset);

    Homie.getLogger() << "Parsed " << name << " settings = >>>" << settings << "<<<" << endl;
    Homie.getLogger() << "Address = " << pDTSetting->addr << endl;
    Homie.getLogger() << "Offset = " << pDTSetting->offset << endl;
    Homie.getLogger() << endl;
}

#pragma clang diagnostic pop
