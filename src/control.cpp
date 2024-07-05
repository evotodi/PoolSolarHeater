#include "control.h"

void doProcess() {
    if (firstRun) {
        return;
    }

//    if (manualHeatingEnable) {
//        return;
//    }
//
//    if (ItoF(pool.get()) >= poolConfigSetPointSetting.get()) {
//        Homie.getLogger() << "Set point reached" << endl;
//        heatOff();
//        atSetpoint = true;
//        return;
//    }
//
//    if (ItoF(pool.get()) < (poolConfigSetPointSetting.get() - poolConfigSetPointSwingSetting.get())) {
//        atSetpoint = false;
//        heatOn();
//    }
}

void setPumpOn() {
    digitalWrite(PUMP_RLY_PIN, HIGH);
    rlyPump = true;
}

void setPumpOff() {
    digitalWrite(PUMP_RLY_PIN, LOW);
    rlyPump = false;
}

void setHeatAuxOn() {
    digitalWrite(AUX_HEAT_RLY_PIN, HIGH);
    rlyHeatAux = true;
}

void setHeatAuxOff() {
    digitalWrite(AUX_HEAT_RLY_PIN, LOW);
    rlyHeatAux = false;
}

void setAuxOn() {
    digitalWrite(AUX_RLY_PIN, HIGH);
    rlyAux = true;
}

void setAuxOff() {
    digitalWrite(AUX_RLY_PIN, LOW);
    rlyAux = false;
}
