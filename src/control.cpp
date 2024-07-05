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

void setAllOff() {
    setPumpOff();
    setHeatAuxOff();
    setAuxOff();
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

bool getEnable() {
#ifdef FORCE_ENABLE
    if (FORCE_ENABLE == 1){
        return true;
    }

    return false;
#else
    return digitalRead(PIN_ENABLE);
#endif
}

bool getForceOn() {
#ifdef FORCE_ON
    if (FORCE_ON == 1){
        return true;
    }

    return false;
#else
    if (digitalRead(PIN_FORCE_ON)) {
        return true;
    }

    return forceOn;
#endif
}