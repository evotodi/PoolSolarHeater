#include "control.h"

void doProcess() {
    if (firstRun) {
        return;
    }

    if (manualHeatingEnable) {
        return;
    }

    if (ItoF(pool.get()) >= poolConfigSetPointSetting.get()) {
        Homie.getLogger() << "Set point reached" << endl;
        heatOff();
        atSetpoint = true;
        return;
    }

    if (ItoF(pool.get()) < (poolConfigSetPointSetting.get() - poolConfigSetPointSwingSetting.get())) {
        atSetpoint = false;
        heatOn();
    }
}

bool heatOn() {
    if (!envAllowHeat()) {
        Homie.getLogger() << "Env not allow heat on" << endl;
        heatOff();
        return false;
    }

    if (!isHeating) {
        Homie.getLogger() << "Heat turned on" << endl;
    }
    setRunStatus(RunStatus::SOLAR);
    setPumpOn();
    isHeating = true;
    displayPageMainUpdate();
    return true;
}

bool heatOff() {
    if (isHeating) {
        Homie.getLogger() << "Heat turned off" << endl;
    }
    setRunStatus(RunStatus::OFF);
    setPumpOff();
    isHeating = false;
    displayPageMainUpdate();

    return true;
}

void setPumpOn() {
    digitalWrite(PUMP_RLY_PIN, HIGH);
    pumpOn = true;
}

void setPumpOff() {
    digitalWrite(PUMP_RLY_PIN, LOW);
    pumpOn = false;
}

void setAuxHeatOn() {
    digitalWrite(AUX_HEAT_RLY_PIN, HIGH);
    propaneOn = true;
}

void setAuxHeatOff() {
    digitalWrite(AUX_HEAT_RLY_PIN, LOW);
    propaneOn = false;
}

void setAuxOn() {
    digitalWrite(AUX_RLY_PIN, HIGH);
    auxOn = true;
}

void setAuxOff() {
    digitalWrite(AUX_RLY_PIN, LOW);
    auxOn = false;
}

bool envAllowHeat() {
    bool rtn = true;
    status.reset();

    if (!poolOk) status << "SENSOR: Pool error\n";
    if (!tinOk) status << "SENSOR: Temp in error\n";
    if (!toutOk) status << "SENSOR: Temp out error\n";
    if (!airOk) status << "SENSOR: Air error\n";

    if (overrideEnv) {
        Homie.getLogger() << "Env: Override enabled !" << endl;
        status << "ENV: Override EN\n";
        return true;
    }

#ifndef NO_ENV_SOLAR_CHECK
    if (!envCheckNoSolar) {
        time_t t = now();
        if (t < daylight.midday && solar.elevation <= poolConfigSunMinElvAMSetting.get()) {
            Homie.getLogger() << "Env: Elevation " << solar.elevation << "° below morning minimum " << poolConfigSunMinElvAMSetting.get() << "°" << endl;
            status << "ENV: Below AM Elv\n";
            rtn = false;
        }
        if (t >= daylight.midday && solar.elevation <= poolConfigSunMinElvPMSetting.get()) {
            Homie.getLogger() << "Env: Elevation " << solar.elevation << "° below evening minimum " << poolConfigSunMinElvPMSetting.get() << "°" << endl;
            status << "ENV: Below PM Elv\n";
            rtn = false;
        }
    } else {
        Homie.getLogger() << "Env: Skip solar check" << endl;
    }
#else
    Homie.getLogger() << "Env: Skip solar check by define" << endl;
#endif

#ifndef NO_ENV_CLOUD_CHECK
    if (!envCheckNoCloud) {
        if (isOvercast && ItoF(air.get()) < poolConfigSetPointSetting.get()) {
            Homie.getLogger() << "Env: Overcast and too cool outside" << endl;
            status << "ENV: Overcast and Cold\n";
            rtn = false;
        }
    } else {
        Homie.getLogger() << "Env: Skip cloud check" << endl;
    }
#else
    Homie.getLogger() << "Env: Skip cloud check by define" << endl;
#endif

#ifndef NO_ENV_AIR_CHECK
    if (!envCheckNoAir) {
        if (ItoF(air.get()) < poolConfigSetPointSetting.get()) {
            if ((ItoF(air.get()) + poolConfigAirPoolDiffSetting.get()) < ItoF(tin.get())) {
                Homie.getLogger() << "Env: Temp in to air not enough diff" << endl;
                status << "ENV: TIN -> Air diff to small\n";
                rtn = false;
            }
        }
    } else {
        Homie.getLogger() << "Env: Skip set-point check" << endl;
    }
#else
    Homie.getLogger() << "Env: Skip set-point check by define" << endl;
#endif

    // todo-evo: This needs fixed tin and tout can only be check while running
//#ifndef NO_ENV_IN_OUT_DIFF_CHECK
//    if(!envCheckNoTDiff) {
//        if (ItoF(tout.get()) < (ItoF(tin.get()) - pshConfigs.tinDiffMax)) {
//            Homie.getLogger() << "Env: Temp out less than temp in" << endl;
//            status << "ENV: TOUT < TIN\n";
//            rtn = false;
//        }
//    }else {
//        Homie.getLogger() << "Env: Skip tin to tout diff check" << endl;
//    }
//#else
//    Homie.getLogger() << "Env: Skip tin to tout diff check by define" << endl;
//#endif

    if (status.getCursor() == 0) {
        status << "ok";
    }

    return rtn;
}