#include "calibration.h"

void calibratePoolTemps()
{
    Homie.getLogger() << "Calibration Started!" << endl;
    digitalWrite(LED_PIN, LOW);
    delay(2000);
    tin.clear();
    tout.clear();
    pool.clear();

    for (int i = 0; i < 10; i++) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        sensors.requestTemperatures();
        delay(100);

        tin.add(FtoI(sensors.getTempF(tempSensorIn)));
        tout.add(FtoI(sensors.getTempF(tempSensorOut)));
        addPoolTemp();
        Homie.getLogger() << "Tin = " << ItoF(tin.getLast()) << " °F  Tin Smooth = " << ItoF(tin.get()) << " °F " << endl;
        Homie.getLogger() << "Tout = " << ItoF(tout.getLast()) << " °F  Tout Smooth = " << ItoF(tout.get()) << " °F " << endl;
        Homie.getLogger() << "Pool = " << ItoF(pool.getLast()) << " °F  Pool Smooth = " << ItoF(pool.get()) << " °F" << endl;

        yield();
    }

    digitalWrite(LED_PIN, LOW);

    if(poolConfigPoolTempInSetting.get() == 0) {
        toutSettings.offset = ItoF(tin.get()) - ItoF(tout.get());
    } else if(poolConfigPoolTempInSetting.get() == 1) {
        tinSettings.offset = ItoF(pool.get()) - ItoF(tin.get());
        toutSettings.offset = ItoF(pool.get()) - ItoF(tout.get());
    }else{
        Homie.getLogger() << F("✖ Invalid poolTemp type: ") << poolConfigPoolTempInSetting.get() << endl;
    }

    Homie.getLogger() << "Offsets: tin = " << tinSettings.offset << " °F  tout = " << toutSettings.offset << " °F" << endl;

    configWrite();

    Homie.getLogger() << F("✔ Calibration Completed!") << endl;
    digitalWrite(LED_PIN, HIGH);

}

void calibrationReset()
{
    Homie.getLogger() << F("✔ Calibration Reset!") << endl;
    digitalWrite(LED_PIN, LOW);
    tinSettings.offset = float(0);
    toutSettings.offset = float(0);
    configWrite();
    delay(500);
    digitalWrite(LED_PIN, HIGH);
}