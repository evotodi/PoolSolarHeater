#include "datetime.h"

time_t getNtpTime() {
    timeClient.forceUpdate();
    Homie.getLogger() << "NTP Updated. Current time is [ Unix: " << timeClient.getEpochTime() << " Human: " << timeClient.getFormattedTime() << " ]" << endl;
#ifdef DEBUG_FORCE_TIME
    return time_t (DEBUG_FORCE_TIME);
#else
    return time_t(timeClient.getEpochTime());
#endif

}

int getTimeOffset(const time_t *t, bool asHours) {
    if (month(*t) == settingTimeDstBeginMonth.get() && day(*t) >= settingTimeDstBeginDay.get()) {
        if (asHours) {
            return settingTimeDstOffset.get();
        } else {
            return settingTimeDstOffset.get() * 60 * 60;
        }
    }

    if (month(*t) == settingTimeDstEndMonth.get() && day(*t) <= settingTimeDstEndDay.get()) {
        if (asHours) {
            return settingTimeStOffset.get();
        } else {
            return settingTimeStOffset.get() * 60 * 60;
        }
    }

    if (month(*t) > settingTimeDstBeginMonth.get() && month(*t) < settingTimeDstEndMonth.get()) {
        if (asHours) {
            return settingTimeDstOffset.get();
        } else {
            return settingTimeDstOffset.get() * 60 * 60;
        }
    }

    if (month(*t) > settingTimeDstEndMonth.get()) {
        if (asHours) {
            return settingTimeStOffset.get();
        } else {
            return settingTimeStOffset.get() * 60 * 60;
        }
    }

    if (asHours) {
        return settingTimeDstOffset.get();
    } else {
        return settingTimeDstOffset.get() * 60 * 60;
    }
}

std::string getTimestamp(bool withOffset, bool human, time_t ts) {
    time_t tNow = now();
    if (ts != 0) {
        tNow = ts;
    }

    int offsetHours = getTimeOffset(&tNow, true);

    if (withOffset) {
        tNow = tNow + (offsetHours * 60 * 60);
    }

    char cDate[32] = "\0"; // Date ends at element 12
    char cYear[5] = "\0";
    char cTime[18] = "\0";

    if (human) {
        sprintf(cYear, "%04d", year(tNow));
        sprintf(cDate, "%0d/%0d/%c%c ", month(tNow), day(tNow), cYear[2], cYear[3]);
        sprintf(cTime, "%02d:%02d:%02d", hour(tNow), minute(tNow), second(tNow));
    } else {
        sprintf(cDate, "%04d-%02d-%02dT", year(tNow), month(tNow), day(tNow));
        if (withOffset) {
            if (offsetHours < 0) {
                sprintf(cTime, "%02d:%02d:%02d.00-00:00", hour(tNow), minute(tNow), second(tNow));
            } else {
                sprintf(cTime, "%02d:%02d:%02d.00+00:00", hour(tNow), minute(tNow), second(tNow));
            }
        } else {
            sprintf(cTime, "%02d:%02d:%02d.00+00:00", hour(tNow), minute(tNow), second(tNow));
        }
    }

    strcat(cDate, cTime);
    Homie.getLogger() << "Timestamp: " << cDate << endl;
    std::string rtn(cDate);

    return rtn;
}
