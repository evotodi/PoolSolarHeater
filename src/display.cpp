#include "display.h"

void displayCenterMessage(const char *str) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);

    tft.setCursor(0, tft.height() / 2);
    tft.println(str);
}

void displayCenterMessage(std::string str) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);

    tft.setCursor(0, tft.height() / 2);
    tft.println(str.c_str());
}

void displayPageMain() {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(3);
    tft.setTextColor(TFT_LT_BLUE);
    tft.setCursor(0, 0);

    tft.println("Status:");
    tft.println("");
    tft.println("");
    tft.println("Pump:    ");
    tft.println("HeatAux: ");
    tft.println("Pool : ");
    tft.println("TIn  : ");
    tft.println("Solar: ");
    tft.println("Heat : ");
    tft.println("Air  : ");
    tft.println("Watts: ");
    tft.println("Post: ");

    displayPageMainUpdate();
}

void displayPageMainUpdate() {
    if (displayPage != DisplayPage::DISP_MAIN) return;

    int16_t x1 = 0;
    int16_t y1 = 0;
    uint16_t w = 0;
    uint16_t h = 0;

    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE);

    tft.getTextBounds("0", 0, 0, &x1, &y1, &w, &h); // Just to get character size

    // Status
    tft.fillRect(0, h, tft.width(), h, TFT_BLACK);
    tft.setCursor(0, h);
    if (runStatus == RunStatus::ERROR) {
        tft.setTextColor(TFT_PINK);
    }
    tft.print(getRunStatusStr());
    tft.setTextColor(TFT_WHITE);

    // Pump
    tft.fillRect(w * 9, h * 3, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 9, h * 3);
    if (rlyPump) {
        tft.setTextColor(TFT_GREEN);
    } else {
        tft.setTextColor(TFT_YELLOW);
    }
    tft.print(boolToStr(rlyPump));
    tft.setTextColor(TFT_WHITE);

    // Propane
    tft.fillRect(w * 9, h * 4, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 9, h * 4);
    if (rlyHeatAux) {
        tft.setTextColor(TFT_GREEN);
    } else {
        tft.setTextColor(TFT_YELLOW);
    }
    tft.print(boolToStr(rlyHeatAux));
    tft.setTextColor(TFT_WHITE);

    // Pool
    tft.fillRect(w * 6, h * 5, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 6, h * 5);
    if (!poolOk) {
        tft.setTextColor(TFT_PINK);
        tft.print("ERROR");
    } else {
        tft.printf("%.1f", ItoF(pool.get()));
    }
    tft.setTextColor(TFT_WHITE);

    // Temp IN
    tft.fillRect(w * 6, h * 6, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 6, h * 6);
    if (!tinOk) {
        tft.setTextColor(TFT_PINK);
        tft.print("ERROR");
    } else {
        tft.printf("%.1f", ItoF(tin.get()));
    }
    tft.setTextColor(TFT_WHITE);

    // Temp OUT Solar
    tft.fillRect(w * 6, h * 7, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 6, h * 7);
    if (!toutSolarOk) {
        tft.setTextColor(TFT_PINK);
        tft.print("ERROR");
    } else {
        tft.printf("%.1f", ItoF(toutSolar.get()));
    }
    tft.setTextColor(TFT_WHITE);

    // Temp OUT Heat
    tft.fillRect(w * 6, h * 8, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 6, h * 8);
    if (!toutHeatOk) {
        tft.setTextColor(TFT_PINK);
        tft.print("ERROR");
    } else {
        tft.printf("%.1f", ItoF(toutHeat.get()));
    }
    tft.setTextColor(TFT_WHITE);

    // Air
    tft.fillRect(w * 6, h * 9, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 6, h * 9);
    if (!airOk) {
        tft.setTextColor(TFT_PINK);
        tft.print("ERROR");
    } else {
        tft.printf("%.1f", ItoF(air.get()));
    }
    tft.setTextColor(TFT_WHITE);

    // Watts
    tft.fillRect(w * 6, h * 10, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 6, h * 10);
    tft.printf("%.1f", ItoF(watts.get()));

    // Last post
    tft.fillRect(0, h * 12, tft.width(), tft.height(), TFT_BLACK);
    tft.setCursor(0, h * 12);
    tft.setTextSize(2);
    tft.println(getTimestamp(false, true, lastPublishData).c_str());
}

void displayPageInfo() {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(0, 0);

    tft.printf("Pool:  %.1f\n", ItoF(pool.get()));
    tft.printf("T-In:  %.1f\n", ItoF(tin.get()));
    tft.printf("T-Solar: %.1f\n", ItoF(toutSolar.get()));
    tft.printf("T-Heat: %.1f\n", ItoF(toutHeat.get()));
    tft.printf("Air: %.1f\n", ItoF(air.get()));
    tft.printf("Light: %d\n", light.get());
    tft.printf("Pool SP: %.1f\n", settingPoolSP.get());
    tft.printf("Heat SP: %.1f\n", settingHeatAuxSP.get());
    tft.printf("Swing: %.1f\n", settingSPHyst.get());
    tft.printf("Az: %.4f\n", solar.azimuth);
    tft.printf("Elv: %.4f\n", solar.elevation);
    tft.printf("Watts: %d\n", watts.get());
    tft.printf("Cloudy: %s\n", boolToStr(isCloudy));
    tft.printf("Overcast: %s\n", boolToStr(isOvercast));
    tft.printf("IP: %s\n", ip);
    tft.printf("Version: %s\n", VERSION);
    tft.printf("Status: %s\n", status.get());
}

void displayPageConfig() {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(0, 0);

    tft.println("CONFIG");
    tft.println("");
    tft.println("Long press:");
    tft.println("calibrate something");
    tft.println("");
    tft.println("Double click:");
    tft.println("calibrate something else");
}