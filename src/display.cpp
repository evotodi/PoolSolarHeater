#include "display.h"
#include "utils.h"

CursorPos cursorPos;
Rectangle cfgAlterRect;
TextBounds cfgAlterTxtBounds;
size_t cfgValLen = 0;
std::basic_string<char> cfgVal = "";
uint8_t cfgValAtCur = 0;
uint8_t cfgValAtCurOrig = 0;
int8_t cfgValCursor = 0;
std::basic_string<char> cfgValPointer = "";

void displayCenterMessage(const char *str) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(txtSize.med.num);
    tft.setTextColor(ILI9341_WHITE);

    tft.setCursor(0, tft.height() / 2);
    tft.println(str);
}

void displayCenterMessage(std::string str) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(txtSize.med.num);
    tft.setTextColor(ILI9341_WHITE);

    tft.setCursor(0, tft.height() / 2);
    tft.println(str.c_str());
}

void displayPageMain() {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(txtSize.large.num);
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

    tft.setTextSize(txtSize.large.num);
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
        if (ItoF(pool.get()) < settingPoolSP.get()) {
            tft.setTextColor(TFT_YELLOW);
        } else {
            tft.setTextColor(TFT_GREEN);
        }
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
        if (toutSolar.get() < pool.get()) {
            tft.setTextColor(TFT_YELLOW);
        } else {
            tft.setTextColor(TFT_GREEN);
        }
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
        if (toutHeat.get() < pool.get()) {
            tft.setTextColor(TFT_YELLOW);
        } else {
            tft.setTextColor(TFT_GREEN);
        }
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
        if (air.get() < pool.get()) {
            tft.setTextColor(TFT_YELLOW);
        } else {
            tft.setTextColor(TFT_GREEN);
        }
        tft.printf("%.1f", ItoF(air.get()));
    }
    tft.setTextColor(TFT_WHITE);

    // Watts
    tft.fillRect(w * 6, h * 10, tft.width(), h, TFT_BLACK);
    tft.setCursor(w * 6, h * 10);
    if (ItoF(watts.get()) < DISPLAY_LOW_WATTS) {
        tft.setTextColor(TFT_YELLOW);
    } else {
        tft.setTextColor(TFT_GREEN);
    }
    tft.printf("%.1f", ItoF(watts.get()));
    tft.setTextColor(TFT_WHITE);

    // Last post
    tft.fillRect(0, h * 12, tft.width(), tft.height(), TFT_BLACK);
    tft.setCursor(0, h * 12);
    tft.setTextSize(txtSize.med.num);
    tft.println(getTimestamp(false, true, lastPublishData).c_str());
}

void displayPageInfo() {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(txtSize.med.num);
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
    // Reset vars
    cfgVal = "";
    cfgValPointer = "";

    // Draw screen
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(txtSize.med.num);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(0, 0);

    tft.println("CONFIG");
    tft.setTextSize(txtSize.small.num);
    tft.println("");
    tft.println("Click: BTN 1: Next BTN 2: Select");
    tft.println("Long Prs BTN 2: Exit");
    tft.println("");
    tft.setTextSize(txtSize.med.num);
    if (configPageSelection == ConfigPageItem::CFG_PG_POOL_SP) {
        tft.println(">Pool Set Point");
    }else{
        tft.println(" Pool Set Point");
    }

    if (configPageSelection == ConfigPageItem::CFG_PG_AUX_HEAT_SP) {
        tft.println(">Aux Heat Set Point");
    }else{
        tft.println(" Aux Heat Set Point");
    }

    if (configPageSelection == ConfigPageItem::CFG_PG_SP_SWING) {
        tft.println(">Set Point Swing");
    }else{
        tft.println(" Set Point Swing");
    }

    if (configPageSelection == ConfigPageItem::CFG_PG_SUN_AM_ELV) {
        tft.println(">Sun AM Elv");
    }else{
        tft.println(" Sun AM Elv");
    }

    if (configPageSelection == ConfigPageItem::CFG_PG_SUN_PM_ELV) {
        tft.println(">Sun PM Elv");
    }else{
        tft.println(" Sun PM Elv");
    }

    if (configPageSelection == ConfigPageItem::CFG_PG_CLOUDY) {
        tft.println(">Cloudy Level");
    }else{
        tft.println(" Cloudy Level");
    }

    if (configPageSelection == ConfigPageItem::CFG_PG_OVERCAST) {
        tft.println(">Overcast Count");
    }else{
        tft.println(" Overcast Count");
    }

    if (configPageSelection == ConfigPageItem::CFG_PG_PUMP_GPM) {
        tft.println(">Pump GPM");
    }else{
        tft.println(" Pump GPM");
    }

    if (configPageSelection == ConfigPageItem::CFG_PG_AUX_HEAT_EN) {
        tft.println(">Aux Heat Enable");
    }else{
        tft.println(" Aux Heat Enable");
    }
}

void displayPageConfigAlter()
{

    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(txtSize.med.num);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(0, 0);

    tft.print("Alter: ");
    tft.println(configPageItemsData[configPageSelection].name);
    tft.setTextSize(txtSize.small.num);
    tft.println("");
    // Store the location and size of first menu text
    cfgAlterTxtBounds.size = txtSize.small;
    cfgAlterTxtBounds.x = tft.getCursorX();
    cfgAlterTxtBounds.y = tft.getCursorY();
    tft.getTextBounds("Click: BTN 1: Select BTN 2: Edit", cfgAlterTxtBounds.x, cfgAlterTxtBounds.y, &cfgAlterTxtBounds.x, &cfgAlterTxtBounds.y, &cfgAlterTxtBounds.w, &cfgAlterTxtBounds.h);
    tft.println("Click: BTN 1: Select BTN 2: Edit");
    // Second menu text
    tft.println("Long Prs BTN 1: Save BTN 2: Cancel");
    tft.print("\n\n");

    tft.setTextSize(txtSize.xl.num);

    if (cfgVal == "") {
        cfgVal = configPageItemsData[configPageSelection].getSettingValueString(PAD_CFG_VALS);
        cfgValLen = tft.printf("%s", cfgVal.c_str());
    } else {
        tft.printf("%s", cfgVal.c_str());
    }
    tft.print("\n");

    if (cfgValPointer == "") {
        // Create the pointer and set cursor var
        cfgValCursor = cfgValLen - 1;
        cfgValPointer = "^";
        padTo(cfgValPointer, cfgValLen);
        tft.print(cfgValPointer.c_str());
        cursorPos.x = tft.getCursorX();
        cursorPos.y = tft.getCursorY();
        tft.print("\n");
        cfgValAtCur = atoi(&cfgVal[cfgValLen - 1]);

        cfgAlterRect.x = 0, cfgAlterRect.y = cursorPos.y - txtSize.xl.height;
        cfgAlterRect.w = cfgValLen * txtSize.xl.width;
        cfgAlterRect.h = txtSize.xl.height * 2;
        cfgAlterRect.color = ILI9341_BLACK;
    }
}

void displayPageConfigAlterSelect()
{
    //Change top menu
    tft.setTextSize(txtSize.small.num);
    tft.fillRect(cfgAlterTxtBounds.x, cfgAlterTxtBounds.y, cfgAlterTxtBounds.w, cfgAlterTxtBounds.h, TFT_BLACK);
    tft.setCursor(cfgAlterTxtBounds.x, cfgAlterTxtBounds.y);
    tft.print("Click: BTN 1: Select BTN 2: Edit");

    tft.setTextSize(txtSize.xl.num);
    tft.setCursor(cfgAlterTxtBounds.x, cfgAlterTxtBounds.y);
    // Move the cursor to far right if all the way left
    if (cfgValCursor <= 0) {
        cfgValCursor = cfgValLen - 1;
    } else {
        cfgValCursor -= 1;
    }

    // Check if . under cursor
    char charAtCur = cfgVal[cfgValCursor];
    if (charAtCur == '.') {
        cfgValCursor -= 1;
        if (cfgValCursor <= 0) {
            cfgValCursor = cfgValLen - 1;
        }
        charAtCur = cfgVal[cfgValCursor];
    }

    cfgValAtCur = charAtCur - 48;
    cfgValAtCurOrig = cfgValAtCur;

    tft.fillRect(cfgAlterRect.x, cfgAlterRect.y, cfgAlterRect.w, cfgAlterRect.h, TFT_BLACK);
    tft.setCursor(cfgAlterRect.x, cfgAlterRect.y);
    tft.println(cfgVal.c_str());
    cfgValPointer = "^";
    padTo(cfgValPointer, cfgValCursor + 1);
    tft.println(cfgValPointer.c_str());
}

void displayPageConfigAlterEdit()
{
    //Change top menu
    tft.setTextSize(txtSize.small.num);
    tft.fillRect(cfgAlterTxtBounds.x, cfgAlterTxtBounds.y, cfgAlterTxtBounds.w, cfgAlterTxtBounds.h, TFT_BLACK);
    tft.setCursor(cfgAlterTxtBounds.x, cfgAlterTxtBounds.y);
    tft.print("Click: BTN 1: Incr BTN 2: Decr");

    //Create rect around value
    tft.setTextSize(txtSize.xl.num);
    tft.fillRect(cfgAlterRect.x, cfgAlterRect.y, cfgAlterRect.w, cfgAlterRect.h, TFT_PINK);
    tft.setCursor(cfgAlterRect.x, cfgAlterRect.y);
    tft.println(cfgVal.c_str());
    cfgValPointer = "^";
    padTo(cfgValPointer, cfgValCursor + 1);
    tft.println(cfgValPointer.c_str());
}