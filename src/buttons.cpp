#include "buttons.h"

void setupButtons() {
    InterruptButton::setMenuCount(MenuPage::MENU_LAST_NUM_OF_PAGES);
    InterruptButton::setMenuLevel(MenuPage::MENU_MAIN);
    InterruptButton::setMode(Mode_Synchronous);

    button1.bind(Event_KeyPress, MenuPage::MENU_MAIN, &menuMainBtn1KeyPress);
    button1.bind(Event_DoubleClick, MenuPage::MENU_MAIN, &menuMainBtn1DblClick);

    button1.bind(Event_KeyPress, MenuPage::MENU_CONFIG, &menuConfigBtn1KeyPress);
    button1.bind(Event_LongKeyPress, MenuPage::MENU_CONFIG, &menuConfigBtn1LongPress);
    button1.bind(Event_DoubleClick, MenuPage::MENU_CONFIG, &menuConfigBtn1DblClick);
}

void menuMainBtn1KeyPress(void) {
    Serial.printf("Menu MAIN, Button 1: Key Down:              %lu ms\n", millis());
    int8_t page = displayPage + 1;
    Serial.printf("Page = %d\n", page);

    switch (page) {
        case DisplayPage::DISP_INFO:
            InterruptButton::setMenuLevel(MenuPage::MENU_MAIN);
            displayPageInfo();
            displayPage = DisplayPage::DISP_INFO;
            break;
        case DisplayPage::DISP_CONFIG:
            InterruptButton::setMenuLevel(MenuPage::MENU_CONFIG);
            displayPageConfig();
            displayPage = DisplayPage::DISP_CONFIG;
            break;
        default:
            InterruptButton::setMenuLevel(MenuPage::MENU_MAIN);
            displayPageMain();
            displayPage = DisplayPage::DISP_MAIN;
            displayPageMainUpdate();
    }
}

void menuMainBtn1DblClick(void) {
    Serial.printf("Menu MAIN, Button 1: Long Press:              %lu ms\n", millis());
    switch (displayPage) {
        case DisplayPage::DISP_MAIN:
            displayPageMain();
            break;
        case DisplayPage::DISP_INFO:
            displayPageInfo();
            break;
        default:
            return;
    }
}

void menuConfigBtn1KeyPress(void) {
    menuMainBtn1KeyPress();
}

void menuConfigBtn1LongPress(void) {
    Serial.printf("Menu CONFIG, Button 1: Long Key Press:              %lu ms\n", millis());
    Serial.println("Heat Pump");
    digitalWrite(AUX_HEAT_RLY_PIN, HIGH);
    delay(1000);
    digitalWrite(AUX_HEAT_RLY_PIN, LOW);
}

void menuConfigBtn1DblClick(void) {
    Serial.printf("Menu CONFIG, Button 1: Double Click:              %lu ms\n", millis());
    Serial.println("Aux Pump");
    digitalWrite(AUX_RLY_PIN, HIGH);
    delay(1000);
    digitalWrite(AUX_RLY_PIN, LOW);
}