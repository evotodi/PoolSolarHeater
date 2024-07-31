#include "buttons.h"

void setupButtons() {
    InterruptButton::setMenuCount(MenuPage::MENU_LAST_NUM_OF_PAGES);
    InterruptButton::setMenuLevel(MenuPage::MENU_MAIN);
    InterruptButton::setMode(Mode_Synchronous);

    button1.bind(Event_KeyPress, MenuPage::MENU_MAIN, &menuMainKP1);

    button1.bind(Event_KeyPress, MenuPage::MENU_CONFIG, &menuConfigKP1_next);
    button2.bind(Event_LongKeyPress, MenuPage::MENU_CONFIG, &menuConfigLP2_exit);
    button2.bind(Event_KeyPress, MenuPage::MENU_CONFIG, &menuConfigKP2_select);

    button1.bind(Event_KeyPress, MenuPage::MENU_CONFIG_ALTER, &menuConfigAlterKP1_select);
    button2.bind(Event_KeyPress, MenuPage::MENU_CONFIG_ALTER, &menuConfigAlterKP2_edit);
    button1.bind(Event_LongKeyPress, MenuPage::MENU_CONFIG_ALTER, &menuConfigAlterLP1_save);
    button2.bind(Event_LongKeyPress, MenuPage::MENU_CONFIG_ALTER, &menuConfigAlterLP2_cancel);

    button1.bind(Event_KeyPress, MenuPage::MENU_CONFIG_ALTER_VAL, &menuConfigAlterValKP1_incr);
    button2.bind(Event_KeyPress, MenuPage::MENU_CONFIG_ALTER_VAL, &menuConfigAlterValKP2_dec);
    button1.bind(Event_LongKeyPress, MenuPage::MENU_CONFIG_ALTER_VAL, &menuConfigAlterValLP1_save);
    button2.bind(Event_LongKeyPress, MenuPage::MENU_CONFIG_ALTER_VAL, &menuConfigAlterValLP2_cancel);

}

// Main Menu
void menuMainKP1(void) {
    Serial.printf("Menu MAIN, Button 1: Press:              %lu ms\n", millis());
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

// Config Menu
void menuConfigKP1_next(void) {
    Serial.printf("Menu CONFIG, Button 1: Press:              %lu ms\n", millis());
    uint8_t item = configPageSelection + 1;
    if (configPageSelection >= ConfigPageItem::CFG_PG_LAST_NUM_OF_PAGES - 1) {
        configPageSelection = ConfigPageItem(0);
    }else {
        configPageSelection = ConfigPageItem(item);
    }
    displayPageConfig();
}

void menuConfigKP2_select(void)
{
    Serial.printf("Menu CONFIG, Button 2: Press:              %lu ms\n", millis());
    displayPageConfigAlter();
    InterruptButton::setMenuLevel(MenuPage::MENU_CONFIG_ALTER);
}

void menuConfigLP2_exit(void) {
    Serial.printf("Menu CONFIG, Button 1: Long Press:              %lu ms\n", millis());
    menuMainKP1();
}

// Config Alter

void menuConfigAlterKP1_select(void)
{
    Serial.printf("Menu CONFIG ALTER, Button 1: Press:              %lu ms\n", millis());
    // SELECT
    displayPageConfigAlterSelect();
}

void menuConfigAlterKP2_edit(void)
{
    Serial.printf("Menu CONFIG ALTER, Button 2: Press:              %lu ms\n", millis());
    InterruptButton::setMenuLevel(MenuPage::MENU_CONFIG_ALTER_VAL);
    displayPageConfigAlterEdit();
}

void menuConfigAlterLP1_save(void)
{
    Serial.printf("Menu CONFIG ALTER, Button 1: Long Press:              %lu ms\n", millis());
    // SAVE

    Homie.getLogger() << "Value: " << cfgVal.c_str() << endl;

    if (configPageItemsData[configPageSelection].setting->isBool()) {
        PoolSetting<bool>* dataSetting = static_cast<PoolSetting<bool>*>(configPageItemsData[configPageSelection].setting);
        dataSetting->set(std::stoi(cfgVal));
        Homie.getLogger() << "Pool Setting " << dataSetting->getName() << "changed to: " << dataSetting->get() << endl;
    }
    else if (configPageItemsData[configPageSelection].setting->isLong()) {
        PoolSetting<long>* dataSetting = static_cast<PoolSetting<long >*>(configPageItemsData[configPageSelection].setting);
        dataSetting->set(std::stol(cfgVal));
        Homie.getLogger() << "Pool Setting " << dataSetting->getName() << "changed to: " << dataSetting->get() << endl;
    }
    else if (configPageItemsData[configPageSelection].setting->isShort()) {
        PoolSetting<int16_t>* dataSetting = static_cast<PoolSetting<int16_t>*>(configPageItemsData[configPageSelection].setting);
        dataSetting->set(std::stoi(cfgVal));
        Homie.getLogger() << "Pool Setting " << dataSetting->getName() << "changed to: " << dataSetting->get() << endl;
    }
    else if (configPageItemsData[configPageSelection].setting->isUShort()) {
        PoolSetting<uint16_t>* dataSetting = static_cast<PoolSetting<uint16_t>*>(configPageItemsData[configPageSelection].setting);
        dataSetting->set(std::stoi(cfgVal));
        Homie.getLogger() << "Pool Setting " << dataSetting->getName() << "changed to: " << dataSetting->get() << endl;
    }
    else if (configPageItemsData[configPageSelection].setting->isDouble()) {
        PoolSetting<double>* dataSetting = static_cast<PoolSetting<double>*>(configPageItemsData[configPageSelection].setting);
        dataSetting->set(std::stod(cfgVal));
        Homie.getLogger() << "Pool Setting " << dataSetting->getName() << "changed to: " << dataSetting->get() << endl;
    }
    else if (configPageItemsData[configPageSelection].setting->isConstChar()) {
        PoolSetting<const char *>* dataSetting = static_cast<PoolSetting<const char *>*>(configPageItemsData[configPageSelection].setting);
        dataSetting->set(cfgVal.c_str());
        Homie.getLogger() << "Pool Setting " << dataSetting->getName() << "changed to: " << dataSetting->get() << endl;
    }

    configWrite();

    displayPageConfig();
    InterruptButton::setMenuLevel(MenuPage::MENU_CONFIG);
}

void menuConfigAlterLP2_cancel(void)
{
    // CANCEL
    displayPageConfig();
    InterruptButton::setMenuLevel(MenuPage::MENU_CONFIG);
}

// Config Alter Val

void menuConfigAlterValKP1_incr(void)
{
    Serial.printf("Menu CONFIG ALTER VAL, Button 1: Press:              %lu ms\n", millis());

    if (configPageItemsData[configPageSelection].varType == VariableType::VAR_TYPE_BOOL) {
        if (cfgValAtCur < 1) {
            cfgValAtCur++;
        }
    } else {
        if (cfgValAtCur < 9) {
            cfgValAtCur++;
        }
    }

    cfgVal[cfgValCursor] = 48 + cfgValAtCur;
    displayPageConfigAlterEdit();
}

void menuConfigAlterValKP2_dec(void)
{
    Serial.printf("Menu CONFIG ALTER VAL, Button 2: Press:              %lu ms\n", millis());

    if (cfgValAtCur > 0) {
        cfgValAtCur--;
    }

    cfgVal[cfgValCursor] = 48 + cfgValAtCur;
    displayPageConfigAlterEdit();
}

void menuConfigAlterValLP1_save(void) {
    Serial.printf("Menu CONFIG ALTER VAL, Button 1: Long Press:              %lu ms\n", millis());
    InterruptButton::setMenuLevel(MenuPage::MENU_CONFIG_ALTER);
    displayPageConfigAlterSelect();
}

void menuConfigAlterValLP2_cancel(void)
{
    Serial.printf("Menu CONFIG ALTER VAL, Button 2: Long Press:              %lu ms\n", millis());
    cfgValAtCur = cfgValAtCurOrig;
    cfgVal[cfgValCursor] = 48 + cfgValAtCur;
    InterruptButton::setMenuLevel(MenuPage::MENU_CONFIG_ALTER);
    displayPageConfigAlterSelect();
}