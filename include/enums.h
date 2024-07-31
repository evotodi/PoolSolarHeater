#pragma once

#include <string>

/**
 * Update the DISP_LAST_NUM_OF_PAGES to equal the total number of pages in the enum
 */
enum DisplayPage {
    DISP_MAIN = 0,
    DISP_INFO,
    DISP_CONFIG,
    DISP_LAST_NUM_OF_PAGES,
};

/**
 * Update the MENU_LAST_NUM_OF_PAGES to equal the total number of pages in the enum
 */
enum MenuPage {
    MENU_MAIN = 0,
    MENU_CONFIG,
    MENU_CONFIG_ALTER,
    MENU_CONFIG_ALTER_VAL,
    MENU_LAST_NUM_OF_PAGES,
};

enum RunStatus {
    OFF = 0,
    STANDBY = 1,
    ENV_STANDBY = 2,
    SOLAR = 3,
    HEAT_AUX = 4,
    MANUAL_SOLAR = 5,
    MANUAL_HEAT_AUX = 6,
    ERROR = 7,
};

enum VariableType
{
    // Don't forget to edit main::setupConfigPageItems
    VAR_TYPE_FLOAT = 0,
    VAR_TYPE_INT = 1,
    VAR_TYPE_BOOL = 2,
};

enum ConfigPageItem
{
    // Don't forget to edit main::setupConfigPageItems
    CFG_PG_POOL_SP = 0,
    CFG_PG_AUX_HEAT_SP,
    CFG_PG_SP_SWING,
    CFG_PG_SUN_AM_ELV,
    CFG_PG_SUN_PM_ELV,
    CFG_PG_CLOUDY,
    CFG_PG_OVERCAST,
    CFG_PG_PUMP_GPM,
    CFG_PG_AUX_HEAT_EN,
    CFG_PG_LAST_NUM_OF_PAGES,
};
