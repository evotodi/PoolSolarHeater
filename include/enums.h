#pragma once

/**
 * Update the DISP_LAST_NUM_OF_PAGES to equal the total number of pages in the enum
 */
enum DisplayPage {
    DISP_MAIN = 0,
    DISP_INFO = 1,
    DISP_CONFIG = 2,
    DISP_LAST_NUM_OF_PAGES = 3,
};

/**
 * Update the MENU_LAST_NUM_OF_PAGES to equal the total number of pages in the enum
 */
enum MenuPage {
    MENU_MAIN = 0,
    MENU_CONFIG = 1,
    MENU_LAST_NUM_OF_PAGES = 2,
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