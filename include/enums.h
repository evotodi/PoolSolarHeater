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
    SOLAR = 1,
    PROPANE = 2,
    MANUAL_SOLAR = 3,
    MANUAL_PROPANE = 4,
    ERROR = 5,
};