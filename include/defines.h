#pragma once

#define VERSION "2.1.63"

// Debugging Defines >>>
#define DEBUG 1
//#define LOG_TO_TELNET 1
//#define NO_ENV_SOLAR_CHECK 1
//#define NO_ENV_AIR_CHECK 1
//#define NO_ENV_CLOUD_CHECK 1
//#define NO_ENV_IN_OUT_DIFF_CHECK 1
#define NO_CHECK_SENSORS_OK 1
//#define DEBUG_FORCE_TIME 1687217416
#define PRINT_POOL_CONFIG_ON_READ_WRITE 1
// <<< Debugging Defines

#define LOOP_DAT_DLY          (5*1E3)
#define LOOP_PROC_DLY         (5*1E3)
#define LOOP_PUB_DLY         (15*1E3)
#define LOOP_PUB_CFG_DLY     (60*1E3)
#define LOOP_HB_DLY           (5*1E3)
#define LOOP_DAYLIGHT_DLY   (1800*1E3)

#define boolToStr(x) ((x)?"Yes":"No")

#define TELNET_PORT 23
#define DS_TEMP_PRECISION 12
#define T_SENSOR_BAD -500

// ESP32 Usable Pins
#ifndef LED_BUILTIN
#  define LED_BUILTIN     2
#endif
#define PIN_AUX_4_SP     4 // SPARE Strapping
#define MCP_CS           5
#define ONE_WIRE_BUS     13
#define BTN2_PIN         14 // Button 2
#define TFT_CS           15
#define TFT_RST          16
#define PIN_AUX_17       17
#define SPI_CLK          18 // MCP_CLK, DISP_SCK
#define SPI_MISO         19 // MCP_DOUT, DISP_MISO
#define LED_PIN          21
#define PIN_AUX_22       22 // SPARE
#define SPI_MOSI         23 // MCP_DIN, DISP_MOSI
#define PUMP_RLY_PIN     25
#define AUX_HEAT_RLY_PIN 26
#define AUX_RLY_PIN      27
#define TFT_LED          32
#define TFT_DC           33
#define PIN_AUX_34       34
#define BTN1_PIN         35
#define PIN_AUX_36_I     36 // SPARE Input Only
#define PIN_AUX_39_I     39 // SPARE Input Only

// MCP3204 ADC Ports
#define ADC_LIGHT      0
#define ADC_AIR        1
#define ADC_POOL       2
#define ADC_AUX        3

#define MAX_JSON_CONFIG_ARDUINOJSON_BUFFER_SIZE_POOL 1024
#define CONFIG_PATH "/pool/config.json"
#define LOOP_DAT_DLY          (5*1E3)

// Colors
#define TFT_WHITE 0xFFFF
#define TFT_BLACK 0x0000
#define TFT_LT_BLUE 0xBDFF
#define TFT_RED 0xC1C7
#define TFT_PINK 0xFAEB
#define TFT_GREEN 0x9733
#define TFT_YELLOW 0xEF94