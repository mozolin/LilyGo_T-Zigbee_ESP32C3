
#include "colors.h"

//-- I2C: SDA/SCL pins & clock speed
#define ESP32_SDA_PIN                 8
#define ESP32_SCL_PIN                 7
#define ESP32_I2C_CLOCK_SPEED         800000L

//-- SSD1306: OLED address
#define SSD1306_OLED_ADDRESS          0x3C

//-- BH1750: address
#define BH1750_ADDR                   0x23

//-- Temperature precision
#define TEMPERATURE_PRECISION         9

//-- Red LED on board
#define CONFIG_ZIGBEE_MODULE_PIN      0
//-- Button pin
#define CONFIG_USR_BUTTON_PIN         2
//-- Blue LED on board
#define CONFIG_BLUE_LIGHT_PIN         3
//-- Reporting perio in seconds
#define REPORTING_PERIOD              10
//-- Tick timeout in seconds
#define TICK_TIMEOUT                  5


#define ZCL_MEASURED_VALUE_ATTR_ID    0x0000
//-- 0x02 – with destination short address and endpoint
#define ZCL_CMD_DST_ADDR_MODE         0x02
//-- Direction of the command: 0 – Client to server, 1 – Server to client.
#define ZCL_CMD_DIRECTION             1
//-- Disable Default Rsp
#define ZCL_CMD_DISABLE_DEFAULT_RSP   0
//-- Data length
#define ZCL_CMD_DATA_LENGTH           2

//-- Zigbee: ID of clusters
#define ZCL_CLUSTER_MS_ILLUMINANCE_MEASUREMENT            0x0400
#define ZCL_CLUSTER_MS_ILLUMINANCE_LEVEL_SENSING_CONFIG   0x0401
#define ZCL_CLUSTER_MS_TEMPERATURE_MEASUREMENT            0x0402
#define ZCL_CLUSTER_MS_PRESSURE_MEASUREMENT               0x0403
#define ZCL_CLUSTER_MS_FLOW_MEASUREMENT                   0x0404
#define ZCL_CLUSTER_MS_RELATIVE_HUMIDITY                  0x0405
#define ZCL_CLUSTER_MS_OCCUPANCY_SENSING                  0x0406

