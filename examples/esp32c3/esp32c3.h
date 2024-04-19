
#include "colors.h"

//-- I2C: SDA/SCL pins & clock speed
#define ESP32_SDA_PIN                 8
#define ESP32_SCL_PIN                 7
#define ESP32_I2C_CLOCK_SPEED         800000L

#define ONE_WIRE_BUS_GPIO             4

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
//-- Endpint for BME280/BH1750 sensors
#define BME280_BH1750_EP              4

//-- using HCSR501 and/or RCWL0516 sensors
#define USE_FIRST_PIR_SENSOR           true
#define USE_SECOND_PIR_SENSOR          false
//- PIN for output PIR sensor: 1=HC-SR501, 6=RCWL-0516
#define HCSR501_SENSOR_GPIO            6
#define RCWL0516_SENSOR_GPIO           33
//- Endpoints for PIR sensors
#define HCSR501_EP                     1
#define RCWL0516_EP                    6
//-- reserved...
#define SOME_PIR_SENSOR_GPIO           33
//- PIN for LED indicator of PIR sensor
#define PIR_SENSOR_LED_GPIO            5
#define PIR2_SENSOR_LED_GPIO           5
//-- task running delay in seconds
#define PIR_TASK_DELAY                 5
#define PIR2_TASK_DELAY                5
//-- motion waiting timeout in seconds
#define PIR_WAIT_MOTION                30
#define PIR2_WAIT_MOTION               30

//-- MQ135: pin
#define MQ135_SENSOR_GPIO              1
#define CHIP_NAME                      "ESP32-C3"
#define GAS_VOLTAGE_RESOLUTION         5
//-- ??? For arduino UNO/MEGA/NANO ???
#define GAS_ADC_BIT_RESOLUTION         10
#define GAS_SENSOR_TYPE                "MQ-135"
//-- RS / R0 = 3.6 ppm  
#define MQ135_RATIO_CLEANAIR           3.6
//-- ratio: 1023/4096 (analog output Arduino / analog output ESP32)
#define MQ135_ESP32_ADC                0.249755859375

//-- HC-SR04: pins
#define HCSR04_TRIGGER_GPIO            1
#define HCSR04_ECHO_GPIO               6
//define sound speed in cm/uS
#define HCSR04_SOUND_SPEED             0.034
#define HCSR04_CM_TO_INCH              0.393701
#define HCSR04_EP                      5


//-- Zigbee: ID of clusters
#define ZCL_CLUSTER_MS_ILLUMINANCE_MEASUREMENT            0x0400
#define ZCL_CLUSTER_MS_ILLUMINANCE_LEVEL_SENSING_CONFIG   0x0401
#define ZCL_CLUSTER_MS_TEMPERATURE_MEASUREMENT            0x0402
#define ZCL_CLUSTER_MS_PRESSURE_MEASUREMENT               0x0403
#define ZCL_CLUSTER_MS_FLOW_MEASUREMENT                   0x0404
#define ZCL_CLUSTER_MS_RELATIVE_HUMIDITY                  0x0405
#define ZCL_CLUSTER_MS_OCCUPANCY_SENSING                  0x0406
#define ZCL_CLUSTER_MS_CO2                                0x040D


