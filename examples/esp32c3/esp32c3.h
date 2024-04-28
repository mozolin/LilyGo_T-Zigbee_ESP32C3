
#include "colors.h"
#include "zb_clusters.h"


#define MIKE_BOARD_NUMBER              2

//— list of pins available on the ESP32-C3 + TLSR8258 board (aka LILYGO® T-Zigbee)
#define ESP32C3_GPIO_01                1 //-> MQ135_SENSOR_GPIO
#define ESP32C3_GPIO_02                2 //-> EXTERNAL_BUTTON_GPIO
#define ESP32C3_GPIO_04                4 //-> ONE_WIRE_BUS_GPIO / HCSR04_ECHO_GPIO
#define ESP32C3_GPIO_05                5 //-> PIR_SENSOR_LED_GPIO / HCSR04_TRIGGER_GPIO
#define ESP32C3_GPIO_06                6 //-> PIR_SENSOR_GPIO
#define ESP32C3_GPIO_07                7 //-> ESP32_SCL_PIN
#define ESP32C3_GPIO_08                8 //-> ESP32_SDA_PIN
#define ESP32C3_GPIO_20                20 //— Serial RX
#define ESP32C3_GPIO_21                21 //— Serial TX

//-- list of zigbee endpoints
#define ZIGBEE_ENDPOINT_01             1
#define ZIGBEE_ENDPOINT_04             4
#define ZIGBEE_ENDPOINT_05             5

//-- I2C: SDA/SCL pins & clock speed
#define ESP32_SDA_PIN                  ESP32C3_GPIO_08
#define ESP32_SCL_PIN                  ESP32C3_GPIO_07
#define ESP32_I2C_CLOCK_SPEED          800000L

#if MIKE_BOARD_NUMBER == 1
  #define ONE_WIRE_BUS_GPIO            ESP32C3_GPIO_04
#endif

//-- SSD1306: OLED address
#define SSD1306_OLED_ADDRESS           0x3C

//-- BH1750: address
#define BH1750_ADDR                    0x23

//-- Temperature precision
#define TEMPERATURE_PRECISION          9

//-- Red LED on board
#define RED_LED_ONBOARD_PIN            0
//-- External button pin
#define EXTERNAL_BUTTON_GPIO           ESP32C3_GPIO_02
//-- Blue LED on board
#define BLUE_LED_ONBOARD_PIN           3
//-- Reporting perio in seconds
#define REPORTING_PERIOD               10
//-- Tick timeout in seconds
#define TICK_TIMEOUT                   5


#define ZCL_MEASURED_VALUE_ATTR_ID     0x0000
#define ZCL_PRESENT_VALUE_ATTR_ID      0x0055
//-- 0x02 – with destination short address and endpoint
#define ZCL_CMD_DST_ADDR_MODE          0x02
//-- Direction of the command: 0 – Client to server, 1 – Server to client.
#define ZCL_CMD_DIRECTION              1
//-- Disable Default Rsp
#define ZCL_CMD_DISABLE_DEFAULT_RSP    0
//-- Data length
#define ZCL_CMD_DATA_LENGTH            2
//-- Endpoint for BME280/BH1750 sensors
#define BME280_BH1750_EP               ZIGBEE_ENDPOINT_04

//- PIN for output PIR sensor
#define PIR_SENSOR_GPIO                ESP32C3_GPIO_06
//- Endpoints for PIR sensors
#define PIR_EP                         ZIGBEE_ENDPOINT_01
//- PIN for LED indicator of PIR sensor
#define PIR_SENSOR_LED_GPIO            ESP32C3_GPIO_05
//-- task running delay in seconds
#define PIR_TASK_DELAY                 5
//-- motion waiting timeout in seconds
#define PIR_WAIT_MOTION                30
//-- ...reserved...
#define SOME_PIR_SENSOR_GPIO           33

//-- MQ135: pin
#define MQ135_SENSOR_GPIO              ESP32C3_GPIO_01
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
#define HCSR04_TRIGGER_GPIO            ESP32C3_GPIO_05
#define HCSR04_ECHO_GPIO               ESP32C3_GPIO_04
//-- define sound speed in cm/uS
#define HCSR04_SOUND_SPEED             0.034
#define HCSR04_CM_TO_INCH              0.393701
#define HCSR04_EP                      ZIGBEE_ENDPOINT_05


#define BME680_SEA_LEVEL_PRESSURE      (1013.25)
#define BME680_EP                      ZIGBEE_ENDPOINT_05
