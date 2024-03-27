/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */

#include <Arduino.h>
#include <zbhci.h>
#include <hci_display.h>
#include <OneButton.h>
#include "esp_task_wdt.h"
#include <DallasTemperature.h>

#include <GyverOLED.h>
#define ESP32_SDA_PIN 8
#define ESP32_SCL_PIN 7
#define SSD1306_OLED_ADDRESS 0x3C
#define ESP32_I2C_CLOCK_SPEED 800000L   // макс. 800'000

#define CONFIG_ZIGBEE_MODULE_PIN 0 //-- red
#define CONFIG_USR_BUTTON_PIN 2
#define CONFIG_BLUE_LIGHT_PIN 3 //-- blue
#define REPORTING_PERIOD 10 //-- second(s)
#define TICK_TIMEOUT 5 //-- second(s)

//-- GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     
//-- Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
//-- Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensor(&oneWire);
DeviceAddress deviceAddress[8];

const uint8_t au8ManufacturerName[] = {13,'M','I','K','E','.','E','S','P','3','2','-','C','3'};
ts_DstAddr sDstAddr;
QueueHandle_t msg_queue;
uint8_t numDevices = 0;
#define TEMPERATURE_PRECISION 9

/**
 * Initialize a new OneButton instance for a button
 * connected to digital pin 4 and GND, which is active low
 * and uses the internal pull-up resistor.
 */
OneButton btn = OneButton(CONFIG_USR_BUTTON_PIN,     // Input pin for the button
                          true,  // Button is active LOW
                          true); // Enable internal pull-up resistor

void useSSD1306(void *pvParameters)
{
  GyverOLED<SSD1306_128x64> oled(SSD1306_OLED_ADDRESS);
  
  oled.init(ESP32_SDA_PIN, ESP32_SCL_PIN);
  
  // битмап создан в ImageProcessor https://github.com/AlexGyver/imageProcessor
  // с параметрами вывода vertical byte (OLED)
  const uint8_t bitmap_32x32[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xE0, 0xF0, 0x70, 0x70, 0x30, 0x30, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF0, 0x70, 0x30, 0x30, 0x20, 0x00, 0x00,
    0x00, 0x30, 0x78, 0xFC, 0x7F, 0x3F, 0x0F, 0x0F, 0x1F, 0x3C, 0x78, 0xF0, 0xE0, 0xC0, 0x80, 0x80, 0x80, 0x40, 0xE0, 0xF0, 0xF8, 0xFC, 0xFF, 0x7F, 0x33, 0x13, 0x1E, 0x1C, 0x1C, 0x0E, 0x07, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF9, 0xF7, 0xEF, 0x5F, 0x3F, 0x7F, 0xFE, 0xFD, 0xFB, 0xF1, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x1E, 0x33, 0x33, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x1F, 0x0E, 0x04, 0x00, 0x00, 0x00, 0x00,
  };
  // --------------------------
  // настройка скорости I2C
  Wire.setClock(ESP32_I2C_CLOCK_SPEED);

  while(1)
  {
    //-- !!!!!!!!!!!!!!!!!!!!!!!!! --
    //   oled.setScale(3); => do not use "3" for scaling, it causes the app's destroying!
    //-- !!!!!!!!!!!!!!!!!!!!!!!!! --

    // --------------------------
    oled.clear();   // очистить дисплей (или буфер)
    oled.update();  // обновить. Только для режима с буфером! OLED_BUFFER

    // --------------------------
    oled.home();            // курсор в 0,0
    oled.print("Hello!");   // печатай что угодно: числа, строки, float, как Serial!
    oled.update();
    delay(1000);

    // --------------------------
    oled.setCursor(5, 1);   // курсор в (пиксель X, строка Y)
    oled.setScale(2);
    oled.print("Hello!");
    oled.update();
    delay(1000);

    // --------------------------
    oled.setCursorXY(15, 30); // курсор в (пиксель X, пиксель Y)
    oled.setScale(2);
    //oled.invertText(true);    // инвертируй текст!
    oled.print("Привет!");
    oled.update();
    delay(1000);

    // --------------------------
    oled.clear();
    oled.home();
    oled.setScale(1);
    oled.invertText(false);
    oled.autoPrintln(true);   // автоматически переносить текст
    oled.print(F("Lorem ipsum dolor sit amet, лорем ипсум долор сит амет привет народ ё, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam"));
    oled.update();
    delay(1000);

    // --------------------------
    oled.home();
    oled.textMode(BUF_ADD);
    // BUF_ADD - наложить текст
    // BUF_SUBTRACT - вычесть текст
    // BUF_REPLACE - заменить (весь прямоугольник буквы)
    oled.home();
    oled.setScale(2);
    oled.print("KEK!");
    oled.update();
    delay(1000);

    // --------------------------
    // СЕРВИС
    //oled.setContrast(10);   // яркость 0..255
    //oled.setPower(true);    // true/false - включить/выключить дисплей
    //oled.flipH(true);       // true/false - отзеркалить по горизонтали
    //oled.flipV(true);       // true/false - отзеркалить по вертикали
    //oled.isEnd();           // возвращает true, если дисплей "кончился" - при побуквенном выводе

    // --------------------------
    oled.clear();
    oled.dot(0, 0);     // точка на x,y
    oled.dot(0, 1, 1);  // третий аргумент: 0 выкл пиксель, 1 вкл пиксель (по умолч)
    oled.line(5, 5, 10, 10);        // линия x0,y0,x1,y1
    //oled.line(5, 5, 10, 10, 0);   // пятый аргумент: 0 стереть, 1 нарисовать (по умолч)
    oled.fastLineH(0, 5, 10);       // горизонтальная линия (y, x1, x2)
    //oled.fastLineH(0, 5, 10, 0);  // четвёртый аргумент: 0 стереть, 1 нарисовать (по умолч)
    oled.fastLineV(0, 5, 10);       // аналогично верт. линия (x, y1, y2)
    oled.rect(20, 20, 30, 25);      // прямоугольник (x0,y0,x1,y1)
    oled.rect(5, 35, 35, 60, OLED_STROKE);      // прямоугольник (x0,y0,x1,y1)
    // параметры фигуры:
    // OLED_CLEAR - очистить
    // OLED_FILL - залить
    // OLED_STROKE - нарисовать рамку
    oled.roundRect(50, 5, 80, 25, OLED_STROKE);  // аналогично скруглённый прямоугольник
    oled.circle(60, 45, 15, OLED_STROKE);        // окружность с центром в (x,y, с радиусом)
    oled.circle(60, 45, 5, OLED_FILL);           // четвёртый аргумент: параметр фигуры

    // битмап
    oled.drawBitmap(90, 16, bitmap_32x32, 32, 32, BITMAP_NORMAL, BUF_ADD);
    //oled.drawBitmap(90, 16, bitmap_32x32, 32, 32);  // по умолч. нормал и BUF_ADD
    // x, y, имя, ширина, высота, BITMAP_NORMAL(0)/BITMAP_INVERT(1), BUF_ADD/BUF_SUBTRACT/BUF_REPLACE
    
    oled.update();
    delay(1000);
  }
}

void BlinkLedOnBoard(int type)
{
    int LED_PIN_COLOR = CONFIG_BLUE_LIGHT_PIN;
    //-- Blue
    if(type == 1) {
        LED_PIN_COLOR = CONFIG_BLUE_LIGHT_PIN;
    }
    //-- Red
    if(type == 2) {
        LED_PIN_COLOR = CONFIG_ZIGBEE_MODULE_PIN;
    }
    //-- light ON
    digitalWrite(LED_PIN_COLOR, true);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    //-- light OFF
    digitalWrite(LED_PIN_COLOR, false);
}

//-- function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}



void updateDs18b20Data(int flag)
{
    if(flag == 1) {
        Serial.printf("DS18B20: updating (getDs18b20Data)...\n");
	}
	if(flag == 2) {
		Serial.printf("DS18B20: updating (reportTask)...\n");
	}

    //-- int type: 1 = Blue, 2 = Red
    BlinkLedOnBoard(1);

	sensor.requestTemperatures();
  	//uint8_t d = sensor.getDeviceCount();
    numDevices = sensor.getDS18Count();

    int16_t t = 0;
    int epNum = 0;
    
    for(int i = 0; i < numDevices; ++i) {
        t = sensor.getTempCByIndex(i) * 100;
        epNum = i + 1;
        Serial.printf("DS18B20 (%d): Temperature = %.2fºC in Endpoint #%d", i, ((float)t)/100, epNum);
        if (!sensor.getAddress(deviceAddress[i], i)) {
            Serial.printf(", unable to find address!");
        } else {
            //oneWire.search(deviceAddress[i]);
            Serial.printf(", addr: ");
            printAddress(deviceAddress[i]);
        }
        Serial.print(", resolution: ");
        sensor.setResolution(deviceAddress[i], TEMPERATURE_PRECISION);
        Serial.print(sensor.getResolution(deviceAddress[i]), DEC);
        Serial.println();
        //-- void zbhci_ZclSendReportCmd(
        //--   uint8_t u8DstAddrMode,
        //--   ts_DstAddr sDstAddr,
        //--   uint8_t u8SrcEp,
        //--   uint8_t u8DstEp,
        //--   uint8_t u8DisableDefaultRsp,
        //--   uint8_t u8Direction,
        //--   uint16_t u16ClusterID,
        //--   uint16_t u16AttrID,
        //--   uint8_t u8DataType,
        //--   uint8_t u8DataLen,
        //--   uint8_t *pu8Data
        //-- )
        zbhci_ZclSendReportCmd(0x02, sDstAddr, epNum, 1, 0, 1, 0x0402, 0x0000, ZCL_DATA_TYPE_DATA16, 2, (uint8_t *)&t);
    }
    
}

void getDs18b20Data()
{
  static unsigned long last_time = 0;
    
  if(millis() - last_time > (TICK_TIMEOUT * 1000)) {
    updateDs18b20Data(1);
    btn.tick();
    last_time = millis();
  }
}


uint8_t ledState = 0;
uint8_t netState = 0;
uint8_t autoReport = 0;

void handleClick(void)
{
    if (netState == 1)
    {
        getDs18b20Data();
        delay(100);
    }
    else
    {
        Serial.println("Not joined the zigbee network");
    }

    digitalWrite(CONFIG_BLUE_LIGHT_PIN, true);
    delay(2000);
    digitalWrite(CONFIG_BLUE_LIGHT_PIN, false);
}

void reportTask(void *pvParameters)
{
    while (autoReport)
    {
        //Serial.printf("DS18B20: reportTask\n");
        //digitalWrite(CONFIG_BLUE_LIGHT_PIN, true);
        //digitalWrite(CONFIG_ZIGBEE_MODULE_PIN, true);
        
        updateDs18b20Data(2);

        //vTaskDelay(100 / portTICK_PERIOD_MS);
        //digitalWrite(CONFIG_BLUE_LIGHT_PIN, false);
        //digitalWrite(CONFIG_ZIGBEE_MODULE_PIN, false);
        vTaskDelay(REPORTING_PERIOD * 1000 / portTICK_PERIOD_MS);
    }
}

void handleDoubleClick(void)
{
    Serial.printf("handleDoubleClick: autoReport=%d\n", autoReport);
    if (autoReport == 0)
    {
        autoReport = 1;
        xTaskCreatePinnedToCore(reportTask, "report", 4096, NULL, 6, NULL, ARDUINO_RUNNING_CORE);
    }
    else
    {
        autoReport = 0;
        Serial.println("Stop report task");
        delay(1000);
    }
}

void handleLongPress()
{
    Serial.printf("handleLongPress: netState=%d\n", netState);
    if (netState == 0)
    {
        Serial.println("Joining the zigbee network");
        zbhci_BdbCommissionSteer();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else if (netState == 1)
    {
        Serial.println("leave the zigbee network");
        zbhci_BdbFactoryReset();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        netState = 0;
    }
}


void zbhciTask(void *pvParameters)
{
    ts_HciMsg sHciMsg;

    while (1)
    {
        bzero(&sHciMsg, sizeof(sHciMsg));
        if (xQueueReceive(msg_queue, &sHciMsg, portMAX_DELAY))
        {
            switch (sHciMsg.u16MsgType)
            {
                case ZBHCI_CMD_ACKNOWLEDGE:
                	//Serial.printf(">> ZBHCI_CMD_ACKNOWLEDGE\n");
                  //displayAcknowledg(&sHciMsg.uPayload.sAckPayload);
                break;

                case ZBHCI_CMD_NETWORK_STATE_RSP:
                	//Serial.printf(">> ZBHCI_CMD_NETWORK_STATE_RSP\n");
                    if (sHciMsg.uPayload.sNetworkStateRspPayloasd.u16NwkAddr == 0x0000)
                    {
                        zbhci_BdbFactoryReset();
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        zbhci_NetworkStateReq();
                    }
                    else if (sHciMsg.uPayload.sNetworkStateRspPayloasd.u16NwkAddr != 0xFFFF)
                    {
                        netState = 1;
                    }
                break;

                case ZBHCI_CMD_NETWORK_STATE_REPORT:
                	//Serial.printf(">> ZBHCI_CMD_NETWORK_STATE_REPORT\n");
                    netState = 1;
                    //zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, 0x0000, 0x0005, ZCL_DATA_TYPE_CHAR_STR, sizeof(au8ManufacturerName), (uint8_t *)&au8ManufacturerName);
                break;

                case ZBHCI_CMD_ZCL_ONOFF_CMD_RCV:
                	//Serial.printf(">> ZBHCI_CMD_ZCL_ONOFF_CMD_RCV\n");
                    /*
                    if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 0)
                    {
                        digitalWrite(CONFIG_BLUE_LIGHT_PIN, LOW);
                        ledState = 0;
                    }
                    else if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 1)
                    {
                        digitalWrite(CONFIG_BLUE_LIGHT_PIN, HIGH);
                        ledState = 1;
                    }
                    else if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 2)
                    {
                        ledState = !ledState;
                        digitalWrite(CONFIG_BLUE_LIGHT_PIN, ledState);
                    }
                    zbhci_ZclSendReportCmd(0x02, sDstAddr, 1, 1, 0, 1, 0x0006, 0x0000, ZCL_DATA_TYPE_BOOLEAN, 1, &ledState);
                    */
                break;

                default:
                    Serial.printf("u16MsgType %d\n", sHciMsg.u16MsgType);
                break;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    sDstAddr.u16DstAddr = 0x0000;
    
    Serial.begin(115200);
    delay(10);
    Serial.printf("Init\n");

    pinMode(CONFIG_ZIGBEE_MODULE_PIN, OUTPUT);
    digitalWrite(CONFIG_ZIGBEE_MODULE_PIN, HIGH);
    delay(500);

    pinMode(CONFIG_BLUE_LIGHT_PIN, OUTPUT);
    digitalWrite(CONFIG_BLUE_LIGHT_PIN, LOW);

    sensor.begin();
    Serial.print("Parasite power is: ");
    if (sensor.isParasitePowerMode()) {
        Serial.println("ON");
    } else {
        Serial.println("OFF");
    }

    btn.attachClick(handleClick);
    btn.attachDoubleClick(handleDoubleClick);
    //btn.setPressTicks(3000);
    btn.attachLongPressStart(handleLongPress);

    msg_queue = xQueueCreate(10, sizeof(ts_HciMsg));
    zbhci_Init(msg_queue);

    xTaskCreatePinnedToCore(zbhciTask, "zbhci", 4096, NULL, 5, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(useSSD1306, "useSSD1306", 4096, NULL, 10, NULL, ARDUINO_RUNNING_CORE);

    // zbhci_BdbFactoryReset();
    delay(100);
    zbhci_NetworkStateReq();
}

void loop()
{
    static unsigned long last_time = 0;
    btn.tick();
    getDs18b20Data();
}