
#include "esp32c3.h"

#include <Arduino.h>
#include <zbhci.h>
#include <hci_display.h>
#include "esp_task_wdt.h"

#include <OneButton.h>
#include <GyverOLED.h>
#include <BH1750.h>
#include <DallasTemperature.h>

#include "zigbee_logo.h"
#include "zigbee_connected.h"
#include "zigbee_disconnected.h"
#include "zigbee_image.h"

//-- GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     
//-- Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
//-- Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature ds18b20(&oneWire);
DeviceAddress deviceAddress[8];

const uint8_t au8ManufacturerName[] = {13,'M','I','K','E','.','E','S','P','3','2','-','C','3'};
ts_DstAddr sDstAddr;
QueueHandle_t msg_queue;
uint8_t numDevices = 0;

GyverOLED<SSD1306_128x64> oled(SSD1306_OLED_ADDRESS);

uint8_t ledState = 0;
uint8_t netState = 0;
uint8_t autoReport = 0;

BH1750 bh1750(BH1750_ADDR);
bool BH1750Inited = false;
float lux = 0;

int16_t gIllum = 0, gHumid = 0, gPres = 0, gTemp = 0;

/**
 * Initialize a new OneButton instance for a button
 * connected to digital pin 4 and GND, which is active low
 * and uses the internal pull-up resistor.
 */
//-- param #1: Input pin for the button
//-- param #2: Button is active LOW
//-- param #3: Enable internal pull-up resistor
OneButton btn = OneButton(CONFIG_USR_BUTTON_PIN, true, true); 

void drawSSD1306()
{
  numDevices = ds18b20.getDS18Count();

  int16_t t = 0, X = 0, Y = 0;
  char temp_data_str[100] = {0};
  char illum_data_str[100] = {0};

  //-- header
  oled.setCursorXY(0, 0);
  oled.setScale(1);
  oled.print("TLSR8258/ESP32C3");

  char ds18b20_str[200] = {0};
  char ds18b20_int[10] = {0};
    
  //-- DS18B20 temperature sensors
  int idx = 0;
  bool newLine = false;
  for(int i = 0; i < numDevices; ++i) {
    t = ds18b20.getTempCByIndex(i) * 100;
    X = 0;
    Y = 16 + idx * 12;

    if(strlen(ds18b20_str) > 0) {
    	strcat(ds18b20_str, ",");
    }
    sprintf(ds18b20_int, "[%d]=%.2f", i, ((float)t)/100);
    strcat(ds18b20_str, ds18b20_int);
    
    oled.setCursorXY(X, Y);
    oled.print(ds18b20_str);
    if(i > 0 && (i+1) % 2 == 0) {
    	idx++;
    	strcpy(ds18b20_str, "");
    	newLine = true;
    } else {
    	newLine = false;
    }
    oled.update();
  }

  if(!newLine) {
  	//-- set cursor to new line
  	idx++;
  }
  //-- BMX280 sensor
  X = 0;
  Y = 16 + idx * 12;
  oled.setCursorXY(X, Y);
  sprintf(temp_data_str, "%.2f, %.2f%%, %.0f", ((float)gTemp/100), ((float)gHumid/100), (float)gPres);
  oled.print(temp_data_str);

  //-- set cursor to new line
  idx++;
  //-- BH1750 illuminance sensor
  X = 0;
  Y = 16 + idx * 12;
  oled.setCursorXY(X, Y);
  //Serial.printf("lux on led: %.2f\n", lux);
  sprintf(illum_data_str, "bh1750: %.2f", lux);
  oled.print(illum_data_str);
  oled.update();
}

void initSSD1306()
{
  oled.init(ESP32_SDA_PIN, ESP32_SCL_PIN);
  // настройка скорости I2C
  Wire.setClock(ESP32_I2C_CLOCK_SPEED);
  
  //-- logo at start
  oled.clear();
  oled.drawBitmap(0, 0, zigbee_logo, 128, 64);
  oled.update();
  delay(5000);
 
  //-- icons
  oled.clear();
  //ssd1306_draw_bitmap(ssd1306_dev, 112, 48, zigbee_image, 16, 16);
  //ssd1306_draw_bitmap(ssd1306_dev, 112, 0, zigbee_connected, 16, 16);
  //ssd1306_draw_bitmap(ssd1306_dev, 112, 0, zigbee_disconnected, 16, 16);
  //ssd1306_draw_bitmap(ssd1306_dev, 0, 16, zigbee, 128, 32);
  oled.drawBitmap(112, 48, zigbee_image, 16, 16);
  oled.drawBitmap(112, 0, zigbee_connected, 16, 16);
}

void initBH1750()
{
  if(bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_ADDR, &Wire)) {
    Serial.println(F("BH1750 initialized!"));
    BH1750Inited = true;
  } else {
    Serial.println(F("Error initialising BH1750..."));
  }
}

void getBH1750(void *pvParameters)
{
  while(1)
  {
    if(BH1750Inited) {
      lux = bh1750.readLightLevel();
      //Serial.printf("BH1750: Illuminance = %.2f lux in Endpoint #%d\n", lux, 666);
    }
    delay(5000);
  }
}

void useSSD1306(void *pvParameters)
{
  while(1)
  {
    drawSSD1306();
    delay(5000);
    
    /*
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
    */
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
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) {
      Serial.print("0");
    }
    Serial.print(deviceAddress[i], HEX);
  }
}


void zb_sendReport(uint8_t u8SrcEp, uint16_t u16ClusterID, uint8_t *pu8Data)
{
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
  zbhci_ZclSendReportCmd(
  	ZCL_CMD_DST_ADDR_MODE,
  	sDstAddr,
  	u8SrcEp,
  	1,
  	ZCL_CMD_DISABLE_DEFAULT_RSP,
  	ZCL_CMD_DIRECTION,
  	u16ClusterID,
  	ZCL_MEASURED_VALUE_ATTR_ID,
  	ZCL_DATA_TYPE_DATA16,
  	ZCL_CMD_DATA_LENGTH,
  	//(uint8_t *)&pu8Data
  	pu8Data
  );
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void updateDs18b20Data(int flag)
{
  Serial.printf("***\n");
  /*
  if(flag == 1) {
    Serial.printf("DS18B20: updating (getDs18b20Data)...\n");
  }
  if(flag == 2) {
    Serial.printf("DS18B20: updating (reportTask)...\n");
  }
  */
  //-- int type: 1 = Blue, 2 = Red
  BlinkLedOnBoard(1);

  ds18b20.requestTemperatures();
  //uint8_t d = ds18b20.getDeviceCount();
  //Serial.printf("TOTAL DEVICES: %d\n", d);
  numDevices = ds18b20.getDS18Count();

  int16_t t = 0;
  int srcEpNum = 0;
  int dstEpNum = 1;
  
  for(int i = 0; i < numDevices; ++i) {
    t = ds18b20.getTempCByIndex(i) * 100;
    srcEpNum = i + 1;
    Serial.printf("DS18B20 (");
    
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%d", i);
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.printf("): Temperature = ");
    
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", ((float)t)/100);
    Serial.print(STYLE_COLOR_RESET);

    Serial.printf("ºC in Endpoint #");
    
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%d", srcEpNum);
    Serial.print(STYLE_COLOR_RESET);
    
    if(!ds18b20.getAddress(deviceAddress[i], i)) {
      Serial.printf(", ");
      Serial.print(FONT_COLOR_STRONG_RED);
      Serial.printf("unable to find address!");
      Serial.print(STYLE_COLOR_RESET);
    } else {
      Serial.printf(", addr: ");
      Serial.print(FONT_COLOR_STRONG_YELLOW);
      printAddress(deviceAddress[i]);
      Serial.print(STYLE_COLOR_RESET);
    }
    Serial.print(", resolution: ");

    Serial.print(FONT_COLOR_STRONG_YELLOW);
    ds18b20.setResolution(deviceAddress[i], TEMPERATURE_PRECISION);
    Serial.print(ds18b20.getResolution(deviceAddress[i]), DEC);
    Serial.print(STYLE_COLOR_RESET);

    Serial.println();
    
  	//-- DS18B20 Temperature Sensor
  	zb_sendReport(srcEpNum, ZCL_CLUSTER_MS_TEMPERATURE_MEASUREMENT, (uint8_t *)&t);
  }

  //-- BMX280 Temperature Sensor
  gTemp = t;
  zb_sendReport(6, ZCL_CLUSTER_MS_TEMPERATURE_MEASUREMENT, (uint8_t *)&gTemp);
  
  //-- BMX280 Humidity Sensor
  gHumid = t;
  zb_sendReport(6, ZCL_CLUSTER_MS_RELATIVE_HUMIDITY, (uint8_t *)&gHumid);
  
  //-- BMX280 Pressure Sensor
  gPres = t;
  zb_sendReport(6, ZCL_CLUSTER_MS_PRESSURE_MEASUREMENT, (uint8_t *)&gPres);
  
  //-- BH1750 Illuminance Sensor
  //-- the real "lux" value is multiplied by 100 for correct conversion in the external Zigbee2MQTT converter
  //-- loot at MIKE.ESP32-C3.js: MIKE => just fixed output format: "XXX" => "(XXX / 100).toFixed(2)"
  gIllum = lux * 100;
  zb_sendReport(6, ZCL_CLUSTER_MS_ILLUMINANCE_MEASUREMENT, (uint8_t *)&gIllum);

  Serial.printf("BMX280|BH1750: ");
  
  Serial.print(FONT_COLOR_STRONG_CYAN);
  Serial.printf("%.2f", ((float)gTemp/100));
  Serial.print(STYLE_COLOR_RESET);

  Serial.printf("ºC, ");
  
  Serial.print(FONT_COLOR_STRONG_CYAN);
  Serial.printf("%.2f", ((float)gHumid/100));
  Serial.print(STYLE_COLOR_RESET);

  Serial.printf("%%, ");
  
  Serial.print(FONT_COLOR_STRONG_CYAN);
  Serial.printf("%.0f", (float)gPres);
  Serial.print(STYLE_COLOR_RESET);

  Serial.printf("hPa | ");
  
  Serial.print(FONT_COLOR_STRONG_CYAN);
  Serial.printf("%.2f", ((float)gIllum/100));
  Serial.print(STYLE_COLOR_RESET);

  Serial.printf("lux\n");
  //Serial.printf("BMX280|BH1750: %.2fºC, %.2f%%, %.0fhPa | %.2flux\n", ((float)gTemp/100), ((float)gHumid/100), (float)gPres, ((float)gIllum/100));

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



void handleClick(void)
{
  if(netState == 1) {
    getDs18b20Data();
    delay(100);
  } else {
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
  if(autoReport == 0) {
    autoReport = 1;
    xTaskCreatePinnedToCore(reportTask, "report", 4096, NULL, 6, NULL, ARDUINO_RUNNING_CORE);
  } else {
    autoReport = 0;
    Serial.println("Stop report task");
    delay(1000);
  }
}

void handleLongPress()
{
  Serial.printf("handleLongPress: netState=%d\n", netState);
  if(netState == 0) {
    Serial.println("Joining the zigbee network");
    zbhci_BdbCommissionSteer();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  } else if (netState == 1) {
    Serial.println("leave the zigbee network");
    zbhci_BdbFactoryReset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    netState = 0;
  }
}


void zbhciTask(void *pvParameters)
{
  ts_HciMsg sHciMsg;

  while(1)
  {
    bzero(&sHciMsg, sizeof(sHciMsg));
    if(xQueueReceive(msg_queue, &sHciMsg, portMAX_DELAY))
    {
      switch(sHciMsg.u16MsgType)
      {
        case ZBHCI_CMD_ACKNOWLEDGE:
          //Serial.printf(">> ZBHCI_CMD_ACKNOWLEDGE\n");
          //displayAcknowledg(&sHciMsg.uPayload.sAckPayload);
          break;
        case ZBHCI_CMD_NETWORK_STATE_RSP:
          //Serial.printf(">> ZBHCI_CMD_NETWORK_STATE_RSP\n");
          if(sHciMsg.uPayload.sNetworkStateRspPayloasd.u16NwkAddr == 0x0000) {
            zbhci_BdbFactoryReset();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            zbhci_NetworkStateReq();
          } else if (sHciMsg.uPayload.sNetworkStateRspPayloasd.u16NwkAddr != 0xFFFF) {
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
          if(sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 0) {
            digitalWrite(CONFIG_BLUE_LIGHT_PIN, LOW);
            ledState = 0;
          } else if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 1) {
            digitalWrite(CONFIG_BLUE_LIGHT_PIN, HIGH);
            ledState = 1;
          } else if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 2) {
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

  ds18b20.begin();
  Serial.print("Parasite power is: ");
  if(ds18b20.isParasitePowerMode()) {
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

  initSSD1306();
  initBH1750();

  xTaskCreatePinnedToCore(zbhciTask, "zbhci", 4096, NULL, 5, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(useSSD1306, "useSSD1306", 4096, NULL, 10, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(getBH1750, "getBH1750", 4096, NULL, 15, NULL, ARDUINO_RUNNING_CORE);

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
