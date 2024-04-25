
#include "esp32c3.h"

#include <Arduino.h>
#include <zbhci.h>
#include <hci_display.h>
#include "esp_task_wdt.h"

#include <OneButton.h>
#include <GyverOLED.h>
#include <BH1750.h>
#include <DallasTemperature.h>
#include "vt_bme280"
#include "pir.h"
#include <MQUnifiedsensor.h>
#include "Adafruit_BME680.h"

//-- pictures
#include "images/danger_image.h"
#include "images/empty_image.h"
#include "images/motion_image.h"
#include "images/toxic_image.h"
#include "images/zigbee_logo.h"
#include "images/zigbee_connected.h"
#include "images/zigbee_disconnected.h"
#include "images/zigbee_image.h"

#if MIKE_BOARD_NUMBER == 1
//-- GPIO where the DS18B20 is connected to
const int oneWireBus = ONE_WIRE_BUS_GPIO;
//-- Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
//-- Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature ds18b20(&oneWire);

DeviceAddress deviceAddress[8];
#endif

const uint8_t au8ManufacturerName[] = {13,'M','I','K','E','.','E','S','P','3','2','-','C','3'};
ts_DstAddr sDstAddr;
QueueHandle_t msg_queue;
uint8_t numDevices = 0;

GyverOLED<SSD1306_128x64> oled(SSD1306_OLED_ADDRESS);
int screenNumber = 1;
bool oledTestScreen = false;

uint8_t ledState = 0;
uint8_t netState = 0;
uint8_t autoReport = 0;

BH1750 bh1750(BH1750_ADDR);
bool BH1750Inited = false;
float bh1750Illum = 0;

using namespace vt;
bme280_t bme280;
float
  bme280Humid = 0,
  bme280Pres = 0,
  bme280Temp = 0;
bool BME280Inited = false;

//-- PIR Occupancy Sensor
bool pirDeviceMotionState = false;

MQUnifiedsensor MQ135(
  CHIP_NAME,
  GAS_VOLTAGE_RESOLUTION,
  GAS_ADC_BIT_RESOLUTION,
  MQ135_SENSOR_GPIO,
  GAS_SENSOR_TYPE
);
bool
  mq135Inited = false,
  mq135CO2Exceeded1000 = false;

float
  mq135CalcR0,
  mq135Analog,
  mq135GasCO,
  mq135GasAlcohol,
  mq135GasCO2,
  mq135GasToluen,
  mq135GasNH4,
  mq135GasAceton;

float
  hcsr04DistanceCm,
  hcsr04DistanceInch;


float
  bme680Humid = 0,
  bme680Pres = 0,
  bme680Temp = 0,
  bme680Gas = 0,
  bme680Alt = 0;
bool BME680Inited = false;
Adafruit_BME680 bme680;

//-- param #1: Input pin for the button
//-- param #2: Button is active LOW
//-- param #3: Enable internal pull-up resistor
OneButton btn = OneButton(EXTERNAL_BUTTON_GPIO, true, true); 

void drawSSD1306()
{
  if(oledTestScreen) {
    #if MIKE_BOARD_NUMBER == 2
      mq135GasCO2 = 1083;
      mq135CO2Exceeded1000 = true;
    #endif
    pirDeviceMotionState = true;
  }
  
  //oled.clear();
  int16_t t = 0, X = 0, Y = 0;
  
  char temp_data_str[100] = {0};
    
  X = 0; Y = 0;
  oled.setCursorXY(X, Y);
  sprintf(temp_data_str, "Board #%d", MIKE_BOARD_NUMBER);
  oled.print(temp_data_str);  
  
  #if MIKE_BOARD_NUMBER == 2
    //-- MQ135 gas sensor
    //-- the first row on yellow
    oled.home();
    oled.rect(0, 0, 77, 7, OLED_CLEAR);
    sprintf(temp_data_str, "CO2=%d", (int)mq135GasCO2);
    oled.print(temp_data_str);
  
    //-- HC-SR04 distance sensor
    //-- the second row on yellow
    oled.rect(0, 8, 77, 15, OLED_CLEAR);
    oled.setCursorXY(0, 8);
    sprintf(temp_data_str, "D=%.2f", hcsr04DistanceCm);
    oled.print(temp_data_str);
  #endif

  oled.update();
  
  #if MIKE_BOARD_NUMBER == 1
    //-- DS18B20 temperature sensors
    numDevices = ds18b20.getDS18Count();
    
    char ds18b20_str[200] = {0};
    char ds18b20_int[10] = {0};
    int idx = 0;
    bool newLine = false;
    for(int i = 0; i < numDevices; ++i) {
      t = ds18b20.getTempCByIndex(i) * 100;
      X = 0;
      Y = 16 + idx * 8;
    
      if(strlen(ds18b20_str) > 0) {
        strcat(ds18b20_str, ", ");
      }
      sprintf(ds18b20_int, "T%d=%.2f", i, ((float)t)/100);
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
      //oled.update();
    }
    
    if(!newLine) {
      //-- set cursor to new line
      idx++;
    }
 
   	//-- BH1750 illuminance sensor
    oled.rect(0, 32, 127, 39, OLED_CLEAR);
    oled.setCursorXY(0, 32);
		sprintf(temp_data_str, "I=%.2f", bh1750Illum);
		oled.print(temp_data_str);

    //-- BME280 sensor
    oled.rect(0, 40, 127, 47, OLED_CLEAR);
    oled.setCursorXY(0, 40);
    sprintf(temp_data_str, "%.2f, %.2f%%, %.0f", bme280Temp, bme280Humid, bme280Pres);
    oled.print(temp_data_str);
    
    //-- BME680 sensor
  	oled.rect(0, 48, 127, 55, OLED_CLEAR);
  	oled.setCursorXY(0, 48);
  	sprintf(temp_data_str, "%.2f, %.2f%%, %.0f", bme680Temp, bme680Humid, bme680Pres);
  	oled.print(temp_data_str);

  	//-- BME680: gas & altitude in the last row
  	oled.rect(0, 56, 127, 63, OLED_CLEAR);
    oled.setCursorXY(0, 56);
   	sprintf(temp_data_str, "%.2fkOm, %.2fm", bme680Gas, bme680Alt);
  	oled.print(temp_data_str);
  #endif
  //-- draw icons in the header
  if(pirDeviceMotionState) {
    oled.drawBitmap(94, 0, motion_image, 16, 16);
  } else {
    oled.rect(94, 0, 108, 15, OLED_CLEAR);
  }
  
  #if MIKE_BOARD_NUMBER == 2
    if(mq135CO2Exceeded1000) {
      //oled.drawBitmap(78, 0, toxic_image, 16, 16);
      oled.drawBitmap(78, 0, danger_image, 16, 16);
      //-- header: CO2 big text
      /*
      oled.setCursor(0, 0);
      oled.setScale(2);
      oled.print("CO");
      oled.setScale(1);
      oled.setCursorXY(24, 6);
      oled.print("2");
      */
    } else {
      //-- clear header row: from 0 to the end of "danger_image" icon
      oled.rect(78, 0, 88, 15, OLED_CLEAR);
    }
  #endif
  //-- redraw Zigbee icons
  oled.drawBitmap(112, 48, zigbee_image, 16, 16);
  oled.drawBitmap(112, 0, zigbee_connected, 16, 16);
  
  oled.update();
}

void drawSSD1306_02()
{
  int16_t X = 0, Y = 0;
  char temp_data_str[100] = {0};

  X = 0; Y = 0;
  oled.setCursorXY(X, Y);
  sprintf(temp_data_str, "Board #%d", MIKE_BOARD_NUMBER);
  oled.print(temp_data_str);  

  #if MIKE_BOARD_NUMBER == 1
    X = 0; Y = 16;
    oled.setCursorXY(X, Y);
    sprintf(temp_data_str, "DS18B20:pin%d,ep1-%d", ONE_WIRE_BUS_GPIO, numDevices);
    oled.print(temp_data_str);
    
    X = 0; Y = 24;
    oled.setCursorXY(X, Y);
    sprintf(temp_data_str, "BME280:sda%d,scl%d,ep%d", ESP32_SDA_PIN, ESP32_SCL_PIN, BME280_BH1750_EP);
    oled.print(temp_data_str);
    
    X = 0; Y = 32;
    oled.setCursorXY(X, Y);
    sprintf(temp_data_str, "BH1750:sda%d,scl%d,ep%d", ESP32_SDA_PIN, ESP32_SCL_PIN, BME280_BH1750_EP);
    oled.print(temp_data_str);
    
    X = 0; Y = 40;
    oled.setCursorXY(X, Y);
    sprintf(temp_data_str, "BME680:sda%d,scl%d,ep%d", ESP32_SDA_PIN, ESP32_SCL_PIN, BME680_EP);
    oled.print(temp_data_str);

    X = 0; Y = 48;
    oled.setCursorXY(X, Y);
    sprintf(temp_data_str, "HCSR501:pin%d,led%d", PIR_SENSOR_GPIO, PIR_SENSOR_LED_GPIO);
    oled.print(temp_data_str);
  #endif

  #if MIKE_BOARD_NUMBER == 2
    X = 0; Y = 16;
    oled.setCursorXY(X, Y);
    sprintf(temp_data_str, "MQ135:pin%d", MQ135_SENSOR_GPIO);
    oled.print(temp_data_str);
    
    X = 0; Y = 24;
    oled.setCursorXY(X, Y);
    sprintf(temp_data_str, "HCSR04:trig%d, echo%d", HCSR04_TRIGGER_GPIO, HCSR04_ECHO_GPIO);
    oled.print(temp_data_str);

    X = 0; Y = 32;
    oled.setCursorXY(X, Y);
    sprintf(temp_data_str, "RCWL0516:pin%d,led%d", PIR_SENSOR_GPIO, PIR_SENSOR_LED_GPIO);
    oled.print(temp_data_str);
  #endif

  oled.update();

  delay(10000);
  screenNumber = 1;
  Serial.print(FONT_COLOR_STRONG_GREEN);
  Serial.println("Switch to default screen...");
  Serial.print(STYLE_COLOR_RESET);
  oled.clear();
}

void initLEDsOnBoard()
{
  pinMode(RED_LED_ONBOARD_PIN, OUTPUT);
  digitalWrite(RED_LED_ONBOARD_PIN, HIGH);
  delay(500);

  pinMode(BLUE_LED_ONBOARD_PIN, OUTPUT);
  digitalWrite(BLUE_LED_ONBOARD_PIN, LOW);
}

void initSSD1306()
{
  //Serial.printf("SSD #1\n");
  oled.init(ESP32_SDA_PIN, ESP32_SCL_PIN);
  //Serial.printf("SSD #2\n");
  //-- set I2C speed
  Wire.setClock(ESP32_I2C_CLOCK_SPEED);
  //Serial.printf("SSD #3\n");
  
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
  //oled.drawBitmap(112, 48, zigbee_image, 16, 16);
  //oled.drawBitmap(112, 0, zigbee_connected, 16, 16);
}

#if MIKE_BOARD_NUMBER == 1
void initDS18B20()
{
  ds18b20.begin();
  /*
  Serial.print("DS18B20: parasite power is: ");
  if(ds18b20.isParasitePowerMode()) {
    Serial.print(FONT_COLOR_STRONG_GREEN);
    Serial.println("ON");
  } else {
    Serial.print(FONT_COLOR_STRONG_RED);
    Serial.println("OFF");
  }
  Serial.print(STYLE_COLOR_RESET);
  */
}

void initBH1750()
{
  if(bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_ADDR, &Wire)) {
    Serial.print(FONT_COLOR_GREEN);
    Serial.println(F("BH1750 initialized!"));
    Serial.print(STYLE_COLOR_RESET);
    BH1750Inited = true;
  } else {
    Serial.print(FONT_COLOR_RED);
    Serial.println(F("BH1750 initialization error..."));
    Serial.print(STYLE_COLOR_RESET);
  }
}

void getBH1750(void *pvParameters)
{
  while(1)
  {
    if(BH1750Inited) {
      bh1750Illum = bh1750.readLightLevel();
      //Serial.printf("BH1750: Illuminance = %.2f lux in Endpoint #%d\n", bh1750Illum, 666);
    }
    delay(5000);
  }
}

void initBME280()
{
  BME280Inited = (bool)bme280.begin();
  if(BME280Inited) {
    Serial.print(FONT_COLOR_GREEN);
    Serial.println(F("BME280 initialized!"));
    Serial.print(STYLE_COLOR_RESET);
  } else {
    Serial.print(FONT_COLOR_RED);
    Serial.println(F("BME280 initialization error..."));
    Serial.print(STYLE_COLOR_RESET);
  }
}
void getBME280(void *pvParameters)
{
  while(1)
  {
    if(BME280Inited) {
      bme280Temp = bme280.read_temperature_c();
      bme280Humid = bme280.read_humidity();
      bme280Pres = bme280.read_pressure();
    }
    delay(5000);
  }
}

void initBME680()
{
  BME680Inited = (bool)bme680.begin();
  if(BME680Inited) {
    Serial.print(FONT_COLOR_GREEN);
    Serial.println(F("BME680 initialized!"));
    Serial.print(STYLE_COLOR_RESET);

    // Set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150); // 320*C for 150 ms

  } else {
    Serial.print(FONT_COLOR_RED);
    Serial.println(F("BME680 initialization error..."));
    Serial.print(STYLE_COLOR_RESET);
  }
}
void getBME680(void *pvParameters)
{
  while(1)
  {
    if(BME680Inited) {
      if(!bme680.performReading()) {
				Serial.println("Failed to perform reading BME680!");
      } else {
      	bme680Humid = bme680.humidity;
  			bme680Pres  = bme680.pressure / 100.0;
  			bme680Temp  = bme680.temperature;
  			bme680Gas   = bme680.gas_resistance / 1000.0;
  			bme680Alt   = bme680.readAltitude(BME680_SEA_LEVEL_PRESSURE);
      }
    }
    delay(5000);
  }
}
#endif

#if MIKE_BOARD_NUMBER == 2
void initHCSR04()
{
  pinMode(HCSR04_TRIGGER_GPIO, OUTPUT); // Sets the trigPin as an Output
  pinMode(HCSR04_ECHO_GPIO, INPUT); // Sets the echoPin as an Input
}
void getHCSR04(void *pvParameters)
{
  while(1) {
    //-- Clears the HCSR04_TRIGGER_GPIO
    digitalWrite(HCSR04_TRIGGER_GPIO, LOW);
    delayMicroseconds(2);
    //-- Sets the HCSR04_TRIGGER_GPIO on HIGH state for 10 micro seconds
    digitalWrite(HCSR04_TRIGGER_GPIO, HIGH);
    delayMicroseconds(10);
    digitalWrite(HCSR04_TRIGGER_GPIO, LOW);
    
    //-- Reads the echoPin, returns the sound wave travel time in microseconds
    long hcsr04Duration = pulseIn(HCSR04_ECHO_GPIO, HIGH);
    
    //-- Calculate the distance
    hcsr04DistanceCm = hcsr04Duration * HCSR04_SOUND_SPEED/2;
    //-- Convert to inches
    hcsr04DistanceInch = hcsr04DistanceCm * HCSR04_CM_TO_INCH;
    
    delay(5000);
  }
}
#endif

void useSSD1306(void *pvParameters)
{
  while(1)
  {
    if(screenNumber == 2) {
      //-- switch to the second screen
      drawSSD1306_02();
    } else {
      //-- switch to default screen
      drawSSD1306();
    }
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
  int LED_PIN_COLOR = BLUE_LED_ONBOARD_PIN;
  //-- Blue
  if(type == 1) {
    LED_PIN_COLOR = BLUE_LED_ONBOARD_PIN;
  }
  //-- Red
  if(type == 2) {
    LED_PIN_COLOR = RED_LED_ONBOARD_PIN;
  }
  //-- light ON
  digitalWrite(LED_PIN_COLOR, true);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  //-- light OFF
  digitalWrite(LED_PIN_COLOR, false);
}

#if MIKE_BOARD_NUMBER == 1
//-- function to print DS18B20 address
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
#endif

void zb_sendReport(uint8_t u8SrcEp, uint16_t u16ClusterID, uint8_t *pu8Data)
{
  //-- void zbhci_ZclSendReportCmd()
  zbhci_ZclSendReportCmd(
    ZCL_CMD_DST_ADDR_MODE,       //-- uint8_t u8DstAddrMode
    sDstAddr,                    //-- ts_DstAddr sDstAddr
    u8SrcEp,                     //-- uint8_t u8SrcEp
    1,                           //-- uint8_t u8DstEp
    ZCL_CMD_DISABLE_DEFAULT_RSP, //-- uint8_t u8DisableDefaultRsp
    ZCL_CMD_DIRECTION,           //-- uint8_t u8Direction
    u16ClusterID,                //-- uint16_t u16ClusterID
    ZCL_MEASURED_VALUE_ATTR_ID,  //-- uint16_t u16AttrID
    ZCL_DATA_TYPE_DATA16,        //-- uint8_t u8DataType
    ZCL_CMD_DATA_LENGTH,         //-- uint8_t u8DataLen
    pu8Data                      //-- uint8_t *pu8Data
  );
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

float gasVolume = 400.0;

void updateAttributes(int flag)
{
  /*
  if(flag == 1) {
    Serial.printf("DS18B20: updating (getData)...\n");
  }
  if(flag == 2) {
    Serial.printf("DS18B20: updating (reportTask)...\n");
  }
  */
  //-- int type: 1 = Blue, 2 = Red
  BlinkLedOnBoard(1);

  #if MIKE_BOARD_NUMBER == 1
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
      Serial.printf("DS18B20[");
      
      Serial.print(FONT_COLOR_STRONG_YELLOW);
      Serial.printf("%d", i);
      Serial.print(STYLE_COLOR_RESET);
      
      Serial.printf("] (GPIO=");
      Serial.print(FONT_COLOR_STRONG_CYAN);
      Serial.printf("%d", ONE_WIRE_BUS_GPIO);
      Serial.print(STYLE_COLOR_RESET);
      Serial.printf("): T = ");
      
      Serial.print(FONT_COLOR_STRONG_YELLOW);
      Serial.printf("%.2f", ((float)t)/100);
      Serial.print(STYLE_COLOR_RESET);
    
      Serial.printf(" ºC in Endpoint #");
      
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
      /*
      Serial.print(", resolution: ");
    
      Serial.print(FONT_COLOR_STRONG_YELLOW);
      ds18b20.setResolution(deviceAddress[i], TEMPERATURE_PRECISION);
      Serial.print(ds18b20.getResolution(deviceAddress[i]), DEC);
      Serial.print(STYLE_COLOR_RESET);
      */
    
      Serial.print("\n");
      
      //-- DS18B20 Temperature Sensor
      zb_sendReport(srcEpNum, ZCL_CLUSTER_MS_TEMPERATURE_MEASUREMENT, (uint8_t *)&t);
    }
    
    //-- BME280 Temperature Sensor
    int16_t tmpTemp = bme280Temp * 100;
    zb_sendReport(BME280_BH1750_EP, ZCL_CLUSTER_MS_TEMPERATURE_MEASUREMENT, (uint8_t *)&tmpTemp);
    
    //-- BME280 Humidity Sensor
    int16_t tmpHumid = bme280Humid * 100;
    zb_sendReport(BME280_BH1750_EP, ZCL_CLUSTER_MS_RELATIVE_HUMIDITY, (uint8_t *)&tmpHumid);
    
    //-- BME280 Pressure Sensor
    int16_t tmpPres = bme280Pres * 1;
    zb_sendReport(BME280_BH1750_EP, ZCL_CLUSTER_MS_PRESSURE_MEASUREMENT, (uint8_t *)&tmpPres);
    
    
    //-- BME680 Temperature Sensor
    int16_t tmpTemp2 = bme680Temp * 100;
    zb_sendReport(BME680_EP, ZCL_CLUSTER_MS_TEMPERATURE_MEASUREMENT, (uint8_t *)&tmpTemp2);
    
    //-- BME680 Humidity Sensor
    int16_t tmpHumid2 = bme680Humid * 100;
    zb_sendReport(BME680_EP, ZCL_CLUSTER_MS_RELATIVE_HUMIDITY, (uint8_t *)&tmpHumid2);
    
    //-- BME680 Pressure Sensor
    int16_t tmpPres2 = bme680Pres * 1;
    zb_sendReport(BME680_EP, ZCL_CLUSTER_MS_PRESSURE_MEASUREMENT, (uint8_t *)&tmpPres2);
 
    //-- BH1750 Illuminance Sensor
    //-- the real "lux" value is multiplied by 100 for correct conversion in the external Zigbee2MQTT converter
    //-- loot at MIKE.ESP32-C3.js: MIKE => just fixed output format: "XXX" => "(XXX / 100).toFixed(2)"
    int16_t tmpIllum = bh1750Illum * 100;
    zb_sendReport(BME280_BH1750_EP, ZCL_CLUSTER_MS_ILLUMINANCE_MEASUREMENT, (uint8_t *)&tmpIllum);
  #endif

  //-- PIR Occupancy Sensor
  pirDeviceMotionState = getPIRState_asLib();
  int16_t tmpOccup = pirDeviceMotionState * 1;
  zb_sendReport(PIR_EP, ZCL_CLUSTER_MS_OCCUPANCY_SENSING, (uint8_t *)&tmpOccup);

  #if MIKE_BOARD_NUMBER == 2
    if(mq135Inited) {
       //-- MQ135 Gas Sensor
      int16_t tmpPPM = (int)mq135GasCO2;
      zb_sendReport(BME280_BH1750_EP, ZCL_CLUSTER_MS_CO2, (uint8_t *)&tmpPPM);
    }
  #endif

  #if MIKE_BOARD_NUMBER == 1
    Serial.printf("BME280 (SDA=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", ESP32_SDA_PIN);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", SCL=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", ESP32_SCL_PIN);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf("): T = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", bme280Temp);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(" ºC, H = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", bme280Humid);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(" %%, P = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.0f", bme280Pres);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(" hPa in Endpoint #");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%d\n", BME280_BH1750_EP);
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.printf("BME680 (SDA=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", ESP32_SDA_PIN);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", SCL=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", ESP32_SCL_PIN);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf("): T = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", bme680Temp);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(" ºC, H = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", bme680Humid);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(" %%, P = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.0f", bme680Pres);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(" hPa, Gas = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", bme680Gas);
    Serial.print(STYLE_COLOR_RESET);
    //Serial.printf(" KOhms, Altitude = ");
    Serial.printf(" KΩ, Altitude = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
  	Serial.printf("%.2f", bme680Alt);
  	Serial.print(STYLE_COLOR_RESET);
  	Serial.printf(" m in Endpoint #");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%d\n", BME680_EP);
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.printf("BH1750 (SDA=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", ESP32_SDA_PIN);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", SCL=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", ESP32_SCL_PIN);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf("): I = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", bh1750Illum);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(" lux in Endpoint #");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%d\n", BME280_BH1750_EP);
    Serial.print(STYLE_COLOR_RESET);
  #endif

  #if MIKE_BOARD_NUMBER == 1
    Serial.printf("HC-SR501 (GPIO=");
  #else
    Serial.printf("RCWL-0516 (GPIO=");
  #endif
  Serial.print(FONT_COLOR_STRONG_CYAN);
  Serial.printf("%d", PIR_SENSOR_GPIO);
  Serial.print(STYLE_COLOR_RESET);
  #if MIKE_BOARD_NUMBER == 1
    Serial.printf(", LED=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", PIR_SENSOR_LED_GPIO);
    Serial.print(STYLE_COLOR_RESET);
  #endif
  Serial.printf("): State = ");
  if(pirDeviceMotionState) {
    Serial.print(FONT_COLOR_STRONG_RED);
  } else {
    Serial.print(FONT_COLOR_STRONG_GREEN);
  }
  Serial.printf("%s", pirDeviceMotionState ? "ON" : "OFF");
  Serial.print(STYLE_COLOR_RESET);
  Serial.printf(" in Endpoint #");
  Serial.print(FONT_COLOR_STRONG_YELLOW);
  Serial.printf("%d\n", PIR_EP);
  Serial.print(STYLE_COLOR_RESET);
  
  #if MIKE_BOARD_NUMBER == 2
    Serial.printf("MQ135 (GPIO=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", MQ135_SENSOR_GPIO);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf("): Analog = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", mq135Analog);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", CO = ");
    if(mq135CO2Exceeded1000) {
      Serial.print(FONT_COLOR_STRONG_RED);
    } else {
      Serial.print(FONT_COLOR_STRONG_YELLOW);
    }
    Serial.printf("%.2f", mq135GasCO);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", Alcohol = ");
    if(mq135CO2Exceeded1000) {
      Serial.print(FONT_COLOR_STRONG_RED);
    } else {
      Serial.print(FONT_COLOR_STRONG_YELLOW);
    }
    Serial.printf("%.2f", mq135GasAlcohol);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", CO2 = ");
    if(mq135CO2Exceeded1000) {
      Serial.print(FONT_COLOR_STRONG_RED);
    } else {
      Serial.print(FONT_COLOR_STRONG_YELLOW);
    }
    Serial.printf("%d", (int)mq135GasCO2);
    Serial.print(STYLE_COLOR_RESET);
    Serial.print(" ppm, Toluen = ");
    if(mq135CO2Exceeded1000) {
      Serial.print(FONT_COLOR_STRONG_RED);
    } else {
      Serial.print(FONT_COLOR_STRONG_YELLOW);
    }
    Serial.printf("%.2f", mq135GasToluen);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", NH4 = ");
    if(mq135CO2Exceeded1000) {
      Serial.print(FONT_COLOR_STRONG_RED);
    } else {
      Serial.print(FONT_COLOR_STRONG_YELLOW);
    }
    Serial.printf("%.2f", mq135GasNH4);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", Aceton = ");
    if(mq135CO2Exceeded1000) {
      Serial.print(FONT_COLOR_STRONG_RED);
    } else {
      Serial.print(FONT_COLOR_STRONG_YELLOW);
    }
    Serial.printf("%.2f\n", mq135GasAceton);
    Serial.print(STYLE_COLOR_RESET);
    
    
    Serial.printf("HC-SR04 (TRIG=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", HCSR04_TRIGGER_GPIO);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", ECHO=");
    Serial.print(FONT_COLOR_STRONG_CYAN);
    Serial.printf("%d", HCSR04_ECHO_GPIO);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf("): Distance (cm) = ");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%.2f", hcsr04DistanceCm);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(" in Endpoint #");
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.printf("%d\n", HCSR04_EP);
    Serial.print(STYLE_COLOR_RESET);
  #endif
  //-- empty row between blocks
  Serial.printf("\n");
}

void getData()
{
  static unsigned long last_time = 0;
    
  if(millis() - last_time > (TICK_TIMEOUT * 1000)) {
    updateAttributes(1);
    btn.tick();
    last_time = millis();
  }
}


//-- Single click: update attributes
void handleClick(void)
{
  /*
  if(netState == 1) {
    getData();
    delay(100);
  } else {
    Serial.print(FONT_COLOR_STRONG_RED);
    Serial.println("Not joined the zigbee network");
    Serial.print(STYLE_COLOR_RESET);
  }
  */
  /*
  Serial.print(FONT_COLOR_STRONG_RED);
  Serial.printf("Soft reset pressed: %s will restart!\n", CHIP_NAME);
  Serial.print(STYLE_COLOR_RESET);
  delay(1000);
  
  //-- soft reset, ESP will restart!
  esp_restart();

  digitalWrite(BLUE_LED_ONBOARD_PIN, true);
  delay(2000);
  digitalWrite(BLUE_LED_ONBOARD_PIN, false);
  */

  //-- switch to the sesond screen
  if(screenNumber == 1) {
    screenNumber = 2;
    
    Serial.print(FONT_COLOR_STRONG_RED);
    Serial.println("Switch to the sesond screen...");
    Serial.print(STYLE_COLOR_RESET);
  } else {
    screenNumber = 1;

    Serial.print(FONT_COLOR_STRONG_GREEN);
    Serial.println("Switch to default screen...");
    Serial.print(STYLE_COLOR_RESET);
  }
  oled.clear();
}

void reportTask(void *pvParameters)
{
  while(autoReport)
  {
    //Serial.printf("DS18B20: reportTask\n");
    //digitalWrite(BLUE_LED_ONBOARD_PIN, true);
    //digitalWrite(RED_LED_ONBOARD_PIN, true);
    
    updateAttributes(2);

    //vTaskDelay(100 / portTICK_PERIOD_MS);
    //digitalWrite(BLUE_LED_ONBOARD_PIN, false);
    //digitalWrite(RED_LED_ONBOARD_PIN, false);
    vTaskDelay(REPORTING_PERIOD * 1000 / portTICK_PERIOD_MS);
  }
}

//-- Double click: stop/start reporting
void handleDoubleClick(void)
{
  /*
  Serial.printf("handleDoubleClick: autoReport=%d\n", autoReport);
  if(autoReport == 0) {
    autoReport = 1;
    xTaskCreatePinnedToCore(reportTask, "report", 4096, NULL, 6, NULL, ARDUINO_RUNNING_CORE);
  } else {
    autoReport = 0;
    Serial.print(FONT_COLOR_STRONG_RED);
    Serial.println("Stop report task");
    Serial.print(STYLE_COLOR_RESET);
    delay(1000);
  }
  */
}

//-- Long click: join/leave the zigbee network
void handleLongPress()
{
  Serial.printf("handleLongPress: netState=%d\n", netState);
  if(netState == 0) {
    Serial.print(FONT_COLOR_STRONG_YELLOW);
    Serial.println("Joining the zigbee network");
    Serial.print(STYLE_COLOR_RESET);
    zbhci_BdbCommissionSteer();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  } else if (netState == 1) {
    Serial.print(FONT_COLOR_STRONG_RED);
    Serial.println("Leave the zigbee network");
    Serial.print(STYLE_COLOR_RESET);
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
            digitalWrite(BLUE_LED_ONBOARD_PIN, LOW);
            ledState = 0;
          } else if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 1) {
            digitalWrite(BLUE_LED_ONBOARD_PIN, HIGH);
            ledState = 1;
          } else if (sHciMsg.uPayload.sZclOnOffCmdRcvPayload.u8CmdId == 2) {
            ledState = !ledState;
            digitalWrite(BLUE_LED_ONBOARD_PIN, ledState);
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

#if MIKE_BOARD_NUMBER == 2
void initMQ135Uni()
{
  //Init the serial port communication - to debug the library
  //Serial.begin(9600); //Init serial port

  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  
  /*****************************  MQ Init ********************************************/ 
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/ 
  MQ135.init(); 
  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ135.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float mq135CalcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    mq135CalcR0 += MQ135.calibrate(MQ135_RATIO_CLEANAIR);
    Serial.print(".");
  }
  MQ135.setR0(mq135CalcR0/10);
  
  Serial.print(FONT_COLOR_STRONG_GREEN);
  Serial.printf(" DONE!\n");
  Serial.print(STYLE_COLOR_RESET);
  
  mq135Inited = true;
  if(isinf(mq135CalcR0)) {
    Serial.print(FONT_COLOR_STRONG_RED);
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    Serial.print(STYLE_COLOR_RESET);
    mq135Inited = false;
  }
  if(mq135CalcR0 == 0) {
    Serial.print(FONT_COLOR_STRONG_RED);
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    Serial.print(STYLE_COLOR_RESET);
    mq135Inited = false;
  }
  /*****************************  MQ CAlibration ********************************************/ 
  //Serial.println("** Values from MQ-135 ****");
  //Serial.println("|    CO   |  Alcohol |   CO2  |  Toluen  |  NH4  |  Aceton  |");  
  if(mq135Inited) {
    Serial.print(FONT_COLOR_GREEN);
    Serial.println(F("MQ135 initialized!"));
    Serial.print(STYLE_COLOR_RESET);
  } else {
    Serial.print(FONT_COLOR_RED);
    Serial.println(F("MQ135 initialization error..."));
    Serial.print(STYLE_COLOR_RESET);
  }
}

float checkPPM(float _a, float _b)
{
  //Serial.print(FONT_COLOR_STRONG_GREEN);
  float
    _sensor_volt = MQ135.getVoltage(),
    _R0 = MQ135.getR0(),
    _RS_Calc = 0,
    _ratio = 0,
    _PPM = 0;

  _RS_Calc = ((GAS_VOLTAGE_RESOLUTION * GAS_ADC_BIT_RESOLUTION) / _sensor_volt) - GAS_ADC_BIT_RESOLUTION;
  if(_RS_Calc < 0) {
    _RS_Calc = 0;
  }
  _ratio = _RS_Calc / _R0;
  if(_ratio <= 0) {
    //-- must not be negative or zero, otherwise it will receive the value INF
    _ratio = 0.1;
  }
  //Serial.printf("_ratio = %.2f, _b = %.2f\n", _ratio, _b);
  _PPM= _a * pow(_ratio, _b);
  if(_PPM < 0) {
    _PPM = 0;
  }
  
  //Serial.printf("((GAS_VOLTAGE_RESOLUTION:%.2f*GAS_ADC_BIT_RESOLUTION:%.2f)/_sensor_volt:%.2f)-GAS_ADC_BIT_RESOLUTION:%.2f -> this->_R0:%.2f => _PPM:%.2f\n", GAS_VOLTAGE_RESOLUTION, GAS_ADC_BIT_RESOLUTION, _sensor_volt, GAS_ADC_BIT_RESOLUTION, _R0, _PPM);
  //Serial.print(STYLE_COLOR_RESET);

  //Serial.print(FONT_COLOR_STRONG_RED);
  //Serial.printf("PPM = %.2f!\n", _PPM);
  //Serial.print(STYLE_COLOR_RESET);

  return _PPM;
}

void getMQ135Uni(void *pvParameters)
{
  float co2PPM = 0;

  while(mq135Inited) {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    
    //-- Sensor will read PPM concentration using the model, a and b values must be specified as parameters
    co2PPM = checkPPM(110.47, -2.862) * MQ135_ESP32_ADC + 400;
    if(co2PPM > 1000) {
      //Serial.print(FONT_COLOR_STRONG_RED);
      //Serial.print(STYLE_COLOR_BOLD);
      //Serial.printf("Warning: High concentration of CO2 (%.2f) detected!\n", co2PPM);
      //Serial.print(STYLE_COLOR_RESET);

      mq135CO2Exceeded1000 = true;
    } else {
      //Serial.print(FONT_COLOR_STRONG_GREEN);
      //Serial.printf("Info: CO2 concentration is normal: %.2f...\n", co2PPM);
      //Serial.print(STYLE_COLOR_RESET);

      mq135CO2Exceeded1000 = false;
    }
    
    mq135Analog = analogRead(MQ135_SENSOR_GPIO) * MQ135_ESP32_ADC;
    //mq135Analog = analogRead(MQ135_SENSOR_GPIO);

    if(mq135CO2Exceeded1000) {
      mq135GasCO = checkPPM(605.18, -3.937) * MQ135_ESP32_ADC;
    } else {
      MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
      mq135GasCO = MQ135.readSensor() * MQ135_ESP32_ADC; // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    }
    
    if(mq135CO2Exceeded1000) {
      mq135GasAlcohol = checkPPM(77.255, -3.18) * MQ135_ESP32_ADC;
    } else {
      MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
      mq135GasAlcohol = MQ135.readSensor() * MQ135_ESP32_ADC; // SSensor will read PPM concentration using the model, a and b values set previously or from the setup
    }

    if(mq135CO2Exceeded1000) {
      mq135GasCO2 = (int)(checkPPM(110.47, -2.862) * MQ135_ESP32_ADC + 400);
    } else {
      MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
      mq135GasCO2 = (int)(MQ135.readSensor() * MQ135_ESP32_ADC + 400); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    }
    if(mq135GasCO2 > 20479) {
      mq135GasCO2 = 20479;
    }
    
    if(oledTestScreen) {
      mq135GasCO2 = 1083;
      mq135CO2Exceeded1000 = true;
    }
    
    if(mq135CO2Exceeded1000) {
      mq135GasToluen = checkPPM(44.947, -3.445) * MQ135_ESP32_ADC;
    } else {
      MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
      mq135GasToluen = MQ135.readSensor() * MQ135_ESP32_ADC; // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    }
    
    if(mq135CO2Exceeded1000) {
      mq135GasNH4 = checkPPM(102.2, -2.473) * MQ135_ESP32_ADC;
    } else {
      MQ135.setA(102.2); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
      mq135GasNH4 = MQ135.readSensor() * MQ135_ESP32_ADC; // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    }
    
    if(mq135CO2Exceeded1000) {
      mq135GasAceton = checkPPM(34.668, -3.369) * MQ135_ESP32_ADC;
    } else {
      MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
      mq135GasAceton = MQ135.readSensor() * MQ135_ESP32_ADC; // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    }
    // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
    /*
    Motivation:
    We have added 400 PPM because when the library is calibrated it assumes the current state of the
    air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.
    https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
    */

    
    
    /*
      Exponential regression:
    GAS      | a      | b
    CO       | 605.18 | -3.937  
    Alcohol  | 77.255 | -3.18 
    CO2      | 110.47 | -2.862
    Toluen   | 44.947 | -3.445
    NH4      | 102.2  | -2.473
    Aceton   | 34.668 | -3.369
    */
    
    delay(5000); //Sampling frequency
  }
}
#endif



void setup()
{
  sDstAddr.u16DstAddr = 0x0000;
  
  Serial.begin(115200);
  delay(10);
  Serial.printf("\n");

  btn.attachClick(handleClick);
  btn.attachDoubleClick(handleDoubleClick);
  //btn.setPressTicks(3000);
  btn.attachLongPressStart(handleLongPress);

  msg_queue = xQueueCreate(10, sizeof(ts_HciMsg));
  zbhci_Init(msg_queue);

  initLEDsOnBoard();
  initSSD1306();
  #if MIKE_BOARD_NUMBER == 1
    initDS18B20();
    initBH1750();
    initBME280();
    initBME680();
  #endif
  initPIR_asLib();
  #if MIKE_BOARD_NUMBER == 2
    initMQ135Uni();
    initHCSR04();
  #endif

  xTaskCreatePinnedToCore(zbhciTask, "zbhci", 4096, NULL, 5, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(useSSD1306, "useSSD1306", 4096, NULL, 6, NULL, ARDUINO_RUNNING_CORE);
  #if MIKE_BOARD_NUMBER == 1
    xTaskCreatePinnedToCore(getBH1750, "getBH1750", 4096, NULL, 7, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(getBME280, "getBME280", 4096, NULL, 8, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(getBME680, "getBME680", 4096, NULL, 9, NULL, ARDUINO_RUNNING_CORE);
  #endif
  xTaskCreatePinnedToCore(getPIR_asLib, "getPIR_asLib", 4096, NULL, 10, NULL, ARDUINO_RUNNING_CORE);
  #if MIKE_BOARD_NUMBER == 2
    xTaskCreatePinnedToCore(getMQ135Uni, "getMQ135Uni", 4096, NULL, 11, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(getHCSR04, "getHCSR04", 4096, NULL, 12, NULL, ARDUINO_RUNNING_CORE);
  #endif

  // zbhci_BdbFactoryReset();
  delay(100);
  zbhci_NetworkStateReq();
}


void loop()
{
  static unsigned long last_time = 0;
  btn.tick();
  getData();
}
