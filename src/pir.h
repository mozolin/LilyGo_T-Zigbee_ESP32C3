
#include "colors.h"
#include "RCWL0516.h"

//-- DIRECT
int
  pir_state = LOW,
  pir_val = 0,
  pirShowLog2 = false;

//-- !!! TIMER (may fire an error) !!!
#define pir_timeSeconds 10
//-- Timer: Auxiliary variables
unsigned long pir_now = millis();
unsigned long pir_lastTrigger = 0;
boolean pir_startTimer = false;
boolean pir_motion = false;

//-- LIBRARY
RCWL0516 pir(PIR_SENSOR_GPIO);
unsigned long
  pirNow = millis(),
  pirLastTrgTime = 0;
bool
  pirMotionState = false,
  pirLastMotionState = false,
  pirShowLog = false;


//------------------------------------------
//
//   DIRECT
//
//------------------------------------------
void initPIR_asDirect()
{
  pinMode(PIR_SENSOR_LED_GPIO, OUTPUT);      // initalize LED as an output
  pinMode(PIR_SENSOR_GPIO, INPUT);
}
bool getPIRState_asDirect()
{
  return (bool)pir_state;
}
void logPIR_asDirect()
{
  if(pirShowLog2) {
    Serial.printf(">> DIRECT: pir_val=");
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.printf("%d", pir_val);
    Serial.print(STYLE_COLOR_RESET);
    Serial.printf(", pir_state=");
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.printf("%d", pir_state);
    Serial.print(STYLE_COLOR_RESET);
    Serial.println("");
  }
}
void getPIR_asDirect(void *pvParameters)
{
  while(1)
  {
    pir_val = digitalRead(PIR_SENSOR_GPIO);   // read sensor value
    if(pir_val == HIGH) {           // check if the sensor is HIGH
      digitalWrite(PIR_SENSOR_LED_GPIO, HIGH);   // turn LED ON
      if(pir_state == LOW) {
        Serial.print(FONT_COLOR_STRONG_RED);
        Serial.println("DIRECT: Motion detected!"); 
        Serial.print(STYLE_COLOR_RESET);
        pir_state = HIGH;       // update variable state to HIGH
      }
    } else {
      digitalWrite(PIR_SENSOR_LED_GPIO, LOW); // turn LED OFF
      if(pir_state == HIGH){
        Serial.print(FONT_COLOR_STRONG_GREEN);
        Serial.println("DIRECT: Motion stopped!");
        Serial.print(STYLE_COLOR_RESET);
        pir_state = LOW;       // update variable state to LOW
      }
    }
    
    logPIR_asDirect();

    delay(PIR_TASK_DELAY * 1000);
    //vTaskDelay(PIR_TASK_DELAY * 1000 / portTICK_PERIOD_MS);

    
  }
}    


//------------------------------------------
//
//   TIMER
//
//------------------------------------------
//-- Checks if pir_motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement_asTimer()
{
  int led_state = digitalRead(PIR_SENSOR_LED_GPIO);
  int sensor_state = digitalRead(SOME_PIR_SENSOR_GPIO);
  
  Serial.print(FONT_COLOR_STRONG_WHITE);
  Serial.printf("TIMER: detectsMovement_asTimer | PIR_SENSOR_LED_GPIO=%d, sensor=%d...\n", led_state, sensor_state);
  Serial.print(STYLE_COLOR_RESET);

  if(led_state == HIGH) {
    digitalWrite(PIR_SENSOR_LED_GPIO, HIGH);
    pir_startTimer = true;
    pir_lastTrigger = millis();
  }
}
void initPIR_asTimer()
{
  //-- PIR Motion Sensor mode INPUT_PULLUP
  pinMode(SOME_PIR_SENSOR_GPIO, INPUT_PULLUP);
  //-- Set PIR_SENSOR_GPIO pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(SOME_PIR_SENSOR_GPIO), detectsMovement_asTimer, RISING);
  //-- Set LED to LOW
  pinMode(PIR_SENSOR_LED_GPIO, OUTPUT);
  digitalWrite(PIR_SENSOR_LED_GPIO, LOW);
}
void getPIR_asTimer(void *pvParameters)
{
  while(1)
  {
    //-- Current time
    pir_now = millis();
    
    int l = digitalRead(PIR_SENSOR_LED_GPIO);
    Serial.printf("%d: getPIR_asTimer: LED=%d, motion=%d\n", pir_now, l, pir_motion);
    
    if((digitalRead(PIR_SENSOR_LED_GPIO) == HIGH) && (pir_motion == false)) {
      Serial.print(FONT_COLOR_STRONG_RED);
      Serial.printf("TIMER: Motion detected!\n");
      Serial.print(STYLE_COLOR_RESET);
      pir_motion = true;
    }
    //-- Turn off the LED after the number of seconds defined in the pir_timeSeconds variable
    if(pir_startTimer && (pir_now - pir_lastTrigger > (pir_timeSeconds*1000))) {
      Serial.print(FONT_COLOR_STRONG_GREEN);
      Serial.printf("TIMER: Motion stopped...\n");
      Serial.print(STYLE_COLOR_RESET);
      digitalWrite(PIR_SENSOR_LED_GPIO, LOW);
      pir_startTimer = false;
      pir_motion = false;
    }
    delay(PIR_TASK_DELAY * 1000);
    //vTaskDelay(PIR_TASK_DELAY * 1000 / portTICK_PERIOD_MS);
  }
}    

//------------------------------------------
//
//   LIBRARY
//
//------------------------------------------
void initPIR_asLib()
{
  pir.activate();
  pirNow = millis();
  #if MIKE_BOARD_NUMBER == 1
    pinMode(PIR_SENSOR_LED_GPIO, OUTPUT);      // initalize LED as an output
  #endif
}
bool getPIRState_asLib()
{
  pirLastTrgTime = pir.lastTriggerTime();
  //-- skip first iteration
  if(pirLastTrgTime == 0) {
    return false;
  }
  //-- if not zero trigger counter
  return (pir.triggeredWithin(PIR_WAIT_MOTION * 1000) != 0);
}
void logPIR_asLib()
{
  if(pirShowLog) {
    Serial.print(">> PIR: trigTime = ");
    
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.print(pirLastTrgTime);
    Serial.print("|");
    Serial.print(pirNow);
    Serial.print("|");
    Serial.print(pirLastTrgTime - pirNow);
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.print(", trigCount = ");
    
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.print(pir.trigCount());
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.printf(", triggeredWithin %ds = ", PIR_WAIT_MOTION);
    
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.print(pir.triggeredWithin(PIR_WAIT_MOTION * 1000));
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.print(", currentTriggers = ");
    
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.print(pir.currentTriggers());
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.print(", MOTION = ");
    
    if(pirMotionState) {
      Serial.print(FONT_COLOR_STRONG_RED);
    } else {
      Serial.print(FONT_COLOR_STRONG_WHITE);
    }
    Serial.println(pirMotionState);
    Serial.print(STYLE_COLOR_RESET);
  }
}
void getPIR_asLib(void *pvParameters)
{
  pirLastTrgTime = 0;
  pirLastMotionState = false;
  while(1)
  {
    //-- get motion state
    pirMotionState = getPIRState_asLib();
    unsigned long pirNowTmp = millis();
    
    //Serial.printf("PIR: Motion state = %s\n", pirMotionState ? "ON" : "OFF");
    //-- state has been change from 0 to 1
    if(pirLastMotionState == false && pirMotionState == true) {
      /*
      Serial.print(FONT_COLOR_STRONG_RED);
      Serial.printf("LIBRARY: Motion detected! %d ms\n", (pirNowTmp - pirLastTrgTime));
      Serial.print(STYLE_COLOR_RESET);
      */
      pirLastMotionState = pirMotionState;
      
      #if MIKE_BOARD_NUMBER == 1
        digitalWrite(PIR_SENSOR_LED_GPIO, HIGH);
      #endif
      //Serial.printf("LED on %d: %d\n", PIR_SENSOR_LED_GPIO, HIGH);
    //-- state has been change from 1 to 0
    } else if(pirLastMotionState == true && pirMotionState == false) {
      /*
      Serial.print(FONT_COLOR_STRONG_GREEN);
      Serial.println("LIBRARY: Motion stopped...");
      Serial.print(STYLE_COLOR_RESET);
      */
      pirLastMotionState = pirMotionState;

      #if MIKE_BOARD_NUMBER == 1
        digitalWrite(PIR_SENSOR_LED_GPIO, LOW);
      #endif
      //Serial.printf("LED on %d: %d\n", PIR_SENSOR_LED_GPIO, LOW);
    }

    //-- show log in Serial terminal
    logPIR_asLib();
    
    delay(PIR_TASK_DELAY * 1000);
    //vTaskDelay(PIR_TASK_DELAY * 1000 / portTICK_PERIOD_MS);
  }
}    
