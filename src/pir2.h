
#include "colors.h"
#if USE_FIRST_PIR_SENSOR
  //-- skip including
#else
  #include "RCWL0516.h"
#endif

//-- LIBRARY #2
RCWL0516 pir2(RCWL0516_SENSOR_GPIO);
unsigned long
  pir2Now = millis(),
  pir2LastTrgTime = 0;
bool
  pir2MotionState = false,
  pir2LastMotionState = false,
  pir2ShowLog = true;


//------------------------------------------
//
//   LIBRARY #2
//
//------------------------------------------
void initPIR2_asLib()
{
  pir2.activate();
  pir2Now = millis();
  pinMode(PIR2_SENSOR_LED_GPIO, OUTPUT);      // initalize LED as an output
}
bool getPIR2State_asLib()
{
  pir2LastTrgTime = pir2.lastTriggerTime();
  //-- skip first iteration
  if(pir2LastTrgTime == 0) {
    return false;
  }
  //-- if not zero trigger counter
  return (pir2.triggeredWithin(PIR2_WAIT_MOTION * 1000) != 0);
}
void logPIR2_asLib()
{
  if(pir2ShowLog) {
    Serial.print(">> PIR2: trigTime = ");
    
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.print(pir2LastTrgTime);
    Serial.print("|");
    Serial.print(pir2Now);
    Serial.print("|");
    Serial.print(pir2LastTrgTime - pir2Now);
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.print(", trigCount = ");
    
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.print(pir2.trigCount());
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.printf(", triggeredWithin %ds = ", PIR2_WAIT_MOTION);
    
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.print(pir2.triggeredWithin(PIR2_WAIT_MOTION * 1000));
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.print(", currentTriggers = ");
    
    Serial.print(FONT_COLOR_STRONG_WHITE);
    Serial.print(pir2.currentTriggers());
    Serial.print(STYLE_COLOR_RESET);
    
    Serial.print(", MOTION = ");
    
    if(pir2MotionState) {
      Serial.print(FONT_COLOR_STRONG_RED);
    } else {
      Serial.print(FONT_COLOR_STRONG_WHITE);
    }
    Serial.println(pir2MotionState);
    Serial.print(STYLE_COLOR_RESET);
  }
}
void getPIR2_asLib(void *pvParameters)
{
  pir2LastTrgTime = 0;
  pir2LastMotionState = false;
  while(1)
  {
    //-- get motion state
    pir2MotionState = getPIR2State_asLib();
    unsigned long pir2NowTmp = millis();
    
    //Serial.printf("PIR2: Motion state = %s\n", pir2MotionState ? "ON" : "OFF");
    //-- state has been change from 0 to 1
    if(pir2LastMotionState == false && pir2MotionState == true) {
    //if(pir2MotionState == true) {
      Serial.print(FONT_COLOR_STRONG_RED);
      Serial.printf("LIBRARY2: Motion detected! %d ms\n", (pir2NowTmp - pir2LastTrgTime));
      Serial.print(STYLE_COLOR_RESET);
      pir2LastMotionState = pir2MotionState;

      digitalWrite(PIR2_SENSOR_LED_GPIO, HIGH);
      //Serial.printf("LED on %d: %d\n", PIR2_SENSOR_LED_GPIO, HIGH);
    //-- state has been change from 1 to 0
    } else if(pir2LastMotionState == true && pir2MotionState == false) {
      Serial.print(FONT_COLOR_STRONG_GREEN);
      Serial.println("LIBRARY2: Motion stopped...");
      Serial.print(STYLE_COLOR_RESET);
      pir2LastMotionState = pir2MotionState;

      digitalWrite(PIR2_SENSOR_LED_GPIO, LOW);
      //Serial.printf("LED on %d: %d\n", PIR2_SENSOR_LED_GPIO, LOW);
    }

    //-- show log in Serial terminal
    logPIR2_asLib();
    
    delay(PIR2_TASK_DELAY * 1000);
    //vTaskDelay(PIR2_TASK_DELAY * 1000 / portTICK_PERIOD_MS);
  }
}    
