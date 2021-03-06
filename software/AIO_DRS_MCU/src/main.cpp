
/* Mihir Savadi 18th April 2021 */

#include <Arduino.h>

#include "aioDRSMCU.hpp"

void setup() {

  //following macro block control in "aioDRSMCU.hpp" header 
  #if includeSerialPrints == true
    Serial.begin(9600);
  #endif

  aioDRSMCU drsMCU;

  // drsMCU.manualServoControl(false);
  
  while(1)
  {
    drsMCU.runDRSservo(true);
    drsMCU.runBBservo();
  }
}

//dont use this. Put all looped code in the while(1) block in void setup for
// tighter control of scope and to just keep code clean.
void loop() {}