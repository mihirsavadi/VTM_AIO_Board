#include <Arduino.h>

#include "aioMainMCU.hpp"

#define includeSerialPrints false

void setup() {

  #if includeSerialPrints == true
    Serial.begin(9600);
  #endif

  aioMainMCU mainMCU;
  
  while(1)
  {
    #if includeSerialPrints == true
      String error;
      if (mainMCU.getError(error))
      {
        Serial.println(error);
      }
      else
      {
        Serial.println("NO ERROR!");
      }
    #endif

    mainMCU.readInputs();
    mainMCU.logAllInputs();
  }
}

//dont use this. Put all looped code in the while(1) block in void setup for
// tighter control of scope and to just keep code clean.
void loop() {}