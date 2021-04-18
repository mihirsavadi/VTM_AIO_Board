#include <Arduino.h>

#include "aioMainMCU.hpp"

void setup() {

  Serial.begin(9600);

  aioMainMCU mainMCU;
  
  while(1)
  {
    String error;
    if (mainMCU.getError(error))
    {
      Serial.println(error);
    }
    else
    {
      Serial.println("NO ERROR!");
    }

    mainMCU.readInputs();
    mainMCU.logAllInputs();
  }
}

//dont use this. Put all looped code in the while(1) block in void setup for
// tighter control of scope and to just keep code clean.
void loop() {}