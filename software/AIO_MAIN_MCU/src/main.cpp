#include <Arduino.h>

#include "aioMainMCU.hpp"

void setup() {

  Serial.begin(9600);

  aioMainMCU mainMCU;
  
  while(1)
  {
    mainMCU.readInputs();
    mainMCU.logAllInputs();
  }
}

//dont use this. Put all looped code in the while(1) block in void setup for
// tighter control of scope and to just keep code clean.
void loop() {}