/*This is an example of the EasyTransfer Library 2way communications.
The sketch is for the Arduino with a potentiometer attached to analog pin 0.
This other Arduino has the servo attached to pin 9.
Both have a putton attached to pin 12 and output a status using the LED on pin 13.
The idea is each arduino will read the status of the button attached to it, and send it
to the other Arduino, which will toggle it's LED based on the others button. The button
should connect pin 12 to ground when pushed.
And the Arduino with the potentiometer will send it's value to the one with the servo.
The servo will move to the position based on the potentiometer.
*/



#include <EasyTransfer.h>

//create two objects
EasyTransfer ETin, ETout;


struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t servovalDA;
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t servovalAA;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;


void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(rxdata), &Serial1);
  ETout.begin(details(txdata), &Serial1);

}

void loop(){

  //first, lets read our potentiometer and button and store it in our data structure
  txdata.servovalAA = analogRead(A16);

  Serial.print("servovalAA = " + String(txdata.servovalAA));

  //then we will go ahead and send that data out
  ETout.sendData();

 //there's a loop here so that we run the recieve function more often then the
 //transmit function. This is important due to the slight differences in
 //the clock speed of different Arduinos. If we didn't do this, messages
 //would build up in the buffer and appear to cause a delay.
  for(int i=0; i<5; i++){
    //remember, you could use an if() here to check for new data, this time it's not needed.
    ETin.receiveData();

    //all recieving variables here


  }

}
