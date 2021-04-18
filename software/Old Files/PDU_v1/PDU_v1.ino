/*
* AIO Board - DAQ MCU V1 code.
*
* Creation Date: 22 Dec 2019
* Last Update: 22 Dec 2019
*/

#include <EasyTransfer.h>

EasyTransfer ETin, ETout;

struct RECEIVE_DATA_STRUCTURE{
 //put your variable definitions here for the data you want to receive
 //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
 int16_t daqSend;
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t pduSend;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;

void setup() {
  Serial.begin(9600);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(rxdata), &Serial1);
  ETout.begin(details(txdata), &Serial1);

}

void loop() {
  //first, lets read our button and store it in our data structure
  txdata.pduSend = 1010;
 //then we will go ahead and send that data out
  ETout.sendData();

 //there's a loop here so that we run the recieve function more often then the
 //transmit function. This is important due to the slight differences in
 //the clock speed of different Arduinos. If we didn't do this, messages
 //would build up in the buffer and appear to cause a delay.

  for(int i=0; i<5; i++){
    //remember, you could use an if() here to check for new data, this time it's not needed.
    ETin.receiveData();
  }

  Serial.println("From DAQ: " + String(rxdata.daqSend));

  //delay for good measure
  delay(10);

}
