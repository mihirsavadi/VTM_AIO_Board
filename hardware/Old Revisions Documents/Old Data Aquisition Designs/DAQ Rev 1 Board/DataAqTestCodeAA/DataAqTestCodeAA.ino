/*
* Active Aero + Canbus Board Test Piece.
*/

//-------------------------LIBRARIES-------------------------------------------
#include <EasyTransfer.h>
#include <Servo.h>

//--------------------------ALL MAIN VARIABLES---------------------------------
  //Variables transmitting from ActiveAero
    int Servo1PositionCommand;
    int Servo2PositionCommand;
    int Servo3PositionCommand;
    int Servo4PositionCommand;
    int Servo5PositionCommand;
    int Servo6PositionCommand;
  //recieveing from main dataAq
    int ActiveAeroButtonPush;
  //Variables recieveing from GPS from main dataAq
    int gpsFix;
    int gpsFixQuality;
    float gpsLatitude;
    float gpsLongitude;
    float gpsSpeedKnots;
    float gpsAngle;
    float gpsAltitude;
    int gpsSatellites;
  //Variables for testing. Sending from here to main dataAq
    int AApotentiometer1;
    float bun;

//----------------------UART / EASYTRANSFER SETUP------------------------------
  EasyTransfer ActiveAeroCanbusUARTIN;
  EasyTransfer ActiveAeroCanbusUARTOUT;

    struct UART_RECIEVE_DATA_STRUCTURE_ACTIVE_AERO{
      //Variables from GPS to send to Active Aero
        int16_t _gpsFix;
        int16_t _gpsFixQuality;
        float _gpsLatitude;
        float _gpsLongitude;
        float _gpsSpeedKnots;
        float _gpsAngle;
        float _gpsAltitude;
        int16_t _gpsSatellites;
      //Control variable
        int16_t _ActiveAeroButtonPush;
        float _bun;

    };
    UART_RECIEVE_DATA_STRUCTURE_ACTIVE_AERO rxUartDataACTIVEAERO;
    struct UART_TRANSMIT_DATA_STRUCTURE_ACTIVE_AERO{
      //Variables from Active Aero subsystem (via UART 1)
        int16_t _Servo1PositionCommand;
        int16_t _Servo2PositionCommand;
        int16_t _Servo3PositionCommand;
        int16_t _Servo4PositionCommand;
        int16_t _Servo5PositionCommand;
        int16_t _Servo6PositionCommand;
      //Variables for testing
        int16_t _AApotentiometer1;
    };
    UART_TRANSMIT_DATA_STRUCTURE_ACTIVE_AERO txUartDataACTIVEAERO;

      void UARTSetupforActiveAero(){
        Serial.println("Setting up UART communication for ActiveAeroCanbus");
        ActiveAeroCanbusUARTIN.begin(details(rxUartDataACTIVEAERO), &Serial1);
        ActiveAeroCanbusUARTOUT.begin(details(txUartDataACTIVEAERO), &Serial1);
        Serial.println("***ActiveAeroCanbus UART Comms Setup DONE!");
      }
      void SendAndRecieveActiveAeroData(){

        //enter data under here to send. [structName].[variablename] = variable
        txUartDataACTIVEAERO._Servo1PositionCommand = 0;
        txUartDataACTIVEAERO._Servo2PositionCommand = 0;
        txUartDataACTIVEAERO._Servo3PositionCommand = 0;
        txUartDataACTIVEAERO._Servo4PositionCommand = 0;
        txUartDataACTIVEAERO._Servo5PositionCommand = 0;
        txUartDataACTIVEAERO._Servo6PositionCommand = 0;

        txUartDataACTIVEAERO._AApotentiometer1 = analogRead(A16);

        //send the data
        ActiveAeroCanbusUARTOUT.sendData();

        //recieve the data
        for(int i=0; i<5; i++){
          ActiveAeroCanbusUARTIN.receiveData();

          //put all variables to recieve here. holderVariable = [structName].[variablename]
          gpsFix = rxUartDataACTIVEAERO._gpsFix;
          gpsFixQuality = rxUartDataACTIVEAERO._gpsFixQuality;
          gpsLatitude = rxUartDataACTIVEAERO._gpsLatitude;
          gpsLongitude = rxUartDataACTIVEAERO._gpsLongitude;
          gpsSpeedKnots = rxUartDataACTIVEAERO._gpsSpeedKnots;
          gpsAngle = rxUartDataACTIVEAERO._gpsAngle;
          gpsAltitude = rxUartDataACTIVEAERO._gpsAltitude;
          gpsSatellites = rxUartDataACTIVEAERO._gpsSatellites;
          ActiveAeroButtonPush = rxUartDataACTIVEAERO._ActiveAeroButtonPush;
          bun = rxUartDataACTIVEAERO._bun;

        }

        Serial.println(String(bun));

      }

//-----------------------------MAIN SETUP & LOOP-------------------------------
  void setup(){

      Serial.begin(9600);
      Serial1.begin(9600);

      UARTSetupforActiveAero();

  }

  void loop(){

      SendAndRecieveActiveAeroData();


  }
