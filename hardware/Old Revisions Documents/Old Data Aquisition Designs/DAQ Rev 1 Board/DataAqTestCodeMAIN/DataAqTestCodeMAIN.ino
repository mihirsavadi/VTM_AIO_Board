/*
* Data acquisition testing code
*
* Mihir Savadi 6Feb2019
*/

//-------------------------LIBRARIES--------------------------------------------
  //library for Adafruit SD card reader https://github.com/arduino-libraries/SD/blob/master/ & https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial/arduino-library
    #include <SPI.h>
    #include <SD.h>
    const int chipSelect = 15; //CS1 is pin 31 on teensy3.5

  //library for Adafruit Ultimate GatherGPS https://github.com/adafruit/Adafruit_GPS & https://github.com/adafruit/Adafruit_GPS/blob/master/examples/echo/echo.pde
    #include <Adafruit_GPS.h>
    Adafruit_GPS GPS(&Serial2);
    #define GPSECHO  false
    boolean usingInterrupt = false;
    void useInterrupt(boolean);
    uint32_t timer = millis();

  //library for Adafruit ADS1115 16Bit ADC
    #include <Adafruit_ADS1015.h>
    Adafruit_ADS1115 ads;

  //libraries for Sparkfun IMU MPU-9250 - https://learn.sparkfun.com/tutorials/mpu-9250-hookup-guide
    #include <quaternionFilters.h>
    #include <MPU9250.h>

    #define AHRS false         // Set to false for basic data read
    #define SerialDebug true  // Set to true to get Serial output for debugging

    #define I2Cclock 400000
    #define I2Cport Wire1
    #define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
    //#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1
    MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);


//--------------------------ALL LOGGED VARIABLES--------------------------------
  //variables from MPU9250 IMU
    int AccelX; int AccelY; int AccelZ; //in g's
    int RollX; int RollY; int RollZ; //in deg/s
    int MagX; int MagY; int MagZ; //milliGauss or mG
    int IMUTemp; //degrees C
  //Variables from Analog Inputs of Pedals
    int ThrottlePosition;
    int BrakePosition;
  //non hardware variables
    int TimeSinceLogStart; //milliseconds
    int TimeSincePowerUp; //milliseconds
  //Variables from ADC1115 for WheelRPM sensing
    int Wheel1RPM;
    int Wheel2RPM;
  //BrakeBias potentiometer readings
    int brakeBiasPotValue;
  //Variables from VL53L1X driver presence lidar sensor
    int DriverLidarValue;
  //Variables from GPS
    int gpsFix;
    int gpsFixQuality;
    float gpsLatitude;
    float gpsLongitude;
    float gpsSpeedKnots;
    float gpsAngle;
    float gpsAltitude;
    int gpsSatellites;
  //ActiveAero Input
    int ActiveAeroButtonPin = 23;
  //Variables from Inverter CANBUS
    int InverterMotorSpeed;
    int InverterRMSMotorCurrent;
    int InverterDCVoltage;
    int InverterDCCurrent;
    int InverterHeatSinkTemp;
    int InverterMotorTemp;
  //Variables from BMS CANBUS
    int BMShighTemp;
    int BMSpackCurrent;
  //Variables from ECU CANBUS (still need to add shit idk what)
    int idkwhatyet;
  //Variables from Active Aero subsystem (via UART 1)
    int Servo1PositionCommand;
    int Servo2PositionCommand;
    int Servo3PositionCommand;
    int Servo4PositionCommand;
    int Servo5PositionCommand;
    int Servo6PositionCommand;
    int ActiveAeroButtonPush; //send to subsystem.
  //Variables from AutoShift (via UART4)
    int ShiftUp; //everytime shifted up, report high or 1
    int ShiftDown; //likewise
    int AutoShiftFaultReport;
  //Variables from Telemetry HC-12 (via UART5)
    int laptime;
    //button on groundstation user press and holds to mark interesting time to
    //view data. Will change from 0 to 1 on press
    int EventHighlight;
  //Variables from Graphics Unit (via UART3)
    //nothing to get from graphics
  //Variables for testing
    int AApotentiometer1;


//----------------------SETUP: UART / EASYTRANSFER------------------------------
  //Instiating EasyTransfer Library for each UART device. Self explanatory naming.
  //ALSO Data structs for EasyTransfer library. Must be exactly same variable names on all UART devices!!
  //ALSO functions for setup and sending and recieving data to all devices

  //library setup for UART communication from https://github.com/madsci1016/Arduino-EasyTransfer/tree/master/EasyTransfer
  #include <EasyTransfer.h>

  EasyTransfer ActiveAeroCanbusUARTIN;
  EasyTransfer ActiveAeroCanbusUARTOUT;

    struct UART_RECIEVE_DATA_STRUCTURE_ACTIVE_AERO{
      //Variables from Active Aero subsystem (via UART 1)
        int16_t _Servo1PositionCommand;
        int16_t _Servo2PositionCommand;
        int16_t _Servo3PositionCommand;
        int16_t _Servo4PositionCommand;
        int16_t _Servo5PositionCommand;
        int16_t _Servo6PositionCommand;
      //Variables for testing
        int16_t _AApotentiometer1;
        int16_t _AApotentiometer2;
    };
    UART_RECIEVE_DATA_STRUCTURE_ACTIVE_AERO rxUartDataACTIVEAERO;
    struct UART_TRANSMIT_DATA_STRUCTURE_ACTIVE_AERO{
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
    };
    UART_TRANSMIT_DATA_STRUCTURE_ACTIVE_AERO txUartDataACTIVEAERO;

      void UARTSetupforActiveAero(){
        Serial.println("Setting up UART communication for ActiveAeroCanbus");
        Serial1.begin(9600);
        ActiveAeroCanbusUARTIN.begin(details(rxUartDataACTIVEAERO), &Serial1);
        ActiveAeroCanbusUARTOUT.begin(details(txUartDataACTIVEAERO), &Serial1);
        Serial.println("***ActiveAeroCanbus UART Comms Setup DONE!");
      }
      void SendAndRecieveActiveAeroData(){

        //enter data under here to send. [structName].[variablename] = variable
        txUartDataACTIVEAERO._gpsFix = gpsFix;
        txUartDataACTIVEAERO._gpsFixQuality = gpsFixQuality;
        txUartDataACTIVEAERO._gpsLatitude = gpsLatitude;
        txUartDataACTIVEAERO._gpsLongitude = gpsLongitude;
        txUartDataACTIVEAERO._gpsSpeedKnots = gpsSpeedKnots;
        txUartDataACTIVEAERO._gpsAngle = gpsAngle;
        txUartDataACTIVEAERO._gpsAltitude = gpsAltitude;
        txUartDataACTIVEAERO._gpsSatellites = gpsSatellites;
        txUartDataACTIVEAERO._ActiveAeroButtonPush = ActiveAeroButtonPush;

        //send the data
        ActiveAeroCanbusUARTOUT.sendData();

        //recieve the data
        for(int i=0; i<5; i++){
          ActiveAeroCanbusUARTIN.receiveData();

          //put all variables to recieve here. holderVariable = [structName].[variablename]
          Servo1PositionCommand = rxUartDataACTIVEAERO._Servo1PositionCommand;
          Servo2PositionCommand = rxUartDataACTIVEAERO._Servo2PositionCommand;
          Servo3PositionCommand = rxUartDataACTIVEAERO._Servo3PositionCommand;
          Servo4PositionCommand = rxUartDataACTIVEAERO._Servo4PositionCommand;
          Servo5PositionCommand = rxUartDataACTIVEAERO._Servo5PositionCommand;
          Servo6PositionCommand = rxUartDataACTIVEAERO._Servo6PositionCommand;
          AApotentiometer1 = rxUartDataACTIVEAERO._AApotentiometer1;
        }

      }

  EasyTransfer DashDisplayUARTIN;
  EasyTransfer DashDisplayUARTOUT;

  EasyTransfer AutoShiftUARTIN;
  EasyTransfer AutoShiftUARTOUT;

  EasyTransfer TelemetryUARTIN;
  EasyTransfer TelemetryUARTOUT;


//----------------------SETUP: MPU9250 IMU--------------------------------------

  //below from https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/examples/MPU9250BasicAHRS_I2C/MPU9250BasicAHRS_I2C.ino

  void SetupForMPU9250(){
      Serial.println("****Beginning MPU9250 Setup & Calibration Sequence****");

      Wire1.begin();

      int imuAddressSelectorPin = 19;
      pinMode(imuAddressSelectorPin, OUTPUT);
      digitalWrite(imuAddressSelectorPin, 0);

      // Read the WHO_AM_I register, this is a good test of communication
     byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
     Serial.print(F("MPU9250 I AM 0x"));
     Serial.print(c, HEX);
     Serial.print(F(" I should be 0x"));
     Serial.println(0x71, HEX);

     if (c == 0x71) // WHO_AM_I should always be 0x71
     {
       Serial.println(F("MPU9250 is online..."));

       // Start by performing self test and reporting values
       myIMU.MPU9250SelfTest(myIMU.selfTest);
       Serial.print(F("x-axis self test: acceleration trim within : "));
       Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
       Serial.print(F("y-axis self test: acceleration trim within : "));
       Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
       Serial.print(F("z-axis self test: acceleration trim within : "));
       Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
       Serial.print(F("x-axis self test: gyration trim within : "));
       Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
       Serial.print(F("y-axis self test: gyration trim within : "));
       Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
       Serial.print(F("z-axis self test: gyration trim within : "));
       Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

       // Calibrate gyro and accelerometers, load biases in bias registers
       myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

       myIMU.initMPU9250();
        // Initialize device for active mode read of acclerometer, gyroscope, and
        // temperature
        Serial.println("MPU9250 initialized for active data mode....");

        // Read the WHO_AM_I register of the magnetometer, this is a good test of
        // communication
        byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
        Serial.print("AK8963 ");
        Serial.print("I AM 0x");
        Serial.print(d, HEX);
        Serial.print(" I should be 0x");
        Serial.println(0x48, HEX);

        if (d != 0x48)
       {
         // Communication failed, stop here
         Serial.println(F("Communication failed, abort!"));
         Serial.flush();
         abort();
       }

       // Get magnetometer calibration from AK8963 ROM
       myIMU.initAK8963(myIMU.factoryMagCalibration);
       // Initialize device for active mode read of magnetometer
       Serial.println("AK8963 initialized for active data mode....");

       if (SerialDebug)
       {
         //  Serial.println("Calibration values: ");
         Serial.print("X-Axis factory sensitivity adjustment value ");
         Serial.println(myIMU.factoryMagCalibration[0], 2);
         Serial.print("Y-Axis factory sensitivity adjustment value ");
         Serial.println(myIMU.factoryMagCalibration[1], 2);
         Serial.print("Z-Axis factory sensitivity adjustment value ");
         Serial.println(myIMU.factoryMagCalibration[2], 2);
       }

       // Get sensor resolutions, only need to do this once
       myIMU.getAres();
       myIMU.getGres();
       myIMU.getMres();

       // The next call delays for 4 seconds, and then records about 15 seconds of
       // data to calculate bias and scale.
    //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
       Serial.println("AK8963 mag biases (mG)");
       Serial.println(myIMU.magBias[0]);
       Serial.println(myIMU.magBias[1]);
       Serial.println(myIMU.magBias[2]);

       Serial.println("AK8963 mag scale (mG)");
       Serial.println(myIMU.magScale[0]);
       Serial.println(myIMU.magScale[1]);
       Serial.println(myIMU.magScale[2]);
    //    delay(2000); // Add delay to see results before serial spew of data

       if(SerialDebug)
       {
         Serial.println("Magnetometer:");
         Serial.print("X-Axis sensitivity adjustment value ");
         Serial.println(myIMU.factoryMagCalibration[0], 2);
         Serial.print("Y-Axis sensitivity adjustment value ");
         Serial.println(myIMU.factoryMagCalibration[1], 2);
         Serial.print("Z-Axis sensitivity adjustment value ");
         Serial.println(myIMU.factoryMagCalibration[2], 2);
       }

     }
     else
      {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);

        // Communication failed, stop here
        Serial.println(F("Communication failed, abort!"));
        Serial.flush();
        abort();
      }

      Serial.println("****MPU9250 Setup Sequence Complete****");
  }

  void GatherMPU9250Data(){
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
   {
     myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

     // Now we'll calculate the accleration value into actual g's
     // This depends on scale being set
     myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
     myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
     myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

     myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

     // Calculate the gyro value into actual degrees per second
     // This depends on scale being set
     myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
     myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
     myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

     myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

     // Calculate the magnetometer values in milliGauss
     // Include factory calibration per data sheet and user environmental
     // corrections
     // Get actual magnetometer value, this depends on scale being set
     myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
                * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
     myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
                * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
     myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
                * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
   }

   myIMU.tempCount = myIMU.readTempData();  // Read the adc values
   // Temperature in degrees Centigrade
   myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;

   AccelX = myIMU.ax;
   AccelY = myIMU.ay;
   AccelZ = myIMU.az;
   RollX = myIMU.gx;
   RollY = myIMU.gy;
   RollZ = myIMU.gz;
   MagX = myIMU.mx;
   MagY = myIMU.my;
   MagZ = myIMU.mz;
   IMUTemp = myIMU.temperature;

   Serial.println( "IMU: " + String(AccelX) + "aX " + String(AccelY) + "aY " + String(AccelZ) + "aZ " +
   String(RollX) + "gX " + String(RollY) + "gY " +String(RollZ) + "gZ " + String(MagX) + "mX " + String(MagY) + "mY "
   + String(MagZ) + "mZ " + "T " + String(IMUTemp) + "C");

  }


//----------------------SETUP: GPS----------------------------------------------

  //below from https://github.com/adafruit/Adafruit_GPS/blob/master/examples/echo/echo.pde & https://learn.adafruit.com/adafruit-ultimate-gps/parsed-data-output
  //some extra shit for GPS library
  #ifdef __AVR__
     // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
     SIGNAL(TIMER0_COMPA_vect) {
         char c = GPS.read();
         // if you want to debug, this is a good time to do it!
       #ifdef UDR0
         if (GPSECHO)
           if (c) UDR0 = c;
           // writing direct to UDR0 is much much faster than Serial.print
           // but only one character can be written at a time.
       #endif
     }
     void useInterrupt(boolean v) {
       if (v) {
         // Timer0 is already used for millis() - we'll just interrupt somewhere
         // in the middle and call the "Compare A" function above
         OCR0A = 0xAF;
         TIMSK0 |= _BV(OCIE0A);
         usingInterrupt = true;
       } else {
         // do not call the interrupt function COMPA anymore
         TIMSK0 &= ~_BV(OCIE0A);
         usingInterrupt = false;
       }
     }
  #endif //#ifdef__AVR__

  void SetupForGPS(){
    Serial.println("****Beginning GPS Setup & Calibration Sequence****");

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
    // Note the position can only be updated at most 5 times a second so it will lag behind serial output. GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    #ifdef __arm__
      usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
    #else
      useInterrupt(true);
    #endif

      delay(1000);

    Serial.println("****GPS Setup Sequence Complete****");
  }

  void GatherGPSData(){
      // in case you are not using the interrupt above, you'll
     // need to 'hand query' the GPS, not suggested :(
     if (! usingInterrupt) {
       // read data from the GPS in the 'main loop'
       char c = GPS.read();
       // if you want to debug, this is a good time to do it!
       if (GPSECHO)
         if (c) Serial.print(c);
     }

     // if a sentence is received, we can check the checksum, parse it...
     if (GPS.newNMEAreceived()) {
       // a tricky thing here is if we print the NMEA sentence, or data
       // we end up not listening and catching other sentences!
       // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
       //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

       if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
         return;  // we can fail to parse a sentence in which case we should just wait for another
     }

     // if millis() or timer wraps around, we'll just reset it
     if (timer > millis())  timer = millis();

     // approximately every 2 seconds or so, print out the current stats
     if (millis() - timer > 2000) {
       timer = millis(); // reset the timer

       gpsFix = GPS.fix;
       gpsFixQuality = GPS.fixquality;
       if (GPS.fix) {
         gpsLatitude = GPS.latitude;
         gpsLongitude = GPS.longitude;

         gpsSpeedKnots = GPS.speed;
         gpsAngle = GPS.angle;
         gpsAltitude = GPS.altitude;
         gpsSatellites = GPS.satellites;
       }

       Serial.print("GPS: " + String(gpsFix) + "fix ");
       Serial.print(String(gpsFixQuality) + "fixQ ");
       Serial.print(String(gpsLatitude) + "lat ");
       Serial.print(String(gpsLongitude) + "long ");
       Serial.print(String(gpsSpeedKnots) + "knots ");
       Serial.print(String(gpsAngle) + "angle ");
       Serial.print(String(gpsAltitude) + "alt ");
       Serial.print(String(gpsSatellites) + "sat ");

    }

   }


//----------------------SETUP: ADS1115 ADC--------------------------------------

  //below from https://github.com/adafruit/Adafruit_ADS1X15/blob/master/examples/singleended/singleended.pde
  void SetupForADS1115(){
    Serial.println("****Beginning ADS1115 Setup & Calibration Sequence****");
    // The ADC input range (or gain) can be changed via the following
    // functions, but be careful never to exceed VDD +0.3V max, or to
    // exceed the upper and lower limits if you adjust the input range!
    // Setting these values incorrectly may destroy your ADC!
    //                                                                ADS1015  ADS1115
    //                                                                -------  -------
    // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    ads.begin();
    Serial.println("****ADS1115 Setup Sequence Complete****");
  }

  void GatherADS1115Data(){

    Wheel1RPM = ads.readADC_SingleEnded(0);
    Wheel2RPM = ads.readADC_SingleEnded(1);

    Serial.println("ADS0: " + String(Wheel1RPM) + " ADS1: " + String(Wheel2RPM));
  }


//----------------------SETUP: SD Card adapter----------------------------------

String dataString =
    String(AccelX) + ", " + String(AccelY) + ", " + String(AccelZ) + ", " +
    String(RollX) + ", " + String(RollY) + ", " + String(RollZ) + ", " +
    String(MagX) + ", " + String(MagY) + ", " + String(MagZ) + ", " +
    String(IMUTemp) + ", " +
    String(ThrottlePosition) + ", " +
    String(BrakePosition) + ", " +
    String(TimeSinceLogStart) + ", " +
    String(TimeSincePowerUp) + ", " +
    String(Wheel1RPM) + ", " +
    String(Wheel2RPM) + ", " +
    String(brakeBiasPotValue) + ", " +
    String(DriverLidarValue) + ", " +
    String(gpsFix) + ", " +
    String(gpsFixQuality) + ", " +
    String(gpsLatitude) + ", " +
    String(gpsLongitude) + ", " +
    String(gpsSpeedKnots) + ", " +
    String(gpsAngle) + ", " +
    String(gpsAltitude) + ", " +
    String(gpsSatellites) + ", " +
    String(InverterMotorSpeed) + ", " +
    String(InverterRMSMotorCurrent) + ", " +
    String(InverterDCVoltage) + ", " +
    String(InverterDCCurrent) + ", " +
    String(InverterHeatSinkTemp) + ", " +
    String(InverterMotorTemp) + ", " +
    String(BMShighTemp) + ", " +
    String(BMSpackCurrent) + ", " +
    String(idkwhatyet) + ", " +
    String(Servo1PositionCommand) + ", " +
    String(Servo2PositionCommand) + ", " +
    String(Servo3PositionCommand) + ", " +
    String(Servo4PositionCommand) + ", " +
    String(Servo5PositionCommand) + ", " +
    String(Servo6PositionCommand) + ", " +
    String(ActiveAeroButtonPush) + ", " +
    String(ShiftUp) + ", " +
    String(ShiftDown) + ", " +
    String(AutoShiftFaultReport) + ", " +
    String(laptime) + ", " +
    String(EventHighlight) + ", " +
    String(AApotentiometer1)
    ;

  //below from https://github.com/arduino-libraries/SD & https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial/arduino-library
  void SetupForSDCardReader(){
    Serial.println("****Beginning SD Card Reader Setup Sequence****");

    Serial.print("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while (1);
    }
    Serial.println("card initialized.");
    Serial.println("**** SD Card Reader setup sequence complete****");

  }

  void WriteToSDCard(){
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(" Logged onto SD Card! ");
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
  }


//-----------------------------OTHER SETUP FUNCTIONS----------------------------



//-----------------------------OTHER LOOP FUNCTIONS----------------------------



//-----------------------------MAIN SETUP & LOOP-------------------------------
  void setup(){
    Serial.begin(9600);
    Serial3.begin(9600);
    Serial4.begin(9600);
    Serial5.begin(9600);

    Serial.println("Ay fam. Starting overall Setup Sequence. Please be patient.");

    UARTSetupforActiveAero();

    SetupForMPU9250();
    SetupForGPS();
    SetupForADS1115();

    Serial.println("Setup sequence complete!");

  }

  void loop(){

    SendAndRecieveActiveAeroData();

    GatherADS1115Data();
    GatherGPSData();
    GatherMPU9250Data();



  }
