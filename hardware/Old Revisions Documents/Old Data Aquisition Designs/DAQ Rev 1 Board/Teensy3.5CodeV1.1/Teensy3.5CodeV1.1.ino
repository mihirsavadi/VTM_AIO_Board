/*
Data Acquisition Teensy 3.5 Central Processor Code
Version 1.1

Change from previous v1.0:
- Changed gps variables from int to FLoat
- Added all library files to ino folder itself
- Integrating Brake Bias into Teensy 3.5 - simple multiturn pot. Removing brake
bias.

~ Mihir Savadi 22/Dec/2018
*/

//library for Adafruit SD card reader https://github.com/arduino-libraries/SD/blob/master/ & https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial/arduino-library
  #include <SPI.h>
  #include <SD.h>
  //#include <SD.h>
  const int chipSelect = 15; //CS1 is pin 31 on teensy3.5

//library for Adafruit Ultimate GatherGPS https://github.com/adafruit/Adafruit_GPS & https://github.com/adafruit/Adafruit_GPS/blob/master/examples/echo/echo.pde
  #include <Adafruit_GPS.h>
  Adafruit_GPS GPS(&Serial2);
  #define GPSECHO  false
  boolean usingInterrupt = false;
  void useInterrupt(boolean);
  uint32_t timer = millis();

//library for Pololu VL53L1X lidar driver presence sensor https://github.com/pololu/vl53l1x-arduino
  #include <VL53L1X.h>
  VL53L1X lidar; //declare VL53l1X instance and name it Lidar cos it is a lidar

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


//VARIABLES FOR DATA LOGGING
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
  //Variables from Active Brake Bias subsystem (only analog and digital in no UART)
    int BrakeBiasPosReport;
    int BrakeBiasFaultReport;
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
  //Variables from Active Aero subsystem (via UART 1)
    int Servo1PositionCommand;
    int Servo2PositionCommand;
    int Servo3PositionCommand;
    int Servo4PositionCommand;
    int Servo5PositionCommand;
    int Servo6PositionCommand;
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

//create string for easy input into SD card. All variables from Data Logging Variables list above.
  String dataString = String(AccelX) + ", " + String(AccelY) + ", " + String(AccelZ) + ", " +
  String(RollX) + ", " + String(RollY) + ", " + String(RollZ) + ", " + String(MagX) + ", " + String(MagY) + ", " + String(MagZ) + String(IMUTemp) + ", " +
  ", " + String(ThrottlePosition) + ", " + String(BrakePosition) + ", " +
  String(TimeSinceLogStart) + ", " + String(Wheel1RPM) + ", " + String(Wheel2RPM) + ", " +
  String(DriverLidarValue) + ", " + String(gpsFix) + ", " + String(gpsFixQuality) + ", " + String(gpsLatitude) +
  ", " + String(gpsLongitude) + ", " + String(gpsSpeedKnots) + ", " + String(gpsAngle) + ", " +
  String(gpsAltitude) + ", " + String(gpsSatellites) + String(Servo1PositionCommand) + ", " + String(BrakeBiasPosReport) + ", " +
  String(BrakeBiasFaultReport);

  //library setup for UART communication from https://github.com/madsci1016/Arduino-EasyTransfer/tree/master/EasyTransfer
    #include <EasyTransfer.h>
    //stuff for Autoshift communication
      EasyTransfer AutoShiftUART;

      struct DATA_STRUCTURE_AUTOSHIFT{
        //PUT STUFF IN HERE TO RECEIVE & SEND (MUST BE SAME ON OTHER END TOO!!!)
      };
      DATA_STRUCTURE_AUTOSHIFT AutoShiftData;

      void UARTSetupforAutoshift(){
        Serial.println("Setting up UART communication for Autoshift");
        AutoShiftUART.begin(details(AutoShiftData), &Serial4);
        Serial.println("***AutoShift UART Comms Setup DONE!");
      }

      void SendAndRecieveAutoshiftData(){
          //check and see if a data packet has come in.
          if(AutoShiftUART.receiveData()){
            //this is how you access the variables. "AutoShiftData.[variable name]"
            //then do whatever you want with variables inside here
          }

          //this is how you access the variables. "AutoShiftData.[variable name] = xyz;"

          //send the data
          AutoShiftUART.sendData();

      }

    //stuff for Telemetry Communication
      EasyTransfer TelemetryUART;

      struct DATA_STRUCTURE_TELEMETRY{
        //PUT STUFF IN HERE to RECEIVE AND SEND (MUST BE SAME ON OTHER END TOO!!!)
      };
      DATA_STRUCTURE_TELEMETRY TelemetryData;

      void UARTSetupforTelemetry(){
        Serial.println("Setting up UART communication for Telemetry");
        TelemetryUART.begin(details(TelemetryData), &Serial5);
        Serial.println("***Telemetry UART Comms Setup DONE!");
      }

      void SendAndRecieveTelemetryData(){
        //check and see if a data packet has come in.
        if(TelemetryUART.receiveData()){
          //this is how you access the variables. "TelemetryData.[variable name]"
          //then do whatever you want with variables inside here
        }

        //this is how you access the variables. "TelemetryData.[variable name] = xyz;"

        //send the data
        TelemetryUART.sendData();

      }

    //stuff for graphics unit communication
      EasyTransfer GraphicsUART;

      struct DATA_STRUCTURE_GRAPHICS{
        //PUT STUFF IN HERE TO RECEIVE AND SEND (MUST BE SAME ON OTHER END TOO!!!)
      };
      DATA_STRUCTURE_GRAPHICS GraphicsData;

      void UARTSetupforGraphics(){
        Serial.println("Setting up UART communication for Graphics");
        GraphicsUART.begin(details(TelemetryData), &Serial3);
        Serial.println("***Graphics UART Comms Setup DONE!");
      }

      void SendAndRecieveGraphicsData(){
        //check and see if a data packet has come in.
        if(GraphicsUART.receiveData()){
          //this is how you access the variables. "GraphicsData.[variable name]"
          //then do whatever you want with variables inside here
        }

        //this is how you access the variables. "GraphicsData.[variable name] = xyz;"

        //send the data
        GraphicsUART.sendData();

      }

    //stuff for Active Aero Module communication
      EasyTransfer ActiveAeroUART;

      struct DATA_STRUCTURE_AERO{
        //PUT STUFF IN HERE TO RECEIVE AND SEND (MUST BE SAME ON OTHER END TOO!!!)
        int16_t Servo1PositionCommand;
        int16_t Servo2PositionCommand;
        int16_t Servo3PositionCommand;
        int16_t Servo4PositionCommand;
        int16_t Servo5PositionCommand;
        int16_t Servo6PositionCommand;

        int16_t gpsFix;
        int16_t gpsFixQuality;
        int16_t gpsSatellites;
        int16_t gpsAngle;
        int16_t gpsSpeedKnots;
        int16_t gpsAltitude;
        float gpsLatitude;
        float gpsLongitude;

      };
      DATA_STRUCTURE_AERO ActiveAeroData;

      void UARTSetupforActiveAero(){
        Serial.println("Setting up UART communication for Graphics");
        ActiveAeroUART.begin(details(ActiveAeroData), &Serial1);
        Serial.println("***Graphics UART Comms Setup DONE!");
      }

      void SendAndRecieveActiveAeroData(){
          //check and see if a data packet has come in.
          if(ActiveAeroUART.receiveData()){
            //this is how you access the variables. "ActiveAeroData.[variable name]"
            //then do whatever you want with variables inside here
            Servo1PositionCommand = ActiveAeroData.Servo1PositionCommand;
            Servo2PositionCommand = ActiveAeroData.Servo2PositionCommand;
            Servo3PositionCommand = ActiveAeroData.Servo3PositionCommand;
            Servo4PositionCommand = ActiveAeroData.Servo4PositionCommand;
            Servo5PositionCommand = ActiveAeroData.Servo5PositionCommand;
            Servo6PositionCommand = ActiveAeroData.Servo6PositionCommand;
          }

          //this is how you access the variables. "ActiveAeroData.[variable name] = xyz;"
          ActiveAeroData.gpsFix = gpsFix;
          ActiveAeroData.gpsFixQuality = gpsFixQuality;
          ActiveAeroData.gpsSatellites = gpsSatellites;
          ActiveAeroData.gpsAngle = gpsAngle;
          ActiveAeroData.gpsSpeedKnots = gpsSpeedKnots;
          ActiveAeroData.gpsAltitude = gpsAltitude;
          ActiveAeroData.gpsLatitude = gpsLatitude;
          ActiveAeroData.gpsLongitude = gpsLongitude;

          //send the data
          ActiveAeroUART.sendData();

      }

void setup(){
  Serial.begin(115200);

  Serial.println("Ay fam. Starting overall Setup Sequence. Please be patient.");

  Wire1.begin(); //begin Wire1 i2c port for all i2 connections.

  SetupForBrakeBiasSense();
  SetupForMPU9250();
  SetupForVL53L1X();
  SetupForADS1115();
  SetupForGPS();
  SetupForSDCardReader();
  UARTSetupforGraphics();
  UARTSetupforAutoshift();
  UARTSetupforTelemetry();
  UARTSetupforActiveAero();

  Serial.println("Setup sequence complete!");
}

void loop(){

  TimeSincePowerUp = millis();
  TimeSinceLogStart = millis();

  GatherPedalPotInfo();
  GatherVL53L1XData();
  GatherMPU9250Data();
  GatherADS1115Data();
  GatherGPSData();
  ReadBrakeBiasInformation();

  SendAndRecieveGraphicsData();
  SendAndRecieveTelemetryData();
  SendAndRecieveAutoshiftData();
  SendAndRecieveActiveAeroData();

  WriteToSDCard();

}

void GatherPedalPotInfo(){
  ThrottlePosition = analogRead(14);
  BrakePosition = analogRead(15);

  Serial.print("PedalPot: " + String(ThrottlePosition) + "T " +
  String(BrakePosition) + "B ");
}

void SetupForBrakeBiasSense(){
  Serial.println("****Setting Up Active Brake Bias GPIO****");

  pinMode(25, INPUT); //fault feedbackpin

  Serial.println("****Active Brake Bias pin input setup done!****");
}

void ReadBrakeBiasInformation(){
  int BrakeBiasFaultPin = 25;
  int BrakeBiasPosPin = A2;

  BrakeBiasPosReport = analogRead(BrakeBiasPosPin);
  //high means fault, low means no fault
  BrakeBiasFaultReport = digitalRead(BrakeBiasFaultPin);

  Serial.print("BrakeB: " + String(BrakeBiasPosReport) + "pos ");
  Serial.print(String(BrakeBiasFaultReport) + "flt ");
}


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


//below from https://github.com/adafruit/Adafruit_GPS/blob/master/examples/echo/echo.pde & https://learn.adafruit.com/adafruit-ultimate-gps/parsed-data-output
void SetupForGPS(){
  Serial.println("****Beginning GPS Setup & Calibration Sequence****");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  // Note the position can only be updated at most 5 times a second so it will lag behind serial output.  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
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

  Serial.print("ADS0: " + String(Wheel1RPM) + " ADS1: " + String(Wheel2RPM));
}


//below from https://github.com/pololu/vl53l1x-arduino/blob/master/examples/Continuous/Continuous.ino
void SetupForVL53L1X(){
  Serial.println("****Beginning VL53L1X Setup & Calibration Sequence****");
  lidar.setTimeout(500);
  if (!lidar.init()){
    Serial.println("Failed to detect and initialize lidar!");
    while (1);
  } else{ Serial.println("lidar Detected and Initialized! All's G fam!");}

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  lidar.setDistanceMode(VL53L1X::Long);
  lidar.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  lidar.startContinuous(50);
  Serial.println("****VL53L1X Setup Sequence Complete****");
}

void GatherVL53L1XData(){

  DriverLidarValue = lidar.read();

  Serial.print( "Lidar: " + String(DriverLidarValue) + " ");
  if (lidar.timeoutOccurred()) { Serial.print ("VL53L1X Lidar TIMEOUT! "); }

}


//below from https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/examples/MPU9250BasicAHRS_I2C/MPU9250BasicAHRS_I2C.ino
void SetupForMPU9250(){
    Serial.println("****Beginning MPU9250 Setup & Calibration Sequence****");
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
