/* implementation for aioMainMCU class */

#include "aioMainMCU.hpp"

//floating point map helper function
float mapfl(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// GPS Library Stuff. Must Setup outside class declaration strangely.
Adafruit_GPS GPS(&Serial3);
// IMU Library stuff. Must Setup outside class declaration strangely.
MPU9250 IMU(Wire1, 0x68);

aioMainMCU::aioMainMCU()
{
    //set up digital input pins
    pinMode(BSPDFAULT_IN, INPUT);
    pinMode(KILLSENSE_IN, INPUT);
    pinMode(DATALOG_BUTTON_IN, INPUT);
    pinMode(BUTTON_IN_NEUTRAL, INPUT);
    pinMode(BUTTON_IN_LAUNCHCONTROL, INPUT);
    pinMode(SD_CARD_DETECT, INPUT);

    //set up ADC input ports for analog input at 12bits
    analog_init();
    analogReadResolution(12);
    analogReadAveraging(5); //set no. of averaging per read.

    //setup GPS
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //ask for type of data
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); //set 10Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA); //request update on antenna

    //setup IMU
    this->sigIns.IMUbeginStatus = IMU.begin();
    if (this->sigIns.IMUbeginStatus < 0)
    {
        this->errorPresent = true;
        this->errorstring.concat("IMU init unsuccessful statVal: " + 
                                  String(this->sigIns.IMUbeginStatus) + ERDELIM);
    }
    // setting the accelerometer full scale range to +/-8G 
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);

    //setup SD card 
    // (add first line on startup to give guide to whats in each column)
    // see logAllInputs() method for order of entry
    pinMode(SPI_CS_0, OUTPUT);
    if (!SD.begin(SPI_CS_0))
    {
        this->errorPresent = true;
        this->errorstring.concat("SD card not present and cant be initialized"
            + ERDELIM);
    }
    this->dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (!this->dataFile)
    {
        this->errorPresent = true;
        this->errorstring.concat("Error opening datalog file" + ERDELIM);
    }
    else
    {
        String firstLine;

        firstLine.concat("shiftSolenoid Current (Amps), ");
        firstLine.concat("GSensor Current (Amps), ");
        firstLine.concat("syncSensor Current (Amps), ");
        firstLine.concat("EGT Current (Amps), ");
        firstLine.concat("injector Current (Amps), ");
        firstLine.concat("turboSolenoid Current (Amps), ");
        firstLine.concat("LTC Current (Amps), ");
        firstLine.concat("fuelPump Current (Amps), ");
        firstLine.concat("ignitionCoil Current (Amps), ");
        firstLine.concat("fan Current (Amps), ");
        firstLine.concat("auxiliaryStage Current (Amps), ");
        firstLine.concat("motec Current (Amps), ");
        firstLine.concat("acdcConverter Current (Amps), ");
        firstLine.concat("battery Current (Amps), ");
        firstLine.concat("servo Current (Amps), ");

        firstLine.concat("acdcConverter Voltage (Volts), ");
        firstLine.concat("battery Voltage (Volts), ");

        firstLine.concat("gearSense ADC read, ");
        firstLine.concat("throttleSignal ADC read, ");
        firstLine.concat("brakeSignal ADC read, ");
        firstLine.concat("bspdFault Digital read, ");
        firstLine.concat("killsense Digital read, ");
        firstLine.concat("dataLogButtonPressed Digital read, ");
        firstLine.concat("neutralButtonPressed Digital read, ");
        firstLine.concat("launchButtonPressed Digital read, ");
        firstLine.concat("sdCardDetected Digital read, ");

        firstLine.concat("year GPSdata (GMT), ");
        firstLine.concat("month GPSdata (GMT), ");
        firstLine.concat("day GPSdata (GMT), ");
        firstLine.concat("hour GPSdata (GMT), ");
        firstLine.concat("min GPSdata (GMT), ");
        firstLine.concat("sec GPSdata (GMT), ");
        firstLine.concat("mSec GPSdata (GMT), ");
        firstLine.concat("gpsFix GPSdata (T/F), ");
        firstLine.concat("fixQual GPSdata (0:Invalid; 1:GPS; 2:DGPS), ");
        firstLine.concat("fixQual3d GPSdata (1:Nofix; 2:2D fix; 3:3D fix), ");
        firstLine.concat("sats GPSdata (no. of satellites in use), ");
        firstLine.concat("latDeg GPSdata (decimal degrees), ");
        firstLine.concat("longDeg GPSdata (decimal degrees), ");
        firstLine.concat("altitude GPSdata (meters above MSL), ");
        firstLine.concat("speed GPSdata (knots), ");
        firstLine.concat("angle GPSdata (course degrees from true north), ");
        firstLine.concat("magVar GPSdata (magnetic variation in degrees vs true north), ");
        firstLine.concat("hdop GPSdata (horizontal relative accuracy), ");
        firstLine.concat("vdop GPSdata (horizontal relative accuracy), ");
        firstLine.concat("pdop GPSdata (horizontal relative accuracy), ");

        firstLine.concat("accelX IMUdata (m/(s^2)), ");
        firstLine.concat("accelY IMUdata (m/(s^2)), ");
        firstLine.concat("accelZ IMUdata (m/(s^2)), ");
        firstLine.concat("gyroX IMUdata (rad/s), ");
        firstLine.concat("gyroY IMUdata (rad/s), ");
        firstLine.concat("gyroZ IMUdata (rad/s), ");
        firstLine.concat("magX IMUdata (uT), ");
        firstLine.concat("magY IMUdata (uT), ");
        firstLine.concat("magZ IMUdata (uT), ");
        firstLine.concat("IMU_dieTemp IMUdata (C), ");

        if(this->errorPresent)
        {
            firstLine.concat(String("NO ERRORS PRESENT"));
        }
        else
        {
            firstLine.concat("ERRORS PRESENT: " + this->errorstring);
        }

        this->dataFile.println(firstLine);
    }
}

void aioMainMCU::readInputs()
{
    //read all current data
    this->current.shiftSolenoid  = ampsFromADC(analogRead(CURRENT_SHIFTSOL));
    this->current.GSensor        = ampsFromADC(analogRead(CURRENT_GSENSOR));
    this->current.syncSensor     = ampsFromADC(analogRead(CURRENT_SYNC));
    this->current.EGT            = ampsFromADC(analogRead(CURRENT_EGT));
    this->current.injector       = ampsFromADC(analogRead(CURRENT_INJECTOR));
    this->current.turboSolenoid  = ampsFromADC(analogRead(CURRENT_TURBOSOL));
    this->current.LTC            = ampsFromADC(analogRead(CURRENT_LTC));
    this->current.fuelPump       = ampsFromADC(analogRead(CURRENT_FP));
    this->current.ignitionCoil   = ampsFromADC(analogRead(CURRENT_IC));
    this->current.fan            = ampsFromADC(analogRead(CURRENT_FAN));
    this->current.auxiliaryStage = ampsFromADC(analogRead(CURRENT_AUX));
    this->current.motec          = ampsFromADC(analogRead(CURRENT_MOTEC));
    this->current.acdcConverter  = ampsFromADC(analogRead(CURRENT_ACDC));
    this->current.battery        = ampsFromADC(analogRead(CURRENT_BATTERY));
    this->current.servo          = ampsFromADC(analogRead(CURRENT_SERVO));

    //read all voltage data
    this->voltage.acdcConverter = mapfl(analogRead(VOLTAGE_ACDC), 0, 4096, 0, 3.3)*5
                                    + V_OFFSET;
    this->voltage.battery       = mapfl(analogRead(VOLTAGE_BATTERY), 0, 4096, 0, 3.3)*5
                                    + V_OFFSET;

    //read all signalInput data
    this->sigIns.gearSense            = analogRead(GEARSENSE_ADC_IN);
    this->sigIns.throttleSignal       = analogRead(THROTTLE_SIG_IN);
    this->sigIns.brakeSignal          = analogRead(BRAKE_SIG_IN);
    this->sigIns.bspdFault            = digitalReadFast(BSPDFAULT_IN);
    this->sigIns.killsense            = digitalReadFast(KILLSENSE_IN);
    this->sigIns.dataLogButtonPressed = digitalReadFast(DATALOG_BUTTON_IN);
    this->sigIns.neutralButtonPressed = digitalReadFast(BUTTON_IN_NEUTRAL);
    this->sigIns.launchButtonPressed  = digitalReadFast(BUTTON_IN_LAUNCHCONTROL);
    this->sigIns.sdCardDetected       = digitalReadFast(SD_CARD_DETECT);

    //read all GPS data
    GPS.read();
    this->sigIns.year       = GPS.year;
    this->sigIns.month      = GPS.month;
    this->sigIns.day        = GPS.day;
    this->sigIns.hour       = GPS.hour;
    this->sigIns.min        = GPS.minute;
    this->sigIns.sec        = GPS.seconds;
    this->sigIns.mSec       = GPS.milliseconds;
    this->sigIns.gpsFix     = GPS.fix;
    this->sigIns.fixQual    = GPS.fixquality;
    this->sigIns.fixQual3d  = GPS.fixquality_3d;
    this->sigIns.sats       = GPS.satellites;
    this->sigIns.latDeg     = GPS.latitudeDegrees;
    this->sigIns.longDeg    = GPS.longitudeDegrees;
    this->sigIns.altitude   = GPS.altitude;
    this->sigIns.speed      = GPS.speed;
    this->sigIns.angle      = GPS.angle;
    this->sigIns.magVar     = GPS.magvariation;
    this->sigIns.hdop       = GPS.HDOP;
    this->sigIns.vdop       = GPS.VDOP;
    this->sigIns.pdop       = GPS.PDOP;

    //read all IMU data
    IMU.readSensor();
    this->sigIns.accelX = IMU.getAccelX_mss();
    this->sigIns.accelY = IMU.getAccelY_mss();
    this->sigIns.accelZ = IMU.getAccelZ_mss();
    this->sigIns.gyroX  = IMU.getGyroX_rads();
    this->sigIns.gyroY  = IMU.getGyroY_rads();
    this->sigIns.gyroZ  = IMU.getGyroZ_rads();
    this->sigIns.magX   = IMU.getMagX_uT();
    this->sigIns.magY   = IMU.getMagY_uT();
    this->sigIns.magZ   = IMU.getMagZ_uT();
    this->sigIns.IMU_dieTemp = IMU.getTemperature_C();
}

void aioMainMCU::logAllInputs()
{
    String dataLine;

    if(this->dataFile)
    {
        // current data
        dataLine.concat(String(this->current.shiftSolenoid) + ", ");
        dataLine.concat(String(this->current.GSensor) + ", ");
        dataLine.concat(String(this->current.syncSensor) + ", ");
        dataLine.concat(String(this->current.EGT) + ", ");
        dataLine.concat(String(this->current.injector) + ", ");
        dataLine.concat(String(this->current.turboSolenoid) + ", ");
        dataLine.concat(String(this->current.LTC) + ", ");
        dataLine.concat(String(this->current.fuelPump) + ", ");
        dataLine.concat(String(this->current.ignitionCoil) + ", ");
        dataLine.concat(String(this->current.fan) + ", ");
        dataLine.concat(String(this->current.auxiliaryStage) + ", ");
        dataLine.concat(String(this->current.motec) + ", ");
        dataLine.concat(String(this->current.acdcConverter) + ", ");
        dataLine.concat(String(this->current.battery) + ", ");
        dataLine.concat(String(this->current.servo) + ", ");

        // voltage data
        dataLine.concat(String(this->voltage.acdcConverter) + ", ");
        dataLine.concat(String(this->voltage.battery) + ", ");

        // signal input data
        dataLine.concat(String(this->sigIns.gearSense) + ", ");
        dataLine.concat(String(this->sigIns.throttleSignal) + ", ");
        dataLine.concat(String(this->sigIns.brakeSignal) + ", ");
        dataLine.concat(String(this->sigIns.bspdFault) + ", ");
        dataLine.concat(String(this->sigIns.killsense) + ", ");
        dataLine.concat(String(this->sigIns.dataLogButtonPressed) + ", ");
        dataLine.concat(String(this->sigIns.neutralButtonPressed) + ", ");
        dataLine.concat(String(this->sigIns.launchButtonPressed) + ", ");
        dataLine.concat(String(this->sigIns.sdCardDetected));

        // GPS data
        dataLine.concat(String(this->sigIns.year) + ", ");
        dataLine.concat(String(this->sigIns.month) + ", ");
        dataLine.concat(String(this->sigIns.day) + ", ");
        dataLine.concat(String(this->sigIns.hour) + ", ");
        dataLine.concat(String(this->sigIns.min) + ", ");
        dataLine.concat(String(this->sigIns.sec) + ", ");
        dataLine.concat(String(this->sigIns.mSec) + ", ");
        dataLine.concat(String(this->sigIns.gpsFix) + ", ");
        dataLine.concat(String(this->sigIns.fixQual) + ", ");
        dataLine.concat(String(this->sigIns.fixQual3d) + ", ");
        dataLine.concat(String(this->sigIns.sats) + ", ");
        dataLine.concat(String(this->sigIns.latDeg) + ", ");
        dataLine.concat(String(this->sigIns.longDeg) + ", ");
        dataLine.concat(String(this->sigIns.altitude) + ", ");
        dataLine.concat(String(this->sigIns.speed) + ", ");
        dataLine.concat(String(this->sigIns.angle) + ", ");
        dataLine.concat(String(this->sigIns.magVar) + ", ");
        dataLine.concat(String(this->sigIns.hdop) + ", ");
        dataLine.concat(String(this->sigIns.vdop) + ", ");
        dataLine.concat(String(this->sigIns.pdop) + ", ");

        // IMU data
        dataLine.concat(String(this->sigIns.accelX) + ", ");
        dataLine.concat(String(this->sigIns.accelY) + ", ");
        dataLine.concat(String(this->sigIns.accelZ) + ", ");
        dataLine.concat(String(this->sigIns.gyroX) + ", ");
        dataLine.concat(String(this->sigIns.gyroY) + ", ");
        dataLine.concat(String(this->sigIns.gyroZ) + ", ");
        dataLine.concat(String(this->sigIns.magX) + ", ");
        dataLine.concat(String(this->sigIns.magY) + ", ");
        dataLine.concat(String(this->sigIns.magZ) + ", ");
        dataLine.concat(String(this->sigIns.IMU_dieTemp) + ", ");

        if(this->errorPresent)
        {
            dataLine.concat(String("NO ERRORS PRESENT"));
        }
        else
        {
            dataLine.concat("ERRORS PRESENT: " + this->errorstring);
        }
    }
    else
    {
        this->errorPresent = true;
        this->errorstring.concat("error opening datalog.csv" + ERDELIM);
        
        dataLine.concat("ERRORS PRESENT: " + this->errorstring);
    }

    this->dataFile.println(dataLine);
    this->dataFile.flush();
}

bool const aioMainMCU::getError(String &errorDescription)
{
    errorDescription = this->errorstring;
    return this->errorPresent;
}

float const aioMainMCU::ampsFromADC(uint16_t ADC_reading)
{
    //see file:///C:/Users/m_sav/Downloads/ACS781-Datasheet.pdf page 2
    // we are running ACS781KLRTR-150B-T which has a 8.8mV/A sensitivity.
    // So we do a floating point scale of ADC_reading, which as input range of
    // 0 to 4096, to an output range of 0 to 3300mV (since the ADC scales linearly from
    // 0 to 3.3V max), then dividing by the ACS781 current sensor data sheet constant
    // of 8.8mV/A, then add to C_OFFSET for calibration.
    return (mapfl(ADC_reading, 0, 4096, 0, 3300) / 8.8) - C_OFFSET;;
}

void const aioMainMCU::printToMonitor() 
{
    Serial.print("BattV: " + String(mapfl(analogRead(VOLTAGE_BATTERY), 0, 4096, 0, 3.3)*5) + "V");
    // Serial.println(", BattA: " + String(this->ampsFromADC(analogRead(CURRENT_BATTERY))) + "A");
    Serial.println(", BattA: " + String(analogRead(CURRENT_BATTERY)) + "A");
}