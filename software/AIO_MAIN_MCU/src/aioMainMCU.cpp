/* implementation for aioMainMCU class */

#include "aioMainMCU.hpp"

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
    analogReadAveraging(10); //set no. of averaging per read.
    
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

        firstLine.concat("shiftSolenoid Current, ");
        firstLine.concat("GSensor Current, ");
        firstLine.concat("syncSensor Current, ");
        firstLine.concat("EGT Current, ");
        firstLine.concat("injector Current, ");
        firstLine.concat("turboSolenoid Current, ");
        firstLine.concat("LTC Current, ");
        firstLine.concat("fuelPump Current, ");
        firstLine.concat("ignitionCoil Current, ");
        firstLine.concat("fan Current, ");
        firstLine.concat("auxiliaryStage Current, ");
        firstLine.concat("motec Current, ");
        firstLine.concat("acdcConverter Current, ");
        firstLine.concat("battery Current, ");
        firstLine.concat("servo Current, ");

        firstLine.concat("acdcConverter Voltage, ");
        firstLine.concat("battery Voltage, ");

        firstLine.concat("gearSense ADC read, ");
        firstLine.concat("throttleSignal ADC read, ");
        firstLine.concat("brakeSignal ADC read, ");
        firstLine.concat("bspdFault Digital read, ");
        firstLine.concat("killsense Digital read, ");
        firstLine.concat("dataLogButtonPressed Digital read, ");
        firstLine.concat("neutralButtonPressed Digital read, ");
        firstLine.concat("launchButtonPressed Digital read, ");
        firstLine.concat("sdCardDetected Digital read, ");

        firstLine.concat("year GPSdata, ");
        firstLine.concat("month GPSdata, ");
        firstLine.concat("day GPSdata, ");
        firstLine.concat("hour GPSdata, ");
        firstLine.concat("min GPSdata, ");
        firstLine.concat("sec GPSdata, ");
        firstLine.concat("mSec GPSdata, ");
        firstLine.concat("gpsFix GPSdata, ");
        firstLine.concat("fixQual GPSdata, ");
        firstLine.concat("fixQual3d GPSdata, ");
        firstLine.concat("sats GPSdata, ");
        firstLine.concat("latDeg GPSdata, ");
        firstLine.concat("longDeg GPSdata, ");
        firstLine.concat("altitude GPSdata, ");
        firstLine.concat("speed GPSdata, ");
        firstLine.concat("angle GPSdata, ");
        firstLine.concat("magVar GPSdata, ");
        firstLine.concat("hdop GPSdata, ");
        firstLine.concat("vdop GPSdata, ");
        firstLine.concat("pdop GPSdata, ");

        //TODO add IMU stuff here

        if(this->errorPresent)
        {
            dataLine.concat(String("NO ERRORS PRESENT"));
        }
        else
        {
            dataLine.concat("ERRORS PRESENT: " + this->errorstring);
        }

        this->dataFile.println(firstLine);
    }

    //setup GPS
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //ask for type of data
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); //set 10Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA); //request update on antenna

    //TODO
    //setup IMU
}

//TODO
void aioMainMCU::readInputs()
{
    //read all current data
    this->current.shiftSolenoid  = analogRead(CURRENT_SHIFTSOL);
    this->current.GSensor        = analogRead(CURRENT_GSENSOR);
    this->current.syncSensor     = analogRead(CURRENT_SYNC);
    this->current.EGT            = analogRead(CURRENT_EGT);
    this->current.injector       = analogRead(CURRENT_INJECTOR);
    this->current.turboSolenoid  = analogRead(CURRENT_TURBOSOL);
    this->current.LTC            = analogRead(CURRENT_LTC);
    this->current.fuelPump       = analogRead(CURRENT_FP);
    this->current.ignitionCoil   = analogRead(CURRENT_IC);
    this->current.fan            = analogRead(CURRENT_FAN);
    this->current.auxiliaryStage = analogRead(CURRENT_AUX);
    this->current.motec          = analogRead(CURRENT_MOTEC);
    this->current.acdcConverter  = analogRead(CURRENT_ACDC);
    this->current.battery        = analogRead(CURRENT_BATTERY);
    this->current.servo          = analogRead(CURRENT_SERVO);

    //read all voltage data
    this->voltage.acdcConverter = analogRead(VOLTAGE_ACDC);
    this->voltage.battery       = analogRead(VOLTAGE_BATTERY);

    //read all signalInput data
    this->sigIns.gearSense            = analogRead(GEARSENSE_ADC_IN);
    this->sigIns.throttleSignal       = analogRead(THROTTLE_SIG_IN);
    this->sigIns.brakeSignal          = analogRead(BRAKE_SIG_IN);
    this->sigIns.bspdFault            = digitalRead(BSPDFAULT_IN);
    this->sigIns.killsense            = digitalRead(KILLSENSE_IN);
    this->sigIns.dataLogButtonPressed = digitalRead(DATALOG_BUTTON_IN);
    this->sigIns.neutralButtonPressed = digitalRead(BUTTON_IN_NEUTRAL);
    this->sigIns.launchButtonPressed  = digitalRead(BUTTON_IN_LAUNCHCONTROL);
    this->sigIns.sdCardDetected       = digitalRead(SD_CARD_DETECT);

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

    //read all IMU data TODO
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

        // TODO ADD IMU DATA

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
}

bool const aioMainMCU::getError(String &errorDescription)
{
    errorDescription = this->errorstring;
    return this->errorPresent;
}