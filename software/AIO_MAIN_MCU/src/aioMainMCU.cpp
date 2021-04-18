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
    // to put lines into datafile from here into a new line just do 
    //  "this->dataFile.println(<insert string here>);"

    //TODO
    //setup IMU

    //TODO
    //setup GPS
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
}

//TODO
void aioMainMCU::logAllInputs()
{
    String dataLine;

    if(dataFile)
    {
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

        dataLine.concat(String(this->voltage.acdcConverter) + ", ");
        dataLine.concat(String(this->voltage.battery) + ", ");

        dataLine.concat(String(this->sigIns.gearSense) + ", ");
        dataLine.concat(String(this->sigIns.throttleSignal) + ", ");
        dataLine.concat(String(this->sigIns.brakeSignal) + ", ");
        dataLine.concat(String(this->sigIns.bspdFault) + ", ");
        dataLine.concat(String(this->sigIns.killsense) + ", ");
        dataLine.concat(String(this->sigIns.dataLogButtonPressed) + ", ");
        dataLine.concat(String(this->sigIns.neutralButtonPressed) + ", ");
        dataLine.concat(String(this->sigIns.launchButtonPressed) + ", ");
        dataLine.concat(String(this->sigIns.sdCardDetected));
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