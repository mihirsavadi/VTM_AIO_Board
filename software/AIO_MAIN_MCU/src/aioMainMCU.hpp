#ifndef AIOMAINMCU_CPP
#define AIOMAINMCU_CPP

/*
    Mihir Savadi 18th April 2021

    DAQ_PDU AIO BOARD REV 2
    
    Main class to abstract away all core functionalities and initializations
    to make programming in main.cpp clean, easy, and consistent.

    TODO: Incorporate GPS and IMU functionality
    TODO: Implement UART comms with DRS MCU
*/
#include <Arduino.h>

#include "pin_definitions.hpp"

// this library for SD card usage https://github.com/PaulStoffregen/SD
#include <SPI.h>
#include <SD.h> 

// GPS Library Stuff. Must Setup outside class declaration strangely.
#include <Adafruit_GPS.h>

// IMU Library stuff. Must Setup outside class declaration strangely.
#include <MPU9250.h>

#define ERDELIM String(". ")
#define V_OFFSET 0 //volts offset for battery and acdcConverter voltage measurements
#define C_OFFSET 0 //amps offset for all current sensor measurements

/* struct to hold all current sense points */
struct currentData
{
    uint16_t shiftSolenoid;
    uint16_t GSensor;
    uint16_t syncSensor;
    uint16_t EGT;
    uint16_t injector;
    uint16_t turboSolenoid;
    uint16_t LTC;
    uint16_t fuelPump;
    uint16_t ignitionCoil;
    uint16_t fan;
    uint16_t auxiliaryStage;
    uint16_t motec;
    uint16_t acdcConverter;
    uint16_t battery;
    uint16_t servo;
};

/* struct to hold all voltage sense data */
struct voltageData
{
    uint16_t acdcConverter;
    uint16_t battery;
};

/* struct to hold other signal inputs */
struct signalInputs
{
    // analog signal
    uint16_t gearSense;
    uint16_t throttleSignal;
    uint16_t brakeSignal;

    // digital signals
    bool bspdFault;
    bool killsense; //active low signal 
    bool dataLogButtonPressed;
    bool neutralButtonPressed;
    bool launchButtonPressed;
    bool sdCardDetected;

    //GPS Data
    uint8_t year, month, day, hour, min, sec;
    uint16_t mSec;
    bool gpsFix;
    uint8_t fixQual;   //< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
    uint8_t fixQual3d; //< 3D fix quality (1, 3, 3 = Nofix, 2D fix, 3D fix)
    uint8_t sats;      //< Number of satellites in use
    float latDeg, longDeg; // Lat and Long in decimal degres
    float altitude;        // alt in meters above mean sea level
    float speed;           // groundspeed in knots
    float angle;           // course in degrees from true north
    float magVar;          // magnetic variation in degrees vs. true north
    float hdop, vdop, pdop;// relative accuracy of horizontal, vertical, and
                           //   overall position respectively

    //IMU Data
    int IMUbeginStatus;           // < 0 is bad.
    float accelX, accelY, accelZ; //accelerometer measurements in m/(s^2)
    float gyroX, gyroY, gyroZ;    //gyro measurements in rad/s
    float magX, magY, magZ;       //magnetometer measurements in uT
    float IMU_dieTemp;            //IMU die temperature in C
};

/*
    Main class to abstract away all core functionalities and initializations
    to make programming in main.cpp clean, easy, and consistent.
*/
class aioMainMCU
{
    public:
        /* Default constructor to initialize all pins and peripherals */
        aioMainMCU();

        /* read all inputs (analog and digital) and update private fields*/
        void readInputs();

        /* log all data in private fields into sd card in csv format.
            See the .cpp for order in which data is printed per line.
            Note all signals are raw 12bit adc values, which need to be converted
            to amps in post processing.
            This can be handled on the microcontroller in the future easily
            but the scaling factors etc will have to be experimentally found.
        */
        void logAllInputs();

        /* getter for errors. returns true if error present, and returns
            error string by reference argument*/
        bool const getError(String &errorDescription);

        /* convert input ADC reading to current value in amps */
        float const ampsFromADC(uint16_t ADC_reading);

    private:
        /* struct to hold sampled current sense data*/
        currentData current;

        /* struct to hold sampled voltage data */
        voltageData voltage;

        /* struct to hold other signal input data */
        signalInputs sigIns;

        /* error flag. high if error*/
        bool errorPresent;
        /* error string. each error delineated by ". " which is an included macro.
            So every addition to this has to be follow code
            errorstring.append(<insert error description here> + ERDELIM)*/
        String errorstring;

        /* File for storage of data */
        File dataFile;
};

#endif