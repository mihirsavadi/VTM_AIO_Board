#ifndef AIOMAINMCU_CPP
#define AIOMAINMCU_CPP

/*
    Mihir Savadi 18th April 2021

    Main class to abstract away all core functionalities and initializations
    to make programming in main.cpp clean, easy, and consistent.
*/
#include <Arduino.h>

#include "pin_definitions.hpp"

// this library for SD card usage https://github.com/PaulStoffregen/SD
#include <SPI.h>
#include <SD.h> 

// #include <Adafruit_GPS.h> 

#define ERDELIM String(". ")

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

        /* log all data in private fields into sd card in csv format
            TODO: establish order of data in line
            TODO: how to log errors
        */
        void logAllInputs();

        /* getter for errors. returns true if error present, and returns
            error string by reference argument*/
        bool const getError(String &errorDescription);

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