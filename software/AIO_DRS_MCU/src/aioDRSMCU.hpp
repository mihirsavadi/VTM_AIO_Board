#ifndef AIODRSMCU_CPP
#define AIODRSMCU_CPP

/*
    Mihir Savadi 18/April/2021

    Main DRS MCU class to abstract away all core functionalities and initializations
    to make programming in main.cpp clean, easy, and consistent.

    TODO: Ignore implementation of debouncing for now. See how servo all hooked
        up works if this is needed or not. 
    TODO: UART communication to MAIN MCU implementation. Far less trivial
        then servo controls. Without this the need to build an object orientated
        system seems excessive, but implementing UART will help this.
*/

#include <Arduino.h>
#include <Servo.h>
// https://github.com/arduino-libraries/Servo

//Pin Macros
#define TOG_SW_PIN4  5 
#define TOG_SW_PIN6  4
#define TOG_SW_PIN1  3
#define DRS_BUTTON   2
#define TX1_MAIN_MCU 1
#define RX1_MAIN_MCU 0
#define DRS_SERV_SIG 23
#define BB_SERV_SIG  22

#define BBSERVO

/* main DRS class */
class aioDRSMCU
{
    public:
        /* default constructor to initialize all pins */
        aioDRSMCU();

        //TODO
        /* Function that drives DRS servo control in while loop 
            Must run in infinite while loop with minimal blockage.
            If input argument is true, operation is in "press to actuate"
            mode, where the wing is in low drag when pressed, and in high
            drag when released. If false, wing position toggles with each
            button press*/
        void runDRSservo(bool pressToActuateModeEnabled);

        //TODO 
        /* Function that drives Brake Bias servo control with input from the 
            3-position toggle switch.
            Must run in infinite while loop with minimal blockage. */
        void runBBservo();

    private:
        Servo DRS_servo;
        Servo BB_servo;
};

#endif