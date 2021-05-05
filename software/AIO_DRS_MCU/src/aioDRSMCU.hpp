#ifndef AIODRSMCU_CPP
#define AIODRSMCU_CPP

/*
    Mihir Savadi 18/April/2021

    Main DRS MCU class to abstract away all core functionalities and initializations
    to make programming in main.cpp clean, easy, and consistent.

    DAQ_PDU AIO BOARD REV 2

    TODO: Ignore implementation of debouncing for now. See how servo all hooked
        up works if this is needed or not. 
    TODO: UART communication to MAIN MCU implementation. Far less trivial
        then servo controls. Without this the need to build an object orientated
        system seems excessive, but implementing UART will help this.
*/

//this macro enables macro blocks in main cpp as well as the manualServoControl()
// method.
#define includeSerialPrints true

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

#define DRS_HIGHDRAG_SERVOANGLE 30
#define DRS_LOWDRAG_SERVOANGLE 40

/* main DRS class */
class aioDRSMCU
{
    public:
        /* default constructor to initialize all pins */
        aioDRSMCU();

        /* Function that drives DRS servo control.
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

        #if includeSerialPrints == true
            /* Function that takes in inputs from serial monitor to move the servo up or down. Type into serial monitor 'u' or 'd' and hit enter. 'u' will move it up, 'd' will move it down by a certain increment. Use this to help manually find positioning of servo in whatever mount. This function already has an infinite while loop inside it so it will be absolutely blocking in main.cpp, and does not need to be placed in an infinite while loop. runBBservo() and runDRSservo() must not be called when using this. If the argument is true, the DRS servo will be controlled, if false the brakebias servo will be controlled. Upload code via platform-io but use the stock arduino monitor for easiest usage*/
            void manualServoControl(bool controlDRS);
        #endif

    private:
        Servo DRS_servo;
        Servo BB_servo;

        bool DRS_LowDragFlag = false;
};

#endif