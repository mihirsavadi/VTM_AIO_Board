#ifndef PIN_DEFINITIONS_CPP
#define PIN_DEFINITIONS_CPP

/* 
    Mihir Savadi 18 April 2021
    File containing all pin definitions for DAQ_PDU AIO BOARD REV 2
*/

// UART PORTS
#define RX1_DASH 0
#define TX1_DASH 1

#define RX3_GPS 7
#define TX3_GPS 8

#define RX9_TELEMETRY 9
#define TX9_TELEMETRY 10

#define RX4_DRS_MCU 31
#define TX4_DRS_MCU 32

// SPI PORTS
#define SPI_MOSI_0 11
#define SPI_MISO_0 12
#define SPI_SCK_0  13
#define SPI_CS_0   15

// I2C PORTS
#define SCL1 37
#define SDA1 38

// CANBUS PORTS
#define CANBUS_TX       29
#define CANBUS_RX       30
#define CANSPEEDSELECT  24

// DIGITAL SIGNAL INPUTS
#define BSPDFAULT_IN            2
#define KILLSENSE_IN            4
#define DATALOG_BUTTON_IN       5
#define BUTTON_IN_NUETRAL       25
#define BUTTON_IN_LAUNCHCONTROL 26
#define SD_CARD_DETECT          27

// ANALOG SIGNAL INPUTS
#define STEERPOS_IN      3 //this is not an ADC port. Hardware error. Do not use.

#define GEARSENSE_ADC_IN A10

#define THROTTLE_SIG_IN  A2
#define BRAKE_SIG_IN     A3

#define VOLTAGE_ACDC     A16
#define VOLTAGE_BATTERY  A17

#define CURRENT_SHIFTSOL A0
#define CURRENT_GSENSOR  A11
#define CURRENT_SYNC     A21
#define CURRENT_EGT      A22
#define CURRENT_INJECTOR A25
#define CURRENT_TURBOSOL A26
#define CURRENT_LTC      A20
#define CURRENT_FP       A4
#define CURRENT_IC       A5
#define CURRENT_FAN      A6
#define CURRENT_AUX      A7
#define CURRENT_MOTEC    A8
#define CURRENT_ACDC     A9
#define CURRENT_BATTERY  A14
#define CURRENT_SERVO    A15


#endif