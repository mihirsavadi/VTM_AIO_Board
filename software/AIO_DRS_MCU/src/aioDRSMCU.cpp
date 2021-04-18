/* Mihir Savadi 18/April/2021 */

#include "aioDRSMCU.hpp"

aioDRSMCU::aioDRSMCU()
{
    DRS_servo.attach(DRS_SERV_SIG);
    BB_servo.attach(BB_SERV_SIG);
}


//see this for reference for next two methods
// https://github.com/arduino-libraries/Servo/blob/master/examples/Knob/Knob.ino


//TODO
void aioDRSMCU::runDRSservo(bool pressToActuateModeEnabled)
{
    if (pressToActuateModeEnabled)
    {
        
    }
    else
    {

    }
}

//TODO
void aioDRSMCU::runBBservo()
{

}
