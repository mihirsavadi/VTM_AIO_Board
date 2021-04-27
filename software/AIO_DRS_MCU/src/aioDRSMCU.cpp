/* Mihir Savadi 18/April/2021 */

#include "aioDRSMCU.hpp"

aioDRSMCU::aioDRSMCU()
{
    DRS_servo.attach(DRS_SERV_SIG);
    BB_servo.attach(BB_SERV_SIG);

    pinMode(DRS_BUTTON, INPUT);
}


//see this for reference for next two methods
// https://github.com/arduino-libraries/Servo/blob/master/examples/Knob/Knob.ino


//TODO
void aioDRSMCU::runDRSservo(bool pressToActuateModeEnabled)
{
    if (pressToActuateModeEnabled)
    {
        if (digitalReadFast(DRS_BUTTON))
        {
            DRS_servo.write(DRS_LOWDRAG_SERVOANGLE);
        }
        else
        {
            DRS_servo.write(DRS_HIGHDRAG_SERVOANGLE);
        }
    }
    else
    {
        if (digitalReadFast(DRS_BUTTON))
        {
            this->DRS_LowDragFlag = !this->DRS_LowDragFlag;
        }
        DRS_servo.write(this->DRS_LowDragFlag);
        delay(10); //for debounce effect
    }
}

//TODO
void aioDRSMCU::runBBservo()
{

}
