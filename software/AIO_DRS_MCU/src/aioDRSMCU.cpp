/* Mihir Savadi 18/April/2021 */

#include "aioDRSMCU.hpp"

aioDRSMCU::aioDRSMCU()
{
    this->DRS_servo.attach(DRS_SERV_SIG);
    this->BB_servo.attach(BB_SERV_SIG);

    pinMode(DRS_BUTTON, INPUT);
}


//see this for reference for next three methods
// https://github.com/arduino-libraries/Servo/blob/master/examples/Knob/Knob.ino

void aioDRSMCU::runDRSservo(bool pressToActuateModeEnabled)
{
    if (pressToActuateModeEnabled)
    {
        if (digitalReadFast(DRS_BUTTON))
            this->DRS_servo.write(DRS_LOWDRAG_SERVOANGLE);
        else
            this->DRS_servo.write(DRS_HIGHDRAG_SERVOANGLE);
    }
    else
    {
        if (digitalReadFast(DRS_BUTTON))
            this->DRS_LowDragFlag = !this->DRS_LowDragFlag;

        if (this->DRS_LowDragFlag)
            this->DRS_servo.write(DRS_LOWDRAG_SERVOANGLE);
        else
            this->DRS_servo.write(DRS_HIGHDRAG_SERVOANGLE);
        delay(10); //for debounce effect
    }
}

//TODO
void aioDRSMCU::runBBservo()
{

}

#if includeSerialPrints == true
    void aioDRSMCU::manualServoControl(bool controlDRS)
    {
        String command;
        int servoPos = 180;
        int increment = 5;

        //set servo to middle position before doing anything else
        Serial.print("Manual Servo Control begun. ");
        if (controlDRS)
        {
            this->DRS_servo.write(servoPos);
            Serial.println("DRS Servo in control, set to: " + String(servoPos));
        }
        else
        {
            this->BB_servo.write(servoPos);
            Serial.println("BB Servo in control, set to: " + String(servoPos));

        }

        while(1)
        {
            if (Serial.available())
            {
                command = Serial.readStringUntil('\n');

                if (command.equals("u"))
                {
                    servoPos = servoPos + increment;
                    if (servoPos > 180)
                        servoPos = 180;
                    
                    if (controlDRS)
                    {
                        this->DRS_servo.write(servoPos);
                        Serial.println("DRS Servo Pos: " + String(servoPos));
                    }
                    else
                    {
                        this->BB_servo.write(servoPos);
                        Serial.println("BB Servo Pos: " + String(servoPos));
                    }
                }
                else if (command.equals("d"))
                {
                    servoPos = servoPos - increment;
                    if (servoPos < 0)
                        servoPos = 0;
                    
                    if (controlDRS)
                    {
                        this->DRS_servo.write(servoPos);
                        Serial.println("DRS Servo Pos: " + String(servoPos));
                    }
                    else
                    {
                        this->BB_servo.write(servoPos);
                        Serial.println("BB Servo Pos: " + String(servoPos));
                    }
                }
            }
        }
    }
#endif
