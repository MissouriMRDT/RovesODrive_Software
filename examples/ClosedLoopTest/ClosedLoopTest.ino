#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
String targetString;
float pos_est, set_pos, tvlimit, trapAPC, talimit, tdlimit;
Error_Axis axerror;
Error_Motor merror;
Error_Encoder enerror;
Error_Controller cerror;

void setup()
{
    Serial.begin(115200);
    Drive1.begin();

    while (!Serial); // wait for Arduino Serial Monitor to open   
    
    char output[255];

    Serial.println("ODriveArduino");
    Serial.println("Setting parameters...");    
    Serial7.write("w axis0.requested_state 8 \n");
    Serial.println("Initialised");
    delay(100);
}

void loop()
{
    if(Serial7.available())
    {
        Serial.print("--->");
        while(Serial7.available())
        {
            Serial.write(Serial7.read());
        }
    }
    
    while (Serial.available()) 
    {
        char c = Serial.read();  
        targetString += c; 
        delay(2);  
    }

    if (targetString.length() >0) 
    {
        Serial.println(targetString);

        if (targetString == "reboot")
        {
            Serial.println("Rebooting");
            Drive1.motor[0].reboot();
        }

        else if (targetString == "pestimate")
        {
            serial.println("Requesting Position Estimate...");
            pos_est =  Drive1.motor[0].requestPosEstimate();
            Serial.println(pos_est);
            pos_est = 0;
        }

        else if (targetString == "axisError")
        {
            serial.println("Checking Axis Errors...");
            axerror =  Drive1.motor[0].checkAxisErrors();
            Serial.println(axerror);

            if(axerror != ERROR_NONE_A)
            {
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axerror = 0;
            }
        }

        else if (targetString == "motorError")
        {
            serial.println("Checking Motor Errors");
            merror =  Drive1.motor[0].checkMotorErrors();
            Serial.println(merror);

            if(merror != ERROR_NONE_M)
            {
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                merror = 0;
            }
        }

        else if (targetString == "encoderError")
        {
            serial.println("Checking Encoder Errors...");
            enerror =  Drive1.motor[0].checkEncoderErrors();
            Serial.println(enerror);

            if(enerror != ERROR_NONE_E)
            {
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                enerror = 0;
            }
        }

        else if (targetString == "controllerError")
        {
            serial.println("Checking Controller Errors");
            cerror =  Drive1.motor[0].checkControllerErrors();
            Serial.println(cerror);
            
            if(cerror != ERROR_NONE_C)
            {
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                cerror = 0;
            }
        }

        else if (targetString == "saveConfig")
        {
            serial.println("Saving Configuration...");
            Serial.println("Rebooting");
            Drive1.motor[0].saveConfig();
        }

        else if (targetString == "eraseConfig")
        {
            serial.println("Erasing Configuration...");
            Serial.println("Rebooting");
            Drive1.motor[0].eraseConfig();
        }

        else if (targetString == "positionSetPoint")
        {
            serial.println("Setting Position Set Point...");
            set_pos = targetString.toFloat();
            Drive1.motor[0].posSetPoint();
        }

        else if (targetString == "trapVLimit")
        {
            Serial.println("Setting Trap Velocity Limit...");
            tvlimit = targetString.toFloat();
            Drive1.motor[0].setTrapVelocityLimit(vlimit_T);
        }

        else if (targetString == "trapALimit")
        {
            serial.println("Setting Trap Acceleration Limit...");
            talimit = targetString.toFloat();
            Drive1.motor[0].setTrapAccelerationLimit(alimit_T);
        }

        else if (targetString == "trapDLimit")
        {
            serial.println("Setting Trap Deceleration Limit...");
            tdlimit = targetString.toFloat();
            Drive1.motor[0].setTrapDecelerationLimit(dlimit_T);
        }        

        else if (targetString == "trapAPC")
        {
            serial.println("Setting Trap Acceleration Per Counts...");
            trapAPC = targetString.toFloat();
            Drive1.motor[0].setTrapAccelerationPerCounts(trapAPC);
        }

        else 
        {
            int command = targetString.toInt();
            char output[255];

            serial.println("Setting Trap Target...");
            Drive1.motor[0].setTrapTarget(command);
        }
        targetString = "";
    }
}